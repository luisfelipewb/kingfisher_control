#!/usr/bin/env python3

import rospy
import yaml
import rospkg
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf
from numpy.linalg import norm
from topic_tools.srv import MuxSelect


class WaypointPublisher:
    def __init__(self):

        self.listener = tf.TransformListener()
        self.odom_sub = rospy.Subscriber('~odom', Odometry, self.odom_callback, queue_size=1)
        self.world_frame = None
        self.wait_for_world_frame()

        self.config_file = rospy.get_param('~config_file', 'empty.yaml')
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('kingfisher_experiments') + '/config/' + self.config_file
        self.points, self.frame_id = self.load_yaml_config(file_path)
        self.pose_array = self.setup_pose_array()

        self.goals_pub = rospy.Publisher('~goals', PoseArray, queue_size=1)
        self.next_goal_pub = rospy.Publisher('~goal', PoseStamped, queue_size=1)
        self.agent_status = True

        self.current_point_idx = 0
        self.dist_threshold = rospy.get_param('~dist_threshold', 1.5)
        self.goal_bounds_threshold = rospy.get_param('~goal_bounds_threshold', 4.0)

        self.last_goal_time = rospy.Time.now()
        self.lost_control = True
        self.goal_mux_srv = rospy.ServiceProxy('/goal_mux/select', MuxSelect)
        self.perception_goal = None
        self.next_waypoint = None

        self.perception_goal_sub = rospy.Subscriber('~perception_goal', PoseStamped, self.monitor_perception_goal_callback, queue_size=1)
        self.agent_status_sub = rospy.Subscriber('~status_topic', Bool, self.agent_status_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.get_next_point)

    def select_mux(self, topic):
        try:
            self.goal_mux_srv(topic)
            rospy.loginfo(f"Select {topic} input")
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

    def wait_for_world_frame(self):
        """Wait for the world frame to be available."""
        while self.world_frame is None:
            rospy.logwarn("Waiting for the world_frame to be set from odom")
            rospy.sleep(1)

    def odom_callback(self, msg):
        if self.world_frame is None:
            self.world_frame = msg.header.frame_id
        else:
            return

    def load_yaml_config(self, file_path):
        """Load the YAML configuration file."""
        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)

        waypoints = config['waypoints']
        frame_id = config['frame_id']
        offset = config['offset']
        for point in waypoints:
            point[0] += offset[0]
            point[1] += offset[1]

        return waypoints, frame_id

    def setup_pose_array(self):
        """Publish points as PoseArray."""
        # Initialize ROS node
        rospy.init_node('waypoint_publisher')

        # Wait for the publisher to connect
        rospy.sleep(1)

        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame_id

        for point in self.points:
            # Create Pose message
            pose = Pose()

            # Fill in the position values from the YAML points
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = 0.0  # Assuming a flat trajectory

            # Assuming zero orientation (no rotation)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            # Append the pose to the pose array
            pose_array.poses.append(pose)

        # Set timestamp
        pose_array.header.stamp = rospy.Time.now()

        # Transform the PoseArray to the world frame using tf
        try:
            self.listener.waitForTransform(pose_array.header.frame_id, self.world_frame, rospy.Time.now(), rospy.Duration(1.0))
            for i in range(len(pose_array.poses)):
                created_pose = PoseStamped()
                created_pose.header = pose_array.header
                created_pose.pose = pose_array.poses[i]
                # update timestamp
                created_pose.header.stamp = rospy.Time(0)
                transformed_pose = self.listener.transformPose(self.world_frame, created_pose)
                pose_array.poses[i] = transformed_pose.pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming PoseArray: {e}")
            return

        # update the frame_id
        pose_array.header.frame_id = self.world_frame
        # Publish the PoseArray message
        rospy.loginfo(f"Created PoseArray with {len(pose_array.poses)} poses")
        # print(pose_array)
        return pose_array

    def monitor_perception_goal_callback(self, msg):

        """ This callback is triggered by a new perception goal. Here we just store the goal in the world frame."""

        try:
            self.listener.waitForTransform(msg.header.frame_id, self.world_frame, rospy.Time(0), rospy.Duration(1.0))
            global_goal = self.listener.transformPose(self.world_frame, msg)
            x = global_goal.pose.position.x
            y = global_goal.pose.position.y
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming goal to world frame: {e}")
            return

        # Update the frame and position of the perception goal
        msg.header.frame_id = self.world_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0


        self.last_goal_time = rospy.Time.now()
        self.perception_goal = msg


    def agent_status_callback(self, msg):
        # Store the agent status
        self.agent_status = msg.data

    def goal_reached(self):
        """Check if the goal point is reached."""

        # transform the next goal to the base_link frame
        self.listener.waitForTransform(self.world_frame, 'base_link', rospy.Time.now(), rospy.Duration(1.0))
        try:
            current_goal = PoseStamped()
            current_goal.header.frame_id = self.world_frame
            current_goal.pose = self.pose_array.poses[self.current_point_idx]
            current_goal.header.stamp = rospy.Time(0)
            transformed_goal = self.listener.transformPose('base_link', current_goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming Pose: {e}")
            return False

        dist = norm([transformed_goal.pose.position.x, transformed_goal.pose.position.y])
        # rospy.loginfo_throttle(1, "Distance to goal: %.2f" % dist)
        if dist < self.dist_threshold:
            return True
        return False


    def get_next_point(self, event):

        if self.goal_reached(): # TODO: Check if reached the point
            self.current_point_idx = (self.current_point_idx + 1) % len(self.pose_array.poses)
        self.next_waypoint = self.pose_array.poses[self.current_point_idx]

    def publish_goal(self, point):
        """Publish a goal point as PoseStamped."""
        goal = PoseStamped()
        goal.header.frame_id = self.world_frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = point.position.x
        goal.pose.position.y = point.position.y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1
        self.next_goal_pub.publish(goal)
        self.goals_pub.publish(self.pose_array)


    def check_goal_bounds(self, point):
        """ Check if the point is close to the next waypoint."""
        if not self.next_waypoint:
            return False
        if not point:
            return False

        dist = norm([point.pose.position.x - self.next_waypoint.position.x,
                      point.pose.position.y - self.next_waypoint.position.y])
        # print(f"Distance between goal and next waypoint: {dist}")
        return dist < self.goal_bounds_threshold

    def loop(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            if not self.agent_status:
                if self.lost_control:
                    self.select_mux('/waypoint_publisher/goal')
                    self.perception_goal = None
                    self.lost_control = False
            elif self.check_goal_bounds(self.perception_goal) and self.agent_status:
                if not self.lost_control:
                    self.select_mux(self.perception_goal_sub.resolved_name)
                    self.lost_control = True
            elif self.next_waypoint:
                if self.lost_control:
                    self.select_mux('/waypoint_publisher/goal')
                    self.lost_control = False

            if self.next_waypoint:
                self.publish_goal(self.next_waypoint)
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_publisher')

        # Create PointsPublisher instance
        points_publisher = WaypointPublisher()
        # Publish points as PoseArray
        points_publisher.loop()
    except rospy.ROSInterruptException:
        pass