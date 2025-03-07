#!/usr/bin/env python3

import rospy
import yaml
import rospkg
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry
import tf
from numpy.linalg import norm
from topic_tools.srv import MuxSelect


class WaypointPublisher:
    def __init__(self, config_file):
        self.config_file = config_file
        self.points = self.load_yaml_config()
        self.goals_pub = rospy.Publisher('~goals', PoseArray, queue_size=1)
        self.next_goal_pub = rospy.Publisher('~goal', PoseStamped, queue_size=1)

        self.odom_sub = rospy.Subscriber('~odom', Odometry, self.odom_callback, queue_size=1)
        self.goal_mon1_sub = rospy.Subscriber('~monitor_goal1', PoseStamped, self.monitor_goal1_callback, queue_size=1)
        self.goal_mon2_sub = rospy.Subscriber('~monitor_goal2', PoseStamped, self.monitor_goal2_callback, queue_size=1)
        self.listener = tf.TransformListener()
        self.world_frame = None
        self.wait_for_world_frame()
        self.pose_array = self.setup_pose_array()
        self.current_point_idx = 0
        self.dist_threshold = rospy.get_param('~dist_threshold', 1.5)
        self.last_goal_time = rospy.Time.now()
        self.lost_control = True
        self.goal_mux_srv = rospy.ServiceProxy('/mux_goal/select', MuxSelect)

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

    def load_yaml_config(self):
        """Load the YAML configuration file."""
        with open(self.config_file, 'r') as file:
            config = yaml.safe_load(file)
        return config['experiments'][0]['points']

    def setup_pose_array(self):
        """Publish points as PoseArray."""
        # Initialize ROS node
        rospy.init_node('waypoint_publisher')

        # Wait for the publisher to connect
        rospy.sleep(1)

        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link"  # You can change this to the relevant frame

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

    def monitor_goal1_callback(self, msg):
        self.last_goal_time = rospy.Time.now() #rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
        self.lost_control = True
        rospy.loginfo(f"Detected goal from {self.goal_mon1_sub.resolved_name}")
        self.select_mux(self.goal_mon1_sub.resolved_name)
    
    def monitor_goal2_callback(self, msg):
        self.last_goal_time =  rospy.Time.now() #rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
        self.lost_control = True
        rospy.loginfo(f"Detected goal from {self.goal_mon2_sub.resolved_name}")
        self.select_mux(self.goal_mon2_sub.resolved_name)
    

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
        rospy.loginfo_throttle(1, "Distance to goal: %.2f" % dist)
        if dist < self.dist_threshold:
            return True
        return False


    def get_next_point(self):

        if self.goal_reached(): # TODO: Check if reached the point
            self.current_point_idx = (self.current_point_idx + 1) % len(self.pose_array.poses)
        point = self.pose_array.poses[self.current_point_idx]
        # print(f"Next point: {point}")
        return point
    
    def publish_goal(self, point):
        """Publish a goal point as PoseStamped."""
        goal = PoseStamped()
        goal.header.frame_id = self.world_frame
        goal.pose.position.x = point.position.x
        goal.pose.position.y = point.position.y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1
        self.next_goal_pub.publish(goal)
        self.goals_pub.publish(self.pose_array)


    
    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # should be out of the loop in case the point is reached while the agent is controlling the movement
            point = self.get_next_point()

            if rospy.Time.now() - self.last_goal_time < rospy.Duration(5): #TODO: parameter
                continue
            
            # Make sure to get the mux back 
            if self.lost_control:
                self.lost_control = False
                self.select_mux('waypoint_publisher/goal')
            
            self.publish_goal(point)

            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_publisher')

        # Load YAML configuration
        rospack = rospkg.RosPack()
        # Get file name from ros param 
        pattern = rospy.get_param('~pattern', 'grid')
        print(pattern)
        file_path = rospack.get_path('kingfisher_experiments') + '/config/' + f'{pattern}.yaml'
        try:
            with open(file_path, 'r') as file:
                pass
        except FileNotFoundError:
            rospy.logerr(f"File {file_path} not found")
            exit(1)

        # Create PointsPublisher instance
        points_publisher = WaypointPublisher(file_path)

        # Publish points as PoseArray
        points_publisher.loop()
    except rospy.ROSInterruptException:
        pass