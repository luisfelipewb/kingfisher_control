#!/usr/bin/env python3

import rospy
import yaml
import rospkg
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry
import tf

class PointsPublisher:
    def __init__(self, config_file):
        self.config_file = config_file
        self.points = self.load_yaml_config()
        self.pub = rospy.Publisher('/goals', PoseArray, queue_size=1)
        rospy.Subscriber('/local_odom', Odometry, self.odom_callback, queue_size=1)
        self.world_frame = None
        self.wait_for_world_frame()

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

    def publish_points(self):
        """Publish points as PoseArray."""
        # Initialize ROS node
        rospy.init_node('goals_publisher')

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
            listener = tf.TransformListener()
            listener.waitForTransform(pose_array.header.frame_id, self.world_frame, rospy.Time(0), rospy.Duration(1.0))
            for i in range(len(pose_array.poses)):
                created_pose = PoseStamped()
                created_pose.header = pose_array.header
                created_pose.pose = pose_array.poses[i]
                # update timestamp
                created_pose.header.stamp = rospy.Time(0)
                transformed_pose = listener.transformPose(self.world_frame, created_pose)
                pose_array.poses[i] = transformed_pose.pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming PoseArray: {e}")
            return

        # update the frame_id
        pose_array.header.frame_id = self.world_frame
        # Publish the PoseArray message
        self.pub.publish(pose_array)
        rospy.loginfo(f"Published PoseArray with {len(pose_array.poses)} poses")
        # print(pose_array)

if __name__ == '__main__':
    try:
        rospy.init_node('goals_publisher')

        # Load YAML configuration
        rospack = rospkg.RosPack()
        # Get file name from ros param 
        pattern = rospy.get_param('~pattern', 'asdf')
        print(pattern)
        file_path = rospack.get_path('kingfisher_experiments') + '/config/' + f'{pattern}.yaml'
        # test if the file exists
        try:
            with open(file_path, 'r') as file:
                pass
        except FileNotFoundError:
            rospy.logerr(f"File {file_path} not found")
            exit(1)

        # Create PointsPublisher instance
        points_publisher = PointsPublisher(file_path)

        # Publish points as PoseArray
        points_publisher.publish_points()
    except rospy.ROSInterruptException:
        pass