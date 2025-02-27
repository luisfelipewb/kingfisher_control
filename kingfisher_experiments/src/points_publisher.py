#!/usr/bin/env python3

import rospy
import yaml
import rospkg
from geometry_msgs.msg import PoseArray, Pose

def load_yaml_config(file_path):
    """Load the YAML configuration file."""
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

def publish_points(points):
    """Publish points as PoseArray."""
    # Initialize ROS node
    rospy.init_node('goals_publisher')

    # Create a publisher
    pub = rospy.Publisher('/goals', PoseArray, queue_size=10)

    # Wait for the publisher to connect
    rospy.sleep(1)

    # Create PoseArray message
    pose_array = PoseArray()
    pose_array.header.frame_id = "base_link"  # You can change this to the relevant frame

    for point in points:
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

    # Publish the PoseArray message
    pub.publish(pose_array)
    rospy.loginfo(f"Published PoseArray with {len(pose_array.poses)} poses")

if __name__ == '__main__':
    try:
        # Load YAML configuration
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('kingfisher_experiments') + '/config/' + 'zigzag.yaml'
        # test if the file exists
        try:
            with open(file_path, 'r') as file:
                pass
                data = yaml.safe_load(file)
        except FileNotFoundError:
            rospy.logerr(f"File {file_path} not found")
        config = load_yaml_config(file_path)

        # Get the points from the YAML configuration
        points = config['experiments'][0]['points']

        # Publish points as PoseArray
        publish_points(points)
    except rospy.ROSInterruptException:
        pass