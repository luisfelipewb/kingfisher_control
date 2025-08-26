#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped

import sensor_msgs.point_cloud2 as pc2

import numpy as np


class GoalPublisher:
    def __init__(self):

        rospy.init_node('goal_publisher', log_level=rospy.DEBUG)

        self.max_goal_distance = rospy.get_param('~max_goal_distance', 5.0)

        # Move base simple goal message definition
        self.goal_msg = PoseStamped()

        # Define the publisher
        self.goal_publisher = rospy.Publisher('~goal', PoseStamped, queue_size=1)

        # Subscribes to a point cloud image
        self.point_cloud_sub = rospy.Subscriber('~detections', PointCloud2, self.point_cloud_cb)


    def update_local_frame(self, msg):
        """
        Updates the pointcloud in the local frame after converting it from global to base_link frame.
        """
        if msg.header.frame_id != "base_link":
            rospy.logwarn("Should check frame transformation")
        return msg
        

    def select_goal(self, msg):
        """
        Selects the closest point from the point cloud and publishes it as a goal message.
        """
        goal_pose = None

        # Convert the PointCloud2 message to a list of points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        if points:
            # Find the closest point
            closest_point = None
            min_distance = float('inf')

            for point in points:
                distance = np.linalg.norm([point[0], point[1], point[2]])
                if distance < min_distance:
                    min_distance = distance
                    closest_point = point

            # Create a PoseStamped message
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = msg.header.frame_id
            goal_pose.header.stamp = msg.header.stamp
            goal_pose.pose.position.x = closest_point[0]
            goal_pose.pose.position.y = closest_point[1]
            goal_pose.pose.position.z = closest_point[2]
            goal_pose.pose.orientation.w = 1.0  # Assuming no orientation for simplicity
            rospy.logdebug(f"Selected goal: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}, {goal_pose.pose.position.z}")

        else:
            rospy.logdebug("No valid points in the point cloud")

        if min_distance > self.max_goal_distance:
            rospy.logwarn(f"Selected goal is too far: {min_distance}m (max allowed: {self.max_goal_distance}m)")
            return None

        return goal_pose
    

    def point_cloud_cb(self, msg):
        """
        Receives a point cloud message with target points. 
        Selects the closest point and publishes it as a goal message.
        """
        rospy.logdebug("Received point cloud message")

        updated_msg = self.update_local_frame(msg)

        selected_goal = self.select_goal(updated_msg)

        if selected_goal is not None:
            self.goal_publisher.publish(selected_goal)


if __name__ == '__main__':
    try:
        goal_publisher = GoalPublisher()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass