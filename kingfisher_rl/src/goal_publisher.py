#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

import sensor_msgs.point_cloud2 as pc2

import numpy as np


class GoalPublisher:
    def __init__(self):

        rospy.init_node('goal_publisher', log_level=rospy.DEBUG)


        # Marker definition
        self.goal_marker = Marker()
        self.goal_marker.header.frame_id = "base_link"
        self.goal_marker.type = Marker.ARROW
        self.goal_marker.action = Marker.ADD
        self.goal_marker.scale.x = -0.5
        self.goal_marker.scale.y = 0.1
        self.goal_marker.scale.z = 0.1
        self.goal_marker.color.a = 0.8
        self.goal_marker.color.r = 1.0
        self.goal_marker.color.g = 0.0
        self.goal_marker.color.b = 0.0
        self.goal_marker.pose.orientation.x = 0.0
        self.goal_marker.pose.orientation.y = 0.7068252
        self.goal_marker.pose.orientation.z = 0.0
        self.goal_marker.pose.orientation.w = 0.7068252

        # Move base simple goal message definition
        self.goal_msg = PoseStamped()


        # Define the publisher
        self.marker_publisher = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        self.goal_publisher = rospy.Publisher('/waste_detector/goal', PoseStamped, queue_size=1)

        # Subscribes to a point cloud image
        self.point_cloud_sub = rospy.Subscriber('/waste_detector/detections', PointCloud2, self.point_cloud_cb)


    def update_local_frame(self, msg):
        """
        Updates the pointcloud in the local frame after converting it from global to base_link frame.
        """
        # TODO
        rospy.loginfo("TODO: update frame")
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
            # Publish the goal marker
            self.goal_marker.pose.position = selected_goal.pose.position
            self.goal_marker.header.stamp = selected_goal.header.stamp
            self.marker_publisher.publish(self.goal_marker)
            rospy.logdebug("Published goal marker")

            # Publish the goal
            self.goal_publisher.publish(selected_goal)


if __name__ == '__main__':
    try:
        goal_publisher = GoalPublisher()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass