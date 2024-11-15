#!/usr/bin/python3

import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Vector3Stamped
from kingfisher_msgs.msg import Drive
from nav_msgs.msg import Odometry

from kingfisher_experiments.srv import SetFloat, SetFloatResponse

import tf2_geometry_msgs
import tf2_ros

import threading


class OdomTransform:

    def __init__(self):
        self.target_frame = rospy.get_param('~target_frame', 'base_link')

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("~odom", Odometry, self.odom_cb, queue_size=10)
        self.odom_pub = rospy.Publisher("~transformed_odom", Odometry, queue_size=10)

    def transform_twist(self, twist, transform):
        """
        Transforms a geometry_msgs/Twist message from one frame to another.

        :param twist: The Twist message to transform.
        :param transform: The transform to apply.
        :return: The transformed Twist message.
        """
        # Create a Vector3Stamped message for the linear and angular components of the twist message
        twist_linear = Vector3Stamped()
        twist_linear.vector = twist.linear
        twist_angular = Vector3Stamped()
        twist_angular.vector = twist.angular

        # Transform the linear and angular components of the twist message
        twist_linear_transformed = tf2_geometry_msgs.do_transform_vector3(twist_linear, transform)
        twist_angular_transformed = tf2_geometry_msgs.do_transform_vector3(twist_angular, transform)

        # Create a new Twist message and assign the transformed components to it
        twist_transformed = Twist()
        twist_transformed.linear = twist_linear_transformed.vector
        twist_transformed.angular = twist_angular_transformed.vector

        return twist_transformed

    def odom_cb(self, msg):
        """
        Callback function for the odometry message. Publishes the transformed odometry message.

        Args:
            msg (nav_msgs.msg.Odometry): The odometry message.

        Returns:
            None
        """
        # Rest of the code...
        child_frame_id = msg.child_frame_id
        frame_id = msg.header.frame_id

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        twist_stamped = TwistStamped()
        twist_stamped.header = msg.header
        twist_stamped.twist = msg.twist.twist

        try:
            # Transform the pose, usually in the imu frame to the desired frame (e.g. base_link)
            transform = self.tf_buffer.lookup_transform(self.target_frame, child_frame_id, rospy.Time())
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Failed to find transform from [{frame_id}] to [{self.target_frame}] frame")
            return
        try:
            # Transform the twist, usually in a world frame to the desired frame (e.g. base_link)
            transform = self.tf_buffer.lookup_transform(self.target_frame, frame_id, rospy.Time())
            twist_transformed = self.transform_twist(twist_stamped.twist, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Failed to find transform from [{frame_id}] to [{self.target_frame}] frame")
            return

        new_odom = Odometry()
        new_odom = msg
        new_odom.child_frame_id = self.target_frame
        #new_odom.pose.pose = pose_transformed.pose
        new_odom.twist.twist = twist_transformed

        rospy.logdebug(f"publishing transofmed odometry message to {self.odom_pub.name}")
        self.odom_pub.publish(new_odom)


if __name__ == '__main__':

    rospy.init_node('odom_transform')

    odom_transform = OdomTransform()

    rospy.spin()
