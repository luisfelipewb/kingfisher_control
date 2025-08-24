#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import tf2_geometry_msgs
import tf2_ros


class OdomTransform:
    """
    ODOM TWIST FRAME CORRECTION NODE

    PURPOSE:
    Corrects twist frame convention to enable seamless deployment between
    simulation and real-world experiments.

    BACKGROUND:
    - Gazebo simulation publishes twist in world frame (non-standard behavior)
    - SBG localization was intentionally configured to match Gazebo's behavior
    - This allows identical code/configuration for both simulation and field tests
    - Standard ROS convention: twist should be in child_frame_id (base_link)

    SOLUTION:
    Transform twist from world frame to base_link frame for both environments.
    Includes configurable localization delay for robustness testing.

    NOTE: SBG behavior is intentional design choice, not a bug.
    """

    def __init__(self):
        rospy.init_node('odom_transform')

        self.target_frame = rospy.get_param('~target_frame', 'base_link')

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        delay_ms = rospy.get_param('~localization_delay', 0)  # milliseconds
        self.localization_delay = rospy.Duration(delay_ms / 1000.0)

        rospy.Subscriber("~odom", Odometry, self.odom_cb, queue_size=10)
        self.odom_pub = rospy.Publisher("~transformed_odom", Odometry, queue_size=1)

        self.odom_buffer = []
        period = 1.0 / 50.0  # seconds per message, assuming 50 Hz frequency
        delay_seconds = self.localization_delay.to_sec()
        self.buffer_length = max(1, int(round(delay_seconds / period)))

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
        Adds artificial localization delay by using older transform data.

        Args:
            msg (nav_msgs.msg.Odometry): The odometry message.

        Returns:
            None
        """
        # Buffer incoming odometry messages to introduce delay
        self.odom_buffer.append(msg)
        if len(self.odom_buffer) < self.buffer_length:
            # Not enough messages buffered yet, skip processing
            return
        # Pop the oldest message to process after buffer is full
        msg = self.odom_buffer.pop(0)


        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, rospy.Time(0))
            twist_transformed = self.transform_twist(msg.twist.twist, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exception:
            rospy.logwarn(f"Failed to find transform from [{msg.header.frame_id}] to [{self.target_frame}] frame: {exception}")
            return

        new_odom = Odometry()
        new_odom.header = msg.header
        new_odom.header.stamp = msg.header.stamp
        new_odom.child_frame_id = self.target_frame
        new_odom.pose.pose = msg.pose.pose # Not changing the pose!
        new_odom.pose.covariance = msg.pose.covariance
        new_odom.twist.twist = twist_transformed
        new_odom.twist.covariance = msg.twist.covariance

        self.odom_pub.publish(new_odom)

if __name__ == '__main__':

    try:
        odom_transform = OdomTransform()
        rospy.spin()

    finally:
        pass
