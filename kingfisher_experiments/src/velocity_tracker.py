#!/usr/bin/python3

import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Vector3Stamped
from heron_msgs.msg import Drive
from nav_msgs.msg import Odometry

from kingfisher_experiments.srv import SetFloat, SetFloatResponse

import tf2_geometry_msgs
import tf2_ros

import threading

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error = 0.0
        self.error_prev = 0.0
        self.integral = 0.0

    def update(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        # Cap the integral term because some targets are not reachable
        self.integral = max(-1.0, min(1.0, self.integral))
        derivative = (error - self.error_prev) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.error_prev = error
        # print(f"error: {error:.2f} integral: {self.integral:.2f} derivative: {derivative:.2f}")
        # print(f"kp: {self.kp:.2f} ki: {self.ki:.2f} kd: {self.kd:.2f}")

        return output

class VelocityTracker:

    def __init__(self):

        rospy.init_node('velocity_tracker')

        self.rate = rospy.get_param('~rate', 20)
        self.dt = 1.0 / self.rate

        self.kp = rospy.get_param('~kp', 5.0)
        self.ki = rospy.get_param('~ki', 0.5)
        self.kd = rospy.get_param('~kd', 0.1)

        self.target_velocity = 0.0
        self.current_velocity = 0.0

        self.odom = None
        self.odom_lock = threading.Lock()

        self.cmd_drive = Drive()

        self.pid_controller = PIDController(self.kp, self.ki, self.kd)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.Subscriber("~odom", Odometry, self.odom_cb, queue_size=1)
        self.cmd_drive_pub = rospy.Publisher('~cmd_drive', Drive, queue_size=1)

        self.set_velocity_service = rospy.Service('~set_target_velocity', SetFloat, self.set_velocity_cb)
        self.set_kp = rospy.Service('~set_kp', SetFloat, self.set_kp_cb)
        self.set_ki = rospy.Service('~set_ki', SetFloat, self.set_ki_cb)
        self.set_kd = rospy.Service('~set_kd', SetFloat, self.set_kd_cb)

    def set_kp_cb(self, req):
        self.pid_controller.kp = req.data
        rospy.loginfo(f"Set kp to {self.pid_controller.kp}")
        return SetFloatResponse(success=True, message=f"kp set to {self.pid_controller.kp}")

    def set_ki_cb(self, req):
        self.pid_controller.ki = req.data
        rospy.loginfo(f"Set ki to {self.pid_controller.ki}")
        return SetFloatResponse(success=True, message=f"ki set to {self.pid_controller.ki}")

    def set_kd_cb(self, req):
        self.pid_controller.kd = req.data
        rospy.loginfo(f"Set kd to {self.pid_controller.kd}")
        return SetFloatResponse(success=True, message=f"kd set to {self.pid_controller.kd}")

    def set_velocity_cb(self, req):

        self.target_velocity = req.data
        rospy.loginfo(f"Set target velocity to {self.target_velocity}")
        return SetFloatResponse(success=True, message=f"Target velocity set to {self.target_velocity}")

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
        Callback function for handling new odometry messages. The robot's current position and heading are stored in
        class variables after transforming to the base_link frame.

        :param msg: The incoming odometry message.
        """
        child_frame_id = msg.child_frame_id
        frame_id = msg.header.frame_id

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        twist_stamped = TwistStamped()
        twist_stamped.header = msg.header
        twist_stamped.twist = msg.twist.twist
        # print(f"twist: \n{msg.twist.twist.linear}")

        try:
            # We want want the pose in the base_link frame. For SBG the odom is not in base_link
            transform = self.tf_buffer.lookup_transform('base_link', child_frame_id, rospy.Time())
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Failed to find transform from [{frame_id}] to [base_link] frame")
            return
        try:
            # We want the twist in the base_link frame and odometry is published in some world frame
            transform = self.tf_buffer.lookup_transform('base_link', frame_id, rospy.Time())
            twist_transformed = self.transform_twist(twist_stamped.twist, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Failed to find transform from [{frame_id}] to [base_link] frame")
            return

        # print(f"\nOriginal: {msg.pose.pose}")
        with self.odom_lock:
            self.odom = msg
            self.odom = Odometry()
            self.odom.header = msg.header
            self.odom.child_frame_id = "base_link"
            self.odom.pose.pose = pose_transformed.pose
            self.odom.twist.twist = twist_transformed


    def publish_cmd_drive(self, left, right):
        self.cmd_drive.left = left
        self.cmd_drive.right = right
        self.cmd_drive_pub.publish(self.cmd_drive)

    def control_loop(self):
        # Calculate the error
        with self.odom_lock:
            if self.odom is None:
                return
            self.current_velocity = self.odom.twist.twist.linear.x

        control_output = self.pid_controller.update(self.target_velocity, self.current_velocity, self.dt)

        # print(f"goal: {self.target_velocity:.2f} current: {self.current_velocity:.2f} cmd: {control_output:.2f}")

        control_output = max(-1.0, min(1.0, control_output))
        # Calculate the left and right wheel speeds
        left = control_output
        right = control_output

        # Publish the drive command
        self.publish_cmd_drive(left, right)

if __name__ == '__main__':


    velocity_controller = VelocityTracker()
    rate = rospy.Rate(velocity_controller.rate)

    while not rospy.is_shutdown():
        velocity_controller.control_loop()
        rate.sleep()
