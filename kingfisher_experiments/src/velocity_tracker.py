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

    def odom_cb(self, msg):
        with self.odom_lock:
            self.odom = msg

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
