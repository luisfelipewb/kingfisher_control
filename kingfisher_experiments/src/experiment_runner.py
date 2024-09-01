#!/usr/bin/python3

import rospy

from sensor_msgs.msg import Joy
from heron_msgs.msg import Drive
from std_msgs.msg import String
from topic_tools.srv import MuxSelect
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Vector3Stamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

from kingfisher_experiments.srv import SetFloat, SetFloatResponse

import numpy as np
import tf2_geometry_msgs
import tf2_ros
import yaml
import csv

import threading

import rospkg


class ExperimentRunner:

    def __init__(self):

        rospy.init_node('experiment_runner')

        self.send_goal = rospy.get_param('~send_goal_button', 7)
        self.send_goal_prev = None

        self.running = True # Assume the agent is running to prevent taking control over it.

        self.odom_lock = threading.Lock()
        self.odom = None
        self.vel_threshold = rospy.get_param('~vel_threshold', 0.1)


        self.cmd_drive = Drive()
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.cmd_drive_pub = rospy.Publisher('~cmd_drive', Drive, queue_size=1)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)


        rospy.Subscriber("~odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/rl_status', Bool, self.status_cb)

        self.drive_mux_srv = rospy.ServiceProxy('/drive_mux/select', MuxSelect)
        self.target_vel_srv = rospy.ServiceProxy('/velocity_tracker/set_target_velocity', SetFloat)

        experiments_file = rospy.get_param('~experiments_file', "sample.yaml")
        self.experiments = self.load_experiments(experiments_file)

        self.wait_for_odom()

    def load_experiments(self, file_path):
        # append the file path to the package path
        rospack = rospkg.RosPack()

        file_path = rospack.get_path('kingfisher_experiments') + '/config/' + file_path
        # test if the file exists
        try:
            with open(file_path, 'r') as file:
                pass
                data = yaml.safe_load(file)
        except FileNotFoundError:
            rospy.logerr(f"File {file_path} not found")
            return None
        return data['experiments']


    def wait_for_odom(self):
        received_odom = False
        while not received_odom:
            with self.odom_lock:
                if self.odom is not None:
                    received_odom = True
            rospy.loginfo_throttle(1, "Waiting for odom...")
            rospy.sleep(0.1)


    def status_cb(self, msg):
        # update the running status of the controller
        self.running = msg.data


    def odom_cb(self, msg):
        with self.odom_lock:
            self.odom = msg

    def select_mux(self, topic):
        try:
            self.drive_mux_srv(topic)
            rospy.loginfo(f"Select {topic} input")
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)


    def joy_callback(self, joy_msg):
        # placeholder for the joy callback to control exeperiment execution
        return
        self.call_mux(joy_msg.buttons[self.mux_button])
        self.publish_cmd_drive(left, right)


    def generate_goal(self, dist, angle):
        angle_rad = np.deg2rad(angle)
        # print(f"angle_rad: {angle_rad}")
        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.pose.position.x = dist * np.cos(angle_rad)
        goal.pose.position.y = dist * np.sin(angle_rad)
        # print(f"goal: {goal.pose.position.x}, {goal.pose.position.y}")
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1
        return goal

    def reach_velocity(self, v0):
        # call the velocity tracker to reach the target velocity
        # /velocity_tracker/set_target_velocity\

        try:
            self.target_vel_srv(v0)
            rospy.loginfo(f"Set target velocity to {v0}")
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

        # pass the control to the velocity tracker
        self.select_mux('velocity_tracker/cmd_drive')

        vel_error = 1.0
        while vel_error > self.vel_threshold:
            with self.odom_lock:
                current_velocity = self.odom.twist.twist.linear.x
            vel_error = abs(v0 - current_velocity)
            rospy.loginfo_throttle(1, f"Current velocity: {current_velocity:.2f} error: {vel_error:.2f}")
            rospy.sleep(0.1)
        rospy.loginfo(f"Reached target velocity {v0}")

    def run_next(self, experiment):

        v0 = experiment['v0']
        dist = experiment['dist']
        bearing = experiment['bearing']

        rospy.loginfo(f"Reaching target velocity {v0}")
        self.reach_velocity(v0)

        rospy.loginfo(f"Publishing a goal with dist: {dist}, angle {bearing}")
        next_goal = self.generate_goal(dist, bearing)
        self.goal_publisher.publish(next_goal)
        self.running = True

        # Pass the mux to the rl_agent
        self.select_mux('rl_agent/cmd_drive')

        rospy.logdebug(f"status {self.running}")
        while self.running:
            rospy.sleep(0.1)
            rospy.loginfo_throttle(5, "Agent running...")
            rospy.sleep(1.0) # wait for the agent to finish and self.running to be updated





if __name__ == '__main__':

    runner = ExperimentRunner()
    rate = rospy.Rate(1)

    try:
        for experiment in runner.experiments:
            runner.run_next(experiment)
            rospy.loginfo("Finished experiments, stopping the boat...")
            runner.reach_velocity(0.0)
            rospy.loginfo("Done!")

    except rospy.ROSInterruptException:
        pass
