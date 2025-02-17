#!/usr/bin/python3

import rospy

from sensor_msgs.msg import Joy
from kingfisher_msgs.msg import Drive
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

        self.running = None # Start as None and way for the first status message.

        self.odom_lock = threading.Lock()
        self.odom = None
        self.vel_threshold = rospy.get_param('~vel_threshold', 0.1)

        self.start_button = rospy.get_param('~start_button', 0)
        self.repeat_button = rospy.get_param('~repeat_button', 3)
        self.skip_button = rospy.get_param('~skip_button', 1)
        self.exp_name = rospy.get_param('~exp_name', 'exp')

        self.start_prev = 1
        self.repeat_prev = 1
        self.skip_prev = 1
        self.start_pressed = False
        self.repeat_pressed = False
        self.skip_pressed = False

        self.mux_select = rospy.get_param('~mux_input', 'control_agent/cmd_drive')
        self.cmd_drive = Drive()
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.cmd_drive_pub = rospy.Publisher('~cmd_drive', Drive, queue_size=1)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.experiment_name_publisher = rospy.Publisher('~experiment_name', String, queue_size=1)

        rospy.Subscriber("~odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('~status_topic', Bool, self.status_cb)

        self.drive_mux_srv = rospy.ServiceProxy('/drive_mux/select', MuxSelect)
        self.target_vel_srv = rospy.ServiceProxy('/velocity_tracker/set_target_velocity', SetFloat)

        experiments_file = rospy.get_param('~experiments_file', "sample.yaml")
        self.experiments = self.load_experiments(experiments_file)

        rospack = rospkg.RosPack()
        self.logpath = rospack.get_path('kingfisher_experiments') + '/experiment_log.csv'


        self.wait_for_status()
        self.wait_for_odom()

    def wait_for_status(self):
        while self.running is None:
            rospy.loginfo_throttle(1, "Waiting for status...")
            rospy.sleep(0.1)

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
        # Update status of the buttons
        if joy_msg.buttons[self.start_button] and not self.start_prev:
            self.start_pressed = True
        if joy_msg.buttons[self.repeat_button] and not self.repeat_prev:
            self.repeat_pressed = True
        if joy_msg.buttons[self.skip_button] and not self.skip_prev:
            self.skip_pressed = True

        self.start_prev = joy_msg.buttons[self.start_button]
        self.repeat_prev = joy_msg.buttons[self.repeat_button]
        self.skip_prev = joy_msg.buttons[self.skip_button]


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
            rospy.loginfo(f"Target velocity to {v0}")
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

    def check_for_skip(self):
        self.skip_pressed = False
        self.start_pressed = False
        rospy.loginfo("Press A to start, B to skip...")
        while not self.start_pressed and not self.skip_pressed:
            rospy.sleep(0.1)
        if self.skip_pressed:
            rospy.loginfo("Skipping the experiment...")
            return True
        else:
            return False

    def check_for_repeat(self):
        self.repeat_pressed = False
        self.start_pressed = False
        rospy.loginfo("Press A for next, Y to repeat...")
        while not self.repeat_pressed and not self.start_pressed:
            rospy.sleep(0.1)
        if self.repeat_pressed:
            rospy.loginfo("Repeating the experiment...")
            return True
        else:
            return False


    def run_next(self, v0, dist, bearing):

        rospy.loginfo(f"Publishing a goal with dist: {dist}, angle {bearing}")
        experiment_string = f"{self.exp_name}_v{v0}_d{dist}_b{bearing}"
        self.experiment_name_publisher.publish(experiment_string)

        next_goal = self.generate_goal(dist, bearing)
        self.goal_publisher.publish(next_goal)
        self.running = True

        start_time = rospy.Time.now().to_sec()
        # Pass the mux to the rl_agent
        self.select_mux(self.mux_select)

        rospy.logdebug(f"status {self.running}")
        rospy.sleep(0.8)
        while self.running:
            self.experiment_name_publisher.publish(experiment_string)
            rospy.sleep(0.1)
            # rospy.loginfo_throttle(5, "Agent running...")
        end_time = rospy.Time.now().to_sec()
        normalized_time = (end_time - start_time) / dist
        rospy.logwarn(f"Experiment time: {end_time - start_time:.2f} Normalized time: {normalized_time:.2f}")
        with open(self.logpath, 'a') as file:
            writer = csv.writer(file)
            duration = round(end_time - start_time, 2)
            normalized_time = round(normalized_time, 2)
            writer.writerow([experiment_string, v0, dist, bearing, duration, normalized_time])



    def run_experiments(self, experiments):

        experiment_index = 0
        total_experiments = len(experiments)

        while experiment_index < total_experiments:
            experiment = experiments[experiment_index]

            v0 = experiment['v0']
            dist = experiment['dist']
            bearing = experiment['bearing']

            # Run the experiment
            rospy.loginfo(f"Experiment {experiment_index}/{total_experiments}: v0: {v0}, dist: {dist}, bearing: {bearing}")

            skip = self.check_for_skip()
            if skip:
                experiment_index += 1
                continue
            else:
                experiment_string = f"{self.exp_name}_v{v0}_d{dist}_b{bearing}"
                self.experiment_name_publisher.publish(experiment_string)
                self.reach_velocity(v0)
                self.run_next(v0, dist, bearing)

            # Repeat the experiment if needed
            if self.check_for_repeat():
                continue
            else:
                experiment_index += 1
            rospy.sleep(1)



if __name__ == '__main__':

    runner = ExperimentRunner()

    try:
        runner.run_experiments(runner.experiments)
        rospy.loginfo("Finished experiments, stopping the boat...")
        runner.reach_velocity(0.0)
        rospy.loginfo("Done!")

    except rospy.ROSInterruptException:
        pass
