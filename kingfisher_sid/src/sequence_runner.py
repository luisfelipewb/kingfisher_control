#!/usr/bin/env python3
import rospy
from kingfisher_msgs.msg import Drive
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import rospkg
import yaml
import threading
from copy import deepcopy
import csv
import os
import numpy as np
import tf

class SequenceRunner:
    def __init__(self):
        rospy.init_node('sequence_runner')

        # Ensuring we're using simulation time
        # if rospy.get_param('/use_sim_time', False):
        #     rospy.loginfo("Using simulation time")
        # else:
        #     rospy.loginfo("Simulation time not enabled. Set '/use_sim_time' parameter to True.")
        #     return
        
        self.current_position = None
        self.current_velocity = None
        self.lock = threading.Lock()

        self.experiment_name = rospy.get_param('~experiment_name', 'default_experiment')
        self.sequence = self.load_sequence()

        self.cmd_drive_pub = rospy.Publisher('cmd', Drive, queue_size=10)
        self.exp_name_pub = rospy.Publisher('experiment_name', String, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odometry_callback)
        self.data = []

        
    def load_sequence(self):
        # Open the experiments.yaml file and load the experiments
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('test_rl')  # Replace 'your_package_name' with the name of your package
        config_path = package_path+'/config/experiments.yaml'  # Assuming the file is in the 'config' directory
        rospy.logdebug(f"Using configuration path: {config_path}")

        with open(config_path, 'r') as file:
            experiments = yaml.safe_load(file)

        # Find the experiment sequence based on the experiment name
        experiment = next((exp for exp in experiments if exp['experiment_name'] == self.experiment_name), None)
        if not experiment:
            rospy.logerr(f"Experiment {self.experiment_name} not found.")
            return
        rospy.loginfo(f"Loaded experiment: {self.experiment_name}")

        sequence = experiment['sections']
        return sequence

    def odometry_callback(self, data):
        with self.lock:
            self.current_position = data.pose.pose
            self.current_velocity = data.twist.twist

    def rotate_velocity(self, orientation, velocity):
        euler = tf.transformations.euler_from_quaternion([orientation.w, orientation.x, orientation.y, orientation.z])
        yaw = euler[0]
        # print(euler)
        # print(f"yaw:{yaw:.2f}")
        yaw = np.arctan2(np.sin(yaw+np.pi), np.cos(yaw+np.pi))  # Ensure angle is within -pi to pi

        rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)],
                                    [np.sin(yaw), np.cos(yaw)]])

        # Rotate the velocity vector
        rotated_velocity = np.dot(rotation_matrix, np.array(velocity))
        # print(rotated_velocity)

        return rotated_velocity.tolist()

    def get_pos_and_vel(self):
        with self.lock:
            pos = deepcopy(self.current_position)
            vel = deepcopy(self.current_velocity)
        # Convert velocity to local frame of the robot
        orientation = pos.orientation
        velocity = vel.linear.x, vel.linear.y
        vx, vy = self.rotate_velocity(orientation, velocity)
        vel.linear.x = vx
        vel.linear.y = vy
        return pos, vel

    def save_output_file(self):
        # Save the results of the experiment in a csv file
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('test_rl')  # Replace 'your_package_name' with the name of your package
        output_dir = os.path.join(package_path, 'output')
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, f"{self.experiment_name}.csv")

        with open(output_file, 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'thr_r', 'thr_l', 'pos_x', 'pos_y', 'lin_x', 'lin_y', 'ang_z', 'roll', 'pitch', 'yaw'])
            writer.writerows(self.data)
            writer = csv.writer(file)
            
        rospy.loginfo(f"Results saved to {output_file}")

    def record_readings(self, time, drive_command, pos, vel):
        # Store the results of the experiment in a csv file
        # TODO Debug and understand the orientation
        euler = tf.transformations.euler_from_quaternion([pos.orientation.w, pos.orientation.x, pos.orientation.y, pos.orientation.z])
        yaw = euler[0]
        yaw = np.arctan2(np.sin(yaw+np.pi), np.cos(yaw+np.pi))
        pitch = euler[1]
        roll = euler[2]
        
        heading_cos = np.cos(yaw)
        heading_sin = np.sin(yaw)

        rospy.logdebug("Storing results...")
        row = [f"{time.to_sec():.3f}"]
        row.extend([drive_command.right, drive_command.left])
        row.extend([f"{pos.position.x:.4f}", f"{pos.position.y:.4f}"])
        row.extend([f"{vel.linear.x:.4f}", f"{vel.linear.y:.4f}", f"{vel.angular.z:.4f}"])
        row.extend([f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}"])
        self.data.append(row)
        return

    def run_experiment(self):

        # make sure the robot has received the odometry message and variables are not None
        while self.current_position is None or self.current_velocity is None:
            rospy.loginfo("Waiting for odometry data...")
            rospy.sleep(1)
    
        
        rate = rospy.Rate(10) # TODO make it a parameter
        start_time = rospy.Time.now()

        for segment in self.sequence:
            drive_command = Drive()
            drive_command.left = segment['tl']
            drive_command.right = segment['tr']
            rospy.loginfo(f"{self.experiment_name} - Executing section: tl={segment['tl']}, tr={segment['tr']}, duration={segment['duration']}s")
            
            segment_time = rospy.Time.now()
            # While loop to wait for the duration of the section
            while (rospy.Time.now() - segment_time).to_sec() < segment['duration']:
                rospy.logdebug(f"Publishing drive command: left={drive_command.left}, right={drive_command.right}")
                
                self.cmd_drive_pub.publish(drive_command)
                self.exp_name_pub.publish(self.experiment_name)

                pos, vel = self.get_pos_and_vel()
                time = rospy.Time.now() - start_time

                self.record_readings(time, drive_command, pos, vel)
                rate.sleep()

        rospy.loginfo("Experiment completed.")
        self.save_output_file()

if __name__ == '__main__':
    try:
        node = SequenceRunner()
        node.run_experiment()
    except rospy.ROSInterruptException:
        pass



