#!/usr/bin/python3

import rospy
import rosbag
import csv
import sys
import os
import numpy as np
import argparse
import tf.transformations as tf
import math

# Topics to extract from simullation:
pose = '/imu/odometry'
cmd_drive = '/cmd_drive'
position_goal = '/move_base_simple/goal'

topic_list = [pose, cmd_drive, position_goal]

parser = argparse.ArgumentParser()
parser.add_argument('-i', '--bag_file', type=str, help="Path to the bag file")
parser.add_argument('-o', '--output_file', type=str, help="Path to the output directory")

args = parser.parse_args()
results_dir = os.path.dirname(args.output_file)
bag_file = args.bag_file
csv_file = args.output_file


def check_rl_running(cmd_drive_l, cmd_drive_r):
    # check if both values are very close to zero
    if abs(cmd_drive_l) < 0.0001 and abs(cmd_drive_r) < 0.0001:
        return False
    else:
        return True

def transform_odom(reference_pose, current_pose):

    # fliped because NED --> ENU
    y_pose = current_pose.pose.pose.position.x - reference_pose.pose.pose.position.x
    x_pose = current_pose.pose.pose.position.y - reference_pose.pose.pose.position.y

    reference_quaternion = [reference_pose.pose.pose.orientation.x,
                            reference_pose.pose.pose.orientation.y,
                            reference_pose.pose.pose.orientation.z,
                            reference_pose.pose.pose.orientation.w]
                
    reference_euler = tf.euler_from_quaternion(reference_quaternion)

    # YAW in the ENU frame
    yaw = math.pi/2 - reference_euler[2]
    # yaw = 0
    
    # if abs(x_pose) < 0.05 and abs(y_pose) < 0.05:
    #     print(f"Reference yaw angle: {reference_euler[2]:.2f}, new yaw angle: {current_euler[2]:.2f}, transformed pose: {x_pose:.2f}, {y_pose:.2}")

    rot_x_pose = x_pose * math.cos(-yaw) - y_pose * math.sin(-yaw)
    rot_y_pose = x_pose * math.sin(-yaw) + y_pose * math.cos(-yaw)

    return rot_x_pose, rot_y_pose



bag = rosbag.Bag(bag_file)

os.makedirs(results_dir, exist_ok=True)

# csv_file = os.path.join(results_dir, 'goto_positions.csv')
count_cmd_drive = 0
count_pose = 0

has_goal = False
rl_running = False
new_status = True

reference_pose = None
current_pose = None

# Open csv file for writing 
f = open(csv_file, 'w') 
writer = csv.writer(f)
writer.writerow(['time', 'goal', 'x', 'y', 'x_goal', 'y_goal', 'distance_to_goal', 'cmd_drive_l', 'cmd_drive_r'])


print(f"Extracting {topic_list} from {bag_file}...")
for topic, msg, t in bag.read_messages(topics=topic_list):
    #print(f"Received message from {topic}")

    if topic == position_goal:
        has_goal = True
        goal = [msg.pose.position.x, msg.pose.position.y]
        goal_string = f"{goal[0]:.1f}_{goal[1]:.1f}"
        new_output_file = f"goto_position_{goal_string}.csv"
        print(f"{t} Received goal message {msg.pose.position.x:.1f}, {msg.pose.position.y:.1f}")
        # Should also have new odometry message
        reference_pose = current_pose
        print(f"{t} Reference pose: {reference_pose.pose.pose.position.x:.2f}, {reference_pose.pose.pose.position.y:.2f}")

    elif topic in [cmd_drive]:
        # new_status = msg.data
        rl_running = check_rl_running(msg.left, msg.right)

        if rl_running != new_status:
            # print(f"RL status changed from {rl_running} to {new_status}")msg.left
            new_status = check_rl_running(msg.left, msg.right)
            #print(f"RL status changed from {rl_running} to {new_status}")
            if new_status:
                print(f"{t} RL is running")
                rl_running = new_status
            else:
                print(f"{t} RL is not running")
                rl_running = new_status
    
    # Always extract pose, but it will be only used if it has a goal and RL is running
    if topic in [pose]:
        time_seconds = float(str(t))/1e9
        # print(f"received message {topic} at {time_seconds}")
        # print(msg)
        x_pose = 0
        y_pose = 0

        current_pose = msg
        if reference_pose is not None:
            x_pose, y_pose = transform_odom(reference_pose, current_pose)
        
        x_pos = format(x_pose, '.4f')
        y_pos = format(y_pose, '.4f')

    if rl_running and has_goal and topic in [cmd_drive]:
        time_seconds = float(str(t))/1e9
        x_goal = format(goal[0], '.5f')
        y_goal = format(goal[1], '.5f')
        np_pos = np.array([x_pos, y_pos], dtype=float)
        np_goal = np.array([x_goal, y_goal], dtype=float)
        distance_to_goal = format(np.linalg.norm(np_pos - np_goal), '.2f')
        cmd_drive_l = format(msg.left, '.4f')
        cmd_drive_r = format(msg.right, '.4f')
        # print(f"received message {topic} at {time_seconds}")

        writer.writerow([time_seconds, goal_string, x_pos, y_pos, x_goal, y_goal, distance_to_goal, cmd_drive_l, cmd_drive_r])


bag.close()
f.close()
