#!/usr/bin/python3

import rospy
import rosbag
import csv
import sys
import os
import numpy as np
import argparse


# Topics to extract from simullation:
pose = '/pose_gt'
cmd_vel = '/cmd_vel'
cmd_drive = '/cmd_drive'
rl_status = '/rl_status'
position_goal = '/move_base_simple/goal'

topic_list = [pose, cmd_drive, rl_status, position_goal]

parser = argparse.ArgumentParser()
parser.add_argument('-i', '--bag_file', type=str, help="Path to the bag file")
parser.add_argument('-o', '--output_file', type=str, help="Path to the output directory")

args = parser.parse_args()
results_dir = os.path.dirname(args.output_file)
bag_file = args.bag_file
csv_file = args.output_file


bag = rosbag.Bag(bag_file)

os.makedirs(results_dir, exist_ok=True)

# csv_file = os.path.join(results_dir, 'goto_positions.csv')
count_cmd_drive = 0
count_pose = 0

has_goal = False
rl_running = False


# Open csv file for writing 
f = open(csv_file, 'w') 
writer = csv.writer(f)
writer.writerow(['time', 'goal', 'x', 'y', 'x_goal', 'y_goal', 'distance_to_goal', 'cmd_drive_l', 'cmd_drive_r'])


print(f"Extracting {topic_list} from {bag_file}...")
for topic, msg, t in bag.read_messages(topics=topic_list):

    if topic == position_goal:
        has_goal = True
        goal = [msg.pose.position.x, msg.pose.position.y]
        goal_string = f"{goal[0]:.1f}_{goal[1]:.1f}"
        new_output_file = f"goto_position_{goal_string}.csv"
        print(f"{t} Received goal message {msg.pose.position.x}, {msg.pose.position.y}")
    elif topic == rl_status:

        new_status = msg.data
        if rl_running != new_status:
            # print(f"RL status changed from {rl_running} to {new_status}")
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
        x_pos = format(msg.pose.pose.position.x, '.4f')
        y_pos = format(msg.pose.pose.position.y, '.4f')

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
