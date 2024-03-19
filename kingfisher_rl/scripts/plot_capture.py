#!/usr/bin/python3
import pandas as pd
import csv
import matplotlib
matplotlib.use('Agg')  # Set the backend to 'Agg'
import matplotlib.pyplot as plt
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-i', '--input_file', type=str, default='../output/goto_positions.csv', help='Path to the input CSV file')
parser.add_argument('-o', '--output_file', type=str, default='../output/combined_plot.png', help='Path to the output image file')
args = parser.parse_args()

df = pd.read_csv(args.input_file)


fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

total_tests = 0
total_reached = 0
distances = []
bearings = []
energies = []

for goal in df['goal'].unique():

    # Plot the positions in the first subplot
    goal_df = df[df['goal'] == goal]
    x_axis = np.array(goal_df['x'])
    y_axis = np.array(goal_df['y'])
    ax1.plot(x_axis, y_axis, label=goal, linewidth=1)
    ax1.scatter(goal_df['x_goal'], goal_df['y_goal'], color='red', s=5)


    # Plot the distance to goal in the second subplot
    goal_df = df[df['goal'] == goal]
    x_axis = np.array(goal_df['time'])
    x_axis = x_axis - x_axis[0]
    y_axis = np.array(goal_df['distance_to_goal'])
    ax2.plot(x_axis, y_axis, label=goal, linewidth=1)

    # Count success rate
    elapsed_time = x_axis[-1] - x_axis[0]
    total_tests += 1
    if elapsed_time < 50:
        total_reached += 1

    initial_distance = np.linalg.norm([goal_df['x_goal'].iloc[0], goal_df['y_goal'].iloc[0]])
    initial_bearing = np.arctan2(goal_df['y_goal'].iloc[0], goal_df['x_goal'].iloc[0])
    initial_bearing = np.rad2deg(initial_bearing)
    accumulated_energy = goal_df['cmd_drive_l'].abs().sum() + goal_df['cmd_drive_r'].abs().sum()
    distances.append(initial_distance)
    bearings.append(initial_bearing)
    energies.append(accumulated_energy)

    # print minimum accumulated energy
    print(f"goal: {goal}, initial_distance: {initial_distance:.2f}, initial_bearing: {initial_bearing:.2f}, accumulated_energy: {accumulated_energy:.2f}")

    # Create a scatter plot of the goal positions on the distance to goal plot on ax3
    # ax3.scatter(initial_distance, initial_bearing, c=accumulated_energy, s=10)

# print(f"goal: {goal}, time: {elapsed_time:.2f}")
print(f"Reached goal {total_reached} out of {total_tests} times. Success rate: {total_reached/total_tests:.2f}")

ax1.set_title('path')
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.axis('equal')
max_dist = 10.5
ax1.set_ylim(-max_dist, max_dist)
ax1.set_xlim(-max_dist, max_dist)

ax2.set_title('distance over time')
ax2.set_xlabel('time (s)')
ax2.set_ylabel('distance to goal (m)')
ax2.set_xlim(0, 60)
# ax2.legend()

# ax1.legend()
print(f"max energy: {max(energies)}")
# scatter = plt.scatter(distances, bearings, c=energies, vmin=0, vmax=max(energies), s=30, marker="s", cmap='coolwarm')
scatter = plt.scatter(distances, bearings, c=energies, vmin=0, vmax=275, s=30, marker="s", cmap='coolwarm')


ax3.set_title('energy vs. initial bearing and distance')
ax3.set_xlabel('initial distance to goal (m)')
ax3.set_ylabel('initial bearing to goal (deg)')
ax3.grid(True)
fig.colorbar(scatter, ax=ax3, label='accumulated energy')

# Set figure title and labels
title = args.input_file.split('/')[-1].split('.')[0]
fig.suptitle(title)

plt.savefig(args.output_file)
plt.close()
