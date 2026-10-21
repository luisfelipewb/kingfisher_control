#!/usr/bin/env python3

import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv
import sys
import rospkg



def plot_experiment(file_name, experiment_name, cmd_ax1, traj_ax ):
    name_token = ''
    if '_gz' in experiment_name:
        name_token = 'GZ'
        vxc = 'red'
        vyc = 'blue'
        vrc = 'green'
        tlc = 'violet'
        trc = 'orange'
        pc = 'blue'
        loc = 1
        linewidth = 1
    elif '_rw' in experiment_name:
        name_token = 'RW'
        vxc = 'darkred'
        vyc = 'darkblue'
        vrc = 'darkgreen'
        tlc = 'darkviolet'
        trc = 'darkorange'
        pc = 'red'
        loc = 4
        linewidth = 2

    # remove _gz or _is from experiment name
    experiment_name = experiment_name.replace('_gz', '')
    experiment_name = experiment_name.replace('_rw', '')
    
    timestamps = []
    tl_values = []
    tr_values = []
    x_values = []
    y_values = []
    z_values = []
    pos_x = []
    pos_y = []

    # Load data from CSV
    with open(file_name, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            timestamps.append(float(row['time']))
            tl_values.append(float(row['thr_l']))
            tr_values.append(float(row['thr_r']))
            x_values.append(float(row['lin_x']))
            y_values.append(float(row['lin_y']))
            z_values.append(float(row['ang_z']))
            pos_x.append(float(row['pos_x']))
            pos_y.append(float(row['pos_y']))

    # Plotting
    cmd_ax2 = cmd_ax1.twinx()

    # Inputs plot (tl and tr)
    cmd_ax1.plot(timestamps, tl_values, label=f'Thruster Left {name_token}', color=tlc, linestyle='--')
    cmd_ax1.plot(timestamps, tr_values, label=f'Thruster Right {name_token}', color=trc, linestyle='--')
    cmd_ax1.set_xlabel('Time (s)')
    cmd_ax1.set_ylabel('Thruster Input')
    cmd_ax1.legend(loc='upper left')
    cmd_ax1.set_title(experiment_name)
    cmd_ax1.set_ylim(-1.1, 1.1)

    # Outputs plot (x, y, z)
    cmd_ax2.plot(timestamps, x_values, label=f'Linear Velocity X {name_token}', color=vxc)
    cmd_ax2.plot(timestamps, y_values, label=f'Linear Velocity Y {name_token}', color=vyc)
    cmd_ax2.plot(timestamps, z_values, label=f'Angular Velocity Z {name_token}', color=vrc)
    cmd_ax2.set_ylabel('Velocity')
    cmd_ax2.legend(loc=loc)
    cmd_ax2.set_ylim(-2.8, 2.8)

    # Trajectory plot (x, y)
    traj_ax.plot(pos_x, pos_y, label=f'Trajectory {name_token}', color=pc, linewidth=3, alpha=0.5)
    traj_ax.legend()
    # if 'acceleration' in experiment_name or 'backwards' in experiment_name:
    #     traj_ax.set_ylim(-0.5, 0.5)
    # else:
    traj_ax.set_aspect('equal')

    plt.tight_layout()
    # remove csv from file name and add png instead
    # fig.savefig(file_name.replace('.csv', '.png'))

def iterate_through_folder(folder):
    experiments = []
    for file in os.listdir(folder):
        if file.endswith(".csv"):
            experiments.append(file)
    experiments.sort()
    
    rows = len(experiments)//2
    fig, axs = plt.subplots(rows, 2, figsize=(15, 4*(rows)))
    # I needs to jump in 2
    for i in range(0, len(experiments), 2):
        print()
        for j in range (2):
            experiment = experiments[i+j]
            print(experiment)
            experiment_name = experiment.replace(".csv", "")
            filepath = folder + "/" + experiment
            plot_experiment(filepath, experiment_name, axs[i//2][0], axs[i//2][1]) 
    
    plt.tight_layout()
    fig.savefig(folder+'/compare_results.png')

if __name__ == "__main__":


    # Load the experiments from the yaml file
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('kingfisher_sid')  # Replace 'your_package_name' with the name of your package
    folder_path = package_path+'/output/'  # Assuming the file is in the 'config' directory

    iterate_through_folder(folder_path)

