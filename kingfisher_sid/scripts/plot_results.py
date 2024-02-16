#!/usr/bin/env python3

import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv
import sys
import rospkg



def plot_experiment(file_name, experiment_name, ax1):
    timestamps = []
    tl_values = []
    tr_values = []
    x_values = []
    y_values = []
    z_values = []

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

    # Plotting
    ax2 = ax1.twinx()

    # Inputs plot (tl and tr)
    ax1.plot(timestamps, tl_values, label='Thruster Left (tl)', color='darkred', linestyle='--')
    ax1.plot(timestamps, tr_values, label='Thruster Right (tr)', color='darkgreen', linestyle='--')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Thruster Input')
    ax1.legend(loc='upper left')
    ax1.set_title(experiment_name)
    ax1.set_ylim(-1.1, 1.1)

    # Outputs plot (x, y, z)
    ax2.plot(timestamps, x_values, label='Linear Velocity X', color='red')
    ax2.plot(timestamps, y_values, label='Linear Velocity Y', color='blue')
    ax2.plot(timestamps, z_values, label='Angular Velocity Z', color='green')
    ax2.set_ylabel('Velocity')
    ax2.legend(loc='upper right')
    ax2.set_ylim(-2.8, 2.8)

    plt.tight_layout()
    # remove csv from file name and add png instead
    # fig.savefig(file_name.replace('.csv', '.png'))

def iterate_through_folder(folder):
    experiments = []
    for file in os.listdir(folder):
        if file.endswith(".csv"):
            experiments.append(file)
    experiments.sort()
    fig, axs = plt.subplots(len(experiments), 1, figsize=(10, 4*len(experiments)))

    for i, experiment in enumerate(experiments):
            experiment_name = experiment.replace(".csv", "")
            filepath = folder + "/" + experiment
            plot_experiment(filepath, experiment_name, axs[i]) 
    plt.tight_layout()
    fig.savefig(folder+'/experiment_results.png')

if __name__ == "__main__":


    # Load the experiments from the yaml file
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('kingfisher_sid')  # Replace 'your_package_name' with the name of your package
    folder_path = package_path+'/output/'  # Assuming the file is in the 'config' directory

    iterate_through_folder(folder_path)
