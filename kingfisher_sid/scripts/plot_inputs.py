#!/usr/bin/env python3

import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rospkg


def run_section(duration, tl, tr, start_time):
    times = []
    tls = []
    trs = []
    for step in range(int(duration)):
        current_time = start_time + step
        times.append(current_time)
        tls.append(tl)
        trs.append(tr)
    return times, tls, trs

# Load the experiments from the yaml file
rospack = rospkg.RosPack()
package_path = rospack.get_path('kingfisher_sid')  # Replace 'your_package_name' with the name of your package
config_path = package_path+'/config/experiments.yaml'  # Assuming the file is in the 'config' directory
with open(config_path, 'r') as file:
    experiments = yaml.safe_load(file)

# Create a new figure with size proportinal to the number of experiments
fig, axs = plt.subplots(len(experiments), 1, figsize=(10, 2*len(experiments)))

# For each experiment

for i, experiment in enumerate(experiments):
    # print(f"experiment: {experiment}")
    experiment_name = experiment['experiment_name']
    # Iterate thorugh the sections
    experiment_results = []
    start_time = 0
    times, tls, trs = [], [], []

    for section in experiment['sections']:
        duration = section['duration']
        tl = section['tl']
        tr = section['tr']
        
        # print(duration, tl, tr)
        section_times, section_tls, section_trs = run_section(duration, tl, tr, start_time)
        start_time += duration
        times.extend(section_times)
        tls.extend(section_tls)
        trs.extend(section_trs)
        
    print(f"Plotting experiment {experiment_name}")
    # Plot the tl and tr commands over time
    axs[i].plot(times, tls, label='tl')
    axs[i].plot(times, trs, label='tr')

    # Set the title and labels
    axs[i].set_title(experiment_name)
    axs[i].set_xlabel('Time')
    axs[i].set_ylabel('Command')
    axs[i].set_ylim(-1.1, 1.1)

    # Add a legend
    axs[i].legend()

# Show the plot
plt.tight_layout()
fig.savefig(package_path+'/output/experiment_inputs.png')
