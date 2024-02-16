#!/usr/bin/env python
import yaml
import subprocess
import rospkg
import argparse
from time import sleep


parser = argparse.ArgumentParser(description='Run a sequence of experiments')
parser.add_argument('--simulation', action='store_true', help='Run the experiments in simulation mode')
args = parser.parse_args()

# Load the experiments from the yaml file
rospack = rospkg.RosPack()
package_path = rospack.get_path('kingfisher_sid')  # Replace 'your_package_name' with the name of your package
config_path = package_path+'/config/experiments.yaml'  # Assuming the file is in the 'config' directory
with open(config_path, 'r') as file:
    experiments = yaml.safe_load(file)

# Check how many experiments we have
print(f"Found {len(experiments)} experiments")

count = 1
# For each experiment
for experiment in experiments:
    # Construct the roslaunch command
    experiment_name = experiment['experiment_name']
    command = [
        'roslaunch',
        'kingfisher_sid',
        'run_sequence.launch',
        'experiment_name:={}'.format(experiment_name)
    ]

    # Run the command
    subprocess.run(command)

    print(f"Finished experiment {count} of {len(experiments)}")
    if args.simulation:
        # create ros command to reset the gazebo environemtn 
        command = [
            'rosservice',
            'call',
            '/gazebo/reset_world'
        ]
        # run the command
        subprocess.run(command)
        # wait for 1 second
        print("Resetting the world...")
        sleep(1)
    else: 
        # Wait for the user to press a key before continuing
        input("Press any key to continue with the next experiment...")
    count += 1