#!/usr/bin/env python3

import yaml 

experiments = []
late_start_duration = 5.0
warmup_duration = 10.0
main_duration = 20.0
cool_down_duration = 10.0
import rospkg

# ACCERATION EXPERIMENTS

accelerations = [1.0, 0.9, 0.8, 0.7, 0.6, 0.5]
for acc in accelerations:
    section = {
        'experiment_name': f'acceleration_{acc}',
        'sections': [
            { 'duration': late_start_duration,  'tl': 0.0, 'tr': 0.0 },
            { 'duration': warmup_duration,      'tl': acc, 'tr': acc },
            { 'duration': main_duration,        'tl': acc, 'tr': acc },
            { 'duration': cool_down_duration,   'tl': 0.0, 'tr': 0.0 }
        ]
    }
    experiments.append(section)

# ROTATION EXPERIMENTS
accelerations_right = [1.0, 0.9, 0.8, 0.7, 0.6]
accelerations_left = [0.5762, 0.5456, 0.5190, 0.4857, 0.4529]

for accl, accr in zip(accelerations_left, accelerations_right):
    section = {
        'experiment_name': f'rotation_{accr}',
        'sections': [
            { 'duration': late_start_duration,  'tl': 0.0,   'tr': 0.0 },
            { 'duration': warmup_duration,      'tl': accl, 'tr': -accr },
            { 'duration': main_duration,        'tl': accl, 'tr': -accr },
            { 'duration': cool_down_duration,   'tl': 0.0,   'tr': 0.0 }
        ]
    }
    experiments.append(section)

    
# CIRCLE EXPERIMENTS
circle_duration = 40
accelerations = [1.0, 0.9, 0.8, 0.7, 0.6, 0.5]
for acc in accelerations:
    section = {
        'experiment_name': f'circle_left_{acc}',
        'sections': [
            { 'duration': late_start_duration,  'tl': 0.0, 'tr': 0.0 },
            { 'duration': warmup_duration,      'tl': 0.0, 'tr': acc },
            { 'duration': circle_duration,        'tl': 0.0, 'tr': acc },
            { 'duration': cool_down_duration,   'tl': 0.0, 'tr': 0.0 }
        ]
    }
    experiments.append(section)
for acc in accelerations:
    section = {
        'experiment_name': f'circle_right_{acc}',
        'sections': [
            { 'duration': late_start_duration,  'tl': 0.0, 'tr': 0.0 },
            { 'duration': warmup_duration,      'tl': acc, 'tr': 0.0 },
            { 'duration': circle_duration,        'tl': acc, 'tr': 0.0 },
            { 'duration': cool_down_duration,   'tl': 0.0, 'tr': 0.0 }
        ]
    }
    experiments.append(section)


# ZIGZAG EXPERIMENTS
zigzag_duration = 3.0
accelerations = [1.0, 0.8]
for acc in accelerations:
    section = {
        'experiment_name': f'zigzag_{acc}',
        'sections': [
                { 'duration': late_start_duration,    'tl': 0.0, 'tr': 0.0 },
                { 'duration': warmup_duration,        'tl': acc, 'tr': acc },
                { 'duration': zigzag_duration,        'tl': acc, 'tr': 0.0 },
                { 'duration': zigzag_duration,        'tl': 0.0, 'tr': acc },
                { 'duration': zigzag_duration,        'tl': acc, 'tr': 0.0 },
                { 'duration': zigzag_duration,        'tl': 0.0, 'tr': acc },
                { 'duration': zigzag_duration,        'tl': acc, 'tr': 0.0 },
                { 'duration': zigzag_duration,        'tl': 0.0, 'tr': acc },
                { 'duration': zigzag_duration,        'tl': acc, 'tr': 0.0 },
                { 'duration': zigzag_duration,        'tl': 0.0, 'tr': acc },
                { 'duration': cool_down_duration,     'tl': 0.0, 'tr': 0.0 }
            ]
    }
    experiments.append(section)



#####################################################
# EXTRA EXPERIMENTS
#####################################################
    

# BACKWARDS EXPERIMENTS
accelerations = [1.0, 0.8, 0.6]
for acc in accelerations:
    section = {
        'experiment_name': f'backwards_{acc}',
        'sections': [
                { 'duration': late_start_duration,    'tl': 0.0,  'tr': 0.0 },
                { 'duration': warmup_duration,        'tl': -acc, 'tr': -acc },
                { 'duration': main_duration,          'tl': -acc, 'tr': -acc },
                { 'duration': cool_down_duration,     'tl': 0.0,  'tr': 0.0 }
            ]
    }
    experiments.append(section)


# Write the experiments to the yaml file
rospack = rospkg.RosPack()
package_path = rospack.get_path('kingfisher_sid')  # Replace 'your_package_name' with the name of your package
config_path = package_path+'/config/experiments.yaml'  # Assuming the file is in the 'config' directory

with open(config_path, 'w') as file:
    yaml.safe_dump(experiments, file)
print(f"Experiments written to {config_path}")
