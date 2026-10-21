"""
The passthrough agent with its goals: goal_publisher, passthrough_thrusters, detection_goal.

Runs on its own next to the boat's bringup or the sim, under `namespace` (default kingfisher).
The agent publishes control/cmd_drive; joystick A hands it cmd_drive through drive_mux.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    goals_config = os.path.join(
        get_package_share_directory('kingfisher_goals'), 'config', 'goals.yaml')
    config = os.path.join(
        get_package_share_directory('kingfisher_rl'), 'config', 'passthrough_thrusters.yaml')
    sim_time = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}
    policy = LaunchConfiguration('policy')

    nodes = [
        Node(
            package='kingfisher_goals', executable='goal_publisher',
            parameters=[goals_config, sim_time],
            output='screen'),
        Node(
            package='kingfisher_rl', executable='passthrough_thrusters',
            parameters=[config, sim_time, {'policy': policy}],
            output='screen'),
        Node(
            package='kingfisher_goals', executable='detection_goal',
            parameters=[goals_config, sim_time],
            condition=IfCondition(LaunchConfiguration('detections')),
            output='screen'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='kingfisher',
            description='ROS namespace of the boat'),
        DeclareLaunchArgument(
            'policy', default_value='passthrough_thrusters/seed44.onnx',
            description='ONNX policy, under share/kingfisher_rl/models/ or absolute'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the sim clock'),
        DeclareLaunchArgument(
            'detections', default_value='false',
            description='Start detection_goal: goals from detections'),
        GroupAction([PushRosNamespace(LaunchConfiguration('namespace')), *nodes]),
    ])
