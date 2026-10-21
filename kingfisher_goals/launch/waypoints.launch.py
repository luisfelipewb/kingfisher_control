"""
Waypoint following, with virtual bottles if asked: waypoint_goal and virtual_detections.

Sends goal; run it next to a controller launch, such as kingfisher_rl's
passthrough_thrusters.launch.py, which starts goal_publisher. scenario is a name under
config/scenarios/ or a path to a scenario YAML.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def nodes(context):
    """Return the nodes, with the scenario resolved to a file."""
    share = get_package_share_directory('kingfisher_goals')
    config = os.path.join(share, 'config', 'goals.yaml')
    scenario = LaunchConfiguration('scenario').perform(context)
    if not scenario.endswith('.yaml'):
        scenario = os.path.join(share, 'config', 'scenarios', scenario + '.yaml')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}

    return [GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='kingfisher_goals', executable='waypoint_goal',
            parameters=[config, sim_time, {'waypoints_file': scenario}],
            output='screen'),
        Node(
            package='kingfisher_goals', executable='virtual_detections',
            parameters=[config, sim_time, {'bottles_file': scenario}],
            condition=IfCondition(LaunchConfiguration('virtual_detections')),
            output='screen'),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='kingfisher',
            description='ROS namespace of the boat'),
        DeclareLaunchArgument(
            'scenario', default_value='grid_small',
            description='Scenario: a name under config/scenarios/, or a .yaml path'),
        DeclareLaunchArgument(
            'virtual_detections', default_value='false',
            description="Start virtual_detections on the scenario's bottles"),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the sim clock'),
        OpaqueFunction(function=nodes),
    ])
