"""
Goals relative to the boat, inside the test area: relative_goal.

Sends goal; run it next to a controller launch, such as kingfisher_rl's
passthrough_thrusters.launch.py, which starts goal_publisher. scenario is a name under
config/scenarios/ or a path to a scenario YAML.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def nodes(context):
    """Return the node, with the scenario resolved to a file."""
    share = get_package_share_directory('kingfisher_goals')
    config = os.path.join(share, 'config', 'goals.yaml')
    scenario = LaunchConfiguration('scenario').perform(context)
    if not scenario.endswith('.yaml'):
        scenario = os.path.join(share, 'config', 'scenarios', scenario + '.yaml')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}

    return [GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='kingfisher_goals', executable='relative_goal',
            parameters=[config, sim_time, {'goals_file': scenario}],
            output='screen'),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='kingfisher',
            description='ROS namespace of the boat'),
        DeclareLaunchArgument(
            'scenario', default_value='relative_small',
            description='Scenario: a name under config/scenarios/, or a .yaml path'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the sim clock'),
        OpaqueFunction(function=nodes),
    ])
