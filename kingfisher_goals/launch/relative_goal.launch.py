"""
Goals relative to the boat, inside the test area: relative_goal, and a bag of the run.

Sends goal; run it next to a controller launch, such as kingfisher_rl's
passthrough_thrusters.launch.py, which starts goal_publisher. scenario is a name under
config/scenarios/ or a path to a scenario YAML. With record, the run is recorded to
bag_dir/<scenario>_<date-time>; relative_goal exits once all goals are done, and the launch with
it, which closes the bag.
"""

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

# in the boat's namespace, except the absolute ones
TOPICS = ['odom', 'joy', 'goal', 'goal/clear', 'control/goal', 'control/cmd_drive', 'cmd_drive',
          'relative_goal/status', 'relative_goal/active', '/tf', '/tf_static', '/sense', 'anemometer/wind', 'anemometer/twist']


def nodes(context):
    """Return the node, the recorder and the shutdown once the node exits."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('kingfisher_goals')
    config = os.path.join(share, 'config', 'goals.yaml')
    scenario = arg('scenario')
    if not scenario.endswith('.yaml'):
        scenario = os.path.join(share, 'config', 'scenarios', scenario + '.yaml')
    sim_time = arg('use_sim_time') == 'true'
    namespace = arg('namespace')

    node = Node(
        package='kingfisher_goals', executable='relative_goal',
        parameters=[config, {'use_sim_time': sim_time}, {'goals_file': scenario}],
        output='screen')
    actions = [
        GroupAction([PushRosNamespace(namespace), node]),
        RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[EmitEvent(event=Shutdown(reason='goals done'))])),
    ]
    if arg('record') == 'true':
        bag_dir = os.path.expanduser(arg('bag_dir'))
        os.makedirs(bag_dir, exist_ok=True)
        name = os.path.splitext(os.path.basename(scenario))[0]
        bag = os.path.join(bag_dir, f'{name}_{datetime.now():%Y-%m-%d_%H-%M-%S}')
        topics = [t if t.startswith('/') else f'/{namespace}/{t}' for t in TOPICS]
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag, *(['--use-sim-time'] if sim_time else []),
                 '--topics', *topics],
            output='screen'))
    return actions


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
        DeclareLaunchArgument(
            'record', default_value='true',
            description='Record the run to a bag'),
        DeclareLaunchArgument(
            'bag_dir', default_value='~/bags/relative_goal',
            description='Folder of the bags, one per run'),
        OpaqueFunction(function=nodes),
    ])
