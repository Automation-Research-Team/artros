import os, yaml
from pathlib                    import Path
from launch                     import LaunchDescription
from launch.actions             import (IncludeLaunchDescription,
                                        OpaqueFunction, RegisterEventHandler)
from launch.conditions          import IfCondition
from launch.event_handlers      import OnProcessExit
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
from launch_ros.substitutions   import FindPackageShare
from moveit_configs_utils       import MoveItConfigsBuilder
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'launch_servo',
        'default':     'false',
        'description': 'Launch moveit_servo?',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Use simulation time if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'warehouse_sqlite_path',
        'default':     os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
        'description': 'Path where the warehouse database should be stored'
    },
    {
        'name':        'publish_robot_description_semantic',
        'default':     'true',
        'description': 'MoveGroup publishes robot description semantic',
        'choices':     ['true', 'false', 'True', 'False']
    },
]

def load_yaml(file_path):
    try:
        with open(file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context):
    moveit_configs = MoveItConfigsBuilder(robot_name='aist_base_scene',
                                          package_name='aist_moveit_config') \
                    .to_moveit_configs()
    wait_robot_description = Node(package='aist_bringup',
                                  executable='wait_for_robot_description',
                                  output='screen')
    servo_yaml = load_yaml(PathJoinSubstitution(
                               [FindPackageShare('aist_moveit_config'),
                                'config', 'ur_servo.yaml']).perform(context))
    return [
        wait_robot_description,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[
                    Node(package='moveit_ros_move_group',
                         executable='move_group',
                         output='screen',
                         parameters=[
                             moveit_configs.to_dict(),
                             {
                               'warehouse_plugin':
                                 'warehouse_ros_sqlite::DatabaseConnection',
                               'warehouse_host':
                                 LaunchConfiguration('warehouse_sqlite_path'),
                               'use_sim_time':
                                 LaunchConfiguration('sim'),
                               'publish_robot_description_semantic':
                                 LaunchConfiguration(
                                     'publish_robot_description_semantic'),
                             }
                         ]),
                    Node(condition=IfCondition(
                                       LaunchConfiguration('launch_servo')),
                         package='moveit_servo',
                         executable='servo_node',
                         parameters=[
                             moveit_configs.to_dict(),
                             {'moveit_servo': servo_yaml}
                         ],
                         output='screen'),
                ]
            ))]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
