from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import declare_launch_arguments


launch_arguments = [
    {
        'name':        'task',
        'default':     'base',
        'description': 'Name of the client',
        'choices':     ['base', 'assembly', 'kitting', 'hmi_demo',
                        'handeye_calibration',
                        'camera_calibration', 'plane_calibration']
    },
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'settings_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_routines'), 'config',
                           'default.yaml']),
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Use simulation time if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
]

def launch_setup(context):
    return [
        Node(name=LaunchConfiguration('task'),
             package='aist_routines',
             executable=LaunchConfiguration('task'),
             parameters=[
                 LaunchConfiguration('settings_file'),
                 {
                     'config_file':
                     PathJoinSubstitution([
                         FindPackageShare('aist_bringup'), 'config',
                         [LaunchConfiguration('config'), '.yaml']]),
                     'use_sim_time': LaunchConfiguration('sim'),
                 }
             ],
             prefix=['gnome-terminal --tab --wait --active --'],
             output='screen')
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
