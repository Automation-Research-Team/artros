from launch                   import LaunchDescription
from launch.actions           import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions     import LaunchConfiguration
from launch_ros.actions       import Node

EXECUTABLES = {
    'butterworth': 'butterworth_lpf_test_node',
    'spline':      'spline_extrapolator_test_node'}

launch_arguments = [
    {'name':        'test_name',
     'default':     'butterworth',
     'description': 'test program name'},
    {'name':        'log_level',
     'default':     'info',
     'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
    {'name':        'output',
     'default':     'both',
     'description': 'pipe node output [screen|log]'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

def launch_setup(context):
    test_name  = LaunchConfiguration('test_name').perform(context)
    executable = EXECUTABLES[test_name]
    return [Node(name='gensig',
                 package='aist_utility',
                 executable='gensig.py',
                 output=LaunchConfiguration('output')),
            Node(name='test',
                 package='aist_utility',
                 executable=executable,
                 remappings=[('/in', 'gensig/out')],
                 output=LaunchConfiguration('output')),
            Node(name='plotjuggler', package='plotjuggler',
                 executable='plotjuggler', output='screen'),
            Node(name='rqt_reconfigure', package='rqt_reconfigure',
                 executable='rqt_reconfigure', output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
