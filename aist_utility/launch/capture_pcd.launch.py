from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               IncludeLaunchDescription,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution,
                                               ThisLaunchFileDir)
from launch.conditions                 import LaunchConfigurationEquals
from launch_ros.actions                import Node, LoadComposableNodes
from launch_ros.substitutions          import FindPackageShare
from launch_ros.descriptions           import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

CAMERAS = {'realsense': {'package':     'realsense2_camera',
                         'launch_file': 'launch.py',
                         'cloud_topic': '/depth/color/points',
                         'depth_topic': '/aligned_depth_to_color/image_raw',
                         'cinfo_topic': '/aligned_depth_to_color/camera_info',
                         'color_topic': '/color/image_raw',
                         'cloud_frame': 'camera_link'},
           'phoxi':     {'package':     'aist_phoxi_camera',
                         'launch_file': 'launch.py',
                         'cloud_topic': '/pointcloud',
                         'depth_topic': '/depth_map',
                         'cinfo_topic': '/camera_info',
                         'color_topic': '/texture',
                         'cloud_frame': 'base_link'}}

launch_arguments = [{'name':        'camera_name',
                     'default':     'realsense',
                     'description': 'camera unique name'},
                    {'name':        'config_dir',
                     'default':     '',
                     'description': 'directory path containing yaml config file for test and camera'},
                    {'name':        'container',
                     'default':     'my_container',
                     'description': 'name of internal or external component container'},
                    {'name':        'log_level',
                     'default':     'info',
                     'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                    {'name':        'output',
                     'default':     'both',
                     'description': 'pipe node output [screen|log]'}]

def declare_launch_arguments(args, defaults={}):
    return [DeclareLaunchArgument(
                arg['name'],
                default_value=str(defaults.get(arg['name'], arg['default'])),
                description=arg['description']) \
            for arg in args]

def launch_setup(context):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera      = CAMERAS[camera_name]
    params      = {'cloud_frame': camera['cloud_frame']}
    return [Node(name='static_transform_publisher',
                 package='tf2_ros', executable='static_transform_publisher',
                 output='screen',
                 arguments=['0.0', '0.0', '0.6', '0.0', '0.0', '3.14',
                            'base_link', camera_name + '_sensor'],
                 condition=LaunchConfigurationEquals('camera_name', 'phoxi')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare(camera['package']), 'launch',
                         camera['launch_file']])),
                launch_arguments={'config_file':
                                  PathJoinSubstitution(
                                      [ThisLaunchFileDir(), '..', 'config',
                                       camera_name + '.yaml']),
                                  'container':
                                  LaunchConfiguration('container'),
                                  'config_dir':
                                  LaunchConfiguration('config_dir'),
                                  'output':
                                  LaunchConfiguration('output'),
                                  'log_level':
                                  LaunchConfiguration('log_level')}.items()),
            LoadComposableNodes(
                target_container=LaunchConfiguration('container'),
                composable_node_descriptions=[
                    ComposableNode(
                        name='capture_pcd',
                        package='aist_utility',
                        plugin='aist_utility::PCDCapturer',
                        remappings=[
                            ('/depth',
                             camera_name + camera['depth_topic']),
                            ('/camera_info',
                             camera_name + camera['cinfo_topic']),
                            ('/color',
                             camera_name + camera['color_topic'])],
                        parameters=[params],
                        extra_arguments=[
                            {'use_intra_process_comms': True}])]),
            Node(name='rviz',
                 package='rviz2', executable='rviz2', output='screen',
                 arguments=['-d',
                            PathJoinSubstitution(
                                [FindPackageShare('aist_utility'), 'launch',
                                 camera_name + '.rviz'])]),
            Node(name='rqt_reconfigure', package='rqt_reconfigure',
                 executable='rqt_reconfigure', output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
