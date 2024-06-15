import os
import yaml
from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               IncludeLaunchDescription,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution,
                                               ThisLaunchFileDir)
from launch_ros.actions                import Node, LoadComposableNodes
from launch_ros.substitutions          import FindPackageShare
from launch_ros.descriptions           import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

CAMERAS = {'realsense': {'package':     'realsense2_camera',
                         'launch_file': 'launch.py',
                         'key_of_id':   'serial_no',
                         'cloud_topic': '/depth/color/points',
                         'depth_topic': '/aligned_depth_to_color/image_raw',
                         'cinfo_topic': '/aligned_depth_to_color/camera_info',
                         'color_topic': '/color/image_raw'},
           'phoxi':     {'package':     'aist_phoxi_camera',
                         'launch_file': 'launch.py',
                         'key_of_id':   'id',
                         'cloud_topic': '/pointcloud',
                         'depth_topic': '/depth_map',
                         'cinfo_topic': '/camera_info',
                         'color_topic': '/texture'}}

launch_arguments = [{'name':        'camera_name',
                     'default':     'realsense',
                     'description': 'camera unique name'},
                    {'name':        'id',
                     'default':     '',
                     'description': 'unique ID of camera'},
                    {'name':        'config_file',
                     'default':     '',
                     'description': 'path to YAML file for configuring detector'},
                    {'name':        'container',
                     'default':     'my_container',
                     'description': 'name of internal or external component container'},
                    {'name':        'log_level',
                     'default':     'info',
                     'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                    {'name':        'output',
                     'default':     'both',
                     'description': 'pipe node output [screen|log]'}]
parameter_arguments = [{'name':        'marker_map_dir',
                        'default':     PathJoinSubstitution(
                                           [ThisLaunchFileDir(),
                                            '..', 'config']),
                        'description': 'directory name containing marker map'},
                       {'name':        'marker_map',
                        'default':     'aruco-26-70x70-5',
                        'description': 'name of marker map'},
                       {'name':        'marker_frame',
                        'default':     'marker_frame',
                        'description': 'marker frame ID'},
                       {'name':        'image_is_rectified',
                        'default':     'false',
                        'description': 'true if image is rectifed'},
                       {'name':        'planarity_tolerance',
                        'default':     '0.001',
                        'description': 'maximum tolerance when checking planarity'}]

def declare_launch_arguments(args, defaults={}):
    num_to_str = lambda x : str(x) if isinstance(x, (bool, int, float)) else x
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=defaults.get(
                                                  arg['name'],
                                                  num_to_str(arg['default'])),
                                  description=arg['description']) \
            for arg in args]

def set_configurable_parameters(args):
    return dict([(arg['name'], LaunchConfiguration(arg['name'])) \
                 for arg in args])

def load_parameters(config_file):
    if config_file == '':
        return {}
    with open(config_file, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, param_args):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera      = CAMERAS[camera_name]
    params      = load_parameters(
                      LaunchConfiguration('config_file').perform(context))
    actions     = declare_launch_arguments(param_args, params)
    params     |= set_configurable_parameters(param_args)
    actions    += [IncludeLaunchDescription(
                       PythonLaunchDescriptionSource(
                           PathJoinSubstitution(
                               [FindPackageShare(camera['package']), 'launch',
                                camera['launch_file']])),
                       launch_arguments={'config_file':
                                         PathJoinSubstitution(
                                             [ThisLaunchFileDir(),
                                              '..', 'config',
                                              camera_name + '.yaml']),
                                         camera['key_of_id']:
                                         LaunchConfiguration('id'),
                                         'container':
                                         LaunchConfiguration('container'),
                                         'output':
                                         LaunchConfiguration('output'),
                                         'log_level':
                                         LaunchConfiguration('log_level')}.items()),
                   LoadComposableNodes(
                       target_container=LaunchConfiguration('container'),
                       composable_node_descriptions=[
                           ComposableNode(
                               name='detector_3d',
                               package='aist_aruco_ros',
                               plugin='aist_aruco_ros::Detector3D',
                               parameters=[params],
                               remappings=[
                                   # ('/pointcloud',
                                   #  camera_name + camera['cloud_topic']),
                                   ('/depth',
                                    camera_name + camera['depth_topic']),
                                   ('/camera_info',
                                    camera_name + camera['cinfo_topic']),
                                   ('/image',
                                    camera_name + camera['color_topic'])],
                               extra_arguments=[
                                   {'use_intra_process_comms': True}])]),
                   Node(name='rviz',
                        package='rviz2', executable='rviz2', output='screen',
                        arguments=['-d',
                                   PathJoinSubstitution(
                                       [FindPackageShare('aist_aruco_ros'),
                                        'launch',
                                        camera_name + '.rviz'])]),
                   Node(name='rqt_reconfigure', package='rqt_reconfigure',
                        executable='rqt_reconfigure', output='screen')]
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
