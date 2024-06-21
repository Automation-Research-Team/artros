import yaml
from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution,
                                               ThisLaunchFileDir)
from launch_ros.actions                import LoadComposableNodes
from launch_ros.descriptions           import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

launch_arguments = [
    {'name':        'name',
     'default':     'detector_3d',
     'description': 'name of detector'},
    {'name':        'camera_info_topic',
     'default':     'phoxi/camera_info',
     'description': 'topic name of camera_info'},
    {'name':        'image_topic',
     'default':     'phoxi/texture',
     'description': 'topic name of intensity/color image'},
    {'name':        'depth_topic',
     'default':     'phoxi/depth_map',
     'description': 'topic name of depth image'},
    {'name':        'cloud_topic',
     'default':     'phoxi/pointcloud',
     'description': 'topic name of pointcloud'},
    {'name':        'config_file',
     'default':     '',
     'description': 'path to YAML file for configuring detector'},
    {'name':        'container',
     'default':     'my_container',
     'description': 'name of external component container'},
    {'name':        'log_level',
     'default':     'info',
     'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
    {'name':        'output',
     'default':     'both',
     'description': 'pipe node output [screen|log]'}]
parameter_arguments = [
    {'name':        'marker_map_dir',
     'default':     PathJoinSubstitution([ThisLaunchFileDir(),
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
    return [DeclareLaunchArgument(
                arg['name'],
                default_value=num_to_str(defaults.get(arg['name'],
                                                      arg['default'])),
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
    params     = load_parameters(
                     LaunchConfiguration('config_file').perform(context))
    actions    = declare_launch_arguments(param_args, params)
    remappings = [('/camera_info', LaunchConfiguration('camera_info_topic'))]
    if LaunchConfiguration('cloud_topic').perform(context) == '':
        remappings += [('/depth', LaunchConfiguration('depth_topic')),
                       ('/image', LaunchConfiguration('image_topic'))]
    else:
        remappings += [('/pointcloud', LaunchConfiguration('cloud_topic'))]
    params  |= set_configurable_parameters(param_args)
    actions += [LoadComposableNodes(
                    target_container=LaunchConfiguration('container'),
                    composable_node_descriptions=[
                        ComposableNode(
                            name=LaunchConfiguration('name'),
                            package='aist_aruco_ros',
                            plugin='aist_aruco_ros::Detector3D',
                            parameters=[params],
                            remappings=remappings,
                            extra_arguments=[
                                {'use_intra_process_comms': True}])])]
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
