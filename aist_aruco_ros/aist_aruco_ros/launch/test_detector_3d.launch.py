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

CAMERAS = {
    'realsense': {'package':           'realsense2_camera',
                  'launch_file':       'launch.py',
                  'key_of_id':         'serial_no',
                  'cloud_topic':       '',
                  'depth_topic':       '/aligned_depth_to_color/image_raw',
                  'camera_info_topic': '/aligned_depth_to_color/camera_info',
                  'image_topic':       '/color/image_raw'},
    'phoxi':     {'package':           'aist_phoxi_camera',
                  'launch_file':       'launch.py',
                  'key_of_id':         'id',
                  'cloud_topic':       '/pointcloud',
                  'depth_topic':       '/depth_map',
                  'camera_info_topic': '/camera_info',
                  'image_topic':       '/texture'}}

launch_arguments = [
    {'name':        'camera_name',
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
    {'name':        'output',
     'default':     'both',
     'description': 'pipe node output [screen|log]'},
    {'name':        'log_level',
     'default':     'info',
     'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
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

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'], default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

def launch_setup(context):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera      = CAMERAS[camera_name]
    return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare(camera['package']), 'launch',
                         camera['launch_file']])),
                launch_arguments={
                    'config_file':        PathJoinSubstitution(
                                              [ThisLaunchFileDir(),
                                               '..', 'config',
                                               camera_name + '.yaml']),
                    camera['key_of_id']:  LaunchConfiguration('id'),
                    'external_container': 'false',
                    'container':          LaunchConfiguration('container'),
                    'output':             LaunchConfiguration('output'),
                    'log_level':          LaunchConfiguration('log_level')
                }.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([ThisLaunchFileDir().perform(context),
                                          'detector_3d.launch.py'])),
                launch_arguments={
                    'camera_info_topic':   camera_name \
                                           + camera['camera_info_topic'],
                    'image_topic':         camera_name + camera['image_topic'],
                    'depth_topic':         camera_name + camera['depth_topic'],
                    'cloud_topic':         '',
                    'container':           LaunchConfiguration('container'),
                    'marker_map':          LaunchConfiguration('marker_map'),
                    'planarity_tolerance': LaunchConfiguration(
                                               'planarity_tolerance')
                }.items()),
            Node(name='rviz',
                 package='rviz2', executable='rviz2', output='screen',
                 arguments=['-d',
                            PathJoinSubstitution(
                                [FindPackageShare('aist_aruco_ros'),
                                 'launch', camera_name + '.rviz'])]),
            Node(name='rqt_reconfigure', package='rqt_reconfigure',
                 executable='rqt_reconfigure', output='screen')]
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
