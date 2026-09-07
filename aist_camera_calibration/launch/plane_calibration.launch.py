from launch                     import LaunchDescription
from launch.actions             import (SetLaunchConfiguration,
                                        IncludeLaunchDescription,
                                        OpaqueFunction)
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch.conditions          import UnlessCondition
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.substitutions   import FindPackageShare
from launch_ros.descriptions    import ComposableNode
from aist_bringup.launch_common import declare_launch_arguments, load_config

launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_camera_calibration'),
                           'config', 'plane_calibration.yaml']),
        'description': 'absolute path to YAML parameter file'
    },
    {
        'name':        'camera_name',
        'default':     'live_camera',
        'description': 'name of the camera to be calibrated'
    },
    {
        'name':        'external_container',
        'default':     'false',
        'description': 'use external container launched in advance',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'container',
        'default':     'plane_calibration_container',
        'description': 'name of the component container for calibration'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'log',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    },
]


def launch_setup(context):
    camera_names = LaunchConfiguration('camera_name').perform(context)\
                                                     .split(',')
    cameras      = load_config(context).get('cameras', {})
    camera_types = [cameras[camera_name]['type']
                    for camera_name in camera_names]

    return [
        SetLaunchConfiguration(
            'param_file',
            PathJoinSubstitution([
                FindPackageShare('aist_camera_calibration'), 'config',
                [LaunchConfiguration('camera_name'), '.yaml']])),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    name='camera_calibrator',
                    package='aist_camera_calibration',
                    plugin='aist_camera_calibration::Calibrator',
                    parameters=[LaunchConfiguration('param_file')],
                    remappings=[('point_correspondences_set',
                                 'multi_detector/point_correspondences_set')],
                    extra_arguments=[{'use_intra_process_comms': True}])
            ]),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_aruco_ros'), 'launch',
                                  'multi_detector.launch.py']),
            launch_arguments=[
                ('detector_name',      'multi_detector'),
                ('camera_name',        camera_names),
                ('camera_type',        camera_types),
                ('param_file',         LaunchConfiguration('param_file')),
                ('external_container', LaunchConfiguration('external_container')),
                ('container',          LaunchConfiguration('container')),
            ]),
        Node(name='plane_calibration',
             package='aist_routines',
             executable='plane_calibration',
             parameters=[
                 LaunchConfiguration('param_file'),
                 {'param_file':
                  PathJoinSubstitution([
                      FindPackageShare('aist_bringup'), 'config',
                      [LaunchConfiguration('config'), '.yaml']])}
             ],
             prefix=['gnome-terminal --tab --wait --active --'],
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')])
    ]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
