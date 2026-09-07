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
        'default':     'camera_calibration_container',
        'description': 'name of the component container for calibration'
    },
    {
        'name':        'check',
        'default':     'false',
        'description': 'Check calibration result if true'
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
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Use simulation time if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
]


def launch_setup(context):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_type = load_config(context).get('cameras', {})[camera_name]['type']

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
                ('detector_name', 'multi_detector'),
                ('camera_type',   camera_type),
            ]),
        Node(name='camera_calibration',
             package='aist_routines',
             executable='camera_calibration',
             parameters=[
                 LaunchConfiguration('param_file'),
                 {'config_file':
                  PathJoinSubstitution([
                      FindPackageShare('aist_bringup'), 'config',
                      [LaunchConfiguration('config'), '.yaml']]),
                  'use_sim_time': LaunchConfiguration('sim')}
             ],
             prefix=['gnome-terminal --tab --wait --active --'],
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')])
    ]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
