# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Toshio Ueshiba
#
import rclpy, sys, time, yaml, re, readline
import numpy as np
import moveit_commander
import tf_transformations as tfs

from cmd                           import Cmd
from math                          import degrees, sqrt, pi
from rclpy.node                    import Node
from rclpy.duration                import Duration
from rclpy.time                    import Time
from rclpy.parameter               import Parameter
from rclpy.callback_groups         import MutuallyExclusiveCallbackGroup
from tf2_ros.buffer                import Buffer
from tf2_ros.transform_listener    import TransformListener
from std_msgs.msg                  import Header
from rcl_interfaces.msg            import ParameterValue
from geometry_msgs.msg             import (PoseStamped, Pose, PoseArray,
                                           PointStamped, Point, Quaternion,
                                           Vector3Stamped, Vector3)
from moveit_msgs.msg               import (RobotTrajectory, PositionIKRequest,
                                           MoveItErrorCodes)
from moveit_msgs.srv               import GetPositionIK
from trajectory_msgs.msg           import JointTrajectoryPoint, JointTrajectory
from controller_manager_msgs.srv   import ListControllers, SwitchController
from action_msgs.msg               import GoalStatus
from aist_utility.fileio           import filepath_from_url
from aist_utility.geometry_msgs    import (transform_matrix, pose_matrix,
                                           pose_from_matrix)
from aist_tasks                    import PickOrPlaceTask, PickOrPlaceToolTask
from aist_collision_object_manager import CollisionObjectManager
from ddynamic_reconfigure2         import declare_read_only_parameter
from .gripper_client               import create_gripper
from .camera_client                import CameraClient

#*********************************************************************
#  global functions                                                  *
#*********************************************************************
def get_grippers(config, name=''):
    if 'grippers' in config:
        grippers={}
        for gripper_name, gripper_config in config['grippers'].items():
            grippers |= get_grippers(gripper_config, gripper_name)
        return grippers
    return {} if name == '' else {name: config}

#*********************************************************************
#  class BaseRoutines                                                *
#*********************************************************************
class BaseRoutines(Node, Cmd):
    """ Collection of basic routines for controlling arms, grippers
    and cameras.
    """
    ControllerTypes = (
        'joint_trajectory_controller/JointTrajectoryController',
        'forward_command_controller/ForwardCommandController',
        'cartesian_motion_controller/CartesianMotionController',
        'cartesian_force_controller/CartesianForceController',
        'cartesian_compliance_controller/CartesianComplianceController',
    )

    def __init__(self, name: str):
        """
        Args:
          name: Node name.
        """
        Node.__init__(self, name)
        Cmd.__init__(self)

        # Create TransformListener
        self._tf2_buffer   = Buffer()
        self._tf2_listener = TransformListener(self._tf2_buffer, self)
        time.sleep(1.0)        # Necessary for listener spinning up

        # MoveIt planning parameters
        self._eef_step        = self.declare_parameter('moveit_eef_step',
                                                       0.0005).value
        self._reference_frame = self.declare_parameter('reference_frame',
                                                       'world').value
        self._move_lin        = self.declare_parameter('move_lin',
                                                       True).value

        # MoveIt RobotCommander and MoveGroup
        moveit_commander.roscpp_initialize(sys.argv)
        self._cmd = moveit_commander.RobotCommander(
                        self.declare_parameter('robot_description',
                                               'robot_description').value)
        for group_name in self._cmd.get_group_names():
            group = self._cmd.get_group(group_name)
            group.set_pose_reference_frame(self.reference_frame)
        self.get_logger().info(
            'planning_frame: %s, reference_frame: %s, eef_step: %f'
            % (self.planning_frame, self.reference_frame, self.eef_step))

        # MoveIt GetPositionIK service client
        self._service_cbg = MutuallyExclusiveCallbackGroup()
        self._compute_ik = self.create_client(GetPositionIK, '/compute_ik',
                                              callback_group=self._service_cbg)

        # Hardware configuration
        with open(self.declare_parameter('config_file', name + '.yaml').value,
                  'r') as f:
            config = yaml.safe_load(f)

        # Controller_manager services for arms
        self._list_controllers_srvs \
            = {name: self.create_client(
                         ListControllers,
                         name + '/controller_manager/list_controllers',
                         callback_group=self._service_cbg)
               for name in config['arms']}

        self._switch_controller_srvs \
            = {name: self.create_client(
                         SwitchController,
                         name + '/controller_manager/switch_controller',
                         callback_group=self._service_cbg)
               for name in config['arms']}

        # Grippers
        self._grippers = {name: create_gripper(self, name, props['type'],
                                               props.get('client_args', {}))
                          for name, props in get_grippers(config).items()}
        self._default_gripper_names = {}
        self._active_grippers       = {}
        for name, props in config.get('arms', {}).items():
            self._default_gripper_names[name] = props['default_gripper']
            self.set_gripper(name, self.default_gripper_name(name))

        # Cameras
        self._cameras = {name: CameraClient.create(self, name, props['type'],
                                                   props.get('client_args',
                                                             {}))
                         for name, props in config.get('cameras', {}).items()}

        # Load setting parameters
        self.declare_parameter('setting_urls', [''])
        self.load_settings()

        # CollisionObjectManager wrapping MoveIt PlanningSceneInterface
        self._com = CollisionObjectManager(self)

        # Pick and place task
        self._pick_or_place  = PickOrPlaceTask(self)

        # Pick and place tool task
        self._pick_or_place_tool = PickOrPlaceToolTask(self)

        # Interpreter stuffs
        self._robot_name   = self.declare_parameter('initial_robot_name',
                                                    self.group_names[0]).value
        self._axis         = self.declare_parameter('initial_axis', 1).value
        self._speed        = self.declare_parameter('initial_speed', 1.0).value
        self._recent_tasks = {}
        delims = readline.get_completer_delims()
        if '/' in delims:
            readline.set_completer_delims(delims.replace('/', ''))

        self.get_logger().info('BaseRoutines initialized.')

    @property
    def tf2_buffer(self)-> Buffer:
        """ TF2 buffer associated with this class.
        """
        return self._tf2_buffer

    @property
    def planning_frame(self)-> str:
        """ MoveIt planning frame.
        """
        return self._cmd.get_planning_frame()

    @property
    def reference_frame(self)-> str:
        """ MoveIt reference frame.
        """
        return self._reference_frame

    @property
    def eef_step(self)-> float:
        """ MoveIt end-effector step.
        """
        return self._eef_step

    @property
    def move_lin(self)-> bool:
        return self._move_lin

    @move_lin.setter
    def move_lin(self, enable):
        self._move_lin = enable

    @property
    def group_names(self)-> list[str]:
        """ Name list of MoveIt groups.
        """
        return self._cmd.get_group_names()

    @property
    def robot_names(self)-> list[str]:
        """ Name list of arms.
        """
        return list(self._list_controllers_srvs.keys())

    @property
    def gripper_names(self)-> list[str]:
        """ Name list of grippers.
        """
        return list(self._grippers.keys())

    @property
    def tool_names(self)-> list[str]:
        """ Name list of tools.
        """
        return [n for n, g in self._grippers.items() if '/' in g.base_link]

    @property
    def camera_names(self)-> list[str]:
        """ Name list of cameras.
        """
        return list(self._cameras.keys())

    @property
    def frame_ids(self)-> list[str]:
        """ ID list of frames.
        """
        return list(yaml.safe_load(self.tf2_buffer.all_frames_as_yaml()) \
                    .keys())

    @property
    def object_frames(self):
        return list(filter(lambda frame_id: '/' in frame_id, self.frame_ids))

    @property
    def com(self)-> CollisionObjectManager:
        """ Client of collision object manager associated with this class.
        """
        return self._com

    @property
    def collision_object_ids(self)-> list[str]:
        return list(self.com.collision_objects.keys())

    @property
    def attached_collision_object_ids(self)-> list[str]:
        return list(self.com.attached_collision_objects.keys())

    @property
    def all_collision_object_ids(self)-> list[str]:
        return self.collision_object_ids + self.attached_collision_object_ids

    @property
    def settings(self)-> dict:
        """ Settings for this class.
        The settings are loaded from files whose names are specified by
        the parameter 'setting_urls'.
        """
        return self._settings

    def load_settings(self)-> None:
        def recursive_merge(d1, d2):
            if type(d1) != dict or type(d2) != dict:
                return d2
            for k2, v2 in d2.items():
                d1[k2] = recursive_merge(d1.get(k2, {}), v2)
            return d1

        # Load setting parameters
        self._settings = {}
        for url in self.get_parameter('setting_urls') \
                       .get_parameter_value().string_array_value:
            try:
                with open(filepath_from_url(url), 'r') as f:
                    self._settings = recursive_merge(self._settings,
                                                     yaml.safe_load(f))
            except Exception as e:
                self.get_logger().error('failed to load setting file: %s' % e)

    #
    # CLI(command line interface) stuffs
    #
    @property
    def prompt(self):
        axes = ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')
        return '{:>5}:{}({})@{}>> '.format(axes[self._axis],
                                           self.format_pose(
                                               self.get_current_pose(
                                                   self._robot_name)),
                                           self._speed, self._robot_name)

    def preloop(self):
        Cmd.prompt = self.prompt
        self.initialize_collision_objects()
        self.go_to_named_pose(self._robot_name, 'home')
        self.do_cmds(None)

    def precmd(self, line):
        try:
            xyzrpy = self.xyzrpy_from_pose(self.get_current_pose(
                self._robot_name))
            xyzrpy[self._axis] = float(line)
            self.go_to_pose_goal(self._robot_name,
                                 self.pose_from_xyzrpy(xyzrpy),
                                 speed=self._speed)
            return ''
        except ValueError:
            pass

        offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if line.startswith('+'):
            offset[self._axis] += 0.01 if self._axis < 3 else 10.0
            self.move_relative(self._robot_name, offset, self._speed)
            return ''
        elif line.startswith('-'):
            offset[self._axis] -= 0.01 if self._axis < 3 else 10.0
            self.move_relative(self._robot_name, offset, self._speed)
            return ''
        return line

    def postcmd(self, stop, line):
        Cmd.prompt = self.prompt
        return stop

    def emptyline(self):
        pass

    def do_cmds(self, dummy):
        """      Print command list."""
        print('=== General commands ===')
        print('  quit/EOF:    quit this program')
        print('  reload:      reload settings')
        print('  robot:       select robot')
        print('  ?|help:      print help messages')
        print('=== Arm commands ===')
        print('  X|Y|Z|R|P|W: select arm axis to be driven')
        print('  +|-:         move arm by 10(mm)/10(deg) along the current axis')
        print('  <numeric>:   move arm to the specified coordinate along the current axis')
        print('  home:        move arm to the home position')
        print('  back:        move arm to the back position')
        print('  named:       move arm to the pose specified by name')
        print('  frame:       move arm to the pose specified by frame')
        print('  clip:        make wrist angle within [-180, 180] deg.')
        print('  speed:       set speed')
        print('  stop:        stop arm immediately')
        print('  jvalues:     get current joint values')
        print('  switch:      switch controller')
        print('  toggle:      toggle motion control handle')
        print('  ftreset:     reset bias of ft-sensor')
        print('  lin:         enforce linear path')
        print('  LIN:         not enforce linear path')
        print('=== Gripper commands ===')
        print('  gripper:     assign gripper to current robot')
        print('  pregrasp:    pregrasp with the current gripper')
        print('  grasp:       grasp with the current gripper')
        print('  postgrasp:   postgrasp with the current gripper')
        print('  release:     release with the current gripper')
        print('  gpos:        set gripper position')
        print('  gvel:        set gripper velocity')
        print('  tighten:     tighten screw')
        print('  loosen:      loosen screw')
        print('  gcancel:     cancel gripper action')
        print('=== Task commands ===')
        print('  pick:        pick object')
        print('  place:       place object')
        print('  pt:          pick/place tool')
        print('  cancel       cancel recent task')
        print('=== Collision object commands ===')
        print('  I:  Initialize all collision objects')
        print('  i:  Show infomation on collision objects')
        print('  ci: Show infomation on child collision object of frame')
        print('  r:  Remove specified collision objects')

    def do_EOF(self, dummy):
        """      Quit program."""
        print('bye')
        return True

    def do_quit(self, dummy):
        """      Quit program."""
        return self.do_EOF(dummy)

    def do_reload(self, dummy):
        """      reload
        Reload settings."""
        self.load_settings()

    def do_robot(self, robot_name):
        """      robot [robot_name]
        Activate the specified robot."""
        if robot_name:
            if robot_name in self.robot_names:
                self._robot_name = robot_name
            else:
                print('     unknown robot name[%s]!' % robot_name)
        else:
            print('     current robot: %s' % self._robot_name)

    def complete_robot(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line, self.robot_names)

    def do_X(self, dummy):
        """      X
        Set control axis to X."""
        self._axis = 0

    def do_Y(self, dummy):
        """      Y
        Set control axis to Y."""
        self._axis = 1

    def do_Z(self, dummy):
        """      Z
        Set control axis to Z."""
        self._axis = 2

    def do_R(self, dummy):
        """      R
        Set control axis to Roll."""
        self._axis = 3

    def do_P(self, dummy):
        """      P
        Set control axis to Pitch."""
        self._axis = 4

    def do_W(self, dummy):
        """      W
        Set control axis to Yaw."""
        self._axis = 5

    def do_home(self, dummy):
        """      home
        Move current active robot to home pose."""
        self.go_to_named_pose(self._robot_name, 'home')

    def do_back(self, dummy):
        """      back
        Move current active robot to back pose."""
        self.go_to_named_pose(self._robot_name, 'back')

    def do_named(self, named_pose):
        """      named [named_pose]
        Move current active robot to the specified named pose."""
        self.go_to_named_pose(self._robot_name, named_pose)

    def complete_named(self, text, line, ib, ie):
        return BaseRoutines._complete_default(
                   text, line,
                   self._cmd.get_group(self._robot_name).get_named_targets())

    def do_frame(self, args):
        """      frame <frame_id> [eef_link] [offset]
        Move current active robot to the specified frame."""
        tokens = args.split()
        if len(tokens) == 0:
            print('      frame ID not specified!')
            return
        frame_id = tokens[0]
        eef_link = ''
        offset = []
        for i, token in enumerate(tokens[1:]):
            try:
                offset.append(float(token))
            except ValueError:
                if i != 0:
                    print('      invalid offset value[%s]!' % token)
                    return
                eef_link = token

        if frame_id not in self.frame_ids:
            print('      unknown frame ID[%s]!' % frame_id)
            return
        if eef_link and \
           eef_link not in self.candidate_eef_links(self._robot_name):
            print('      invalid end-effector link[%s]!' % eef_link)
            return

        self.go_to_frame(self._robot_name, frame_id, offset,
                         speed=self._speed, end_effector_link=eef_link)

    def complete_frame(self, text, line, ib, ie):
        candidate_eef_links = self.candidate_eef_links(self._robot_name)
        frame_ids = list(set(self.frame_ids) - set(candidate_eef_links))
        return BaseRoutines._complete_default(text, line,
                                              frame_ids, candidate_eef_links)

    def do_clip(self, dummy):
        """      clip
        Clip wrist joint value."""
        self.clip_wrist_joint_value(self._robot_name)

    def do_speed(self, speed_value):
        """      speed [speed_value]
        Set speed value of the current robot."""
        if speed_value:
            try:
                self._speed = float(speed_value)
            except ValueError:
                print('      invalid speed value[%s]!' % speed_value)
        else:
            print('      current speed: %f' % self._speed)

    def do_stop(self, dummy):
        """      stop
        Stop robot immediately."""
        self.stop(self._robot_name)

    def do_jvalues(self, dummy):
        """      jvalues
        Print current joint values of the robot."""
        print(self.get_current_joint_values(self._robot_name))

    def do_switch(self, controller_name):
        """      switch [controller_name]
        Switch controller of the current robot to the specified one."""
        if controller_name:
            self.switch_controller(self._robot_name, controller_name)
        else:
            active_controller = self.active_controller(self._robot_name)
            if active_controller:
                print('      current active controller: %s'
                      % active_controller.name)
            else:
                print('      no active controllers')

    def complete_switch(self, text, line, ib, ie):
        controller_names = [c.name
                            for c in self.list_controllers(self._robot_name)]
        return BaseRoutines._complete_default(text, line, controller_names)

    def do_toggle(self, dummy):
        """      toggle
        Toggle active/inactive state of motion control handle."""
        self.toggle_motion_control_handle(self._robot_name)

    def do_ftreset(self, dummy):
        """      ftreset
        Reset bias of FT-sensor to all-zero."""
        self.ftsensor_reset_bias(self._robot_name)

    def do_lin(self, dummy):
        """      lin
        Enforce linear path."""
        self.move_lin = True

    def do_LIN(self, dummy):
        """      lin
        Not enforce linear path."""
        self.move_lin = False

    def do_gripper(self, gripper_name):
        """      gripper [gripper_name]
        Assign gripper to current active robot."""
        if gripper_name:
            self.set_gripper(self._robot_name, gripper_name)
        else:
            print('      current gripper: %s'
                  % self.gripper(self._robot_name).name)

    def complete_gripper(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line, self.gripper_names)

    def do_pregrasp(self, dummy):
        """      pregrasp
        Pregrasp with current gripper."""
        self.pregrasp(self._robot_name)

    def do_grasp(self, dummy):
        """      grasp
        Grasp with current gripper."""
        self.grasp(self._robot_name)

    def do_postgrasp(self, dummy):
        """      postgrasp
        Postgrasp with current gripper."""
        self.postgrasp(self._robot_name)

    def do_release(self, dummy):
        """      release
        Release with current gripper."""
        self.release(self._robot_name)

    def do_gpos(self, pos):
        """      gpos <position>
        Move gripper to the specified position."""
        try:
            position = float(pos)
            self.set_gripper_position(self._robot_name, position)
        except ValueError:
            print('      invalid position value[%s]' % pos)

    def do_gvel(self, vel):
        """      gvel <velocity>
        Set velocity value of the gripper."""
        try:
            velocity = float(vel)
            self.set_gripper_velocity(self._robot_name, velocity)
        except ValueError:
            print('      invalid velocity value[%s]' % vel)

    def do_tighten(self, dummy):
        """      tighten
        Tighten screw."""
        self.tighten(self._robot_name)

    def do_loosen(self, dummy):
        """      loosen
        Loosen screw."""
        self.loosen(self._robot_name)

    def do_gcancel(self, dummy):
        """      gcancel
        Cancel gripper action."""
        self.gripper_cancel(self._robot_name)

    def do_pick(self, object_frame):
        """      pick <object_frame>
        Pick a collision object at the specified object frame."""
        if object_frame not in self.object_frames:
            print('      unknown object frame[%s]' % object_frame)
            return
        self._recent_tasks[self._robot_name] = self._pick_or_place
        self.pick_at_frame(self._robot_name,
                           BaseRoutines._get_object_id(object_frame),
                           object_frame, timeout_sec=0.0)

    def complete_pick(self, text, line, ib, ie):
        object_frames = list(set(self.object_frames) -
                             set(self.candidate_eef_links(self._robot_name)))
        return BaseRoutines._complete_default(text, line, object_frames)

    def do_place(self, args):
        """      place <object_frame> <place_frame>
        Place a collision object at the specified place frame."""
        tokens = args.split()
        if len(tokens) < 2:
            print('      object frame and/or place frame not specified!')
            return
        object_frame = tokens[0]
        place_frame  = tokens[1]
        if object_frame not in self.candidate_eef_links(self._robot_name):
            print('      invalid object frame[%s]!' % object_frame)
            return
        if place_frame not in self.frame_ids:
            print('      unknown place frame[%s]!' % place_frame)
            return
        self._recent_tasks[self._robot_name] = self._pick_or_place
        self.place_at_frame(self._robot_name,
                            BaseRoutines._get_object_id(object_frame),
                            place_frame, eef_link=object_frame,
                            timeout_sec=0.0)

    def complete_place(self, text, line, ib, ie):
        object_frames = self.candidate_eef_links(self._robot_name)
        place_frames  = list(set(self.frame_ids) - set(object_frames))
        return BaseRoutines._complete_default(text, line,
                                              object_frames, place_frames)

    def do_pt(self, tool_name):
        """      pt [tool_name]
        Pick tool with specified name. Place tool if no name specified."""
        self._recent_tasks[self._robot_name] = self._pick_or_place_tool
        if tool_name:
            self.pick_tool(self._robot_name, tool_name, timeout_sec=0.0)
        else:
            self.place_tool(self._robot_name, timeout_sec=0.0)

    def complete_pt(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line, self.tool_names)

    def do_cancel(self, dummy):
        """      cancel
        Cancel task currently executed by the current robot."""
        recent_task = self._recent_tasks.pop(self._robot_name, None)
        if recent_task:
            recent_task.cancel_goal(self._robot_name)

    def do_I(self, dummy):
        """      I
        Erase all collision objects and then recreate them."""
        self.initialize_collision_objects()

    def do_i(self, object_id):
        """      i <object_id>
        Show information on collision object with specified ID."""
        info = self.com.get_object_info(object_id)
        if info:
            self.print_object_info(info)
        else:
            print('      unknown objet ID[%s]' % object_id)

    def complete_i(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line,
                                              self.all_collision_object_ids)

    def do_di(self, frame_id):
        """      di <frame_id>
        Show infomation on collision objects which are descendants of specified frame."""
        info_list = self.com.get_attached_descendant_objects_info(frame_id)
        for info in info_list:
            self.print_object_info(info)
            print('----------------')

    def complete_di(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line, self.frame_ids)

    def do_r(self, object_id):
        """      r [object_id]
        Remove collision object if specified, or all objects otherwise."""
        self.com.remove_object(object_id)

    def complete_r(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line,
                                              self.all_collision_object_ids)

    @staticmethod
    def _complete_default(text, line, *completions):
        nargs = len(line.split())
        return completions[nargs-1] if not text else \
               [c for c in completions[nargs-2] if c.startswith(text)]

    #
    # Joint motion stuffs
    #
    def get_joint_names(self, robot_name):
        return self._cmd.get_group(robot_name).get_active_joints()

    def get_current_joint_values(self, robot_name):
        return self._cmd.get_group(robot_name).get_current_joint_values()

    def get_named_joint_values(self, robot_name, named_pose):
        target_values = self._cmd.get_group(robot_name)\
                                 .get_named_target_values(named_pose)
        return [target_values[joint_name]
                for joint_name in self.get_joint_names(robot_name)]

    def remember_joint_values(self, robot_name, name, joint_values=None):
        self._cmd.get_group(robot_name).remember_joint_values(name,
                                                              joint_values)

    def go_to_named_pose(self, robot_name, named_pose, speed=1.0, accel=1.0):
        group = self._cmd.get_group(robot_name)
        try:
            group.set_named_target(named_pose)
        except moveit_commander.exception.MoveItCommanderException as e:
            self.get_logger().error('AistBaseRoutines.go_to_named_pose(): %s'
                                    % e)
            return False
        return self._go(group, speed, accel)

    def go_to_joint_value_target(self, robot_name, joint_values,
                                 speed=1.0, accel=1.0):
        group = self._cmd.get_group(robot_name)
        group.set_joint_value_target(joint_values)
        return self._go(group, speed, accel)

    def clip_wrist_joint_value(self, robot_name, speed=1.0, accel=1.0):
        joint_values = self.get_current_joint_values(robot_name)
        if joint_values[-1] >= -pi:
            if joint_values[-1] <= pi:
                return True
            joint_values[-1] -= 2*pi
        else:
            joint_values[-1] += 2*pi
        return self.go_to_joint_value_target(robot_name, joint_values,
                                             speed, accel)

    def go_directly_to_joint_value_target(self, robot_name,
                                          joint_values, duration):
        joint_trajectory \
            = JointTrajectory(
                joint_names=self.joint_names,
                points=[
                    JointTrajectoryPoint(
                        positions=self.get_current_joint_values(robot_name),
                        time_from_start=Duration(seconds=0)),
                    JointTrajectoryPoint(positions=joint_values,
                                         time_from_start=duration)])
        return self.execute_path(robot_name,
                                 RobotTrajectory(
                                     joint_trajectory=joint_trajectory))

    def _go(self, group, speed=1.0, accel=1.0):
        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))
        group.set_max_acceleration_scaling_factor(np.clip(accel, 0.0, 1.0))
        success = group.go(wait=True)
        if not success:
            self.get_logger().error('Failed to go to target.')
        group.clear_pose_targets()
        return success

    #
    # Cartesian motion stuffs
    #
    def get_current_pose(self, robot_name: str)-> PoseStamped:
        """ Get current pose of the specified robot.

        Args:
          robot_name: Robot name.

        Returns:
          PoseStamped: Current pose of the robot.
        """
        group    = self._cmd.get_group(robot_name)
        eef_link = self.gripper(robot_name).tip_link
        if eef_link in self._cmd.get_link_names():
            return group.get_current_pose()

        default_eef_link = self.default_gripper(robot_name).tip_link
        pose = group.get_current_pose(default_eef_link)
        tfm = self.lookup_transform(default_eef_link, eef_link, Time(),
                                    Duration(seconds=10)).transform
        T = tfs.translation_matrix((pose.pose.position.x,
                                    pose.pose.position.y,
                                    pose.pose.position.z)) \
          @ tfs.quaternion_matrix((pose.pose.orientation.x,
                                   pose.pose.orientation.y,
                                   pose.pose.orientation.z,
                                   pose.pose.orientation.w)) \
          @ tfs.translation_matrix((tfm.translation.x,
                                    tfm.translation.y,
                                    tfm.translation.z)) \
          @ tfs.quaternion_matrix((tfm.rotation.x, tfm.rotation.y,
                                   tfm.rotation.z, tfm.rotation.w))
        t = tfs.translation_from_matrix(T)
        q = tfs.quaternion_from_matrix(T)
        pose.pose = Pose(position=Point(x=t[0], y=t[1], z=t[2]),
                         orientation=Quaternion(x=q[0], y=q[1],
                                                z=q[2], w=q[3]))
        return pose

    def move_relative(self, robot_name: str, offset,
                      speed: float=1.0, accel: float=1.0,
                      end_effector_link: str='')-> bool:
        """ Move the end-effector from current pose by given offset values.
        Offset is specified by a tuple with three, six of seven float values.

        Args:
          robot_name: Robot name.
          offset:
          speed: Upper-limit ratio relative to the maximum speed.
          accel: Upper-limit ratio relative to the maximum acceleration.
          end_effector_link: Link name of the end-effector. If empty,
            use tip_link of the gripper currently attached to the robot.

        Returns:
          bool: `True` iff success.
        """
        return self.go_to_pose_goal(robot_name,
                                    self.get_current_pose(robot_name),
                                    offset, speed, accel, end_effector_link)

    def go_to_frame(self, robot_name, target_frame, offset=(),
                    speed=1.0, accel=1.0, end_effector_link=''):
        return self.go_to_pose_goal(robot_name,
                                    PoseStamped(
                                        header=Header(frame_id=target_frame),
                                        pose=Pose(
                                            position=Point(
                                                x=0.0, y=0.0, z=0.0),
                                            orientation=Quaternion(
                                                x=0.0, y=0.0, z=0.0, w=1.0))),
                                    offset, speed, accel, end_effector_link)

    def go_to_pose_goal(self, robot_name, target_pose, offset=(),
                        speed=1.0, accel=1.0, end_effector_link=''):
        return self.go_along_poses(robot_name,
                                   PoseArray(header=target_pose.header,
                                             poses=[target_pose.pose]),
                                   offset, speed, accel, end_effector_link)

    def go_along_poses(self, robot_name, poses, offset=(),
                       speed=1.0, accel=1.0, end_effector_link=''):
        path = self.create_path(robot_name, poses, offset,
                                speed, accel, end_effector_link)
        if path is None:
            return False
        # group = self._cmd.get_group(robot_name)
        # return self.execute_path(robot_name,
        #                          group.retime_trajectory(
        #                              self._cmd.get_current_state(), path,
        #                              velocity_scaling_factor=speed,
        #                              acceleration_scaling_factor=accel))
        return self.execute_path(robot_name, path)

    def execute_path(self, robot_name, path):
        success = self._cmd.get_group(robot_name).execute(path, wait=True)
        if not success:
            self.get_logger().error('Failed to execute path.')
        self.stop(robot_name)
        return success

    def create_path(self, robot_name, poses, offset=(),
                    speed=1.0, accel=1.0, end_effector_link=''):
        if end_effector_link == '':
            end_effector_link = self.gripper(robot_name).tip_link
        group = self._cmd.get_group(robot_name)
        group.set_end_effector_link(end_effector_link)

        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))
        group.set_max_acceleration_scaling_factor(np.clip(accel, 0.0, 1.0))
        transformed_poses = self.transform_poses_to_target_frame(poses, offset)

        if not self.move_lin:
            group.set_start_state_to_current_state()
            group.set_pose_target(transformed_poses.poses[-1],
                                  end_effector_link)
            success, path, planning_time, error_code = group.plan()
            if not success:
                self.get_logger().error('Failed to compute non-linear path: planning_time=%f, error=%s@%s[%d]]'
                                        % (planning_time, error_code.message,
                                           error_code.source, error_code.val))
                return None
            return path[0]

        try:
            path, fraction = group.compute_cartesian_path(
                                 transformed_poses.poses, self._eef_step, 0.0)
        except Exception as e:
            self.get_logger().error(e)
            return None

        if fraction < 0.995:
            self.get_logger().error('Computed only %3.1f%% of cartesian path.'
                                    % (100.0*fraction))
            return None
        self.get_logger().info('Computed %3.1f%% of cartesian path.'
                               % (100.0*fraction))
        return path

    def create_timed_path(self, robot_name, poses, times_from_start):
        robot_state = self._cmd.get_current_state()
        robot_state.joint_state.header.stamp = poses.header.stamp
        req = PositionIKRequest(group_name=robot_name,
                                robot_state=robot_state)
        joint_trajectory = JointTrajectory(req.robot_state.joint_state.header,
                                           req.robot_state.joint_state.name,
                                           [])
        transformed_poses = self.transform_poses_to_target_frame(poses)
        for pose, time_from_start in zip(transformed_poses.poses,
                                         times_from_start):
            req.pose_stamped = PoseStamped(transformed_poses.header, pose)
            res = self._compute_ik(req)
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error('Failed to solve IK[%d]'
                                        % res.error_code.val)
                return None
            joint_state = res.solution.joint_state
            point = JointTrajectoryPoint(positions=joint_state.position,
                                         velocities=joint_state.velocity,
                                         effort=joint_state.effort)
            point.time_from_start = time_from_start
            joint_trajectory.points.append(point)
        return RobotTrajectory(joint_trajectory=joint_trajectory)

    def stop(self, robot_name):
        group = self._cmd.get_group(robot_name)
        group.stop()
        group.clear_pose_targets()

    #
    # Controller stuffs
    #
    def list_controllers(self, robot_name: str):
        """ List of controllers associated with the robot.

        Args:
          robot_name: Robot name.

        Returns:
          List of controllers.
        """
        return list(filter(lambda x: x.type in BaseRoutines.ControllerTypes,
                           self._list_controllers_srvs[robot_name].call(
                               ListControllers.Request()).controller))

    def active_controller(self, robot_name: str):
        """ Current active controller associated with the robot.

        Args:
          robot_name: Robot name.

        Returns:
          Active contoller, if exists. `None`, if no active controllers.
        """
        return next(filter(lambda c: c.state == 'active',
                           self.list_controllers(robot_name)), None)

    def switch_controller(self, robot_name: str, controller_name: str)-> bool:
        """ Current active controller associated with the robot.

        Args:
          robot_name:      Robot name.
          controller_name: Name of the controller to be switched to.

        Returns:
          bool: `True`, if success. `False`, if failure.
        """
        controller = next(filter(lambda c: c.name == controller_name,
                                 self.list_controllers(robot_name)), None)
        if controller is None:
            self.get_logger().error('Specified controller[%s] not found'
                                    % controller_name)
            return False

        if controller.state == 'active':
            self.get_logger().warn('Already active[%s]' % controller_name)
            return True
        elif controller.state in ('unconfigured', 'inactive'):
            # if controller.type == 'cartesian_force_controller/CartesianForceController' or \
                #    controller.type == 'cartesian_compliance_controller/CartesianComplianceController':
            #     self.ftsensor_reset_bias()
            active_controller = self.active_controller(robot_name)
            req = SwitchController.Request()
            req.activate_controllers   = [controller_name]
            req.deactivate_controllers = [active_controller.name] \
                                         if active_controller else []
            req.strictness             = SwitchController.Request.STRICT
            req.activate_asap          = True
            req.timeout                = Duration(seconds=1).to_msg()
            res = self._switch_controller_srvs[robot_name].call(req)
            time.sleep(0.5)
            if res.ok:
                self.get_logger().info('Succesfully switched to controller[%s]'
                                       % controller_name)
            else:
                self.get_logger().error('Failed to switch to controller[%s]'
                                        % controller_name)
            return res.ok
        else:
            self.get_logger().warn("Controller state is '%', returning True."
                                   % controller.state)
            return True

    def toggle_motion_control_handle(self, robot_name: str)-> bool:
        """ Activate/deactivate motion control handle.

        Args:
          robot_name: Robot name.

        Returns:
          bool: `True`, if successfully switched. `False`, if failure.
        """
        motion_control_handle = next(filter(lambda c:
                                            c.name == 'motion_control_handle',
                                            self.list_controllers(robot_name)),
                                     None)
        if motion_control_handle is None:
            return False

        req = SwitchController.Request()
        if motion_control_handle.state == 'active':
            req.activate_controllers   = []
            req.deactivate_controllers = [motion_control_handle.name]
            message = 'deactivated ' + motion_control_handle.name
            self.switch_controller(robot_name, 'joint_trajectory_controller')
        else:
            self.switch_controller(robot_name,
                                   'cartesian_compliance_controller')
            req.activate_controllers   = [motion_control_handle.name]
            req.deactivate_controllers = []
            message = 'activated ' + motion_control_handle.name
        req.strictness = SwitchControllerRequest.STRICT
        req.start_asap = True
        req.timeout    = Duration(seconds=1).to_msg()
        res = self._switch_controller.call(req)
        time.sleep(0.5)
        if res.ok:
            self.get_logger().info('Succesfully %s' % message)
        else:
            self.get_logger().error('Failed to %s' % message)
        return res.ok

    #
    # Gripper stuffs
    #
    def _create_gripper(self, name, type_name, props):
        gripper_client_class = globals().get(type_name)
        if gripper_client_class is None:
            raise RuntimeError('unknown type[%s] of the gripper[%s]'
                               % (type_name, name))
        return gripper_client_class(self, name, **props)

    def default_gripper_name(self, robot_name):
        return self._default_gripper_names[robot_name]

    def default_gripper(self, robot_name):
        return self._grippers[self.default_gripper_name(robot_name)]

    def set_gripper(self, robot_name, gripper_name):
        gripper = self._grippers.get(gripper_name)
        if gripper is None:
            raise RuntimeError('unknown gripper[%s]' % gripper_name)
        self._active_grippers[robot_name] = gripper

    def gripper(self, robot_name):
        return self._active_grippers[robot_name]

    def set_gripper_parameters(self, robot_name, parameters):
        self.gripper(robot_name).set_parameters(parameters)

    def gripper_parameters(self, robot_name):
        return self.gripper(robot_name).parameters

    def pregrasp(self, robot_name):
        return self.gripper(robot_name).pregrasp()

    def grasp(self, robot_name):
        return self.gripper(robot_name).grasp()

    def postgrasp(self, robot_name):
        return self.gripper(robot_name).postgrasp()

    def release(self, robot_name):
        return self.gripper(robot_name).release()

    def set_gripper_position(self, robot_name, position):
        return self.gripper(robot_name).move(position)

    def set_gripper_velocity(self, robot_name, velocity):
        return self.gripper(robot_name).set_velocity(velocity)

    def tighten(self, robot_name):
        self.gripper(robot_name).tighten()

    def loosen(self, robot_name):
        self.gripper(robot_name).loosen()

    def gripper_cancel(self, robot_name):
        self.gripper(robot_name).cancel_goal()

    #
    # Camera stuffs
    #
    def camera(self, camera_name):
        return self._cameras[camera_name]

    def continuous_shot(self, camera_name, enable):
        return self.camera(camera_name).continuous_shot(enable)

    def trigger_frame(self, camera_name):
        return self.camera(camera_name).trigger_frame()

    #
    # Pick and place action stuffs
    #
    def pick(self, robot_name, object_id, target_pose, *, timeout_sec=None):
        picking_parameters = self.settings.get('picking_parameters', {})
        params             = picking_parameters.get(object_id)
        if params is None:
            params = picking_parameters[
                         self.com.get_object_info(object_id).object_type]
        self.set_gripper_parameters(robot_name,
                                    params.get('gripper_parameters', {}))
        return self._pick_or_place.send_goal(robot_name, True, target_pose,
                                             params['pick_offset'],
                                             params['approach_offset'],
                                             params['departure_offset'],
                                             params['speed_fast'],
                                             params['speed_slow'],
                                             end_effector_link='',
                                             timeout_sec=timeout_sec)

    def place(self, robot_name, object_id, target_pose,
              *, eef_link='', timeout_sec=None):
        picking_parameters = self.settings.get('picking_parameters', {})
        picking_params     = picking_parameters.get(object_id)
        if picking_params is None:
            picking_params = picking_parameters[
                                 self.com.get_object_info(object_id).object_type]
        placing_parameters = self.settings.get('placing_parameters', {})
        placing_params     = placing_parameters.get(
                                 target_pose.header.frame_id, picking_params)
        if not placing_params.get('place_offset'):
            placing_params = placing_parameters['default']
        self.set_gripper_parameters(robot_name,
                                    picking_params.get('gripper_parameters',
                                                       {}))
        return self._pick_or_place.send_goal(robot_name, False, target_pose,
                                             placing_params['place_offset'],
                                             placing_params['approach_offset'],
                                             placing_params['departure_offset'],
                                             picking_params['speed_fast'],
                                             picking_params['speed_slow'],
                                             end_effector_link=eef_link,
                                             timeout_sec=timeout_sec)

    def pick_at_frame(self, robot_name, object_id, target_frame,
                      *, offset=(), timeout_sec=None):
        return self.pick(robot_name, object_id,
                         self.pose_from_xyzrpy(offset, target_frame),
                         timeout_sec=timeout_sec)

    def pick_or_place_wait(self, robot_name,
                           *, target_stage=None, timeout_sec=None):
        return self._pick_or_place.wait(robot_name, target_stage=target_stage,
                                        timeout_sec=timeout_sec)

    def pick_or_place_cancel_goal(self, robot_name):
        self._pick_or_place.cancel_goal(robot_name)

    def place_at_frame(self, robot_name, object_id, target_frame,
                       *, offset=(), eef_link='', timeout_sec=None):
        return self.place(robot_name, object_id,
                          self.pose_from_xyzrpy(offset, target_frame),
                          eef_link=eef_link, timeout_sec=timeout_sec)

    def pick_tool(self, robot_name, tool_name, *, timeout_sec=None):
        return self._pick_or_place_tool.send_goal(robot_name, tool_name,
                                                  timeout_sec=timeout_sec)

    def place_tool(self, robot_name, *, timeout_sec=None):
        return self._pick_or_place_tool.send_goal(robot_name, '',
                                                  timeout_sec=timeout_sec)

    #
    # Utility functions
    #
    def initialize_collision_objects(self):
        self.com.remove_object()
        for object_type, config in self.settings.get('initial_object_config',
                                                     {}).items():
            self.com.create_object(object_type,
                                   self.pose_from_xyzrpy(
                                       config.get('offset', ()),
                                       config['parent_link']),
                                   config.get('subframe', 'base_link'))
            self.com.allow_collision(object_type, config['parent_link'])
            if config.get('attach', False):
                self.com.attach_object(object_type, config['parent_link'])

    def print_object_info(self, info):
        print('    object_id:   %s\n    type:        %s\n    parent_link: %s\n    attach_link: %s\n    touch_links: %s\n    acm_allowed: %s\n    pose:        %s@%s'
              % (info.object_id, info.object_type, info.parent_link,
                 info.attach_link, info.touch_links, info.acm_allowed,
                 self.format_pose(info.pose, info.pose.header.frame_id),
                 info.pose.header.frame_id))

    def candidate_eef_links(self, robot_name):
        frames_dict = yaml.safe_load(self.tf2_buffer.all_frames_as_yaml())
        frame_ids = []

        def _candidate_eef_links(frame_id):
            frame_ids.append(frame_id)
            for child_frame_id, child_frame_props in frames_dict.items():
                if child_frame_props['parent'] == frame_id:
                    _candidate_eef_links(child_frame_id)

        gripper = self.gripper(robot_name)
        gripper_link = gripper.base_link if '/' in gripper.tip_link else \
                       gripper.tip_link
        _candidate_eef_links(gripper_link)
        return frame_ids

    def lookup_transform(self, target_frame, source_frame,
                         time=Time(), timeout=Duration()):
        try:
            return self._tf2_buffer.lookup_transform(target_frame,
                                                     source_frame,
                                                     time, timeout)
        except Exception as e:
            self.get_logger().error('BaseRoutines.lookup_transform(): %s' % e)
            raise e

    def transform_point_to_target_frame(self, point, target_frame=''):
        points = self.transform_points_to_target_frame(point.header,
                                                       [point.point],
                                                       target_frame)
        return PointStamped(header=Header(frame_id=target_frame),
                            point=points[0])

    def transform_points_to_target_frame(self, header, points,
                                         target_frame=''):
        if target_frame == '':
            target_frame = self._reference_frame

        T = transform_matrix(self.lookup_transform(
                                 target_frame, header.frame_id,
                                 header.stamp, Duration(seconds=10)).transform)
        transformed_points = []
        for point in points:
            p = T @ (point.x, point.y, point.z, 1.0)
            transformed_points.append(Point(x=p[0], y=p[1], z=p[2]))
        return transformed_points

    def transform_pose_to_target_frame(self, pose, offset=(), target_frame=''):
        poses = self.transform_poses_to_target_frame(
                    PoseArray(header=pose.header, poses=[pose.pose]),
                    offset, target_frame)
        return PoseStamped(header=poses.header, pose=poses.poses[0])

    def transform_poses_to_target_frame(self, poses,
                                        offset=(), target_frame=''):
        if target_frame == '':
            target_frame = self.reference_frame

        T = transform_matrix(self.lookup_transform(
                                 target_frame, poses.header.frame_id,
                                 poses.header.stamp,
                                 Duration(seconds=10)).transform)
        S = tfs.translation_matrix(self._position_from_offset(offset[0:3])) \
          @ tfs.quaternion_matrix(self._orientation_from_offset(offset[3:]))
        return PoseArray(header=Header(frame_id=target_frame,
                                       stamp=poses.header.stamp),
                         poses=[pose_from_matrix(T @ pose_matrix(pose) @ S)
                                for pose in poses.poses])

    def add_offset_to_pose(self, pose, offset):
        T = pose_matrix(pose.pose) \
          @ tfs.translation_matrix(self._position_from_offset(offset[0:3])) \
          @ tfs.quaternion_matrix(self._orientation_from_offset(offset[3:]))
        return PoseStamped(header=pose.header, pose=pose_from_matrix(T))

    def correct_orientation(self, pose):
        poses = self.correct_orientations(PoseArray(header=pose.header,
                                                    poses=[pose.pose]))
        return PoseStamped(header=poses.header, pose=poses.poses[0])

    def correct_orientations(self, poses):
        up = self._tf2_buffer.transformVector3(
                 poses.header.frame_id,
                 Vector3Stamped(Header(stamp=poses.header.stamp,
                                       frame_id=self.reference_frame),
                                Vector3(x=0, y=0, z=1)))
        return PoseArray(header=poses.header,
                         poses=[Pose(position=pose.position,
                                     orientation=self._correct_orientation(
                                         pose.orientation, up.vector))
                                for pose in poses.poses])

    def pose_from_xyzrpy(self, xyzrpy, frame_id=''):
        if frame_id == '':
            frame_id = self.reference_frame
        t = self._position_from_offset(xyzrpy[0:3])
        q = self._orientation_from_offset(xyzrpy[3:])
        return PoseStamped(header=Header(frame_id=frame_id),
                           pose=Pose(position=Point(x=t[0], y=t[1], z=t[2]),
                                     orientation=Quaternion(x=q[0], y=q[1],
                                                            z=q[2], w=q[3])))

    def xyzrpy_from_pose(self, pose, target_frame=''):
        transformed_pose = self.transform_pose_to_target_frame(
                               pose, target_frame=target_frame).pose
        rpy = tfs.euler_from_quaternion((transformed_pose.orientation.x,
                                         transformed_pose.orientation.y,
                                         transformed_pose.orientation.z,
                                         transformed_pose.orientation.w))
        return [transformed_pose.position.x,
                transformed_pose.position.y,
                transformed_pose.position.z,
                degrees(rpy[0]), degrees(rpy[1]), degrees(rpy[2])]

    def format_pose(self, target_pose, target_frame=''):
        return '[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'.format(
            *self.xyzrpy_from_pose(target_pose, target_frame))

    #
    # Private functions
    #
    def _position_from_offset(self, offset):
        return np.array((0.0, 0.0, 0.0) if len(offset) < 3 else offset[0:3])

    def _orientation_from_offset(self, offset):
        return np.array((0.0, 0.0, 0.0, 1.0)) if len(offset) < 3 else \
               tfs.quaternion_from_euler(
                   *np.radians(offset[0:3])) if len(offset) == 3 else \
               np.array(offset[0:4])

    def _correct_orientation(self, orientation, up):
        q     = (orientation.x, orientation.y, orientation.z, orientation.w)
        r     = tfs.quaternion_matrix(q)[0:3, 2]  # current up vector
        n     = (up.x, up.y, up.z)                # desired up vector
        a     = np.cross(r, n)                    # rotation axis
        dq    = np.empty(4)
        dq[3] = sqrt(0.5 + 0.5*np.dot(r, n))
        if abs(dq[3]) < 1e-7:                     # n == -r ?
            dq[0:3] = (sqrt(0.5), sqrt(0.5), 0.0) # swap x and y, then flip z
        else:
            dq[0:3] = (0.5/dq[3])*a
        q_corrected = tfs.quaternion_multiply(q, dq)
        return Quaternion(x=q_corrected.x, y=q_corrected.y,
                          z=q_corrected.z, w=q_corrected.w)

    @staticmethod
    def _get_object_id(link_name):
        tokens = link_name.rsplit('/', 1)
        return tokens[0] if len(tokens) == 2 else ''
