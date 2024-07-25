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
import sys
import rospy
import numpy as np
import moveit_commander

from math                              import degrees, sqrt
from tf                                import (TransformListener,
                                               transformations as tfs)
from std_msgs.msg                      import Header
from geometry_msgs.msg                 import (PoseStamped, Pose, Point,
                                               Quaternion, PoseArray,
                                               Vector3, Vector3Stamped)
from moveit_msgs.msg                   import (RobotTrajectory,
                                               PositionIKRequest,
                                               MoveItErrorCodes)
from moveit_msgs.srv                   import GetPositionIK
from trajectory_msgs.msg               import (JointTrajectoryPoint,
                                               JointTrajectory)
from aist_routines.GripperClient       import GripperClient, VoidGripper
from aist_routines.CameraClient        import CameraClient
from aist_routines.FasteningToolClient import FasteningToolClient
from aist_routines.MarkerPublisher     import MarkerPublisher
from aist_routines.PickOrPlaceAction   import PickOrPlace
from aist_utility.compat               import *

######################################################################
#  global functions                                                  #
######################################################################
def paramtuples(d):
    fields = set()
    for params in d.values():
        for field in params.keys():
            fields.add(field)
    ParamTuple = collections.namedtuple('ParamTuple', ' '.join(fields))

    params = {}
    for key, param in d.items():
        params[key] = ParamTuple(**param)
    return params

######################################################################
#  class AISTBaseRoutines                                            #
######################################################################
class AISTBaseRoutines(object):
    def __init__(self, reference_frame='', eef_step=None):
        super().__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self._listener = TransformListener()
        rospy.sleep(1.0)        # Necessary for listner spinning up

        # MoveIt planning parameters
        self._eef_step = eef_step if eef_step is not None else \
                         rospy.get_param('~moveit_eef_step', 0.0005)
        self._reference_frame = reference_frame if reference_frame != '' else \
                                rospy.get_param('~moveit_pose_reference_frame',
                                                'workspace_center')

        # MoveIt RobotCommander and MoveGroup
        self._cmd = moveit_commander.RobotCommander('robot_description')
        for group_name in self._cmd.get_group_names():
            group = self._cmd.get_group(group_name)
            group.set_pose_reference_frame(self.reference_frame)

        rospy.loginfo('planning_frame: %s, reference_frame: %s, eef_step: %f',
                      self.planning_frame, self.reference_frame, self.eef_step)

        # MoveIt GetPositionIK service client
        self._compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

        # Grippers
        self._grippers = {}
        for gripper_name, props in rospy.get_param('~grippers', {}).items():
            self._grippers[gripper_name] = GripperClient.create(gripper_name,
                                                                props)

        # Robots
        self._default_gripper_names = {}
        self._active_grippers  = {}
        for robot_name, props in rospy.get_param('~robots', {}).items():
            self._default_gripper_names[robot_name] = props['default_gripper']
            self.set_gripper(robot_name, self.default_gripper_name(robot_name))

        # Cameras
        self._cameras = {}
        for camera_name, props in rospy.get_param('~cameras', {}).items():
            self._cameras[camera_name] = CameraClient.create(camera_name,
                                                             props)

        # Fastening tools
        self._fastening_tools = {}
        for tool_name, props in rospy.get_param('~fastening_tools', {}).items():
            self._fastening_tools[tool_name] \
                = FasteningToolClient.create(tool_name, props)

        # Search graspabilities
        if rospy.has_param('~graspability_parameters'):
            from aist_graspability import GraspabilityClient
            self._graspability_params \
                = rospy.get_param('~graspability_parameters')
            self._graspabilityClient = GraspabilityClient()

        # Pick and place action
        if rospy.has_param('~picking_parameters'):
            self._picking_params = rospy.get_param('~picking_parameters')
            self._pick_or_place  = PickOrPlace(self)
        else:
            self._pick_or_place = None

        # Marker publisher
        self._markerPublisher = MarkerPublisher()

        rospy.loginfo('AISTBaseRoutines initialized.')

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        if self._pick_or_place:
            self._pick_or_place.shutdown()
        rospy.signal_shutdown('AISTBaseRoutines() completed.')
        return False  # Do not forward exceptions

    @property
    def listener(self):
        return self._listener

    @property
    def planning_frame(self):
        return self._cmd.get_planning_frame()

    @property
    def reference_frame(self):
        return self._reference_frame

    @property
    def eef_step(self):
        return self._eef_step

    # Interactive stuffs
    def print_help_messages(self):
        print('=== General commands ===')
        print('  quit:        quit this program')
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
        print('  speed:       set speed')
        print('  stop:        stop arm immediately')
        print('  jvalues:     get current joint values')
        print('=== Gripper commands ===')
        print('  gripper:     assign gripper to current robot')
        print('  pregrasp:    pregrasp with the current gripper')
        print('  grasp:       grasp with the current gripper')
        print('  postgrasp:   postgrasp with the current gripper')
        print('  release:     release with the current gripper')
        print('=== Fastening tool commands ===')
        print('  gripper:     assign gripper to current robot')
        print('  tighten:     tighten screw')
        print('  loosen:      loosen screw')
        print('  fcancel:     cancel tighten/loosen action')

    def interactive(self, key, robot_name, axis, speed=1.0):
        def _is_num(s):
            try:
                float(s)
            except ValueError:
                return False
            else:
                return True

        def _get_offset():
            offset = []
            for s in raw_input('  offset? ').split():
                if _is_num(s):
                    offset.append(float(s))
                else:
                    return None
            return offset

        if key == 'quit':
            self.go_to_named_pose(robot_name, 'home')  # Reset pose
            rospy.signal_shutdown('manual shutdown')
        elif key == 'robot':
            robot_name = raw_input('  robot name? ')
        elif key == '?' or key == 'help':
            self.print_help_messages()
            print('')

        # Arm stuffs
        elif key == 'X':
            axis = 'X'
        elif key == 'Y':
            axis = 'Y'
        elif key == 'Z':
            axis = 'Z'
        elif key == 'R':
            axis = 'Roll'
        elif key == 'P':
            axis = 'Pitch'
        elif key == 'W':
            axis = 'Yaw'
        elif key == '+':
            offset = [0, 0, 0, 0, 0, 0]
            if axis == 'X':
                offset[0] = 0.01
            elif axis == 'Y':
                offset[1] = 0.01
            elif axis == 'Z':
                offset[2] = 0.01
            elif axis == 'Roll':
                offset[3] = 10.0
            elif axis == 'Pitch':
                offset[4] = 10.0
            else:
                offset[5] = 10.0
            self.move_relative(robot_name, offset, speed)
        elif key == '-':
            offset = [0, 0, 0, 0, 0, 0]
            if axis == 'X':
                offset[0] = -0.01
            elif axis == 'Y':
                offset[1] = -0.01
            elif axis == 'Z':
                offset[2] = -0.01
            elif axis == 'Roll':
                offset[3] = -10.0
            elif axis == 'Pitch':
                offset[4] = -10.0
            else:
                offset[5] = -10.0
            self.move_relative(robot_name, offset, speed)
        elif _is_num(key):
            xyzrpy = self.xyzrpy_from_pose(self.get_current_pose(robot_name))
            print(xyzrpy)
            if axis == 'X':
                xyzrpy[0] = float(key)
            elif axis == 'Y':
                xyzrpy[1] = float(key)
            elif axis == 'Z':
                xyzrpy[2] = float(key)
            elif axis == 'Roll':
                xyzrpy[3] = float(key)
            elif axis == 'Pitch':
                xyzrpy[4] = float(key)
            else:
                xyzrpy[5] = float(key)
            self.go_to_pose_goal(robot_name,
                                 self.pose_from_xyzrpy(xyzrpy), speed=speed)
        elif key == 'home':
            self.go_to_named_pose(robot_name, 'home')
        elif key == 'back':
            self.go_to_named_pose(robot_name, 'back')
        elif key == 'named':
            pose_name = raw_input('  pose name? ')
            try:
                self.go_to_named_pose(robot_name, pose_name)
            except rospy.ROSException as e:
                rospy.logerr('Unknown pose: %s' % e)
        elif key == 'frame':
            frame  = raw_input('  frame? ')
            offset = _get_offset()
            try:
                self.go_to_frame(robot_name, frame, offset)
            except Exception as e:
                rospy.logerr('Unknown frame: %s', frame)
        elif key == 'speed':
            speed = float(raw_input('  speed value? '))
        elif key == 'stop':
            self.stop(robot_name)
        elif key == 'jvalues':
            print(self.get_current_joint_values(robot_name))

        # Gripper stuffs
        elif key == 'gripper':
            gripper_name = raw_input('  gripper name? ')
            try:
                self.set_gripper(robot_name, gripper_name)
            except KeyError as e:
                rospy.logerr('Unknown gripper: %s' % e)
        elif key == 'pregrasp':
            self.pregrasp(robot_name)
        elif key == 'grasp':
            self.grasp(robot_name)
        elif key == 'postgrasp':
            self.postgrasp(robot_name)
        elif key == 'release':
            self.release(robot_name)

        # Fastening tool stuffs
        elif key == 'tighten':
            tool_name = raw_input('  tool name? ')
            self.tighten(tool_name, rospy.Duration(-1))
        elif key == 'loosen':
            tool_name = raw_input('  tool name? ')
            self.loosen(tool_name, rospy.Duration(-1))
        elif key == 'fcancel':
            tool_name = raw_input('  tool name? ')
            self.cancel_fastening(tool_name)

        else:
            print('  unknown command! [%s]' % key)
        return robot_name, axis, speed

    # Joint motion stuffs
    def get_joint_names(self, robot_name):
        return self._cmd.get_group(robot_name).get_active_joints()

    def get_current_joint_values(self, robot_name):
        return self._cmd.get_group(robot_name).get_current_joint_values()

    def get_named_joint_values(self, robot_name, named_pose):
        target_values = self._cmd.get_group(robot_name)\
                                 .get_named_target_values(named_pose)
        return [target_values[joint_name]
                for joint_name in self.get_joint_names(robot_name)]

    def remember_joint_values(self, robot_name, name, joint_values):
        self._cmd.get_group(robot_name).remember_joint_values(name,
                                                              joint_values)

    def go_to_named_pose(self, robot_name, named_pose, speed=1.0, accel=1.0):
        group = self._cmd.get_group(robot_name)
        try:
            group.set_named_target(named_pose)
        except moveit_commander.exception.MoveItCommanderException as e:
            rospy.logerr('AistBaseRoutines.go_to_named_pose(): %s', e)
            return False
        return self._go(group, speed, accel)

    def go_to_joint_value_target(self, robot_name, joint_values,
                                 speed=1.0, accel=1.0):
        group = self._cmd.get_group(robot_name)
        group.set_joint_value_target(joint_values)
        return self._go(group, speed, accel)

    def go_directly_to_joint_value_target(self, robot_name,
                                          joint_values, duration):
        joint_trajectory \
            = JointTrajectory(joint_names=self.joint_names,
                              points=[
                                  JointTrajectoryPoint(
                                      positions=self.get_current_joint_values(robot_name),
                                      time_from_start=rospy.Duration(0)),
                                  JointTrajectoryPoint(
                                      positions=joint_values,
                                      time_from_start=duration)])
        return self.execute_path(robot_name,
                                 RobotTrajectory(
                                     joint_trajectory=joint_trajectory))

    def _go(self, group, speed=1.0, accel=1.0):
        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))
        group.set_max_acceleration_scaling_factor(np.clip(accel, 0.0, 1.0))
        success = group.go(wait=True)
        if not success:
            rospy.logerr('Failed to go to target.')
        group.clear_pose_targets()
        return success

    # Cartesian motion stuffs
    def get_current_pose(self, robot_name):
        return self._cmd.get_group(robot_name).get_current_pose()

    def move_relative(self, robot_name, offset,
                      speed=1.0, accel=1.0, end_effector_link=''):
        return self.go_to_pose_goal(robot_name,
                                    self.get_current_pose(robot_name),
                                    offset, speed, accel, end_effector_link)

    def go_to_frame(self, robot_name, target_frame, offset=(),
                    speed=1.0, accel=1.0, end_effector_link=''):
        return self.go_to_pose_goal(robot_name,
                                    PoseStamped(Header(frame_id=target_frame),
                                                Pose(Point(0, 0, 0),
                                                     Quaternion(0, 0, 0, 1))),
                                    offset, speed, accel, end_effector_link)

    def go_to_pose_goal(self, robot_name, target_pose, offset=(),
                        speed=1.0, accel=1.0, end_effector_link=''):
        return self.go_along_poses(robot_name, PoseArray(target_pose.header,
                                                         [target_pose.pose]),
                                   offset, speed, accel, end_effector_link)

    def go_along_poses(self, robot_name, poses, offset=(),
                       speed=1.0, accel=1.0, end_effector_link=''):
        group = self._cmd.get_group(robot_name)
        path  = self.create_path(robot_name, poses, offset,
                                 speed, accel, end_effector_link)
        if path is None:
            return False, group.get_current_pose()

        return (self.execute_path(robot_name,
                                  group.retime_trajectory(
                                      self._cmd.get_current_state(), path,
                                          velocity_scaling_factor=speed,
                                          acceleration_scaling_factor=accel)),
                group.get_current_pose())

    def execute_path(self, robot_name, path):
        success = self._cmd.get_group(robot_name).execute(path, wait=True)
        if not success:
            rospy.logerr('Failed to execute path.')
        self.stop(robot_name)
        return success

    def create_path(self, robot_name, poses, offset=(),
                    speed=1.0, accel=1.0, end_effector_link=''):
        group = self._cmd.get_group(robot_name)

        if end_effector_link == '':
            end_effector_link = self.gripper(robot_name).tip_link
        group.set_end_effector_link(end_effector_link)

        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))
        group.set_max_acceleration_scaling_factor(np.clip(accel, 0.0, 1.0))
        transformed_poses = self.transform_poses_to_target_frame(poses, offset)

        try:
            path, fraction = group.compute_cartesian_path(
                                 transformed_poses.poses, self._eef_step, 0.0)
        except Exception as e:
            fraction = 0.0
            rospy.logerr(e)

        if fraction < 0.995:
            rospy.logerr('Computed only %3.1f%% of cartesian path.',
                         100*fraction)
            return None
        rospy.loginfo('Computed %3.1f%% of cartesian path.', 100*fraction)
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
                rospy.logerr('Failed to solve IK[%d]', res.error_code.val)
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

    # Gripper stuffs
    def default_gripper_name(self, robot_name):
        return self._default_gripper_names[robot_name]

    def set_gripper(self, robot_name, gripper_name):
        self._active_grippers[robot_name] = self._grippers[gripper_name]

    def gripper(self, robot_name):
        return self._active_grippers[robot_name]

    def set_gripper_parameters(self, robot_name, parameters):
        self.gripper(robot_name).parameters = parameters

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

    # Camera stuffs
    def camera(self, camera_name):
        return self._cameras[camera_name]

    def continuous_shot(self, camera_name, enable):
        return self.camera(camera_name).continuous_shot(enable)

    def trigger_frame(self, camera_name):
        return self.camera(camera_name).trigger_frame()

    # Fasteing tool stuffs
    def fastening_tool(self, tool_name):
        return self._fastening_tools[tool_name]

    def tighten(self, tool_name, timeout=rospy.Duration()):
        self.fastening_tool(tool_name).tighten(timeout)

    def loosen(self, tool_name, timeout=rospy.Duration()):
        self.fastening_tool(tool_name).loosen(timeout)

    def cancel_fastening(self, tool_name):
        self.fastening_tool(tool_name).cancel()

    # Marker stuffs
    def delete_all_markers(self):
        self._markerPublisher.delete_all()

    def add_marker(self, marker_type, pose, endpoint=None,
                   text='', lifetime=15):
        self._markerPublisher.add(marker_type, pose, endpoint, text, lifetime)

    def publish_marker(self):
        self._markerPublisher.publish()

    # Graspability stuffs
    def create_mask_image(self, camera_name, nmasks):
        self.camera(camera_name).trigger_frame()
        return self._graspabilityClient.create_mask_image(nmasks)

    def graspability_send_goal(self, robot_name, part_id, mask_id,
                               one_shot=True):
        params = self._graspability_params[part_id]
        self._graspabilityClient.set_parameters(params)

        # Send goal first to be ready for subscribing image,
        self._graspabilityClient.send_goal(mask_id,
                                           self.gripper(robot_name).type,
                                           None if one_shot else
                                           self._graspability_feedback_cb)

    def graspability_cancel_goal(self):
        self._graspabilityClient.cancel_goal()

    def graspability_wait_for_result(self, target_frame='', pose_filter=None,
                                     marker_lifetime=0):
        graspabilities = self._graspabilityClient.wait_for_result()

        print('*** graspability stamp: [{:0>10}.{:0>9}]'
              .format(graspabilities.poses.header.stamp.secs,
                      graspabilities.poses.header.stamp.nsecs))
        #  We have to transform the poses to reference frame before moving
        #  because graspability poses are represented w.r.t. camera frame
        #  which will change while moving in the case of "eye on hand".
        graspabilities.contact_points = self._transform_points_to_target_frame(
                                            graspabilities.poses.header,
                                            graspabilities.contact_points,
                                            target_frame)
        graspabilities.poses          = self.transform_poses_to_target_frame(
                                            graspabilities.poses, (),
                                            target_frame)
        if pose_filter is not None:
            poses          = []
            gscores        = []
            contact_points = []
            for pose, gscore, contact_point \
                in zip(graspabilities.poses.poses, graspabilities.gscores,
                       graspabilities.contact_points):
                filtered_pose = pose_filter(pose)
                if filtered_pose is not None:
                    poses.append(filtered_pose)
                    gscores.append(gscore)
                    contact_points.append(contact_point)
            graspabilities.poses.poses    = poses
            graspabilities.gscores        = gscores
            graspabilities.contact_points = contact_points
        self._graspability_publish_marker(graspabilities, marker_lifetime)
        return graspabilities

    def _graspability_publish_marker(self, graspabilities, marker_lifetime=0):
        self.delete_all_markers()
        for i, pose in enumerate(graspabilities.poses.poses):
            self.add_marker('graspability',
                            PoseStamped(graspabilities.poses.header, pose),
                            graspabilities.contact_points[i],
                            '{}[{:.3f}]'.format(i, graspabilities.gscores[i]),
                            lifetime=marker_lifetime)
        self.publish_marker()

    def _graspability_feedback_cb(self, feedback):
        self._graspability_publish_marker(feedback.graspabilities)

    # Pick and place action stuffs
    def pick(self, robot_name, target_pose, part_id,
             wait=True, done_cb=None, active_cb=None):
        params = self._picking_params[part_id]
        if 'gripper_name' in params:
            self.set_gripper(robot_name, params['gripper_name'])
        if 'gripper_parameters' in params:
            self.set_gripper_parameters(robot_name,
                                        params['gripper_parameters'])
        return self._pick_or_place.send_goal(robot_name, target_pose, True,
                                             params['grasp_offset'],
                                             params['approach_offset'],
                                             params['departure_offset'],
                                             params['speed_fast'],
                                             params['speed_slow'],
                                             wait, done_cb, active_cb)

    def place(self, robot_name, target_pose, part_id,
              wait=True, done_cb=None, active_cb=None):
        params = self._picking_params[part_id]
        if 'gripper_name' in params:
            self.set_gripper(robot_name, params['gripper_name'])
        if 'gripper_parameters' in params:
            self.set_gripper_parameters(robot_name,
                                        params['gripper_parameters'])
        return self._pick_or_place.send_goal(robot_name, target_pose, False,
                                             params['place_offset'],
                                             params['approach_offset'],
                                             params['departure_offset'],
                                             params['speed_fast'],
                                             params['speed_slow'],
                                             wait, done_cb, active_cb)

    def pick_at_frame(self, robot_name, target_frame, part_id,
                      offset=(), wait=True, done_cb=None, active_cb=None):
        return self.pick(robot_name,
                         PoseStamped(Header(frame_id=target_frame),
                                     self.pose_from_offset(offset)),
                         part_id, wait, done_cb, active_cb)

    def place_at_frame(self, robot_name, target_frame, part_id,
                       offset=(), wait=True, done_cb=None, active_cb=None):
        return self.place(robot_name,
                          PoseStamped(Header(frame_id=target_frame),
                                      self.pose_from_offset(offset)),
                          part_id, wait, done_cb, active_cb)

    def pick_or_place_wait_for_stage(self, stage, timeout=rospy.Duration()):
        return self._pick_or_place.wait_for_stage(stage, timeout)

    def pick_or_place_wait_for_result(self, timeout=rospy.Duration()):
        if self._pick_or_place.wait_for_result(timeout):
            return self._pick_or_place.get_result().result

    def pick_or_place_cancel_goal(self):
        self._pick_or_place.cancel_goal()

    # Utility functions
    def transform_pose_to_target_frame(self, pose, offset=(), target_frame=''):
        poses = self.transform_poses_to_target_frame(PoseArray(pose.header,
                                                               [pose.pose]),
                                                     offset, target_frame)
        return PoseStamped(poses.header, poses.poses[0])

    def transform_poses_to_target_frame(self, poses,
                                        offset=(), target_frame=''):
        if target_frame == '':
            target_frame = self.reference_frame

        try:
            self._listener.waitForTransform(target_frame,
                                            poses.header.frame_id,
                                            poses.header.stamp,
                                            rospy.Duration(10))
            mat44 = self._listener.asMatrix(target_frame, poses.header)
        except Exception as e:
            rospy.logerr('AISTBaseRoutines.transform_poses_to_target_frame(): {}'.format(e))
            raise e

        transformed_poses = PoseArray(Header(frame_id=target_frame,
                                             stamp=poses.header.stamp),
                                      [])
        for pose in poses.poses:
            T = tfs.concatenate_matrices(
                    mat44,
                    self._listener.fromTranslationRotation(
                        (pose.position.x, pose.position.y, pose.position.z),
                        (pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w)),
                    self._listener.fromTranslationRotation(
                        self._position_from_offset(offset[0:3]),
                        self._orientation_from_offset(offset[3:])))
            transformed_poses.poses.append(
                Pose(Point(*tuple(tfs.translation_from_matrix(T))),
                     Quaternion(*tuple(tfs.quaternion_from_matrix(T)))))
        return transformed_poses

    def correct_orientation(self, pose):
        poses = self.correct_orientations(PoseArray(pose.header, [pose.pose]))
        return PoseStamped(poses.header, poses.poses[0])

    def correct_orientations(self, poses):
        up = self._listener.transformVector3(
                 poses.header.frame_id,
                 Vector3Stamped(Header(stamp=poses.header.stamp,
                                       frame_id=self.reference_frame),
                                Vector3(0, 0, 1)))
        corrected_poses = PoseArray(poses.header, [])
        for pose in poses.poses:
            corrected_poses.poses.append(Pose(pose.position,
                                              self._correct_orientation(
                                                  pose.orientation, up.vector)))
        return corrected_poses

    def pose_from_xyzrpy(self, xyzrpy):
        return PoseStamped(Header(frame_id=self.reference_frame),
                           self.pose_from_offset(xyzrpy))

    def xyzrpy_from_pose(self, pose):
        transformed_pose = self.transform_pose_to_target_frame(pose).pose
        rpy = tfs.euler_from_quaternion((transformed_pose.orientation.x,
                                         transformed_pose.orientation.y,
                                         transformed_pose.orientation.z,
                                         transformed_pose.orientation.w))
        return [transformed_pose.position.x,
                transformed_pose.position.y,
                transformed_pose.position.z,
                degrees(rpy[0]), degrees(rpy[1]), degrees(rpy[2])]

    def format_pose(self, target_pose):
        return '[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'.format(
            *self.xyzrpy_from_pose(target_pose))

    def pose_from_offset(self, offset):
        return Pose(Point(*self._position_from_offset(offset[0:3])),
                    Quaternion(*self._orientation_from_offset(offset[3:])))

    # Private functions
    def _position_from_offset(self, offset):
        return np.array((0, 0, 0) if len(offset) < 3 else offset[0:3])

    def _orientation_from_offset(self, offset):
        return np.array((0, 0, 0, 1)) if len(offset) < 3 else \
               tfs.quaternion_from_euler(*np.radians(offset[0:3])) if len(offset) == 3 else \
               np.array(offset[0:4])

    def _transform_points_to_target_frame(self, header, points,
                                          target_frame=''):
        if target_frame == '':
            target_frame = self._reference_frame

        try:
            self._listener.waitForTransform(target_frame, header.frame_id,
                                            header.stamp, rospy.Duration(10))
            mat44 = self._listener.asMatrix(target_frame, header)
        except Exception as e:
            rospy.logerr('AISTBaseRoutines._transform_points_to_target_frame(): {}'.format(e))
            raise e

        return [ Point(*tuple(np.dot(mat44,
                                     np.array((p.x, p.y, p.z, 1.0)))[:3]))
                 for p in points ]

    def _correct_orientation(self, orientation, up):
        q     = (orientation.x, orientation.y, orientation.z, orientation.w)
        r     = tfs.quaternion_matrix(q)[0:3, 2]  # current up vector
        n     = (up.x, up.y, up.z)                # desired up vector
        a     = np.cross(r, n)                    # rotation axis
        dq    = np.empty(4)
        dq[3] = sqrt(0.5 + 0.5*np.dot(r, n))
        if abs(dq[3]) < 1e-7:                     # n == -r ?
            dq[0:3] = (sqrt(0.5), sqrt(0.5), 0)   # swap x and y, then flip z
        else:
            dq[0:3] = (0.5/dq[3])*a
        return Quaternion(*tfs.quaternion_multiply(q, dq))
