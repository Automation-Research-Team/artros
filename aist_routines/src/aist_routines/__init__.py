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
import copy
import collections
import rospy
import numpy as np

from math import pi, radians, degrees, cos, sin, sqrt
from tf import TransformListener, transformations as tfs
import moveit_commander
from moveit_commander.conversions import pose_to_list

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PoseArray

from aist_routines.GripperClient     import GripperClient, VoidGripper
from aist_routines.CameraClient      import CameraClient
from aist_routines.MarkerPublisher   import MarkerPublisher
from aist_routines.PickOrPlaceAction import PickOrPlace
from aist_routines.SweepAction       import Sweep
from aist_utility.compat             import *

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
        super(AISTBaseRoutines, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self._listener = TransformListener()
        rospy.sleep(1.0)        # Necessary for listner spinning up

        # MoveIt planning parameters
        self._reference_frame = reference_frame if reference_frame != '' else \
                                rospy.get_param('~moveit_pose_reference_frame',
                                                'workspace_center')
        self._eef_step        = eef_step if eef_step is not None else \
                                rospy.get_param('~moveit_eef_step', 0.0005)
        rospy.loginfo('reference_frame = {}, eef_step = {}'
                      .format(self._reference_frame, self._eef_step))

        # MoveIt RobotCommander
        self._cmd = moveit_commander.RobotCommander('robot_description')
        print('*** planning_frame=%s' % self._cmd.get_planning_frame())
        for group_name in self._cmd.get_group_names():
            group = self._cmd.get_group(group_name)
            print('*** [%s] pose_reference_frame=%s'
                  % (group_name, group.get_pose_reference_frame()))

        # Grippers
        d = rospy.get_param('~grippers', {})
        self._grippers = {'void_gripper': VoidGripper('void_gripper_base_link')}
        for gripper_name, props in d.items():
            self._grippers[gripper_name] = GripperClient.create(props['type'],
                                                                props['args'])

        # Robots
        d = rospy.get_param('~robots', {})
        self._active_grippers = {}
        for robot_name, props in d.items():
            if 'default_gripper' in props:
                self.set_gripper(robot_name, props['default_gripper'])
            else:
                self.set_gripper(robot_name, 'void_gripper')

        # Cameras
        d = rospy.get_param('~cameras', {})
        self._cameras = {}
        for camera_name, type_name in d.items():
            self._cameras[camera_name] = CameraClient.create(camera_name,
                                                             type_name)

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

        # Sweep action
        if rospy.has_param('~sweep_parameters'):
            self._sweep_params = rospy.get_param('~sweep_parameters')
            self._sweep        = Sweep(self)
        else:
            self._sweep = None

        # Marker publisher
        self._markerPublisher = MarkerPublisher()

        rospy.loginfo('AISTBaseRoutines initialized.')

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        if self._pick_or_place:
            self._pick_or_place.shutdown()
        if self._sweep:
            self._sweep.shutdown()
        rospy.signal_shutdown('AISTBaseRoutines() completed.')
        return False  # Do not forward exceptions

    @property
    def listener(self):
        return self._listener

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
        print('=== Gripper commands ===')
        print('  gripper:     select gripper')
        print('  pregrasp:    pregrasp with the current gripper')
        print('  grasp:       grasp with the current gripper')
        print('  postgrasp:   postgrasp with the current gripper')
        print('  release:     release with the current gripper')

    def interactive(self, key, robot_name, axis, speed=1.0):
        def _is_num(s):
            try:
                float(s)
            except ValueError:
                return False
            else:
                return True

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
                offset[3] = radians(10)
            elif axis == 'Pitch':
                offset[4] = radians(10)
            else:
                offset[5] = radians(10)
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
                offset[3] = radians(-10)
            elif axis == 'Pitch':
                offset[4] = radians(-10)
            else:
                offset[5] = radians(-10)
            self.move_relative(robot_name, offset, speed)
        elif _is_num(key):
            xyzrpy = self.xyzrpy_from_pose(self.get_current_pose(robot_name))
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
            self.go_to_pose_goal(robot_name, self.pose_from_xyzrpy(xyzrpy),
                                 speed)
        elif key == 'home':
            self.go_to_named_pose(robot_name, "home")
        elif key == 'back':
            self.go_to_named_pose(robot_name, "back")
        elif key == 'named':
            pose_name = raw_input("  pose name? ")
            try:
                self.go_to_named_pose(robot_name, pose_name)
            except rospy.ROSException as e:
                rospy.logerr('Unknown pose: %s' % e)
        elif key == 'frame':
            frame = raw_input("  frame? ")
            self.go_to_frame(robot_name, frame)
        elif key == 'speed':
            speed = float(raw_input("  speed value? "))
        elif key == 'stop':
            self.stop(robot_name)

        # Gripper stuffs
        elif key == 'gripper':
            gripper_name = raw_input("  gripper name? ")
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

        else:
            print('  unknown command! [%s]' % key)
        return robot_name, axis, speed

    # Basic motion stuffs
    def go_to_named_pose(self, robot_name, named_pose):
        group = self._cmd.get_group(robot_name)  # get MoveGroupCommander
        try:
            group.set_named_target(named_pose)
        except moveit_commander.exception.MoveItCommanderException as e:
            rospy.logerr('AISTBaseRoutines.go_to_named_pose(): {}'
                         .format(e))
            return False
        group.set_max_velocity_scaling_factor(1.0)
        #group.set_max_acceleration_scaling_factor(1.0)
        success = group.go(wait=True)
        group.clear_pose_targets()
        return success

    def go_to_frame(self, robot_name, target_frame, offset=(0, 0, 0),
                    speed=1.0, accel=1.0,
                    end_effector_link='', high_precision=False, move_lin=True):
        target_pose = PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose            = Pose(Point(0, 0, 0),
                                           Quaternion(0, 0, 0, 1))
        return self.go_to_pose_goal(robot_name,
                                    self.effector_target_pose(target_pose,
                                                              offset),
                                    speed, accel, end_effector_link,
                                    high_precision, move_lin)

    def go_to_pose_goal(self, robot_name, target_pose, speed=1.0, accel=1.0,
                        end_effector_link='', high_precision=False,
                        move_lin=True):
        self.add_marker('pose', target_pose)
        self.publish_marker()

        if move_lin:
            return self.go_along_poses(robot_name,
                                       PoseArray(target_pose.header,
                                                 [target_pose.pose]),
                                       speed, accel,
                                       end_effector_link, high_precision)

        group = self._cmd.get_group(robot_name)

        if end_effector_link == '':
            end_effector_link = self.gripper(robot_name).tip_link
        group.set_end_effector_link(end_effector_link)

        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))
        group.set_max_acceleration_scaling_factor(np.clip(accel, 0.0, 1.0))
        group.set_pose_target(target_pose)
        success      = group.go(wait=True)
        current_pose = group.get_current_pose()
        is_all_close = self._all_close(target_pose.pose,
                                       current_pose.pose, 0.01)
        return (success, is_all_close, current_pose)

    def go_along_poses(self, robot_name, poses, speed=1.0, accel=1.0,
                       end_effector_link='', high_precision=False):
        group = self._cmd.get_group(robot_name)

        try:
            transformed_poses = self.transform_poses_to_target_frame(
                                    poses, group.get_planning_frame()).poses
        except Exception as e:
            return (False, False, group.get_current_pose())

        if end_effector_link == '':
            end_effector_link = self.gripper(robot_name).tip_link
        group.set_end_effector_link(end_effector_link)

        if high_precision:
            goal_tolerance = group.get_goal_tolerance()
            planning_time  = group.get_planning_time()
            group.set_goal_tolerance(.000001)
            group.set_planning_time(10)

        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))
        group.set_max_acceleration_scaling_factor(np.clip(accel, 0.0, 1.0))
        plan, fraction = group.compute_cartesian_path(transformed_poses,
                                                      self._eef_step, 0.0)
        if fraction > 0.995:
            success = group.execute(group.retime_trajectory(
                                        self._cmd.get_current_state(),
                                        plan,
                                        velocity_scaling_factor=speed,
                                        acceleration_scaling_factor=accel),
                                    wait=True)
            group.stop()
            if success:
                rospy.loginfo('Executed plan with %3.1f%% computed cartesian path.',
                              100*fraction)
            else:
                rospy.logerr('Computed %3.1f%% of cartesian path but failed to execute.',
                             100*fraction)
        else:
            success = False
            rospy.logwarn('Computed only %3.1f%% of the total cartesian path.',
                          100*fraction)

        group.clear_pose_targets()

        if high_precision:
            group.set_goal_tolerance(goal_tolerance[1])
            group.set_planning_time(planning_time)

        current_pose = group.get_current_pose()
        is_all_close = self._all_close(transformed_poses[-1],
                                       current_pose.pose, 0.01)
        return (success, is_all_close, current_pose)

    def move_relative(self, robot_name, offset,
                      speed=1.0, accel=1.0, end_effector_link='',
                      high_precision=False, move_lin=True):
        return self.go_to_pose_goal(
                   robot_name,
                   self.shift_pose(self.get_current_pose(robot_name,
                                                         end_effector_link),
                                   offset),
                   speed, accel, end_effector_link, high_precision, move_lin)

    def stop(self, robot_name):
        group = self._cmd.get_group(robot_name)
        group.stop()
        group.clear_pose_targets()

    def get_current_pose(self, robot_name, end_effector_link=''):
        if end_effector_link == '' and robot_name in self._grippers:
            end_effector_link = self.gripper(robot_name).tip_link
        group = self._cmd.get_group(robot_name)
        if len(end_effector_link) > 0:
            group.set_end_effector_link(end_effector_link)
        return group.get_current_pose()

    # Gripper stuffs
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
                                            graspabilities.poses, target_frame)
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
            self.gripper(robot_name).parameters = params['gripper_parameters']
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
            self.gripper(robot_name).parameters = params['gripper_parameters']
        return self._pick_or_place.send_goal(robot_name, target_pose, False,
                                             params['place_offset'],
                                             params['approach_offset'],
                                             params['departure_offset'],
                                             params['speed_fast'],
                                             params['speed_slow'],
                                             wait, done_cb, active_cb)

    def pick_at_frame(self, robot_name, target_frame, part_id,
                      offset=(0, 0, 0),
                      wait=True, done_cb=None, active_cb=None):
        target_pose = PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose = Pose(Point(*offset[0:3]),
                                Quaternion(
                                    *self._quaternion_from_offset(offset[3:])))
        return self.pick(robot_name, target_pose, part_id,
                         wait, done_cb, active_cb)

    def place_at_frame(self, robot_name, target_frame, part_id,
                       offset=(0, 0, 0),
                       wait=True, done_cb=None, active_cb=None):
        target_pose = PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose = Pose(Point(*offset[0:3]),
                                Quaternion(
                                    *self._quaternion_from_offset(offset[3:])))
        return self.place(robot_name, target_pose, part_id,
                          wait, done_cb, active_cb)

    def pick_or_place_wait_for_result(self, timeout=rospy.Duration()):
        if self._pick_or_place.wait_for_result(timeout):
            return self._pick_or_place.get_result().result
        else:
            return None

    def pick_or_place_cancel(self):
        self._pick_or_place.cancel_goal()

    def pick_or_place_wait_for_stage(self, stage, timeout=rospy.Duration()):
        return self._pick_or_place.wait_for_stage(stage, timeout)

    # Sweep action stuffs
    def sweep(self, robot_name, target_pose, sweep_dir, part_id,
              wait=True, feedback_cb=None):
        R = tfs.quaternion_matrix((target_pose.pose.orientation.x,
                                   target_pose.pose.orientation.y,
                                   target_pose.pose.orientation.z,
                                   target_pose.pose.orientation.w))
        xdir = np.cross(sweep_dir, R[0:3, 2])   # sweep_dir ^ surface_normal
        R[0:3, 0] = xdir/np.linalg.norm(xdir)
        R[0:3, 1] = sweep_dir/np.linalg.norm(sweep_dir)
        R[0:3, 2] = np.cross(R[0:3, 0], R[0:3, 1])
        target_pose.pose.orientation = Quaternion(
                                           *tfs.quaternion_from_matrix(R))
        params = self._sweep_params[part_id]
        return self._sweep.execute(robot_name, target_pose,
                                   params['sweep_length'],
                                   params['sweep_offset'],
                                   params['approach_offset'],
                                   params['departure_offset'],
                                   params['speed_fast'],
                                   params['speed_slow'],
                                   wait, feedback_cb)

    # Utility functions
    def shift_pose(self, pose, offset):
        m44 = tfs.concatenate_matrices(self._listener.fromTranslationRotation(
                                           (pose.pose.position.x,
                                            pose.pose.position.y,
                                            pose.pose.position.z),
                                           (pose.pose.orientation.x,
                                            pose.pose.orientation.y,
                                            pose.pose.orientation.z,
                                            pose.pose.orientation.w)),
                                       self._listener.fromTranslationRotation(
                                           offset[0:3],
                                           self._quaternion_from_offset(
                                               offset[3:])))
        return PoseStamped(
                   pose.header,
                   Pose(Point(*tuple(tfs.translation_from_matrix(m44))),
                        Quaternion(*tuple(tfs.quaternion_from_matrix(m44)))))

    def transform_pose_to_target_frame(self, pose, target_frame=''):
        poses = self.transform_poses_to_target_frame(PoseArray(pose.header,
                                                               [pose.pose]),
                                                     target_frame)
        return PoseStamped(poses.header, poses.poses[0])

    def transform_poses_to_target_frame(self, poses, target_frame=''):
        if target_frame == '':
            target_frame = self._reference_frame

        try:
            self._listener.waitForTransform(target_frame,
                                            poses.header.frame_id,
                                            poses.header.stamp,
                                            rospy.Duration(10))
            mat44 = self._listener.asMatrix(target_frame, poses.header)
        except Exception as e:
            rospy.logerr('AISTBaseRoutines.transform_poses_to_target_frame(): {}'.format(e))
            raise e

        transformed_poses = PoseArray()
        transformed_poses.header.frame_id = target_frame
        transformed_poses.header.stamp    = poses.header.stamp
        for pose in poses.poses:
            m44 = tfs.concatenate_matrices(
                        mat44,
                        self._listener.fromTranslationRotation(
                            (pose.position.x,
                             pose.position.y,
                             pose.position.z),
                            (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)))
            transformed_poses.poses.append(
                Pose(Point(*tuple(tfs.translation_from_matrix(m44))),
                     Quaternion(*tuple(tfs.quaternion_from_matrix(m44)))))
        return transformed_poses

    def xyzrpy_from_pose(self, pose, deg=True):
        transformed_pose = self.transform_pose_to_target_frame(pose).pose
        rpy = tfs.euler_from_quaternion((transformed_pose.orientation.x,
                                         transformed_pose.orientation.y,
                                         transformed_pose.orientation.z,
                                         transformed_pose.orientation.w))
        if deg:
            rpy = list(map(degrees, rpy))
        return [transformed_pose.position.x,
                transformed_pose.position.y,
                transformed_pose.position.z,
                rpy[0], rpy[1], rpy[2]]

    def pose_from_xyzrpy(self, xyzrpy, deg=True):
        rpy = list(map(radians, xyzrpy[3:6])) if deg else xyzrpy[3:6]
        pose = PoseStamped()
        pose.header.frame_id = self._reference_frame
        pose.pose = Pose(Point(*xyzrpy[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(*rpy)))
        return pose

    def format_pose(self, target_pose):
        xyzrpy = self.xyzrpy_from_pose(target_pose)
        return '[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]' \
              .format(*xyzrpy)

    def effector_target_pose(self, target_pose, offset):
        poses = self.effector_target_poses(PoseArray(target_pose.header,
                                                     [target_pose.pose]),
                                           [offset])
        return PoseStamped(poses.header, poses.poses[0])

    def effector_target_poses(self, target_poses, offsets):
        poses = PoseArray(target_poses.header, [])
        for target_pose, offset in zip(target_poses.poses, offsets):
            T = tfs.concatenate_matrices(
                    self._listener.fromTranslationRotation(
                        (target_pose.position.x,
                         target_pose.position.y,
                         target_pose.position.z),
                        (target_pose.orientation.x,
                         target_pose.orientation.y,
                         target_pose.orientation.z,
                         target_pose.orientation.w)),
                    self._listener.fromTranslationRotation(
                        offset[0:3],
                        self._quaternion_from_offset(offset[3:])),
                    self._listener.fromTranslationRotation(
                        (0, 0, 0),
                        tfs.quaternion_from_euler(0, radians(90), 0)))
            poses.poses.append(Pose(Point(*tfs.translation_from_matrix(T)),
                                    Quaternion(*tfs.quaternion_from_matrix(T))))
        return poses

    # Private functions
    def _create_device(self, type_name, kwargs):
        Device = globals()[type_name]
        if rospy.get_param('/use_real_robot', False):
            return Device(**kwargs)
        else:
            return Device.base(**kwargs)

    def _all_close(self, goal, actual, tolerance):
        goal_list   = pose_to_list(goal)
        actual_list = pose_to_list(actual)
        for i in range(len(goal_list)):
            if abs(actual_list[i] - goal_list[i]) > tolerance:
                return False
        return True

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

    def _quaternion_from_offset(self, offset):
        return (0, 0, 0, 1) if len(offset) < 3 else \
               tfs.quaternion_from_euler(offset[0],
                                         offset[1],
                                         offset[2]) if len(offset) == 3 else \
               offset[0:4]
