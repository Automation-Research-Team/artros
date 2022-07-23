#!/usr/bin/env python
#
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
import rospy, collections
from math                     import pi, radians, degrees
from geometry_msgs.msg        import QuaternionStamped, PoseStamped
from aist_routines            import AISTBaseRoutines
from aist_routines.msg        import pickOrPlaceResult, sweepResult
from finger_pointing_msgs.msg import (RequestHelpAction, RequestHelpGoal,
                                      RequestHelpResult, request_help)
from actionlib                import SimpleActionClient
from actionlib_msgs.msg       import GoalStatus
from tf                       import TransformListener, transformations as tfs

######################################################################
#  class HMIRoutines                                                 #
######################################################################
class HMIRoutines(AISTBaseRoutines):
    """Implements HMI routines for aist robot system."""

    def __init__(self, server='hmi_server'):
        super(HMIRoutines, self).__init__()

        self._bin_props         = rospy.get_param('~bin_props')
        self._part_props        = rospy.get_param('~part_props')
        self._former_robot_name = None
        self._fail_poses        = []
        self._listener          = TransformListener()
        self._request_help      = SimpleActionClient(server + '/request_help',
                                                     RequestHelpAction)
        self._request_help.wait_for_server()

    @property
    def nbins(self):
        return len(self._bin_props)

    @property
    def former_robot_name(self):
        return self._former_robot_name

    ###----- main procedure
    def demo(self, bin_id, max_attempts=1):
        while kitting.attempt_bin(bin_id, 5):
            pass
        self.go_to_named_pose(self.former_robot_name, 'home')

    def search(self, bin_id, max_slant=pi/4):
        bin_props  = self._bin_props[bin_id]
        part_props = self._part_props[bin_props['part_id']]
        self.graspability_send_goal(part_props['robot_name'],
                                    bin_props['part_id'], bin_props['mask_id'])
        self.camera(part_props['camera_name']).trigger_frame()

        orientation = QuaternionStamped()
        orientation.header.frame_id = self.reference_frame
        orientation.quaternion.x = 0
        orientation.quaternion.y = 0
        orientation.quaternion.z = 0
        orientation.quaternion.w = 1
        return self.graspability_wait_for_result(orientation, max_slant)

    def attempt_bin(self, bin_id, max_attempts=5):
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']

        # If using a different robot from the former, move it back to home.
        if self._former_robot_name is not None and \
           self._former_robot_name != robot_name:
            self.go_to_named_pose(self._former_robot_name, 'back')
        self._former_robot_name = robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(robot_name, part_props['camera_name']):
            self.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        # Search for graspabilities.
        poses, _ = self.search(bin_id)

        # Attempt to pick the item.
        nattempts = 0
        for i, p in enumerate(poses.poses):
            if nattempts == max_attempts:
                break

            pose = PoseStamped(poses.header, p)
            if self._is_close_to_fail_poses(pose):
                continue

            result = self.pick(robot_name, pose, part_id)
            if result == pickOrPlaceResult.SUCCESS:
                result = self.place_at_frame(robot_name,
                                             part_props['destination'],
                                             part_id)
                return result == pickOrPlaceResult.SUCCESS
            elif result == pickOrPlaceResult.MOVE_FAILURE or \
                 result == pickOrPlaceResult.APPROACH_FAILURE:
                self._fail_poses.append(pose)
            elif result == pickOrPlaceResult.DEPARTURE_FAILURE:
                self.release(robot_name)
                raise RuntimeError('Failed to depart from pick/place pose')
            elif result == pickOrPlaceResult.GRASP_FAILURE:
                if not self.request_help(robot_name, part_id, pose):
                    rospy.logerr('(hmi_demo) no response received to the request')
                    self._fail_poses.append(pose)
                    nattempts += 1
                    continue
                res = self._request_help.get_result().response
                if res.pointing_state == pointing.SWEEP_RES:
                    rospy.loginfo('(hmi_demo) given sweep direction.')
                    self.sweep_bin(bin_id,
                                   self._compute_sweep_dir(res.header,
                                                           res.finger_pos,
                                                           res.finger_dir))
                elif res.pointing_state == pointing.RECAPTURE_RES:
                    rospy.loginfo('(hmi_demo) commanded recapture.')
                    return False
                else:
                    raise RuntimeError('received unknown command!')

            self.release(robot_name)

        return False

    def request_help(self, robot_name, part_id, pose):
        req = request_help()
        req.robot_name = robot_name
        req.item_id    = part_id
        req.pose       = self._listenr.transform_pose('ground', pose)
        req.request    = request_help.SWEEP_DIR_REQ
        req.message    = 'Picking failed! Please specify sweep direction.'
        return self._request_help.send_goal_and_wait(RequestHelpGoal(req)) \
               is GoalStatus.SUCCEEDED

    def sweep_bin(self, bin_id):
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']

        # If using a different robot from the former, move it back to home.
        if self._former_robot_name is not None and \
           self._former_robot_name != robot_name:
            self.go_to_named_pose(self._former_robot_name, 'back')
        self._former_robot_name = robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(robot_name, part_props['camera_name']):
            self.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        # Search for graspabilities.
        poses, _ = self.search(bin_id, 0.0)

        # Attempt to sweep the item.
        p = poses.poses[0]
        pose = PoseStamped(poses.header, p)

        result = self.sweep(robot_name, pose, part_id)
        return result == sweepResult.SUCCESS

    def clear_fail_poses(self):
        self._fail_poses = []

    def _is_eye_on_hand(self, robot_name, camera_name):
        return camera_name == robot_name + '_camera'

    def _is_close_to_fail_poses(self, pose):
        for fail_pose in self._fail_poses:
            if self._is_close_to_fail_pose(pose, fail_pose, 0.005):
                return True
        return False

    def _is_close_to_fail_pose(self, pose, fail_pose, tolerance):
        position      = pose.pose.position
        fail_position = fail_pose.pose.position
        if abs(position.x - fail_position.x) > tolerance or \
           abs(position.y - fail_position.y) > tolerance or \
           abs(position.z - fail_position.z) > tolerance:
            return False
        return True

    def _compute_sweep_dir(self, header, finger_pos, finger_dir):
        pass


if __name__ == '__main__':

    rospy.init_node('hmi_demo', anonymous=True)

    with HMIRoutines() as hmi:
        while not rospy.is_shutdown():
            print('============ hmi_ procedures ============ ')
            print('  b: Create a backgroud image')
            print('  m: Create a mask image')
            print('  s: Search graspabilities')
            print('  a: Attempt to pick and place')
            print('  A: Repeat attempts to pick and place')
            print('  w: sWeep')
            print('  d: Perform Demo')
            print('  g: Grasp')
            print('  r: Release')
            print('  H: Move all robots to Home')
            print('  B: Move all robots to Back')
            print('  q: Quit')

            try:
                key = raw_input('>> ')
                if key == 'q':
                    if hmi.former_robot_name is not None:
                        hmi.go_to_named_pose(hmi.former_robot_name, 'home')
                    break
                elif key == 'H':
                    hmi.go_to_named_pose('all_bots', 'home')
                elif key == 'B':
                    hmi.go_to_named_pose('all_bots', 'back')
                elif key == 'b':
                    hmi.create_background_image('a_phoxi_m_camera')
                elif key == 'm':
                    hmi.create_mask_image('a_phoxi_m_camera', hmi.nbins)
                elif key == 's':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.search(bin_id)
                elif key == 'a':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.clear_fail_poses()
                    hmi.attempt_bin(bin_id, 5)
                    hmi.go_to_named_pose(hmi.former_robot_name, 'home')
                elif key == 'A':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.clear_fail_poses()
                    while hmi.attempt_bin(bin_id, 5):
                        pass
                    hmi.go_to_named_pose(hmi.former_robot_name, 'home')
                elif key == 'c':
                    self.pick_or_place_cancel()
                elif key == 'w':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.clear_fail_poses()
                    hmi.sweep_bin(bin_id)
                    hmi.go_to_named_pose(hmi.former_robot_name, 'home')
                elif key == 'd':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.clear_fail_poses()
                    hmi.demo(bin_id, 1)
                elif key == 'g':
                    if hmi.former_robot_name is not None:
                        hmi.grasp(hmi.former_robot_name)
                elif key == 'r':
                    if hmi.former_robot_name is not None:
                        hmi.release(hmi.former_robot_name)
            except Exception as e:
                print('(hmi_demo) ' + e.message)
