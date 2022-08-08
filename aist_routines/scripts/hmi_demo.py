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
import rospy
import numpy as np
from math                     import pi, radians, degrees
from geometry_msgs.msg        import (QuaternionStamped, PoseStamped,
                                      PointStamped, Vector3Stamped, Vector3)
from aist_routines            import AISTBaseRoutines
from aist_routines.msg        import PickOrPlaceResult, SweepResult
from finger_pointing_msgs.msg import (RequestHelpAction, RequestHelpGoal,
                                      RequestHelpResult, request_help)
from actionlib                import SimpleActionClient
from actionlib_msgs.msg       import GoalStatus
from tf                       import transformations as tfs

######################################################################
#  class HMIRoutines                                                 #
######################################################################
class HMIRoutines(AISTBaseRoutines):
    """Implements HMI routines for aist robot system."""

    def __init__(self, server='hmi_server'):
        super(HMIRoutines, self).__init__()

        self._ground_frame       = rospy.get_param('~ground_frame', 'ground')
        self._bin_props          = rospy.get_param('~bin_props')
        self._part_props         = rospy.get_param('~part_props')
        self._current_robot_name = None
        self._fail_poses         = []
        self._request_help       = SimpleActionClient(server + '/request_help',
                                                      RequestHelpAction)
        self._request_help.wait_for_server()

    @property
    def nbins(self):
        return len(self._bin_props)

    @property
    def current_robot_name(self):
        return self._current_robot_name

    ###----- main procedure
    def demo(self, bin_id, max_attempts=1):
        while kitting.attempt_bin(bin_id, 5):
            pass
        self.go_to_named_pose(self.current_robot_name, 'home')

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
        if self._current_robot_name is not None and \
           self._current_robot_name != robot_name:
            self.go_to_named_pose(self._current_robot_name, 'back')
        self._current_robot_name = robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(robot_name, part_props['camera_name']):
            self.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        # Search for graspabilities.
        poses, _ = self.search(bin_id)

        # Attempt to pick the item.
        nattempts = 0
        for p in poses.poses:
            if nattempts == max_attempts:
                break

            pose = PoseStamped(poses.header, p)
            if self._is_close_to_fail_poses(pose):
                continue

            result = self.pick(robot_name, pose, part_id)

            if result == PickOrPlaceResult.SUCCESS:
                result = self.place_at_frame(robot_name,
                                             part_props['destination'],
                                             part_id)
                return result == PickOrPlaceResult.SUCCESS
            elif result == PickOrPlaceResult.MOVE_FAILURE or \
                 result == PickOrPlaceResult.APPROACH_FAILURE:
                self._fail_poses.append(pose)
            elif result == PickOrPlaceResult.DEPARTURE_FAILURE:
                raise RuntimeError('Failed to depart from pick/place pose')
            elif result == PickOrPlaceResult.GRASP_FAILURE:
                rospy.logwarn('(hmi_demo) Pick failed. Request help!')
                message = 'Picking failed! Please specify sweep direction.'
                while self.request_help_and_sweep(robot_name, pose, part_id,
                                                  message):
                    message = 'Planning for sweeping failed! Please specify another sweep direction.'
                return True

        return False

    def request_help_and_sweep(self, robot_name, pose, part_id, message):
        req = request_help()
        req.robot_name = robot_name
        req.item_id    = part_id
        req.pose       = self.listener.transformPose(self._ground_frame, pose)
        req.request    = request_help.SWEEP_DIR_REQ
        req.message    = message

        if self._request_help.send_goal_and_wait(RequestHelpGoal(req)) \
           != GoalStatus.SUCCEEDED:
            return False

        res = self._request_help.get_result().response

        if res.pointing_state == pointing.SWEEP_RES:
            rospy.loginfo('(hmi_demo) Sweep direction given.')
            result = self.sweep(robot_name, pose,
                                self._compute_sweep_dir(pose, res), part_id)
            if result == SweepResult.MOVE_FAILURE or \
               result == SweepResult.APPROACH_FAILURE:
                return True                     # Need to send request again
            elif result == SweepResult.DEPARTURE_FAILURE:
                raise RuntimeError('Failed to depart from sweep pose')
        elif res.pointing_state == pointing.RECAPTURE_RES:
            rospy.loginfo('(hmi_demo) Recapture required.')
        else:
            rospy.logerr('(hmi_demo) Unknown command received!')

        return False                            # No more requests required.

    def clear_fail_poses(self):
        self._fail_poses = []

    def sweep_bin(self, bin_id):
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']

        # If using a different robot from the former, move it back to home.
        if self._current_robot_name is not None and \
           self._current_robot_name != robot_name:
            self.go_to_named_pose(self._current_robot_name, 'back')
        self._current_robot_name = robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(robot_name, part_props['camera_name']):
            self.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        # Search for graspabilities.
        poses, _ = self.search(bin_id, 0.0)

        # Attempt to sweep the item along y-axis.
        pose  = PoseStamped(poses.header, poses.poses[0])
        R     = tfs.quaternion_matrix((pose.pose.orientation.x,
                                       pose.pose.orientation.y,
                                       pose.pose.orientation.z,
                                       pose.pose.orientation.w))
        result = self.sweep(robot_name, pose, R[0:3, 1], part_id)
        return result == SweepResult.SUCCESS

    def request_help_bin(self, bin_id):
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']
        message    = '[Request testing] Please specify sweep direction.'

        # Search for graspabilities.
        poses, _ = self.search(bin_id)
        pose     = PoseStamped(poses.header, poses.poses[0])

        # Create request.
        req = request_help()
        req.robot_name = robot_name
        req.item_id    = part_id
        req.pose       = self.listener.transformPose(self._ground_frame, pose)
        req.request    = request_help.SWEEP_DIR_REQ
        req.message    = message

        # Send request.
        if self._request_help.send_goal_and_wait(RequestHelpGoal(req)) \
           != GoalStatus.SUCCEEDED:
            print('Failed to request help!')
            return

        # Receive response and print.
        res = self._request_help.get_result().response
        print('respose=%s' % str(res))

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

    def _compute_sweep_dir(self, pose, res):
        fpos = self.listener.transformPoint(pose.header.frame_id,
                                            PointStamped(res.header,
                                                         res.finger_pos))
        fdir = self.listener.transformVector3(pose.header.frame_id,
                                              Vector3Stamped(res.header,
                                                             res.finger_dir))
        ppos = pose.pose.position
        fnrm = np.cross((fdir.x, fdir.y, fdir.z),
                        (ppos.x - fpos.x, ppos.y - fpos.y, ppos.z - fpos.z))
        gnrm = self.listener.transformVector3(pose.header.frame_id,
                                              Vector3Stamped(res.header,
                                                             Vector3(0, 0, 1)))
        sdir = np.cross(fnrm, gnrm)
        sdir = sdir / np.linalg.norm(sdir)

        return (sdir[0], sdir[1], sdir[2])


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
            print('  w: attempts to sWeep')
            print('  h: request help')
            print('  g: Grasp')
            print('  r: Release')
            print('  H: Move all robots to Home')
            print('  B: Move all robots to Back')
            print('  q: Quit')

            try:
                key = raw_input('>> ')
                if key == 'q':
                    if hmi.current_robot_name is not None:
                        hmi.go_to_named_pose(hmi.current_robot_name, 'home')
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
                    hmi.go_to_named_pose(hmi.current_robot_name, 'home')
                elif key == 'A':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.clear_fail_poses()
                    while hmi.attempt_bin(bin_id, 5):
                        pass
                    hmi.go_to_named_pose(hmi.current_robot_name, 'home')
                elif key == 'w':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.clear_fail_poses()
                    hmi.sweep_bin(bin_id)
                    hmi.go_to_named_pose(hmi.current_robot_name, 'home')
                elif key == 'h':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.request_help_bin(bin_id)
                elif key == 'c':
                    self.pick_or_place_cancel()
                elif key == 'g':
                    if hmi.current_robot_name is not None:
                        hmi.grasp(hmi.current_robot_name)
                elif key == 'r':
                    if hmi.current_robot_name is not None:
                        hmi.release(hmi.current_robot_name)
            except Exception as e:
                print('(hmi_demo) ' + e.message)
