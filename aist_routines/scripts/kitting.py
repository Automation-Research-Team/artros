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
from math              import pi, radians, degrees
from geometry_msgs.msg import PoseStamped, QuaternionStamped
from aist_routines     import AISTBaseRoutines
from aist_routines.msg import PickOrPlaceResult, SweepResult

######################################################################
#  class KittingRoutines                                             #
######################################################################
class KittingRoutines(AISTBaseRoutines):
    """Implements kitting routines for aist robot system."""

    def __init__(self):
        super(KittingRoutines, self).__init__()

        self._bin_props         = rospy.get_param('~bin_props')
        self._part_props        = rospy.get_param('~part_props')
        self._former_robot_name = None
        self._fail_poses        = []
        #self.go_to_named_pose('all_bots', 'home')

    @property
    def nbins(self):
        return len(self._bin_props)

    @property
    def former_robot_name(self):
        return self._former_robot_name

    ###----- main procedure
    def run(self):
        self.go_to_named_pose('all_bots', 'back')
        for bin_id in self._binprops.keys():
            if rospy.is_shutdown():
                break
            self.attempt_bin(bin_id, 1)
        self.go_to_named_pose('all_bots', 'home')

    def demo(self):
        bin_ids = ('bin_1', 'bin_4', 'bin_5')
#        bin_ids = ('bin_1', 'bin_4')

        while True:
            completed = False

            for bin_id in bin_ids:
                kitting.clear_fail_poses()
                success = kitting.attempt_bin(bin_id, 5)
                completed = completed and not success

            if completed:
                break

        kitting.go_to_named_pose(kitting.former_robot_name, 'home')

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
        for p in poses.poses:
            if nattempts == max_attempts:
                break

            pose = PoseStamped(poses.header, p)
            if self._is_close_to_fail_poses(pose):
                continue

            result = self.pick(robot_name, pose, part_id)
            if result is PickOrPlaceResult.SUCCESS:
                result = self.place_at_frame(robot_name,
                                             part_props['destination'],
                                             part_id)
                return result is PickOrPlaceResult.SUCCESS
            elif result is PickOrPlaceResult.MOVE_FAILURE or \
                 result is PickOrPlaceResult.APPROACH_FAILURE:
                self._fail_poses.append(pose)
            elif result is PickOrPlaceResult.DEPARTURE_FAILURE:
                self.release(robot_name)
                raise RuntimeError('Failed to depart from pick/place pose')
            elif result is PickOrPlaceResult.GRASP_FAILURE:
                self._fail_poses.append(pose)
                nattempts += 1

            self.release(robot_name)

        return False

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
        return result is SweepResult.SUCCESS

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


if __name__ == '__main__':

    rospy.init_node('kitting', anonymous=True)

    with KittingRoutines() as kitting:
        while not rospy.is_shutdown():
            print('============ Kitting procedures ============ ')
            print('  b: Create a backgroud image')
            print('  m: Create a mask image')
            print('  s: Search graspabilities')
            print('  a: Attempt to pick and place')
            print('  A: Repeat attempts to pick and place')
            print('  w: sWeep')
            print('  d: Perform small demo')
            print('  k: Do kitting task')
            print('  g: Grasp')
            print('  r: Release')
            print('  H: Move all robots to home')
            print('  B: Move all robots to back')
            print('  q: Quit')

            try:
                key = raw_input('>> ')
                if key == 'q':
                    if kitting.former_robot_name is not None:
                        kitting.go_to_named_pose(kitting.former_robot_name,
                                                 'home')
                    break
                elif key == 'H':
                    kitting.go_to_named_pose('all_bots', 'home')
                elif key == 'B':
                    kitting.go_to_named_pose('all_bots', 'back')
                elif key == 'b':
                    kitting.create_background_image('a_phoxi_m_camera')
                elif key == 'm':
                    kitting.create_mask_image('a_phoxi_m_camera',
                                              kitting.nbins)
                elif key == 's':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    kitting.search(bin_id)
                elif key == 'a':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    kitting.clear_fail_poses()
                    kitting.attempt_bin(bin_id, 5)
                    kitting.go_to_named_pose(kitting.former_robot_name, 'home')
                elif key == 'A':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    kitting.clear_fail_poses()
                    while kitting.attempt_bin(bin_id, 5):
                        pass
                    kitting.go_to_named_pose(kitting.former_robot_name, 'home')
                elif key == 'c':
                    self.pick_or_place_cancel()
                elif key == 'w':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    kitting.clear_fail_poses()
                    kitting.sweep_bin(bin_id)
                    kitting.go_to_named_pose(kitting.former_robot_name, 'home')
                elif key == 'd':
                    kitting.demo()
                elif key == 'k':
                    kitting.run()
                elif key == 'g':
                    if kitting.former_robot_name is not None:
                        kitting.grasp(kitting.former_robot_name)
                elif key == 'r':
                    if kitting.former_robot_name is not None:
                        kitting.release(kitting.former_robot_name)
            except Exception as e:
                print(e.message)
