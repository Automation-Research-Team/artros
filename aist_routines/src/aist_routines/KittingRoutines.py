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
import rospy, numpy as np
from math                           import pi, radians, degrees, cos, sin, sqrt
from geometry_msgs.msg              import Quaternion
from aist_routines.ur               import URRoutines
from aist_routines.AssemblyRoutines import AssemblyRoutines
from aist_routines.AttemptBinAction import AttemptBin
from aist_utility.compat            import *
from tf                             import transformations as tfs

######################################################################
#  class KittingRoutines                                             #
######################################################################
class KittingRoutines(AssemblyRoutines):
    """Implements kitting routines for aist robot system."""

    def __init__(self, do_error_recovery=None, cancel_error_recovery=None):
        super().__init__()

        self._bin_props   = rospy.get_param('~bin_props')
        self._part_props  = rospy.get_param('~part_props')
        self._attempt_bin = AttemptBin(self, do_error_recovery,
                                       cancel_error_recovery)

    @property
    def current_robot_name(self):
        #return self._attempt_bin.current_robot_name
        return 'b_bot'

    def run(self):
        axis = 'Y'

        while not rospy.is_shutdown():
            self.print_help_messages()
            print('')

            prompt = '{:>5}:{}>> '.format(axis,
                                          self.format_pose(
                                              self.get_current_pose(
                                                  self.current_robot_name))) \
                     if self.current_robot_name else '>> '
            key = raw_input(prompt)

            try:
                _, axis, _ = self.interactive(key, self.current_robot_name,
                                              axis, 1.0)
            except Exception as e:
                print(e)

    # Interactive stuffs
    def print_help_messages(self):
        if self.current_robot_name:
            super().print_help_messages()
        print('=== Kitting commands ===')
        print('  m: Create a mask image')
        print('  s: Search graspabilities with normal parameters')
        print('  a: Attempt to pick and place')
        print('  A: Repeat attempts to pick and place')
        print('  c: Cancel attempts to pick and place')
        print('  d: Perform small demo')
        print('  H: Move all robots to home')
        print('  B: Move all robots to back')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'm':
            self.create_mask_image('a_motioncam', len(self._bin_props))
        elif key == 's':
            bin_id = 'bin_' + raw_input('  bin id? ')
            self.search_bin(bin_id)
        elif key == 'a':
            bin_id = 'bin_' + raw_input('  bin id? ')
            self.pick_tool(self.current_robot_name, 'suction_tool')
            self.go_to_named_pose(self.current_robot_name, 'home')
            self._attempt_bin.send_goal(bin_id, False, 5, self._done_cb)
        elif key == 'A':
            bin_id = 'bin_' + raw_input('  bin id? ')
            self.pick_tool(self.current_robot_name, 'suction_tool')
            self.go_to_named_pose(self.current_robot_name, 'home')
            self._attempt_bin.send_goal(bin_id, True, 5, self._done_cb)
        elif key == 'c':
            self._attempt_bin.cancel_goal()
        elif key == 'd':
            self.demo()
        elif key == 'H':
            self.go_to_named_pose('all_bots', 'home')
        elif key == 'B':
            self.go_to_named_pose('all_bots', 'back')
        elif robot_name:
            return super().interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    # Commands
    def search_bin(self, bin_id,
                   min_height=0.004, max_height=0.045, max_slant=pi/4):
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        self.graspability_send_goal(part_props['robot_name'],
                                    part_id, bin_props['mask_id'])
        self.camera(part_props['camera_name']).trigger_frame()
        return self.graspability_wait_for_result(
                   bin_props['name'],
                   lambda pose, min_height=min_height, max_height=max_height, max_slant=max_slant:
                       self._pose_filter(pose,
                                         min_height, max_height, max_slant))

    def demo(self):
        bin_ids = ('bin_1', 'bin_4', 'bin_5')

        while True:
            completed = False

            for bin_id in bin_ids:
                self._attempt_bin.send_goal(bin_id, False)
                completed = completed and not success

            if completed:
                break

        self.go_to_named_pose(self.current_robot_name, 'home')

    # Utilities
    def _pose_filter(self, pose, min_height, max_height, max_slant):
        if pose.position.z < min_height or pose.position.z > max_height:
            return None

        T = tfs.quaternion_matrix((pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w))
        normal = T[0:3, 2]      # local Z-axis at the graspability point
        up     = np.array((0, 0, 1))
        a = np.dot(normal, up)
        b = cos(max_slant)
        if a < b:
            p = sqrt((1.0 - b*b)/(1.0 - a*a))
            q = b - a*p
            R = np.identity(4, dtype=np.float32)
            R[0:3, 2] = p*normal + q*up                   # fixed Z-axis
            R[0:3, 1] = self._normalize(np.cross(R[0:3, 2], T[0:3, 0]))
            R[0:3, 0] = np.cross(R[0:3, 1], R[0:3, 2])
            pose.orientation = Quaternion(*tfs.quaternion_from_matrix(R))
        return pose

    def _normalize(self, x):
        return x / sqrt(np.dot(x, x))

    def _done_cb(self, state, result):
        rospy.sleep(1)          # Pause required after cancelling arm motion
        if self.current_robot_name:
            self.go_to_named_pose(self.current_robot_name, 'home')
            # self.place_tool(self.current_robot_name)
            # self.go_to_named_pose(self.current_robot_name, 'home')
