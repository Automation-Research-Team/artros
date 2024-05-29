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
import rospy, collections, copy, numpy as np
from math                          import pi
from tf                            import transformations as tfs
from geometry_msgs.msg             import (PoseStamped, PointStamped,
                                           Vector3Stamped,
                                           Point, Quaternion, Vector3)
from aist_routines             import AISTBaseRoutines
from aist_routines.RepeatSweepAction import RepeatSweep
from aist_msgs.msg                   import SweepResult
from aist_msgs.msg                 import (RepeatSweepAction, RepeatSweepGoal,
                                           RepeatSweepResult,
                                           RepeatSweep, Pointing)
from visualization_msgs.msg        import Marker
from std_msgs.msg                  import ColorRGBA
from aist_utility.compat           import *

######################################################################
#  class RepeatSweepRoutines                                         #
######################################################################
class RepeatSweepRoutines(KittingRoutines):
    """Implements RepeatSweep routines for aist robot system."""

    def __init__(self, server='hmi_server'):
        super(RepeatSweepRoutines, self).__init__()

        self._ground_frame      = rospy.get_param('~ground_frame', 'ground')
        self._sweep_params      = rospy.get_param('~sweep_parameters')
        self._repeat_sweep_clnt = RepeatSweep(self)

    # Interactive stuffs
    def print_help_messages(self):
        super(RepeatSweepRoutines, self).print_help_messages()
        print('=== RepeatSweep commands ===')
        print('  w: sWeep')
        print('  r: Repeat sweep')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'w':
            bin_id = 'bin_' + raw_input('  bin id? ')
            self._attempt_bin._clear_fail_poses()
            self.sweep_bin(bin_id)
            self.go_to_named_pose(robot_name, 'home')
        elif key == 'r':
            bin_id = 'bin_' + raw_input('  bin id? ')
            self.repeat_sweep_bin(bin_id)
        else:
            return super(RepeatSweepRoutines,
                         self).interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    # Graspability stuffs
    def search_bin(self, bin_id, min_height=0.004, max_slant=pi/4):
        return super(HMIRoutines, self).search_bin(
                   bin_id, min_height,
                   0 if self.using_hmi_graspability_params else max_slant)

    @property
    def using_hmi_graspability_params(self):
        return not self._graspability_params_back is None

    def set_hmi_graspability_params(self, bin_id):
        bin_props = self._bin_props[bin_id]
        part_id   = bin_props['part_id']
        if self.using_hmi_graspability_params:
            rospy.logwarn('(hmi_demo) Already using graspability paramters for HMI demo.')
            return
        self._graspability_params_back \
            = copy.deepcopy(self._graspability_params[part_id])
        self._graspability_params[part_id] \
            = copy.deepcopy(self._hmi_graspability_params[part_id])
        rospy.loginfo('(hmi_demo) Set graspability paramters for HMI demo.')

    def restore_original_graspability_params(self, bin_id):
        print('*** restore_original_graspability_params')
        bin_props = self._bin_props[bin_id]
        part_id   = bin_props['part_id']
        if not self.using_hmi_graspability_params:
            rospy.logwarn('(hmi_demo) Already using original graspability paramters.')
            return
        self._graspability_params[part_id] \
            = copy.deepcopy(self._graspability_params_back)
        self._graspability_params_back = None
        rospy.loginfo('(hmi_demo) Restore original graspability paramters.')

    # Sweep stuffs
    def repeat_sweep_bin(self, bin_id):
        """
        Search graspability points from the specified bin and sweep the one
        with the highest score.

        @type  bin_id: str
        @param bin_id: ID specifying the bin
        @return:       True if sweep succeeded
        """
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        # if self._is_eye_on_hand(robot_name, part_props['camera_name']):
        #     self.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        # Search for graspabilities.
        self.set_hmi_graspability_params(bin_id)
        graspabilities = self.search_bin(bin_id)
        self.restore_original_graspability_params(bin_id)

        # Attempt to sweep the item along y-axis.
        pose = PoseStamped(graspabilities.poses.header,
                           graspabilities.poses.poses[0])
        R    = tfs.quaternion_matrix((pose.pose.orientation.x,
                                      pose.pose.orientation.y,
                                      pose.pose.orientation.z,
                                      pose.pose.orientation.w))
        result = self._sweep(robot_name, pose, R[0:3, 1], part_id)
        return result == SweepResult.SUCCESS

    def _sweep(self, robot_name, target_pose, sweep_dir, part_id, wait=True):
        R = tfs.quaternion_matrix((target_pose.pose.orientation.x,
                                   target_pose.pose.orientation.y,
                                   target_pose.pose.orientation.z,
                                   target_pose.pose.orientation.w))
        nz = R[0:3, 2]
        ny = sweep_dir - nz * np.dot(nz, sweep_dir)
        R[0:3, 1] = ny/np.linalg.norm(ny)
        R[0:3, 0] = np.cross(R[0:3, 1], nz)
        target_pose.pose.orientation = Quaternion(
                                           *tfs.quaternion_from_matrix(R))
        params = self._sweep_params[part_id]
        return self._sweep_clnt.send_goal(robot_name, target_pose,
                                          params['sweep_length'],
                                          params['sweep_offset'],
                                          params['approach_offset'],
                                          params['departure_offset'],
                                          params['speed_fast'],
                                          params['speed_slow'],
                                          wait)

    def _compute_sweep_dir(self, pose, response):
        ppos = pose.pose.position
        fpos = self.listener.transformPoint(pose.header.frame_id,
                                            PointStamped(response.header,
                                                         response.point)).point
        sdir = (fpos.x - ppos.x, fpos.y - ppos.y, fpos.z - ppos.z)
        return tuple(sdir / np.linalg.norm(sdir))
