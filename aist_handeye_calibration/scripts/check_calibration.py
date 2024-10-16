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
from math                import radians
from tf                  import transformations as tfs
from geometry_msgs       import msg as gmsg
from aist_routines       import AISTBaseRoutines
from aist_utility.compat import *

######################################################################
#  class CheckCalibrationRoutines                                    #
######################################################################
class CheckCalibrationRoutines(AISTBaseRoutines):
    def __init__(self):
        super(CheckCalibrationRoutines, self).__init__(
            rospy.get_param('~robot_base_frame', 'workspace_center'))

        self._camera_name      = rospy.get_param('~camera_name',
                                                 'realsenseD435')
        self._robot_name       = rospy.get_param('~robot_name', 'b_bot')
        self._robot_effector_frame \
                               = rospy.get_param('~robot_effector_frame',
                                                 'b_bot_ee_link')
        self._robot_effector_tip_frame \
                               = rospy.get_param('~robot_effector_tip_frame',
                                                 '')
        self._initpose         = rospy.get_param('~initpose', [])
        self._speed            = rospy.get_param('~speed', 1)

    def run(self):
        # Reset pose
        self.go_to_named_pose(self._robot_name, "home")
        self.print_help_messages()
        print('')

        axis = 'Y'

        while not rospy.is_shutdown():
            prompt = '{:>5}:{}>> '.format(axis, self.format_pose(
                                                    self.get_current_pose(
                                                        self._robot_name)))
            key = raw_input(prompt)
            _, axis, _ = self.interactive(key, self._robot_name, axis,
                                          self._speed)

        self.go_to_named_pose(self._robot_name, 'home')

    # interactive stuffs
    def print_help_messages(self):
        super(CheckCalibrationRoutines, self).print_help_messages()
        print('=== Checking commands ===')
        print('  init:  go to initial pose')
        print('  RET:   go to marker')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'init':
            self.go_to_initpose()
        elif key == '':
            self.go_to_marker()
        else:
            return super(CheckCalibrationRoutines, self) \
                  .interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    def go_to_initpose(self):
        pose = self.pose_from_xyzrpy(self._initpose)
        print('  move to %s' % self.format_pose(pose))
        success = self.go_to_pose_goal(self._robot_name, pose,
                                       speed=self._speed,
                                       end_effector_link=self._robot_effector_frame)
        print('  reached %s' %
              self.format_pose(self.get_current_pose(self._robot_name)))

    def go_to_marker(self):
        self.trigger_frame(self._camera_name)
        try:
            marker_pose = rospy.wait_for_message('/aruco_detector_3d/pose',
                                                 gmsg.PoseStamped, 5)
        except rospy.exceptions.ROSException as e:
            rospy.logerr(e)
            return

        #  We must transform the marker pose to reference frame before moving
        #  to the approach pose because the marker pose is given w.r.t. camera
        #  frame which will change while moving in the case of "eye on hand".
        marker_pose = self.transform_pose_to_target_frame(marker_pose)
        success = self.go_to_pose_goal(self._robot_name,
                                       marker_pose, (0, 0, 0.05),
                                       speed=self._speed,
                                       end_effector_link=self._robot_effector_tip_frame)
        print('  reached %s' %
              self.format_pose(self.get_current_pose(self._robot_name)))
        rospy.sleep(1)
        print('  move to %s' % self.format_pose(marker_pose))
        success = self.go_to_pose_goal(self._robot_name,
                                       marker_pose, speed=0.05,
                                       end_effector_link=self._robot_effector_tip_frame)
        print('  reached %s' %
              self.format_pose(self.get_current_pose(self._robot_name)))


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    rospy.init_node('check_calibration')

    routines = CheckCalibrationRoutines()
    routines.run()
