#!/usr/bin/env python2.7
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

import rospy
from math          import radians
from tf            import transformations as tfs
from geometry_msgs import msg as gmsg
from aist_routines import AISTBaseRoutines

######################################################################
#  class CheckCalibrationRoutines                                    #
######################################################################
class CheckCalibrationRoutines(AISTBaseRoutines):
    """Wrapper of MoveGroupCommander specific for this script"""

    def __init__(self):
        super(CheckCalibrationRoutines, self).__init__()
        self._camera_name      = rospy.get_param('~camera_name',
                                                'realsenseD435')
        self._robot_name       = rospy.get_param('~robot_name', 'b_bot')
        self._robot_base_frame = rospy.get_param('~robot_base_frame',
                                                'workspace_center')
        self._robot_effector_frame \
                                = rospy.get_param('~robot_effector_frame',
                                                'b_bot_ee_link')
        self._robot_effector_tip_frame \
                                = rospy.get_param('~robot_effector_tip_frame',
                                                '')
        self._initpose         = rospy.get_param('~initpose', [])
        self._speed            = rospy.get_param('~speed', 1)

    def move(self, pose):
        poseStamped = gmsg.PoseStamped()
        poseStamped.header.frame_id = self._robot_base_frame
        poseStamped.pose = gmsg.Pose(gmsg.Point(*pose[0:3]),
                                    gmsg.Quaternion(
                                         *tfs.quaternion_from_euler(
                                             *map(radians, pose[3:6]))))
        print('  move to ' + self.format_pose(poseStamped))
        (success, _, current_pose) \
            = self.go_to_pose_goal(
                self._robot_name, poseStamped, self._speed,
                end_effector_link=self._robot_effector_frame,
                move_lin=True)
        print('  reached ' + self.format_pose(current_pose))
        return success

    def move_to_marker(self):
        self.trigger_frame(self._camera_name)
        marker_pose = rospy.wait_for_message('/aruco_detector/pose',
                                            gmsg.PoseStamped, 10)
        approach_pose = self.effector_target_pose(marker_pose, (0, 0, 0.05))

        #  We have to transform the target pose to reference frame before moving
        #  to the approach pose because the marker pose is given w.r.t. camera
        #  frame which will change while moving in the case of "eye on hand".
        target_pose = self.transform_pose_to_target_frame(
                        self.effector_target_pose(marker_pose, (0, 0, 0)))
        print('  move to ' + self.format_pose(approach_pose))
        (success, _, current_pose) \
            = self.go_to_pose_goal(
                self._robot_name, approach_pose, self._speed,
                end_effector_link=self._robot_effector_tip_frame,
                move_lin=True)
        print('  reached ' + self.format_pose(current_pose))
        rospy.sleep(1)
        print('  move to ' + self.format_pose(target_pose))
        (success, _, current_pose) \
            = self.go_to_pose_goal(
                self._robot_name, target_pose, 0.05,
                end_effector_link=self._robot_effector_tip_frame,
                move_lin=True)
        print('  reached ' + self.format_pose(current_pose))

    def run(self):
        self.go_to_named_pose(self._robot_name, 'home')

        while not rospy.is_shutdown():
            try:
                print('\n  RET: go to the marker')
                print('  i  : go to initial position')
                print('  h  : go to home position')
                print('  q  : go to home position and quit')
                key = raw_input('>> ')
                if key == 'i':
                    self.move(self._initpose)
                elif key == 'h':
                    self.go_to_named_pose(self._robot_name, 'home')
                elif key == 'q':
                    break
                else:
                    self.move_to_marker()
            except rospy.ROSException as ex:
                rospy.logwarn(ex.message)
            except Exception as ex:
                rospy.logerr(ex)
                break

        self.go_to_named_pose(self._robot_name, 'home')


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    rospy.init_node('check_calibration')

    with CheckCalibrationRoutines() as routines:
        routines.run()
