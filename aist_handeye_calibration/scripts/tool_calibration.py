#!/usr/bin/env python3
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
import rospy, copy
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf                import transformations as tfs
from math              import radians, degrees
from aist_routines     import AISTBaseRoutines

######################################################################
#  class ToolCalibrationRoutines                                     #
######################################################################
class ToolCalibrationRoutines(AISTBaseRoutines):
    refposes = {
        'a_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians( 90)],
        'b_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians(-90)],
        'c_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians( 90)],
        'd_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians(  0)],
    }

    def __init__(self):
        super(ToolCalibrationRoutines, self).__init__()

        self._robot_name = rospy.get_param('~robot_name', 'b_bot')
        self._speed      = rospy.get_param('~speed', 0.1)
        self._refpose    = ToolCalibrationRoutines.refposes[self._robot_name]

        gripper = self.gripper(self._robot_name)

        self._R0  = self.listener.fromTranslationRotation(
                        *self.listener.lookupTransform(
                            self._parent_frame(gripper.base_link),
                            gripper.base_link,
                            rospy.Time(0)))
        self._D0  = self.listener.fromTranslationRotation(
                        *self.listener.lookupTransform(
                            gripper.base_link,
                            gripper.tip_link,
                            rospy.Time(0)))
        self._rpy = list(tfs.euler_from_matrix(self._R0))

    def run(self):
        axis  = 'Pitch'

        while not rospy.is_shutdown():
            prompt = '{:>5}:[p={:.3f},y={:.3f}]>> ' \
                   .format(axis, degrees(self._rpy[1]), degrees(self._rpy[2]))
            key = raw_input(prompt)
            _, axis, _ = self.interactive(key, self._robot_name, axis,
                                          self._speed)

        # Reset pose
        self.go_to_named_pose(self._robot_name, 'home')
        self.print_tip_link()

    # interactive stuffs
    def print_help_messages(self):
        super(ToolCalibrationRoutines, self).print_help_messages()
        print('=== Tool calibration commands ===')
        print('  o: Move to reference pose')
        print('  r: Perform rolling motion')
        print('  p: Perform pitching motion')
        print('  y: Perform yawing motion')
        print('  t: Display tip link')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'o':
            self.move(self._refpose)
        elif key == 'r':
            self.rolling_motion()
        elif key == 'p':
            self.pitching_motion()
        elif key == 'y':
            self.yawing_motion()
        elif key == 't':
            self.rolling_motion()
            self.pitching_motion()
            self.yawing_motion()
        elif key == 'd':
            self.print_tip_link()
        else:
            robot_name, axis, speed = super(ToolCalibrationRoutines, self) \
                                     .interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    def move(self, pose):
        R = self.listener.fromTranslationRotation(
                tfs.translation_from_matrix(self._R0),
                tfs.quaternion_from_euler(*self._rpy))
        T = tfs.concatenate_matrices(
                self.listener.fromTranslationRotation(
                    (pose[0], pose[1], pose[2]),
                    tfs.quaternion_from_euler(pose[3], pose[4], pose[5])),
                tfs.inverse_matrix(self._D0),
                tfs.inverse_matrix(R),
                self._R0,
                self._D0)
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.pose = Pose(Point(*tfs.translation_from_matrix(T)),
                                Quaternion(*tfs.quaternion_from_matrix(T)))
        print('move to ' + self.format_pose(target_pose))
        success, _, current_pose = self.go_to_pose_goal(self._robot_name,
                                                        target_pose,
                                                        self._speed,
                                                        move_lin=True,
                                                        high_precision=True)
        print('reached ' + self.format_pose(current_pose))
        return success

    def rolling_motion(self):
        pose = copy.deepcopy(self._refpose)
        for i in range(4):
            pose[3] += radians(30)
            self.move(pose)
        for i in range(8):
            pose[3] -= radians(30)
            self.move(pose)
        for i in range(4):
            pose[3] += radians(30)
            self.move(pose)

    def pitching_motion(self):
        pose = copy.deepcopy(self._refpose)
        for i in range(5):
            pose[4] += radians(6)
            self.move(pose)
        for i in range(10):
            pose[4] -= radians(6)
            self.move(pose)
        for i in range(5):
            pose[4] += radians(6)
            self.move(pose)

    def yawing_motion(self):
        pose = copy.deepcopy(self._refpose)
        pose[3] = radians(-90)
        pose[5] = radians(180)
        for i in range(5):
            pose[4] += radians(6)
            self.move(pose)
        for i in range(10):
            pose[4] -= radians(6)
            self.move(pose)
        for i in range(5):
            pose[4] += radians(6)
            self.move(pose)

    def print_tip_link(self):
        R   = self.listener.fromTranslationRotation(
                tfs.translation_from_matrix(self._R0),
                tfs.quaternion_from_euler(*self._rpy))
        D   = tfs.concatenate_matrices(R, self._D0)
        xyz = tfs.translation_from_matrix(D)
        q   = tfs.quaternion_from_matrix(D)
        rpy = map(degrees, tfs.euler_from_quaternion(q))
        print('<origin xyz="{0[0]} {0[1]} {0[2]}" rpy="${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}"/>'
              .format(xyz, rpy))

    def _parent_frame(self, frame):
        tm    = rospy.Time(0)
        chain = self.listener.chain('workspace_center', tm, frame, tm,
                                    'workspace_center')
        return chain[chain.index(frame) + 1]


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':

    rospy.init_node('tool_calibration', anonymous=True)

    tool_calibration = ToolCalibrationRoutines()
    tool_calibration.run()
