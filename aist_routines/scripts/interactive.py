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
import rospy
from geometry_msgs    import msg as gmsg
from tf               import transformations as tfs
from math             import radians
from aist_routines.ur import URRoutines
from aist_routines    import AISTBaseRoutines

######################################################################
#  global functions                                                  #
######################################################################
def is_num(s):
    try:
        float(s)
    except ValueError:
        return False
    else:
        return True

######################################################################
#  class InteractiveRoutines                                         #
######################################################################
class InteractiveRoutines(AISTBaseRoutines):
    refposes = {
        'a_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians( 90)],
        'b_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians(-90)],
        'c_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians( 90)],
        'd_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians(-90)],
        'a_khi': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians( 90)],
        'b_khi': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians(-90)],
    }

    def __init__(self):
        super(InteractiveRoutines, self).__init__()

        self._robot_name  = rospy.get_param('~robot_name',  'b_bot')
        self._camera_name = rospy.get_param('~camera_name', 'a_phoxi_m_camera')
        self._speed       = rospy.get_param('~speed',       0.1)
        self._ur_movel    = False

    def move(self, xyzrpy):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.pose = gmsg.Pose(
            gmsg.Point(xyzrpy[0], xyzrpy[1], xyzrpy[2]),
            gmsg.Quaternion(
                *tfs.quaternion_from_euler(xyzrpy[3], xyzrpy[4], xyzrpy[5])))
        if self._ur_movel:
            (success, _, current_pose) = self.ur_movel(self._robot_name,
                                                       target_pose,
                                                       velocity=self._speed)
        else:
            (success, _, current_pose) = self.go_to_pose_goal(
                                                self._robot_name, target_pose,
                                                self._speed,
                                                move_lin=True)
        return success

    def run(self):
        # Reset pose
        self.go_to_named_pose(self._robot_name, "home")

        axis = 'Y'

        while not rospy.is_shutdown():
            current_pose = self.get_current_pose(self._robot_name)
            prompt = '{:>5}:{}{:>9}>> ' \
                   .format(axis, self.format_pose(current_pose),
                           'urscript' if self._ur_movel else 'moveit')

            key = input(prompt)

            if key == 'q':
                break
            elif key == 'r':
                self._robot_name = raw_input('  robot name? ')
            elif key == 'c':
                self._camera_name = raw_input('  camera name? ')
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
                self.move_relative(self._robot_name, offset, self._speed)
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
                self.move_relative(self._robot_name, offset, self._speed)
            elif is_num(key):
                xyzrpy = self.xyz_rpy(current_pose)
                if axis == 'X':
                    xyzrpy[0] = float(key)
                elif axis == 'Y':
                    xyzrpy[1] = float(key)
                elif axis == 'Z':
                    xyzrpy[2] = float(key)
                elif axis == 'Roll':
                    xyzrpy[3] = radians(float(key))
                elif axis == 'Pitch':
                    xyzrpy[4] = radians(float(key))
                else:
                    xyzrpy[5] = radians(float(key))
                self.move(xyzrpy)
            elif key == 's':
                self.stop(self._robot_name)
            elif key == 'f':
                frame = raw_input("  frame? ")
                self.go_to_frame(self._robot_name, frame)

            elif key == 'gripper':
                gripper_name = raw_input("  gripper name? ")
                try:
                    self.set_gripper(self._robot_name, gripper_name)
                except KeyError as e:
                    rospy.logerr('Unknown gripper: %s' % e)
            elif key == 'pregrasp':
                self.pregrasp(self._robot_name)
            elif key == 'grasp':
                self.grasp(self._robot_name)
            elif key == 'release':
                self.release(self._robot_name)
            elif key == 'cont':
                self.continuous_shot(self._camera_name, True)
            elif key == 'stopcont':
                self.continuous_shot(self._camera_name, False)
            elif key == 'trigger':
                self.trigger_frame(self._camera_name)
            elif key == 'mask':
                self.create_mask_image(self._camera_name,
                                       int(raw_input('  #bins? ')))
            elif key == 'search':
                self.graspability_send_goal(self._robot_name,
                                            '04_37D-GEARMOTOR-50-70', 0)
                self.trigger_frame(self._camera_name)
                (poses, gscore, success) = \
                    self.graspability_wait_for_result(self._camera_name, 0)
                for gs in gscore:
                    print(str(gs))
                print(str(poses))
            elif key == 'ur':
                self._ur_movel = not self._ur_movel
            elif key == 'push':
                self.ur_linear_push(self._robot_name, wait=False)
            elif key == 'spiral':
                self.ur_spiral_motion(self._robot_name, wait=False)
            elif key == 'insertion':
                self.ur_insertion(self._robot_name, wait=False)
            elif key == 'hinsertion':
                self.ur_horizontal_insertion(self._robot_name, wait=False)
            elif key == 'spiral':
                self.ur_spiral_motion(self._robot_name, wait=False)
            elif key == 'o':
                self.move(InteractiveRoutines.refposes[self._robot_name])
            elif key == 'h':
                self.go_to_named_pose(self._robot_name, "home")
            elif key == 'b':
                self.go_to_named_pose(self._robot_name, "back")
            elif key == 'n':
                pose_name = raw_input("  pose name? ")
                try:
                    self.go_to_named_pose(pose_name, self._robot_name)
                except rospy.ROSException as e:
                    rospy.logerr('Unknown pose: %s' % e)

            else:
                print('  unknown command! [%s]' % key)

        # Reset pose
        self.go_to_named_pose(self._robot_name, "home")


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':

    rospy.init_node('interactive', anonymous=True)

    with InteractiveRoutines() as routines:
        routines.run()
