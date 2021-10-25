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
from geometry_msgs       import msg as gmsg
from tf                  import transformations as tfs
from aist_routines.fetch import FetchRoutines

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
class InteractiveRoutines(FetchRoutines):
    def __init__(self):
        super(InteractiveRoutines, self).__init__()

        self._robot_name  = 'arm'
        self._camera_name = 'head_camera'
        self._speed       = rospy.get_param('~speed', 0.1)

    def move(self, pose):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.pose = gmsg.Pose(
            gmsg.Point(pose[0], pose[1], pose[2]),
            gmsg.Quaternion(
                *tfs.quaternion_from_euler(pose[3], pose[4], pose[5])))
        (success, _, current_pose) = self.go_to_pose_goal(
                                        self._robot_name, target_pose,
                                        self._speed, move_lin=True)
        return success

    def go_home(self):
        self.move_head(0, 0)
        self.go_to_named_pose(self._robot_name, 'ready')

    def run(self):
        # Reset pose
        self.go_home()

        axis = 'X'

        while not rospy.is_shutdown():
            current_pose = self.get_current_pose(self._robot_name)
            prompt = '{:>5}:{}>> '.format(axis, self.format_pose(current_pose))

            key = raw_input(prompt)

            if key == 'q':
                break
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
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] += 0.01
                elif axis == 'Y':
                    goal_pose[1] += 0.01
                elif axis == 'Z':
                    goal_pose[2] += 0.01
                elif axis == 'Roll':
                    goal_pose[3] += radians(10)
                elif axis == 'Pitch':
                    goal_pose[4] += radians(10)
                else:
                    goal_pose[5] += radians(10)
                self.move(goal_pose)
            elif key == '-':
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] -= 0.01
                elif axis == 'Y':
                    goal_pose[1] -= 0.01
                elif axis == 'Z':
                    goal_pose[2] -= 0.01
                elif axis == 'Roll':
                    goal_pose[3] -= radians(10)
                elif axis == 'Pitch':
                    goal_pose[4] -= radians(10)
                else:
                    goal_pose[5] -= radians(10)
                self.move(goal_pose)
            elif is_num(key):
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] = float(key)
                elif axis == 'Y':
                    goal_pose[1] = float(key)
                elif axis == 'Z':
                    goal_pose[2] = float(key)
                elif axis == 'Roll':
                    goal_pose[3] = radians(float(key))
                elif axis == 'Pitch':
                    goal_pose[4] = radians(float(key))
                else:
                    goal_pose[5] = radians(float(key))
                self.move(goal_pose)
            elif key == 's':
                self.stop(self._robot_name)
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
            elif key == 'bg':
                self.create_background_image(self._camera_name)
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
            elif key == 'tucking':
                self.go_to_named_pose(self._robot_name, 'tucking')
            elif key == 'home':
                self.go_to_named_pose(self._robot_name, 'ready')
            elif key == 'ready':
                self.go_to_named_pose(self._robot_name, 'ready')
            elif key == 'pick_ready':
                self.go_to_named_pose(self._robot_name, 'pick_ready')
            elif key == 'torso':
                position = float(raw_input('  position = '))
                self.move_torso(position)
            elif key == 'head':
                pan  = radians(float(raw_input('  head pan  = ')))
                tilt = radians(float(raw_input('  head tilt = ')))
                self.move_head(pan, tilt)
            elif key == 'move_base':
                x     = float(raw_input('  x     = '))
                y     = float(raw_input('  y     = '))
                theta = radians(float(raw_input('  theta = ')))
                self.move_base(x, y, theta)
            elif key == 'move_base_to_frame':
                self.move_base_to_frame(raw_input(' frame = '))
            elif key == 'shake_head':
                self.shake_head(radians(30), radians(30))
            elif key == 'gaze':
                self.gaze_frame(raw_input('  frame = '))

        # Reset pose
        self.go_home()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':

    rospy.init_node('fetch_interactive', anonymous=True)

    with InteractiveRoutines() as routines:
        routines.run()
