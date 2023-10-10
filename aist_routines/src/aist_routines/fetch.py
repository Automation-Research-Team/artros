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

from actionlib              import SimpleActionClient
from geometry_msgs.msg      import Point, PointStamped
from math                   import pi, sin, radians
from numpy                  import clip
from control_msgs.msg       import PointHeadAction, PointHeadGoal
from FollowTrajectoryClient import FollowTrajectoryClient
from aist_routines          import AISTBaseRoutines
from freight                import FreightRoutines

######################################################################
#  class FetchRoutines                                               #
######################################################################
class FetchRoutines(AISTBaseRoutines, FreightRoutines):
    def __init__(self):
        super(FetchRoutines, self).__init__()

        self._torso_controller = FollowTrajectoryClient("torso_controller",
                                                        ["torso_lift_joint"])
        self._head_controller  = FollowTrajectoryClient("head_controller",
                                                        ["head_pan_joint",
                                                         "head_tilt_joint"])
        self._point_head = SimpleActionClient("head_controller/point_head",
                                              PointHeadAction)
        self._point_head.wait_for_server()

    # Interactive stuffs
    def print_help_messages(self):
        AISTBaseRoutines.print_help_messages(self)
        print('=== Fetch specific commands ===')
        print('  tucking:     move arm to the tacking position')
        print('  ready:       move arm to the ready position')
        print('  pick_ready:  move arm to the pick_ready position')
        print('  torso:       move torso')
        print('  head:        move haed')
        print('  shake_head:  shake head')
        print('  gaze:        gase the specified frame')

    def interactive(self, key, robot_name, axis, speed=1.0):
        if key == 'tucking':
            self.go_to_named_pose(robot_name, 'tucking')
        elif key == 'ready':
            self.go_to_named_pose(robot_name, 'ready')
        elif key == 'pick_ready':
            self.go_to_named_pose(robot_name, 'pick_ready')
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
        else:
            return AISTBaseRoutines.interactive(self,
                                                key, robot_name, axis, speed)
        return robot_name, axis, speed

    @property
    def torso_position(self):
        return self._torso_controller.current_position

    @property
    def head_position(self):
        return self._head_controller.current_position

    def move_torso(self, position, duration=5.0):
        return self._torso_controller.move_to([clip(position, 0.0, 0.4)],
                                              duration)

    def move_head(self, pan, tilt, duration=5.0):
        return self._head_controller.move_to([pan, tilt], duration)

    def shake_head(self, mag_pan, mag_tilt, nsteps=20, duration=10.0):
        positions_list = []
        for i in range(nsteps + 1):
            pan  = mag_pan  * sin(4.0*pi*i/nsteps)
            tilt = mag_tilt * sin(2.0*pi*i/nsteps)
            positions_list.append([pan, tilt])
        return self._head_controller.follow(positions_list, duration)

    def gaze(self, target_point):
        goal = PointHeadGoal()
        goal.target.header.stamp    = rospy.Time.now()
        goal.target.header.frame_id = target_point.header.frame_id
        goal.target.point           = target_point.point
        goal.min_duration           = rospy.Duration(1.0)
        self._point_head.send_goal(goal)
        self._point_head.wait_for_result()

    def gaze_frame(self, target_frame):
        target_point                 = PointStamped()
        target_point.header.frame_id = target_frame
        target_point.point           = Point(0, 0, 0)
        self.gaze(target_point)
