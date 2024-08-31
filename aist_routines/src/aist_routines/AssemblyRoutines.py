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
import rospy, re
from std_msgs.msg                  import Header
from geometry_msgs.msg             import PoseStamped
from aist_routines.ur              import URRoutines
from aist_utility.compat           import *

######################################################################
#  class AssemblyRoutines                                            #
######################################################################
class AssemblyRoutines(URRoutines):
    """Implements assembly routines for aist robot system."""

    def __init__(self):
        super().__init__()
        self._initialize_collision_objects()

    def run(self):
        robot_name = list(rospy.get_param('~robots').keys())[0]
        axis       = 'Y'

        while not rospy.is_shutdown():
            self.print_help_messages()
            print('')

            prompt = '{:>5}:{}>> '.format(axis,
                                          self.format_pose(
                                              self.get_current_pose(
                                                  robot_name)))
            key = raw_input(prompt)

            try:
                robot_name, axis, _ = self.interactive(key, robot_name,
                                                       axis, 1.0)
            except Exception as e:
                print(e)

    # Interactive stuffs
    def print_help_messages(self):
        super().print_help_messages()
        print('=== Assembly commands ===')
        print('  t: Pick tool')
        print('  T: Place tool')
        print('  s: Pick screw')
        print('  i: Initialize all collision objects')
        print('  r: Remove specified collision objects')
        print('  H: Move all robots to home')
        print('  B: Move all robots to back')

    def interactive(self, key, robot_name, axis, speed):
        if key == 't':
            tool_name = raw_input('  tool name? ')
            self.pick_tool(robot_name, tool_name)
        elif key == 'T':
            self.place_tool(robot_name)
        elif key == 's':
            screw_name = raw_input('  screw name? ')
            self.pick_screw(robot_name, screw_name)
        elif key == 'i':
            self._initialize_collision_objects()
        elif key == 'r':
            object_id   = raw_input('  object_id? ')
            attach_link = raw_input('  attach_link? ') if object_id == '' else\
                          ''
            self.com.remove_object(object_id, attach_link)
            self._num_screw_m3 = 0
            self._num_screw_m4 = 0
        elif key == 'H':
            self.go_to_named_pose('all_bots', 'home')
        elif key == 'B':
            self.go_to_named_pose('all_bots', 'back')
        else:
            return super().interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    def pick_tool(self, robot_name, tool_name):
        if self.gripper(robot_name).name == tool_name:
            return True
        elif self.gripper(robot_name).name != \
             self.default_gripper_name(robot_name):
            self.place_tool(robot_name)
        if self.pick_at_frame(robot_name, tool_name + '/base_link',
                              tool_name, attach=True):
            return False
        self.set_gripper(robot_name, tool_name)
        return True

    def place_tool(self, robot_name):
        tool_name            = self.gripper(robot_name).name
        default_gripper_name = self.default_gripper_name(robot_name)
        if tool_name == default_gripper_name:
            return True
        self.set_gripper(robot_name, default_gripper_name)
        if self.place_at_frame(robot_name, tool_name + '_holder_link',
                               tool_name, attach=True):
            self.set_gripper(robot_name, tool_name)
            return False
        return True

    def pick_screw(self, robot_name, screw_name):
        tool_name = 'screw_tool_' + screw_name[-2:]
        if not self.pick_tool(robot_name, tool_name):
            return False
        feeder_name = 'screw_feeder_' + screw_name[-2:]
        screw_id    = self._screw_id(screw_name)
        if self.pick_at_frame(robot_name, screw_id + '/screw_head',
                              screw_id, attach=True):
            return False
        self._generate_screw(screw_name)
        return True

    def _initialize_collision_objects(self):
        for object_type, pose \
            in rospy.get_param('~initial_object_poses', {}).items():
            self.com.create_object(object_type,
                                   PoseStamped(
                                       Header(frame_id=pose['holder']),
                                       self.pose_from_offset(pose['offset'])))
        self._screw_m3_id = 0
        self._screw_m4_id = 0
        self._generate_screw('screw_m3')
        self._generate_screw('screw_m4')

    def _generate_screw(self, screw_name):
        if screw_name == 'screw_m3':
            self._screw_m3_id += 1
        else:
            self._screw_m4_id += 1
        feeder_name = 'screw_feeder_' + screw_name[-2:]
        self.com.create_object(screw_name,
                               PoseStamped(
                                   Header(frame_id=feeder_name + '_outlet_link'),
                                   self.pose_from_offset()),
                               self._screw_id(screw_name))

    def _screw_id(self, screw_name):
        return screw_name + '_' + str(self._screw_m3_id) \
               if screw_name == 'screw_m3' else \
               screw_name + '_' + str(self._screw_m4_id)
