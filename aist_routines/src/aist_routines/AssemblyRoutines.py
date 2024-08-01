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
from std_msgs.msg        import Header
from geometry_msgs.msg   import PoseStamped
from aist_routines.ur    import URRoutines
from aist_utility.compat import *

######################################################################
#  class AssemblyRoutines                                            #
######################################################################
class AssemblyRoutines(URRoutines):
    """Implements assembly routines for aist robot system."""

    def __init__(self):
        super().__init__()

        tool_descriptions = rospy.get_param('~tool_descriptions', {})
        self.psi.load_object_descriptions(tool_descriptions)
        for tool_name in tool_descriptions.keys():
            self.psi.attach_object(
                tool_name,
                PoseStamped(Header(frame_id=tool_name + '_holder_link'),
                            self.pose_from_offset((0, 0, 0, 0, 0, 0))))
        for name in self.psi.get_attached_objects():
            print('*** collision_object: ' + name)

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
        print('  a: Add all collision objects')
        print('  c: Clear all collision objects')
        print('  H: Move all robots to home')
        print('  B: Move all robots to back')

    def interactive(self, key, robot_name, axis, speed):
        if key == 't':
            tool_name = raw_input(' tool name? ')
            self.pick_tool(robot_name, tool_name)
        elif key == 'T':
            tool_name = raw_input(' tool name? ')
            self.place_tool(robot_name, tool_name)
        elif key == 'a':
            for tool_name, tool_props in rospy.get_param('~tools').items():
                self.psi.attach_object(
                    tool_name,
                    PoseStamped(Header(frame_id=tool_name + '_holder_link'),
                                self.pose_from_offset(())))
        elif key == 'c':
            self.psi.remove_attached_object()
        elif key == 'H':
            self.go_to_named_pose('all_bots', 'home')
        elif key == 'B':
            self.go_to_named_pose('all_bots', 'back')
        elif robot_name:
            return super().interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    def pick_tool(self, robot_name, tool_name):
        if self.gripper(robot_name).name == tool_name:
            return True
        elif self.gripper(robot_name).name != \
             self.default_gripper_name(robot_name):
            self.place_tool(robot_name)
        self.psi.add_touch_links_to_attached_object(
            tool_name, self.gripper(robot_name).touch_links)
        if self.pick_at_frame(robot_name, tool_name + '/base_link',
                              tool_name, attach=True):
            return False
        # self.set_gripper(robot_name, tool_name)
        return True

    def place_tool(self, robot_name, tool_name):
        #tool_name = self.gripper(robot_name).name
        print('*** current gripper = %s' % tool_name)
        if tool_name == self.default_gripper_name(robot_name):
            return True
        if self.place_at_frame(robot_name, tool_name + '_holder_link',
                               tool_name, attach=True):
            return False
        # self.set_gripper(robot_name, self.default_gripper_name(robot_name))
        return True
