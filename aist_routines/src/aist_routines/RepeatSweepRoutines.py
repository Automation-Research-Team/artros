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
#  * Redistributions of source code must retain the above copyight
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
from std_msgs.msg                    import Header
from geometry_msgs.msg               import PoseStamped
from aist_routines.KittingRoutines   import KittingRoutines
from aist_routines.RepeatSweepAction import RepeatSweep
from aist_utility.compat             import *

######################################################################
#  class RepeatSweepRoutines                                         #
######################################################################
class RepeatSweepRoutines(KittingRoutines):
    """Implements RepeatSweep routines for aist robot system."""

    def __init__(self, server='hmi_server'):
        super(RepeatSweepRoutines, self).__init__()

        self._repeat_sweep_params = rospy.get_param('~repeat_sweep_parameters')
        self._repeat_sweep_offset = (0.0, 0.0, 0.02)
        self._repeat_sweep        = RepeatSweep(self)

    @property
    def current_robot_name(self):
        return self._repeat_sweep.current_robot_name

    def print_help_messages(self):
        super(RepeatSweepRoutines, self).print_help_messages()
        print('=== RepeatSweep commands ===')
        print('  r: Repeat sweep')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'r':
            bin_id   = 'bin_' + raw_input('  bin id? ')
            x_offset = float(raw_input('  x_offset? '))
            self.repeat_sweep_bin(bin_id, x_offset)
        else:
            return super(RepeatSweepRoutines,
                         self).interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    # RepeatSweep stuffs
    def repeat_sweep_bin(self, bin_id, x_offset):
        """
        Search graspability points from the specified bin and sweep the one
        with the highest score.

        @type  bin_id: str
        @param bin_id: ID specifying the bin
        @return:       True if sweep succeeded
        """
        bin_props  = self._bin_props[bin_id]
        frame_id   = bin_props['name']
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']
        offset     = (x_offset, 0.0, 0.020)
        pose       = PoseStamped(Header(frame_id=frame_id),
                                 self.pose_from_offset(offset))
        params     = self._repeat_sweep_params
        result     = self._repeat_sweep.send_goal(robot_name, pose,
                                                  params['sweep_length'],
                                                  params['sweep_offset'],
                                                  params['approach_offset'],
                                                  params['departure_offset'],
                                                  params['speed_fast'],
                                                  params['speed_slow'],
                                                  params['direction_range'],
                                                  self._done_cb)
