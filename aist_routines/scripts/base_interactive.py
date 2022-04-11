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
import argparse

from aist_routines.MoveBaseRoutines import MoveBaseRoutines

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
#  global functions                                                  #
######################################################################
if __name__ == '__main__':

    rospy.init_node('base_interactive', anonymous=True)

    routines = MoveBaseRoutines()

    while not rospy.is_shutdown():
        prompt = '{}>> '.format(
            routines.format_odom(routines.current_odom))
        key = raw_input(prompt)

        if key == 'q':
            break
        elif key == 'move_base':
            x     = float(raw_input('  x     = '))
            y     = float(raw_input('  y     = '))
            theta = float(raw_input('  theta = '))
            routines.move_base(x, y, theta)
        elif key == 'move_base_to_frame':
            routines.move_base_to_frame(raw_input(' frame = '))
