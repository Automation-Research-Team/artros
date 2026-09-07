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
import rclpy, sys, threading
from rclpy.executors import MultiThreadedExecutor


def _command_line_interface(node):
    node.cmdloop()
    node.destroy_node()
    rclpy.shutdown()

def _main(name, routines):
    rclpy.init(args=sys.argv)
    node = routines(name)

    threading.Thread(target=lambda: _command_line_interface(node),
                     daemon=True).start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

#*********************************************************************
#  entry points                                                      *
#*********************************************************************
def base():
    from aist_routines import BaseRoutines

    _main('base', BaseRoutines)

def assembly():
    from aist_routines import AssemblyRoutines

    _main('assembly', AssemblyRoutines)

def kitting():
    from aist_routines import KittingRoutines

    _main('kitting', KittingRoutines)

def hmi_demo():
    from aist_routines import HMIRoutines

    _main('hmi_demo', HMIRoutines)

def handeye_calibration():
    from aist_handeye_calibration import HandEyeCalibrationRoutines

    _main('handeye_calibration', HandEyeCalibrationRoutines)
