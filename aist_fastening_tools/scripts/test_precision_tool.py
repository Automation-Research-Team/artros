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
import rclpy, sys, threading
from rclpy.node           import Node
from aist_fastening_tools import PrecisionTool


class TestPrecisionTool(Node):
    def __init__(self, name):
        super().__init__(name)

        gripper_name = self.declare_parameter('device_name',
                                              'precision_tool').value
        self._gripper = PrecisionTool(self, gripper_name)
        self.get_logger().info('started')

        threading.Thread(target=self.interactive, daemon=True).start()

    def interactive(self):
        def is_float(s):
            try:
                float(s)
            except ValueError:
                return False
            else:
                return True

        while rclpy.ok():
            print('==== Available commands ====')
            print('  g:         Grasp')
            print('  r:         Release')
            print('  <numeric>: Open gripper with a specified gap value')
            print('  q:         Quit\n')

            key = input('>> ')
            if key == 'g':
                status, result = self._gripper.grasp(timeout_sec=None)
            elif key == 'r':
                status, result = self._gripper.release(timeout_sec=None)
            elif is_float(key):
                status, result = self._gripper.move(float(key))
            elif key=='q':
                break
            else:
                print('unknown command: %s' % key)
                continue

            print('---- Result ----')
            print(result)
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init(args=sys.argv)

    test = TestPrecisionTool('test_precision_tool')
    rclpy.spin(test)

if __name__ == '__main__':
    main()
