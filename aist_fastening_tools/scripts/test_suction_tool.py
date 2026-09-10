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
from rclpy.executors      import MultiThreadedExecutor
from rclpy.node           import Node
from aist_fastening_tools import SuctionTool


class TestSuctionTool(Node):
    def __init__(self, name):
        super().__init__(name)

        device_name = self.declare_parameter('device_name',
                                             'suction_tool').value
        self._suction_tool = SuctionTool(self, device_name)
        self.get_logger().info('started')

        threading.Thread(target=self.interactive, daemon=True).start()

    def interactive(self):
        while rclpy.ok():
            print('====')
            print('  q: quit this program')
            print('  g: grasp')
            print('  r: release')
            print('  m: set min period')
            print('  w: wait for result for two seconds')
            print('  c: cancel')

            key = input('[suck_min_period=%f]> '
                        % self._suction_tool.parameters['suck_min_period'])

            if key == 'q':
                break
            elif key == 'g':
                self._suction_tool.grasp(timeout_sec=0.0)
            elif key == 'r':
                self._suction_tool.release(timeout_sec=None)
            elif key == 'm':
                suck_min_period = float(input('  suck_min_period? '))
                self._suction_tool.parameters = {'suck_min_period':
                                                 suck_min_period}
            elif key == 'w':
                status, result = self._gripper.wait(timeout_sec=2.0)
                print(result)
            elif key == 'c':
                self._suction_tool.cancel_goal()
            else:
                print('Unknown command[%s]' % key)
        self.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        rclpy.init(args=sys.argv)

        test = TestSuctionTool('test_suction_tool')
        executor = MultiThreadedExecutor()
        executor.add_node(test)
        executor.spin()
    except Exception as e:
        print('*** Terminate the node due to exception: %s' % e)
