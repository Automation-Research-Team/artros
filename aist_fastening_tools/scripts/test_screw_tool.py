#!/usr/bin/env python3

import rclpy, sys, threading
import numpy as np
from rclpy.node           import Node
from aist_fastening_tools import ScrewTool


class TestScrewTool(Node):
    def __init__(self, name):
        super().__init__(name)

        device_name = self.declare_parameter('device_name',
                                             'screw_tool_m3').value
        self._screw_tool = ScrewTool(self, device_name)
        self.get_logger().info('started')

        threading.Thread(target=self.interactive, daemon=True).start()

    def interactive(self):
        while rclpy.ok():
            print('====')
            print('  q: quit this program')
            print('  t: tighten the screw')
            print('  l: loosen the screw')
            print('  g: grasp')
            print('  r: release')
            print('  m: set min period')
            print('  w: wait for tightening/loosening completed')
            print('  c: cancel tightening/loosening')
            print('  s: set tool speed')

            key = input('[speed: %f]> ' % self._screw_tool.parameters['speed'])

            if key == 'q':
                break
            elif key == 't':
                self._screw_tool.tighten(timeout_sec=0.0)
            elif key == 'l':
                self._screw_tool.loosen(timeout_sec=0.0)
            elif key == 'g':
                self._screw_tool.grasp(timeout_sec=0.0)
            elif key == 'r':
                self._screw_tool.release()
            elif key == 'm':
                suck_min_period = float(input('  suck_min_period? '))
                self._screw_tool.parameters = {'suck_min_period':
                                               suck_min_period}
            elif key == 'w':
                status, result = self._screw_tool.wait()
                print(result)
            elif key == 'c':
                self._screw_tool.cancel_goal()
            elif key == 's':
                speed = np.clip(float(input('  speed? ')), 0.0, 1.0)
                self._screw_tool.parameters['speed'] = speed
            else:
                print('Unknown command[%s]' % key)
        self.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    test = TestScrewTool('test_screw_tool')
    rclpy.spin(test)
