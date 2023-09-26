#!/usr/bin/env python

import sys, rospy
import numpy as np
from actionlib                import SimpleActionClient
from aist_fastening_tools.msg import (ScrewToolCommandAction,
                                      ScrewToolCommandGoal)
from aist_utility.compat      import *


class ScrewToolTest(object):
    def __init__(self):
        self._client = SimpleActionClient('screw_tool_controller/command',
                                          ScrewToolCommandAction)
        self._client.wait_for_server()

    def run(self):
        tool_name = 'screw_tool_m3'
        speed     = 100

        while not rospy.is_shutdown():
            key = raw_input('[%s:%d]> ' % (tool_name, speed))

            print('====')
            print('  q: quit this program')
            print('  t: tighten the screw')
            print('  l: loosen the screw')
            print('  s: set tool speed')
            print('  3: switch the tool to m3')
            print('  4: switch the tool to m4')

            if key == 'q':
                break
            elif key == 't':
                self._send_command(tool_name, 'tighten')
            elif key == 'l':
                self._send_command(tool_name, 'loosen')
            elif key == 's':
                speed = np.clip(int(raw_input('  speed? ')), 0, 1023)
            elif key == '3':
                tool_name = 'screw_tool_m3'
            elif key == '4':
                tool_name = 'screw_tool_m4'
            else:
                print('Unknown command[%s]' % key)

    def _send_command(self, tool_name, speed, direction):
        goal = ScrewToolCommandGoal()
        goal.fastening_tool_name = tool_name
        goal.speed               = speed
        goal.direction           = direction
        self._client.send_goal_and_wait(goal,
                                        rospy.Duration(10), rospy.Duration(10))
        self._client.wait_for_result()

        return self._client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('screw_tool_test')

        test = ScrewToolTest()
        test.run()

    except rospy.ROSInterruptException:
        print('program interrupted before completion', file=sys.stderr)
