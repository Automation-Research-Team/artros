#!/usr/bin/env python

import sys, rospy
from actionlib                import SimpleActionClient
from aist_fastening_tools.msg import (SuctionToolCommandAction,
                                      SuctionToolCommandGoal)
from aist_utility.compat      import *


class SuctionToolTest(object):
    def __init__(self):
        self._client = SimpleActionClient('screw_tool_controller/command',
                                          SuctionToolCommandAction)
        self._client.wait_for_server()

    def run(self):
        tool_name = 'screw_tool_m3'
        suction   = False
        eject     = False

        while not rospy.is_shutdown():
            key = raw_input('%s:(suction=%d, eject=%d)> '
                            % (tool_name, suction, eject))

            print('====')
            print('  q: quit this program')
            print('  s: toggle suction')
            print('  e: toggle eject')
            print('  3: switch the tool to m3')
            print('  4: switch the tool to m4')
            print('  6: switch the tool to m6')
            print('  S: switch the tool to suction_tool')

            if key == 'q':
                break
            elif key == 's':
                suction = not suction
                success = self._send_command(tool_name, suction, eject)
                print('  %s' % 'succeded' if success else 'failed')
            elif key == 'e':
                eject   = not eject
                success = self._send_command(tool_name, suction, eject)
                print('  %s' % 'succeded' if success else 'failed')
            elif key == '3':
                tool_name = 'screw_tool_m3'
            elif key == '4':
                tool_name = 'screw_tool_m4'
            elif key == '6':
                tool_name = 'screw_tool_m6'
            elif key == 'S':
                tool_name = 'suction_tool'
            else:
                print('Unknown command[%s]' % key)

    def _send_command(self, tool_name, suction, eject):
        goal = SuctionToolCommandGoal()
        goal.fastening_tool_name = tool_name
        goal.turn_suction_on     = suction
        goal.eject               = eject
        self._client.send_goal_and_wait(goal,
                                        rospy.Duration(30), rospy.Duration(10))
        self._client.wait_for_result()

        return self._client.get_result().success


if __name__ == '__main__':
    try:
        rospy.init_node('suction_tool_test')

        test = SuctionToolTest()
        test.run()

    except rospy.ROSInterruptException:
        print('program interrupted before completion', file=sys.stderr)
