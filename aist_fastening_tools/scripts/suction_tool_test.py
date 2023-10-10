#!/usr/bin/env python

import rospy
from actionlib                import SimpleActionClient
from aist_fastening_tools.msg import (SuctionToolCommandAction,
                                      SuctionToolCommandGoal)
from aist_utility.compat      import *


class SuctionToolTest(object):
    def __init__(self, controller_ns):
        self._client = SimpleActionClient(controller_ns + '/command',
                                          SuctionToolCommandAction)
        self._client.wait_for_server()

    def run(self):
        suck       = False
        min_period = 0.5

        while not rospy.is_shutdown():
            key = raw_input('[suck=%d, min_period=%f]> ' % (suck, min_period))

            print('====')
            print('  q: quit this program')
            print('  s: toggle suction')
            print('  m: set min period')
            print('  w: wait for ten seconds')
            print('  c: cancel')

            if key == 'q':
                break
            elif key == 's':
                suck = not suck
                self._send_command(suck, min_period)
            elif key == 'm':
                min_period = float(raw_input('  min_period? '))
            elif key == 'w':
                self._wait()
            elif key == 'c':
                self._client.cancel_goal()
            else:
                print('Unknown command[%s]' % key)

    def _send_command(self, suck, min_period):
        self._client.send_goal(
            SuctionToolCommandGoal(suck, rospy.Duration(min_period)))

    def _wait(self, timeout=rospy.Duration(5)):
        self._client.wait_for_result(timeout)
        status = self._client.get_state()
        if status == GoalStatus.SUCCEEDED:
            print("  succeeded")
        elif status == GoalStatus.ABORTED:
            print("  aborted")
        elif status == GoalStatus.PREEMPTED:
            print("  preempted")


if __name__ == '__main__':
    try:
        rospy.init_node('suction_tool_test')

        controller_ns = rospy.get_param('~controller_ns',
                                        'screw_tool_m3_controller')
        test = SuctionToolTest(controller_ns)
        test.run()

    except rospy.ROSInterruptException:
        print('program interrupted before completion')
