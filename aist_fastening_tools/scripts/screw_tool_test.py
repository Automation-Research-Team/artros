#!/usr/bin/env python

import rospy
import numpy as np
from actionlib                import SimpleActionClient
from actionlib_msgs.msg       import GoalStatus
from aist_fastening_tools.msg import (ScrewToolCommandAction,
                                      ScrewToolCommandGoal)
from aist_utility.compat      import *


class ScrewToolTest(object):
    def __init__(self, controller_ns):
        self._client = SimpleActionClient(controller_ns + '/command',
                                          ScrewToolCommandAction)
        self._client.wait_for_server()

    def run(self):
        speed = 1.0

        while not rospy.is_shutdown():
            key = raw_input('[speed: %d]> ' % speed)

            print('====')
            print('  q: quit this program')
            print('  t: tighten the screw')
            print('  l: loosen the screw')
            print('  w: wait for tightening/loosening completed')
            print('  c: cancel tightening/loosening')
            print('  s: set tool speed')

            if key == 'q':
                break
            elif key == 't':
                self._send_command(speed, True)
            elif key == 'l':
                self._send_command(-speed, False)
            elif key == 'w':
                self._wait()
            elif key == 'c':
                self._client.cancel_goal()
            elif key == 's':
                speed = np.clip(float(raw_input('  speed? ')), 0.0, 1.0)
            else:
                print('Unknown command[%s]' % key)

    def _send_command(self, speed, tighten):
        self._client.send_goal(ScrewToolCommandGoal(speed, tighten))

    def _wait(self, timeout=rospy.Duration(10)):
        self._client.wait_for_result(timeout)
        status = self._client.get_state()
        if status == GoalStatus.SUCCEEDED:
            print("  succeeded")


if __name__ == '__main__':
    try:
        rospy.init_node('screw_tool_test')

        controller_ns = rospy.get_param('~controller_ns',
                                        'screw_tool_m3_fastening_controller')
        test = ScrewToolTest(controller_ns)
        test.run()

    except rospy.ROSInterruptException:
        print('program interrupted before completion')
