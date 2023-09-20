#! /usr/bin/env python

import sys
import rospy
from actionlib                import SimpleActionClient
from aist_fastening_tools.msg import (ScrewToolCommandAction,
                                      ScrewToolCommandGoal)


class ScrewToolController:
    def fasten(self, name):
        client = SimpleActionClient('screw_tool_controller/command',
                                    ScrewToolCommandAction)
        client.wait_for_server()

        goal = ScrewToolCommandGoal()
        goal.fastening_tool_name = name
        goal.speed = 100
        goal.direction = "tighten"

        client.send_goal_and_wait(goal, rospy.Duration(10), rospy.Duration(10))
        client.wait_for_result()

        return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('screw_tool_test')

        controller = ScrewToolController()

        name_list = ["screw_tool_m4", "screw_tool_m3"]

        for name in name_list:
            if not controller.fasten(name):
                break

        print('')

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
