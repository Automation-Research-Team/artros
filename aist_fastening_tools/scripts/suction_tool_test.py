#! /usr/bin/env python

import sys
import rospy
from actionlib                import SimpleActionClient
from aist_fastening_tools.msg import (SuctionToolControlAction,
                                      SuctionToolControlGoal)

class SuctionToolControllerClient:
    def suction(self, name, turn_suction_on, eject_screw):
        client = SimpleActionClient('suction_tool_controller/suction_tool_control',
                                    SuctionToolControlAction)
        client.wait_for_server()
        goal = SuctionToolControlGoal()

        goal.fastening_tool_name = name
        goal.turn_suction_on     = turn_suction_on
        goal.eject_screw         = eject_screw

        client.send_goal_and_wait(goal,rospy.Duration(30), rospy.Duration(10))
        client.wait_for_result()

        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('suction_tool_test')

        controller = SuctionToolControllerClient()

        name_list = [
            "screw_tool_m4"
        ]

        for name in name_list:
            res = controller.suction(name, True, False)
            rospy.sleep(10)
            res = controller.suction(name, False, False)

            if not res.success:
                rospy.logerr("Can not pick screw")

            rospy.sleep(1)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
