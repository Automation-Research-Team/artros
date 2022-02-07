#!/usr/bin/env python
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
import rospy, threading
from geometry_msgs import msg as gmsg
from control_msgs  import msg as cmsg
from actionlib     import SimpleActionClient

######################################################################
#  class PointHeadClient                                             #
######################################################################
class PointHeadClinet(object):
    """Implements kitting routines for aist robot system."""

    def __init__(self, server):
        super(PointHeadClinet, self).__init__()

        self._point_head_client = SimpleActionClient(server + '/point_head',
                                                     cmsg.PointHeadAction)
        thread = threading.Thread(target=self._interactive)
        thread.start()

    def send_goal(self,
                  frame_id, min_duration, max_velocity, feedback_cb=None):
        goal = cmsg.PointHeadGoal()
        goal.target.header.stamp    = rospy.Time.now()
        goal.target.header.frame_id = frame_id
        goal.target.point           = gmsg.Point(x=0, y=0, z=0)
        goal.pointing_axis          = gmsg.Vector3(x=0, y=0, z=-1)
        goal.pointing_frame         = ''
        goal.min_duration           = min_duration
        goal.max_velocity           = max_velocity

        self._point_head_client.send_goal(goal, feedback_cb=feedback_cb)

    def wait_for_result(self, timeout=rospy.Duration()):
        if not self._point_head_client.wait_for_result(timeout):
            self._point_head_client.cancel_goal()  # timeout expired
            return False
        return self._point_head_client.get_state() == GoalStatus.SUCCEEDED

    def cancel_goal(self):
        self._point_head_client.cancel_goal()

    def _interactive(self):
        while not rospy.is_shutdown():
            try:
                key = raw_input('>> ')
                if key == 'q':
                    break
                self.send_goal()
            except Exception as e:
                print(e.message)


if __name__ == '__main__':

    rospy.init_node('point_head_client', anonymous=True)

    server = '/biclops_point_head_tracker'
    point_head_client = PointHeadClient(server)
    rospy.spin()
