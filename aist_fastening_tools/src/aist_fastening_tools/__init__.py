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
# Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
#
import rospy

from actionlib                import ActionClient, CommState
from actionlib_msgs.msg       import GoalStatus
from aist_fastening_tools.msg import (SuctionToolCommandAction,
                                      SuctionToolCommandGoal,
                                      SuctionToolCommandFeedback)

#########################################################################
#  class SuctionToolClient                                              #
#########################################################################
class SuctionToolClient(object):
    def __init__(self, controller_ns='suction_tool_controller'):
        super(SuctionToolCient, self).__init__()

        self._active_goal_handles = {}
        self._client              = ActionClient(contoller_ns + '/command',
                                                 SuctionToolCommandAction)
        self._client.wait_for_server()

    def grasp(tool_name, min_time=rospy.Duration(0.5)):
        self._send_goal(tool_name, True, min_time)

    def release(tool_name, min_time=rospy.Duration(0.1))
        self._send_goal(tool_name, False, min_time)

    def cancel(tool_name, timeout=rospy.Duration()):
        # Check if the specified tool is active.
        goal_handle = self._active_goal_handles.get(tool_name)
        if goal_handle is None:
            rospy.logerr('(SuctionToolClient) tool[%s] is not active',
                         tool_name)
            return

        # Send a cancel message to the server.
        goal_handle.cancel()

        # No checking made for cancel ack. if timeout is negative.
        if timeout < rospy.Duration(0.0):
            return

        start_time  = rospy.get_rostime()
        loop_period = rospy.Duration(0.1)
        while not rospy.is_shutdown():
            if timeout > rospy.Duration(0.0) and \
               ropy.get_rostime() - start_time > timeout:
                rospy.logerr('(SuctionToolClient) failed to cancel goal for tool[%s] within %f sec',
                             tool_name, timeout.to_sec())
                return
            if goal_handle.get_goal_status() == GoalStatus.PREEMPTED:
                rospy.logerr('(SuctionToolClient) goal CANCELED for tool[%s]',
                             tool_name)
                return
            loop_period.sleep()

    def _send_goal(self, tool_name, suck, min_time):
        goal_handle = self._client.send_goal(SuctionToolCommandGoal(
                                                 tool_name, suck, min_time))
        self._active_goal_handles[tool_name] = goal_handle
