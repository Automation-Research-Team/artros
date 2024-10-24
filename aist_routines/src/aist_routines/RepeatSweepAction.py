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
import rospy, copy, numpy as np
from geometry_msgs.msg         import Quaternion
from tf                        import transformations as tfs
from actionlib                 import SimpleActionServer, SimpleActionClient
from std_srvs.srv              import Trigger, TriggerResponse
from aist_routines.SweepAction import Sweep
from aist_msgs.msg             import (RepeatSweepAction, RepeatSweepGoal,
                                       RepeatSweepResult, RepeatSweepFeedback,
                                       UpsetResult, SweepResult)
from aist_utility.compat       import *

######################################################################
#  class RepeatSweep                                                 #
######################################################################
class RepeatSweep(SimpleActionClient):
    def __init__(self, routines):
        SimpleActionClient.__init__(self, 'repeat_sweep', RepeatSweepAction)

        self._routines = routines
        self._current_robot_name = None
        self._sweep  = Sweep(routines)
        self._server = SimpleActionServer("repeat_sweep", RepeatSweepAction,
                                          self._execute_cb, False)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self.wait_for_server()

        self._query_success_srv = rospy.Service('query_success', Trigger,
                                                  self._query_success_cb)
        self._query_success     = rospy.ServiceProxy('query_success', Trigger)

    @property
    def current_robot_name(self):
        return self._current_robot_name

    def _query_success_cb(self, req):
        rospy.logwarn('### upset success? ')
        res = TriggerResponse()
        res.success = (raw_input() == 'y')
        return res

    # Client stuffs
    def send_goal(self, robot_name, pose, sweep_length, sweep_offset,
                  approach_offset, departure_offset, speed_fast, speed_slow,
                  direction_range, done_cb=None, active_cb=None):
        SimpleActionClient.send_goal(self,
                                     RepeatSweepGoal(robot_name, pose,
                                                     sweep_length,
                                                     sweep_offset,
                                                     approach_offset,
                                                     departure_offset,
                                                     speed_fast, speed_slow,
                                                     direction_range),
                                     done_cb, active_cb)

    # Server stuffs
    def _execute_cb(self, goal):
        rospy.loginfo('*** Repeat sweeping ***')

        # If using a different robot from the former, move it back to home.
        if self.current_robot_name is not None and \
           self.current_robot_name != goal.robot_name:
            routines.go_to_named_pose(self.current_robot_name, 'back')
        self._current_robot_name = goal.robot_name

        result    = RepeatSweepResult(SweepResult.SUCCESS, [])
        direction = goal.direction_range[0]
        while direction < goal.direction_range[1]:
            if not self._server.is_active():
                return

            pose = copy.deepcopy(goal.pose)
            pose.pose.orientation = Quaternion(
                                        *tfs.quaternion_multiply(
                                            (pose.pose.orientation.x,
                                             pose.pose.orientation.y,
                                             pose.pose.orientation.z,
                                             pose.pose.orientation.w),
                                            tfs.quaternion_about_axis(
                                                np.radians(direction),
                                                (0, 0, 1))))
            sweep_result = self._sweep.send_goal(goal.robot_name, pose,
                                                 goal.sweep_length,
                                                 goal.sweep_offset,
                                                 goal.approach_offset,
                                                 goal.departure_offset,
                                                 goal.speed_fast,
                                                 goal.speed_slow, True)
            if sweep_result != SweepResult.SUCCESS:
                result.result = sweep_result
                result.upset_results = []
                self._server.set_aborted(result, 'Failed to execute sweep')
                return

            upset_success = self._query_success().success
            rospy.logwarn('### upset success=' + str(upset_success))
            result.upset_results.append(UpsetResult(direction, upset_success))

            direction += goal.direction_range[2]

        self._server.set_succeeded(result, "Succeeded")

    def _preempt_cb(self):
        goal = self._server.current_goal.get_goal()
        self._routines.stop(goal.robot_name)
        self._server.set_preempted(RepeatSweepResult(SweepResult.PREEMPTED,
                                                     []))
        rospy.logwarn('--- RepeatSweep cancelled. ---')
