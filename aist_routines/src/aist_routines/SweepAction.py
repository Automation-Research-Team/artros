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
import rospy, threading, copy, numpy as np
from actionlib          import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg  import Transform, Vector3, Quaternion
from aist_msgs.msg      import (SweepAction, SweepGoal,
                                SweepResult, SweepFeedback)
from tf                 import transformations as tfs

######################################################################
#  class Sweep                                                       #
######################################################################
class Sweep(SimpleActionClient):
    def __init__(self, routines):
        SimpleActionClient.__init__(self, 'sweep', SweepAction)

        self._routines      = routines
        self._current_stage = SweepFeedback.IDLING
        self._target_stage  = None
        self._condition     = threading.Condition()
        self._server        = SimpleActionServer("sweep", SweepAction,
                                                 self._execute_cb, False)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self.wait_for_server()

    # Client stuffs
    def send_goal(self, robot_name, pose, sweep_length, sweep_offset,
                  approach_offset, departure_offset, speed_fast, speed_slow,
                  wait=True, done_cb=None, active_cb=None):
        SimpleActionClient.send_goal(self,
                                     SweepGoal(robot_name, pose, sweep_length,
                                               sweep_offset, approach_offset,
                                               departure_offset,
                                               speed_fast, speed_slow),
                                     done_cb, active_cb, self._feedback_cb)
        if wait:
            self.wait_for_result()
            return self.get_result().result

    def wait_for_stage(self, stage, timeout=rospy.Duration()):
        self._target_stage = stage          # Set stage to be waited for
        timeout_time = rospy.get_rostime() + timeout
        loop_period  = rospy.Duration(0.1)
        with self._condition:
            # Loop to avoid spurious wakeup
            while self._current_stage != self._target_stage:
                time_left = timeout_time - rospy.get_rostime()
                if timeout   >  rospy.Duration(0.0) and \
                   time_left <= rospy.Duration(0.0):
                    return False            # Timeout has expired
                if time_left > loop_period or timeout == rospy.Duration():
                    time_left = loop_period
                self._condition.wait(time_left.to_sec())
        return True

    def _feedback_cb(self, feedback):
        self._current_stage = feedback.stage
        if self._current_stage == self._target_stage:
            with self._condition:
                self._condition.notifyAll()

    # Server stuffs
    def shutdown(self):
        self._server.__del__()

    def _execute_cb(self, goal):
        rospy.loginfo("*** Do sweeping ***")
        routines = self._routines
        feedback = SweepFeedback()
        result   = SweepResult()

        # Go to approach pose.
        rospy.loginfo("--- Go to approach pose. ---")
        feedback.stage = SweepFeedback.MOVING
        self._server.publish_feedback(feedback)
        success, _ = routines.go_to_pose_goal(goal.robot_name,
                                              goal.pose, goal.approach_offset,
                                              goal.speed_fast)
        if not self._server.is_active():
            return
        if not success:
            result.result = SweepResult.MOVE_FAILURE
            self._server.set_aborted(result, "Failed to go to approach pose")
            return

        # Approach sweep pose.
        feedback.stage = SweepFeedback.APPROACHING
        self._server.publish_feedback(feedback)
        success, _ = routines.go_to_pose_goal(goal.robot_name,
                                              goal.pose, goal.sweep_offset,
                                              goal.speed_slow)
        if not self._server.is_active():
            return
        if not success:
            result.result = SweepResult.APPROACH_FAILURE
            self._server.set_aborted(result, "Failed to approach target")
            return

        # Sweep.
        rospy.loginfo("--- Sweep. ---")
        feedback.stage = SweepFeedback.SWEEPING
        self._server.publish_feedback(feedback)
        offset = list(goal.sweep_offset)
        offset[1] += goal.sweep_length
        success, _ = routines.go_to_pose_goal(goal.robot_name,
                                              goal.pose, offset,
                                              goal.speed_fast)
        if not self._server.is_active():
            return
        if not success:
            result.result = SweepResult.SWEEP_FAILURE
            self._server.set_aborted(result, "Failed to sweep")
            return

        # Go back to departure(pick) or approach(place) pose.
        rospy.loginfo("--- Go back to departure pose. ---")
        feedback.stage = SweepFeedback.DEPARTING
        self._server.publish_feedback(feedback)
        success, _ = routines.go_to_pose_goal(goal.robot_name,
                                              goal.pose, goal.departure_offset,
                                              goal.speed_fast)
        if not self._server.is_active():
            return
        if not success:
            result.result = SweepResult.DEPARTURE_FAILURE
            self._server.set_aborted(result, "Failed to depart from target")
            return

        result.result = SweepResult.SUCCESS
        self._server.set_succeeded(result, "Succeeded")

    def _preempt_cb(self):
        goal = self._server.current_goal.get_goal()
        self._routines.stop(goal.robot_name)
        self._server.set_preempted(SweepResult(SweepResult.PREEMPTED))
        rospy.logwarn('--- Sweep cancelled. ---')
