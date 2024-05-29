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
from actionlib                 import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg        import GoalStatus
from geometry_msgs.msg         import Transform, Vector3, Quaternion
from aist_routines.SweepAction import Sweep
from aist_msgs.msg             import (RepeatSweepAction, RepeatSweepGoal,
                                       RepeatSweepResult, RepeatSweepFeedback,
                                       UpsetResult, SweepResult)
from tf                        import transformations as tfs

######################################################################
#  class RepeatSweep                                                 #
######################################################################
class RepeatSweep(SimpleActionClient):
    def __init__(self, routines):
        SimpleActionClient.__init__(self, 'repeat_sweep', RepeatSweepAction)

        self._sweep_clnt = Sweep(routines)
        self._server     = SimpleActionServer("repeat_sweep", RepeatSweepAction,
                                              self._execute_cb, False)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self.wait_for_server()

    # Client stuffs
    def send_goal(self, robot_name, pose, sweep_length, sweep_offset,
                  approach_offset, departure_offset, speed_fast, speed_slow,
                  angle_range, done_cb=None, active_cb=None):
        SimpleActionClient.send_goal(self,
                                     RepeatSweepGoal(robot_name, pose,
                                                     sweep_length,
                                                     sweep_offset,
                                                     approach_offset,
                                                     departure_offset,
                                                     speed_fast, speed_slow,
                                                     anrgle_range),
                                     done_cb, active_cb, self._feedback_cb)

    def _feedback_cb(self, feedback):
        success = raw_input('  *** sweep success?')
        self._upset_results += UpsetResult(feedback.direction, success)
        self._current_stage = feedback.stage
        if self._current_stage == self._target_stage:
            with self._condition:
                self._condition.notifyAll()

    # Server stuffs
    def _execute_cb(self, goal):
        rospy.loginfo('*** Do sweeping ***')
        routines = self._routines
        feedback = RepeatSweepFeedback()
        result   = RepeatSweepResult(SweepResult.Success, [])

        direction = goal.direction_range[0]
        while direction < goal.direction_range[1]:
            if not self._server.is_active():
                return

            pose = copy.deepcopy(goal.pose)
            pose.pose.orientation = tfs.quaternion_mutiply(
                                        pose.pose.orientation,
                                        tfs.quaternion_about_axis(
                                            np.radians(direction), (0, 0, 1)))
            sweep_result = self._sweep_clnt.send_goal(goal.robot_name, pose,
                                                      goal.sweep_length,
                                                      goal.sweep_offset,
                                                      goal.approach_offset,
                                                      goal.departure_offset,
                                                      goal.speed_fast,
                                                      goal.speed.slow, True)
            if result != SweepResult.SUCCESS:
                result.result = sweep_result
                result.upset_results = []
                self._server.set_aborted(result, 'Failed to execute sweep')
                return


            direction += goal.direction_range[2]

        # Go to approach pose.
        rospy.loginfo("--- Go to approach pose. ---")
        feedback.stage = RepeatSweepFeedback.MOVING
        self._server.publish_feedback(feedback)
        success, _ = routines.go_to_pose_goal(goal.robot_name,
                                              goal.pose, goal.approach_offset,
                                              goal.speed_fast)
        if not self._server.is_active():
            return
        if not success:
            result.result = RepeatSweepResult.MOVE_FAILURE
            self._server.set_aborted(result, "Failed to go to approach pose")
            return

        # Approach sweep pose.
        feedback.stage = RepeatSweepFeedback.APPROACHING
        self._server.publish_feedback(feedback)
        success, _ = routines.go_to_pose_goal(goal.robot_name,
                                              goal.pose, goal.sweep_offset,
                                              goal.speed_slow)
        if not self._server.is_active():
            return
        if not success:
            result.result = RepeatSweepResult.APPROACH_FAILURE
            self._server.set_aborted(result, "Failed to approach target")
            return

        # RepeatSweep.
        rospy.loginfo("--- RepeatSweep. ---")
        feedback.stage = RepeatSweepFeedback.SWEEPING
        self._server.publish_feedback(feedback)
        offset = list(goal.sweep_offset)
        offset[1] += goal.sweep_length
        success, _ = routines.go_to_pose_goal(goal.robot_name,
                                              goal.pose, offset,
                                              goal.speed_fast)
        if not self._server.is_active():
            return
        if not success:
            result.result = RepeatSweepResult.SWEEP_FAILURE
            self._server.set_aborted(result, "Failed to sweep")
            return

        # Go back to departure(pick) or approach(place) pose.
        rospy.loginfo("--- Go back to departure pose. ---")
        feedback.stage = RepeatSweepFeedback.DEPARTING
        self._server.publish_feedback(feedback)
        success, _ = routines.go_to_pose_goal(goal.robot_name,
                                              goal.pose, goal.departure_offset,
                                              goal.speed_fast)
        if not self._server.is_active():
            return
        if not success:
            result.result = RepeatSweepResult.DEPARTURE_FAILURE
            self._server.set_aborted(result, "Failed to depart from target")
            return

        result.result = RepeatSweepResult.SUCCESS
        self._server.set_succeeded(result, "Succeeded")

    def _preempt_cb(self):
        goal = self._server.current_goal.get_goal()
        self._routines.stop(goal.robot_name)
        self._server.set_preempted(RepeatSweepResult(RepeatSweepResult.PREEMPTED))
        rospy.logwarn('--- RepeatSweep cancelled. ---')
