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
import numpy as np
from actionlib          import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg  import Transform, Vector3, Quaternion
from aist_routines.msg  import (PickOrPlaceAction, PickOrPlaceGoal,
                                PickOrPlaceResult, PickOrPlaceFeedback)
from tf                 import transformations as tfs

######################################################################
#  class PickOrPlace                                                 #
######################################################################
class PickOrPlace(object):
    def __init__(self, routines):
        super(PickOrPlace, self).__init__()

        self._routines  = routines
        self._condition = threading.Condition()
        self._status    = PickOrPlaceFeedback.IDLING
        self._server    = SimpleActionServer("pickOrPlace", PickOrPlaceAction,
                                             self._execute_cb, False)
        self._server.start()
        self._client    = SimpleActionClient("pickOrPlace", PickOrPlaceAction)
        self._client.wait_for_server()

    # Client stuffs
    def execute(self, robot_name, pose_stamped, pick, offset,
                approach_offset, departure_offset, speed_fast, speed_slow,
                wait=True, feedback_cb=None):
        if not feedback_cb:
            feedback_cb = self._feedback_cb
        goal = PickOrPlaceGoal()
        goal.robot_name       = robot_name
        goal.pose             = pose_stamped
        goal.pick             = pick
        goal.offset           = self._create_transform(offset)
        goal.approach_offset  = self._create_transform(approach_offset)
        goal.departure_offset = self._create_transform(departure_offset)
        goal.speed_fast       = speed_fast
        goal.speed_slow       = speed_slow
        self._client.send_goal(goal, feedback_cb=feedback_cb)
        return self.wait_for_result() if wait else None

    def wait_for_result(self):
        self._client.wait_for_result()
        return self._client.get_result().result

    def cancel(self):
        if self._client.get_state() in ( GoalStatus.PENDING,
                                         GoalStatus.ACTIVE ):
            self._client.cancel_goal()

    def wait_for_status(self, status):
        self._status = status                   # Set target status.
        with self._condition:
            # Loop for avoiding spurious wakeup
            while self._status != PickOrPlaceFeedback.IDLING:
                self._condition.wait()

    def _feedback_cb(self, feedback):
        if self._status == feedback.status:     # Reached target status?
            with self._condition:
                self._status = PickOrPlaceFeedback.IDLING
                self._condition.notifyAll()

    # Server stuffs
    def shutdown(self):
        self._server.__del__()

    def _create_transform(self, offset):
        xyz = (0, 0, 0)    if len(offset) < 3 else offset[0:3]
        q   = (0, 0, 0, 1) if len(offset) < 6 else \
              tfs.quaternion_from_euler(
                  *np.radians(offset[3:6])) if len(offset) == 6 else \
              offset[3:7]
        return Transform(Vector3(*xyz), Quaternion(*q))

    def _execute_cb(self, goal):
        rospy.loginfo("*** Do %s ***", "picking" if goal.pick else "placing")
        routines = self._routines
        gripper  = routines.gripper(goal.robot_name)
        result   = PickOrPlaceResult()

        # Go to approach pose.
        rospy.loginfo("--- Go to approach pose. ---")
        if self._check_if_canceled(PickOrPlaceFeedback.MOVING):
            return
        scaling_factor = goal.speed_fast if goal.pick else goal.speed_slow
        success, _, _ = routines.go_to_pose_goal(
                             goal.robot_name,
                             routines.effector_target_pose(
                                 goal.pose,
                                 (goal.approach_offset.translation.x,
                                  goal.approach_offset.translation.y,
                                  goal.approach_offset.translation.z,
                                  goal.approach_offset.rotation.x,
                                  goal.approach_offset.rotation.y,
                                  goal.approach_offset.rotation.z,
                                  goal.approach_offset.rotation.w)),
                             scaling_factor, scaling_factor)
        if not success:
            result.result = PickOrPlaceResult.MOVE_FAILURE
            self._server.set_aborted(result, "Failed to go to approach pose")
            return

        # Approach pick/place pose.
        rospy.loginfo("--- Go to %s pose. ---",
                      "pick" if goal.pick else "place")
        if self._check_if_canceled(PickOrPlaceFeedback.APPROACHING):
            return
        if goal.pick:
            gripper.pregrasp(-1)               # Pregrasp (not wait)
        target_pose \
            = routines.effector_target_pose(goal.pose,
                                            (goal.offset.translation.x,
                                             goal.offset.translation.y,
                                             goal.offset.translation.z,
                                             goal.offset.rotation.x,
                                             goal.offset.rotation.y,
                                             goal.offset.rotation.z,
                                             goal.offset.rotation.w))
        routines.add_marker("pick_pose" if goal.pick else "place_pose",
                            target_pose)
        routines.publish_marker()
        success, _, _ = routines.go_to_pose_goal(goal.robot_name, target_pose,
                                                 goal.speed_slow)
        if not success:
            result.result = PickOrPlaceResult.APPROACH_FAILURE
            self._server.set_aborted(result, "Failed to approach target")
            if goal.pick:
                gripper.release()
            return

        # Grasp/release at pick/place pose.
        if self._check_if_canceled(PickOrPlaceFeedback.GRASPING_OR_RELEASING):
            return
        if goal.pick:
            gripper.wait()                      # Wait for pregrasp completed
            gripper.grasp()
        else:
            gripper.release()

        # Go back to departure(pick) or approach(place) pose.
        rospy.loginfo("--- Go back to departure pose. ---")

        if self._check_if_canceled(PickOrPlaceFeedback.DEPARTING):
            return
        if goal.pick:
            gripper.postgrasp(-1)    # Postgrap (not wait)
            offset         = goal.departure_offset
            scaling_factor = goal.speed_slow
        else:
            offset         = goal.approach_offset
            scaling_factor = goal.speed_fast
        success, _, _ = routines.go_to_pose_goal(goal.robot_name,
                                                 routines.effector_target_pose(
                                                     goal.pose,
                                                     (offset.translation.x,
                                                      offset.translation.y,
                                                      offset.translation.z,
                                                      offset.rotation.x,
                                                      offset.rotation.y,
                                                      offset.rotation.z,
                                                      offset.rotation.w)),
                                                 scaling_factor, scaling_factor)
        if not success:
            result.result = PickOrPlaceResult.DEPARTURE_FAILURE
            self._server.set_aborted(result, "Failed to depart from target")
            gripper.release()
            return

        if goal.pick and not gripper.wait():  # Wait for postgrasp completed
            rospy.logwarn("--- Pick failed. ---")
            result.result = PickOrPlaceResult.GRASP_FAILURE
            self._server.set_aborted(result, "Failed to grasp")
            gripper.release()
            return

        rospy.loginfo("--- %s succeeded. ---",
                      "Pick" if goal.pick else "Place")
        result.result = PickOrPlaceResult.SUCCESS
        self._server.set_succeeded(result, "Succeeded")

    def _check_if_canceled(self, state):
        self._server.publish_feedback(PickOrPlaceFeedback(state))
        if self._server.is_preempt_requested():
            self._server.set_preempted(PickOrPlaceResult(state))
            return True
        return False
