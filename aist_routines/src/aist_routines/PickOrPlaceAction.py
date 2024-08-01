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
from actionlib          import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from aist_msgs.msg      import (PickOrPlaceAction, PickOrPlaceGoal,
                                PickOrPlaceResult, PickOrPlaceFeedback)

######################################################################
#  class PickOrPlace                                                 #
######################################################################
class PickOrPlace(SimpleActionClient):
    def __init__(self, routines, ns='pick_or_place'):
        SimpleActionClient.__init__(self, ns, PickOrPlaceAction)

        self._routines      = routines
        self._current_stage = PickOrPlaceFeedback.IDLING
        self._target_stage  = None
        self._condition     = threading.Condition()
        self._server        = SimpleActionServer(ns, PickOrPlaceAction,
                                                 self._execute_cb, False)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self.wait_for_server()

    @property
    def current_stage(self):
        return self._current_stage

    # Client stuffs
    def send_goal(self, robot_name, pose, target_frame, pick, offset,
                  approach_offset, departure_offset, speed_fast, speed_slow,
                  object_name='', wait=True, done_cb=None, active_cb=None):
        self._current_stage = PickOrPlaceFeedback.IDLING
        self._target_stage  = PickOrPlaceFeedback.IDLING
        SimpleActionClient.send_goal(self,
                                     PickOrPlaceGoal(robot_name, object_name,
                                                     pose, pick,
                                                     offset, approach_offset,
                                                     departure_offset,
                                                     speed_fast, speed_slow),
                                     done_cb, active_cb, self._feedback_cb)
        if wait:
            self.wait_for_result()
            return self.get_result().result

    def cancel_goal(self):
        if self.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            SimpleActionClient.cancel_goal(self)

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
        rospy.loginfo('*** Do %s ***', 'picking' if goal.pick else 'placing')
        routines = self._routines
        gripper  = routines.gripper(goal.robot_name)
        feedback = PickOrPlaceFeedback()
        result   = PickOrPlaceResult()

        # Go to approach pose.
        rospy.loginfo('--- Go to approach pose. ---')
        feedback.stage = PickOrPlaceFeedback.MOVING
        self._server.publish_feedback(feedback)
        speed      = goal.speed_fast if goal.pick else goal.speed_slow
        success, _ = routines.go_to_pose_goal(goal.robot_name, goal.pose,
                                              goal.approach_offset, speed)
        if not self._server.is_active():
            return
        if not success:
            result.result = PickOrPlaceResult.MOVE_FAILURE
            self._server.set_aborted(result, 'Failed to go to approach pose')
            return

        # Go to pick/place pose.
        rospy.loginfo('--- Go to %s pose. ---',
                      'pick' if goal.pick else 'place')
        feedback.stage = PickOrPlaceFeedback.APPROACHING
        self._server.publish_feedback(feedback)
        if goal.pick:
            gripper.pregrasp()                  # Pregrasp (not wait)
            gripper.wait()                      # Wait for pregrasp completed
        success, _ = routines.go_to_pose_goal(goal.robot_name, goal.pose,
                                              goal.offset, goal.speed_slow)
        if not self._server.is_active():
            return
        if not success:
            result.result = PickOrPlaceResult.APPROACH_FAILURE
            self._server.set_aborted(result, 'Failed to approach target')
            if goal.pick:
                gripper.release()
            return

        # Grasp/release at pick/place pose.
        feedback.stage = PickOrPlaceFeedback.GRASPING_OR_RELEASING
        self._server.publish_feedback(feedback)

        if goal.pick:
            success = gripper.grasp()
            if success and goal.object_name != '':
                object_pose = self._lookup_pose(gripper.tip_link,
                                                goal.pose.header.frame_id)
                if object_pose is None:
                    gripper.release()
                    result.result == PickOrPlaceResult.GRASP_OR_RELEASE_FAILURE
                    self._server.set_aborted(result,
                                             'Failed to lookup object pose')
                    return
                self.psi.move_attached_object(goal.object_name, object_pose,
                                              gripper.touch_links)
        else:
            success = gripper.release()

        # Go back to departure(pick) or approach(place) pose.
        rospy.loginfo('--- Go back to departure pose. ---')
        feedback.stage = PickOrPlaceFeedback.DEPARTING
        self._server.publish_feedback(feedback)
        if goal.pick:
            gripper.postgrasp()                 # Postgrasp (not wait)
            offset = goal.departure_offset
            speed  = goal.speed_slow
        else:
            offset = goal.approach_offset
            speed  = goal.speed_fast
        success, _ = routines.go_to_pose_goal(goal.robot_name,
                                              goal.pose, offset, speed)
        if not self._server.is_active():
            return
        if not success:
            gripper.release()
            result.result = PickOrPlaceResult.DEPARTURE_FAILURE
            self._server.set_aborted(result, 'Failed to depart from target')
            return

        if goal.pick and not gripper.wait():    # Wait for postgrasp completed
            gripper.release()
            result.result = PickOrPlaceResult.GRASP_OR_RELEASE_FAILURE
            self._server.set_aborted(result, 'Failed to grasp')
            rospy.logwarn('--- Pick failed. ---')
            return

        result.result = PickOrPlaceResult.SUCCESS
        self._server.set_succeeded(result, 'Succeeded')
        rospy.loginfo('--- %s succeeded. ---',
                      'Pick' if goal.pick else 'Place')

    def _preempt_cb(self):
        goal = self._server.current_goal.get_goal()
        self._routines.stop(goal.robot_name)
        self._routines.gripper(goal.robot_name).release()
        self._server.set_preempted(PickOrPlaceResult(
                                       PickOrPlaceResult.PREEMPTED))
        rospy.logwarn('--- %s cancelled. ---',
                      'Pick' if goal.pick else 'Place')

    def _lookup_pose(self, frame_id, child_frame_id):
        try:
            t, q = self._routines.listener.lookupTransform(frame_id,
                                                           child_frame_id,
                                                           rospy.Time(0))
        except Exception as e:
            rospy.logerr('PickOrPlaceAction._lookup_transform(): %s', str(e))
            return None
        return PoseStamped(Header(frame_id=frame_id),
                           Pose(Point(*t), Quaternion(*q)))
