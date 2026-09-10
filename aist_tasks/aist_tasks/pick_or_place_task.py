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
import numpy as np
import tf_transformations as tfs

from rclpy.action               import GoalResponse, CancelResponse
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup
from action_msgs.msg            import GoalStatus
from aist_msgs.action           import PickOrPlace
from geometry_msgs.msg          import Point, Quaternion, Pose, PoseStamped
from task_wrappers              import ActionServer, GroupedSimpleActionClient
from aist_utility.geometry_msgs import (pose_matrix, pose_from_matrix,
                                        pose_from_xyzrpy)

#************************************************************************
#  class PickOrPlaceTaskClient                                          *
#************************************************************************
class PickOrPlaceTaskClient(GroupedSimpleActionClient):
    def __init__(self, node, server_ns='pick_or_place'):
        super().__init__(node, PickOrPlace, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def send_goal(self, robot_name, pick, pose, offset,
                  approach_offset, departure_offset, speed_fast, speed_slow,
                  *, end_effector_link='', timeout_sec=0.0):
        return super().send_goal(PickOrPlace.Goal(
                                     robot_name=robot_name,
                                     pick=pick,
                                     pose=pose,
                                     offset=offset,
                                     approach_offset=approach_offset,
                                     departure_offset=departure_offset,
                                     speed_fast=speed_fast,
                                     speed_slow=speed_slow,
                                     end_effector_link=end_effector_link),
                                 feedback_callback=self.stage_feedback_cb,
                                 timeout_sec=timeout_sec)

#************************************************************************
#  class PickOrPlaceTaskServer                                          *
#************************************************************************
class PickOrPlaceTaskServer(ActionServer):
    def __init__(self, node, server_ns='pick_or_place'):
        super().__init__(node, PickOrPlace, server_ns, self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def _execute_cb(self, goal_handle):
        def _get_object_id(link_name):
            tokens = link_name.rsplit('/', 1)
            return tokens[0] if len(tokens) == 2 else ''

        def _offset_from_matrix(T):
            return (*tfs.translation_from_matrix(T),
                    *tfs.quaternion_from_matrix(T))

        request = goal_handle.request
        node    = self.node
        com     = node.com
        gripper = node.gripper(request.robot_name)
        if request.pick:
            object_id   = _get_object_id(request.pose.header.frame_id)
            eef_link    = ''
        else:
            object_id   = _get_object_id(request.end_effector_link)
            eef_link    = request.end_effector_link
        old_root_id = ''

        try:
            stop = lambda: node.stop(request.robot_name)

            # [1] 'move' stage: Go to approach pose.
            with ActionServer.Stage(self, goal_handle, 'move', stop) as stage:
                speed   = request.speed_fast if request.pick else \
                          request.speed_slow
                success = node.go_to_pose_goal(request.robot_name,
                                               request.pose,
                                               request.approach_offset, speed,
                                               end_effector_link=eef_link)
                if not success:
                    raise ActionServer.Error('Failed to go to approach pose',
                                             stage=stage.name)

            # [2] 'approach' stage: Go to pick/place pose.
            with ActionServer.Stage(self, goal_handle, 'approach',
                                    stop) as stage:
                if request.pick:
                    gripper.pregrasp()  # Pregrasp (not wait)
                    gripper.wait()      # Wait for pregrasp completed
                    if object_id != '':
                        com.allow_collision(object_id, gripper.tip_link)
                elif object_id != '':
                    com.allow_collision(object_id,
                                        request.pose.header.frame_id)

                success = node.go_to_pose_goal(request.robot_name,
                                               request.pose,
                                               request.offset,
                                               request.speed_slow,
                                               end_effector_link=eef_link)
                if not success:
                    if request.pick:
                        gripper.release()
                    raise ActionServer.Error('Failed to approach target',
                                             stage=stage.name)

            # [3] 'grasp/release' stage: Grasp or release at pick/place pose.
            with ActionServer.Stage(self, goal_handle,
                                    'grasp' if request.pick else
                                    'release') as stage:
                if request.pick:
                    gripper.grasp()
                    if object_id != '':
                        old_root_id, old_root_pose \
                            = com.attach_object(object_id, gripper.tip_link)
                else:
                    gripper.release()
                    if object_id != '':
                        old_root_id, old_root_pose \
                            = com.detach_object(
                                object_id, request.pose.header.frame_id,
                                _get_object_id(gripper.tip_link))

            # [4] 'depart' stage: Go back to departure(pick) or approach(place)
            #      pose.
            with ActionServer.Stage(self, goal_handle, 'depart',
                                    stop) as stage:
                if request.pick:
                    gripper.postgrasp()                 # Postgrasp (not wait)
                    speed  = request.speed_slow
                    offset = request.departure_offset
                else:
                    speed = request.speed_fast
                    if object_id != '':
                        offset = _offset_from_matrix(
                                     pose_matrix(
                                         pose_from_xyzrpy(
                                             request.approach_offset)) @
                                     tfs.inverse_matrix(
                                         pose_matrix(old_root_pose.pose) @
                                         pose_matrix(
                                             com.relative_frame_pose(
                                                 eef_link, old_root_id).pose)))
                    else:
                        offset = request.approach_offset
                success = node.go_to_pose_goal(request.robot_name,
                                               request.pose, offset, speed)
                if not success:
                    if request.pick:
                        gripper.release()
                        if object_id != '':
                            com.detach_object(old_root_id,
                                              old_root_pose.header.frame_id,
                                              _get_object_id(gripper.tip_link))
                            com.move_object(old_root_id, old_root_pose)
                    raise ActionServer.Error('Failed to depart from target',
                                             stage=stage.name)

            # [5] 'verify' stage: Verify success of postgrasp.
            with ActionServer.Stage(self, goal_handle, 'verify') as stage:
                if request.pick and \
                   not node.get_parameter('use_sim_time') \
                           .get_parameter_value().bool_value and \
                   not gripper.grasped():
                    gripper.release()
                    if object_id != '':
                        com.detach_object(old_root_id,
                                          old_root_pose.header.frame_id,
                                          _get_object_id(gripper.tip_link))
                        com.move_object(old_root_id, old_root_pose)
                    raise ActionServer.Error('Failed to grasp',
                                             stage=stage.name)

            # [Final] Goal succeeded.
            goal_handle.succeed()
            return PickOrPlace.Result(stage='')

        finally:
            if (object_id != ''):
                com.reset_collision(object_id)
                if (old_root_id != '' and old_root_id != object_id):
                    com.reset_collision(old_root_id)

#************************************************************************
#  class PickOrPlaceTask                                                *
#************************************************************************
class PickOrPlaceTask(PickOrPlaceTaskClient):
    def __init__(self, node, server_ns='pick_or_place'):
        super().__init__(node, server_ns)
        self._server = PickOrPlaceTaskServer(node, server_ns)
        self.wait_for_server()
