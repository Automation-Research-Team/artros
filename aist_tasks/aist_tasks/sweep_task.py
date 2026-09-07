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
from rclpy.node                  import Node
from rclpy.callback_groups       import MutuallyExclusiveCallbackGroup
from task_wrappers.action_server import ActionServer
from task_wrappers.action_client import GroupedSimpleActionClient
from aist_msgs.action            import Sweep

#*********************************************************************
#  class SweepTaskClient                                             *
#*********************************************************************
class SweepTaskClient(GroupedSimpleActionClient):
    def __init__(self, node: Node, server_ns: str='sweep'):
        super().__init__(node, Sweep, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def send_goal(self, robot_name, pose, sweep_length, sweep_offset,
                  approach_offset, departure_offset, speed_fast, speed_slow,
                  *, timeout_sec=0.0):
        return super().send_goal(Sweep.Goal(robot_name=robot_name, pose=pose,
                                            sweep_length=sweep_length,
                                            sweep_offset=sweep_offset,
                                            approach_offset=approach_offset,
                                            departure_offset=departure_offset,
                                            speed_fast=speed_fast,
                                            speed_slow=speed_slow),
                                 feedback_callback=self.stage_feedback_cb,
                                 timeout_sec=timeout_sec)

#*********************************************************************
#  class SweepTaskServer                                             *
#*********************************************************************
class SweepTaskServer(ActionServer):
    def __init__(self, node, server_ns='sweep'):
        super().__init__(node, Sweep, server_ns, self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def _execute_cb(self, goal_handle):
        request = goal_handle.request
        node    = self.node
        stop    = lambda: node.stop(request.robot_name)
        # [1] Move stage: Go to approach pose.
        with ActionServer.Stage(self, goal_handle, 'move', stop) as stage:
            success = node.go_to_pose_goal(request.robot_name, request.pose,
                                           request.approach_offset,
                                           request.speed_fast)
            if not success:
                raise ActionServer.Error('Failed to go to approach pose',
                                         stage=stage.name)

        # [2] Approach stage: Go to sweep pose.
        with ActionServer.Stage(self, goal_handle, 'approach', stop) as stage:
            success = node.go_to_pose_goal(request.robot_name, request.pose,
                                           request.sweep_offset,
                                           request.speed_slow)
            if not success:
                raise ActionServer.Error('Failed to go to sweep pose',
                                         stage=stage.name)

        # [3] Sweep stage: Sweep the object.
        with ActionServer.Stage(self, goal_handle, 'sweep', stop) as stage:
            offset = list(request.sweep_offset)
            offset[1] += request.sweep_length
            success = node.go_to_pose_goal(request.robot_name, request.pose,
                                           offset, request.speed_fast)
            if not success:
                raise ActionServer.Error('Failed to sweep', stage=stage.name)

        # [4] Depart stage: Go back to departure pose.
        with ActionServer.Stage(self, goal_handle, 'depart', stop) as stage:
            success = node.go_to_pose_goal(request.robot_name, request.pose,
                                           request.departure_offset,
                                           request.speed_fast)
            if not success:
                raise ActionServer.Error('Failed to go to departure pose',
                                         stage=stage.name)

        # [Final] Goal succeeded.
        goal_handle.succeed()
        return Sweep.Result(stage='')

#************************************************************************
#  class SweepTask                                                      *
#************************************************************************
class SweepTask(SweepTaskClient):
    def __init__(self, node, server_ns='sweep'):
        super().__init__(node, server_ns)
        self._server = SweepTaskServer(node, server_ns)
        self.wait_for_server()
