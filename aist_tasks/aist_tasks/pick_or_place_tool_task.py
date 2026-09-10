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
from rclpy.node            import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from action_msgs.msg       import GoalStatus
from aist_msgs.action      import PickOrPlaceTool
from task_wrappers         import ActionServer, GroupedSimpleActionClient

#*********************************************************************
#  class PickOrPlaceToolTaskClient                                   *
#*********************************************************************
class PickOrPlaceToolTaskClient(GroupedSimpleActionClient):
    def __init__(self, node: Node, server_ns: str='pick_or_place_tool'):
        super().__init__(node, PickOrPlaceTool, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def send_goal(self, robot_name, tool_name, *, timeout_sec=0.0):
        return super().send_goal(PickOrPlaceTool.Goal(robot_name=robot_name,
                                                      tool_name=tool_name),
                                 feedback_callback=self.stage_feedback_cb,
                                 timeout_sec=timeout_sec)

#*********************************************************************
#  class PickOrPlaceToolTaskServer                                   *
#*********************************************************************
class PickOrPlaceToolTaskServer(ActionServer):
    def __init__(self, node, server_ns='pick_or_place_tool'):
        super().__init__(node, PickOrPlaceTool, server_ns, self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def _execute_cb(self, goal_handle):
        request              = goal_handle.request
        node                 = self.node
        current_gname        = node.gripper(request.robot_name).name
        default_gname        = node.default_gripper_name(request.robot_name)
        pick_or_place_cancel = lambda: node.pick_or_place_cancel_goal(
                                           request.robot_name)

        if current_gname == request.tool_name:
            goal_handle.succeed()
            return PickOrPlaceTool.Result(stage='')

        if current_gname in node.tool_names:
            # [1] 'place' stage: Place current tool.
            with ActionServer.Stage(self, goal_handle, 'place',
                                    pick_or_place_cancel) as stage:
                node.set_gripper(request.robot_name, default_gname)
                status, result = node.place_at_frame(
                                     request.robot_name, current_gname,
                                     current_gname + '_holder_link',
                                     eef_link=current_gname + '/base_link')
                if status is GoalStatus.STATUS_ABORTED:
                    if result.stage in ('move', 'approach', 'release'):
                        node.set_gripper(request.robot_name, current_gname)
                    raise ActionServer.Error('Failed to place tool',
                                             stage=stage.extend_name(
                                                       result.stage))

        if request.tool_name == '':
            goal_handle.succeed()
            return PickOrPlaceTool.Result(stage='')
        elif request.tool_name not in node.tool_names:
            raise ActionServer.Error('Unknwon tool name[%s]'
                                     % request.tool_name,
                                     stage='check')

        # [2] 'pick' stage: Pick requested tool.
        with ActionServer.Stage(self, goal_handle, 'pick',
                                pick_or_place_cancel) as stage:
            status, result = node.pick_at_frame(
                                 request.robot_name, request.tool_name,
                                 request.tool_name + '/base_link')
            if status is GoalStatus.STATUS_ABORTED:
                raise ActionServer.Error('Failed to pick tool',
                                         stage=stage.extend_name(result.stage))
            node.set_gripper(request.robot_name, request.tool_name)

        # [Final] Goal succeeded.
        goal_handle.succeed()
        return PickOrPlaceTool.Result(stage='')

#************************************************************************
#  class PickOrPlaceToolTask                                            *
#************************************************************************
class PickOrPlaceToolTask(PickOrPlaceToolTaskClient):
    def __init__(self, node, server_ns='pick_or_place_tool'):
        super().__init__(node, server_ns)
        self._server = PickOrPlaceToolTaskServer(node, server_ns)
        self.wait_for_server()
