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
from aist_msgs.action      import PickOrPlaceScrew
from task_wrappers         import ActionServer, GroupedSimpleActionClient

#*********************************************************************
#  class PickOrPlaceScrewTaskClient                                  *
#*********************************************************************
class PickOrPlaceScrewTaskClient(GroupedSimpleActionClient):
    def __init__(self, node: Node, server_ns: str='pick_or_place_screw'):
        super().__init__(node, PickOrPlaceScrew, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def send_goal(self, robot_name, screw_type, *, timeout_sec=0.0):
        return super().send_goal(PickOrPlaceScrew.Goal(robot_name=robot_name,
                                                      screw_type=screw_type),
                                 feedback_callback=self.stage_feedback_cb,
                                 timeout_sec=timeout_sec)

#*********************************************************************
#  class PickOrPlaceScrewTaskServer                                  *
#*********************************************************************
class PickOrPlaceScrewTaskServer(ActionServer):
    def __init__(self, node, server_ns='pick_or_place_screw'):
        super().__init__(node, PickOrPlaceScrew, server_ns, self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def _execute_cb(self, goal_handle):
        if goal_handle.request.screw_type:
            return self._pick_screw(goal_handle)
        else:
            return self._place_screw(goal_handle)

    def _pick_screw(self, goal_handle):
        request              = goal_handle.request
        node                 = self.node
        tool_name            = 'screw_tool_' + request.screw_type[-2:]
        pick_or_place_cancel = lambda: node.pick_or_place_cancel_goal(
                                           request.robot_name)

        if node.gripper(request.robot_name).name != tool_name:
            # [1] 'pick_tool' stage: Pick screw tool for requested screw type.
            with ActionServer.Stage(self, goal_handle, 'pick_tool',
                                    pick_or_place_cancel) as stage:
                status, result = node.pick_tool(request.robot_name, tool_name)
                if status is GoalStatus.STATUS_ABORTED:
                    raise ActionServer.Error('Failed to pick tool!',
                                             stage=stage.extend_name(
                                                       result.stage))

        # [2] 'pick_screw' stage: Place current tool.
        with ActionServer.Stage(self, goal_handle, 'pick_screw',
                                pick_or_place_cancel) as stage:
            screw_id = node._get_screw_id(request.screw_type)
            status, result = node.pick_at_frame(request.robot_name, screw_id,
                                                screw_id + '/head')
            if status is GoalStatus.STATUS_ABORTED:
                raise ActionServer.Error('Failed to pick screw!',
                                         stage=stage.extend_name(result.stage))

        # [Final] Goal succeeded.
        goal_handle.succeed()
        return PickOrPlaceScrew.Result(stage='', screw_id=screw_id)

    def _place_screw(self, goal_handle):
        request              = goal_handle.request
        node                 = self.node
        screw_tip_link       = next(filter(lambda frame_id:
                                           frame_id.startswith('screw_m') and
                                           frame_id.endswith('/tip_link'),
                                           node.candidate_eef_links(
                                               request.robot_name)),
                                    None)
        pick_or_place_cancel = lambda: node.pick_or_place_cancel_goal(
                                           request.robot_name)

        if screw_tip_link is None:
            raise ActionServer.Error('No screw grasped!', stage='')

        screw_id    = node._get_object_id(screw_tip_link)
        screw_type  = screw_id.rsplit('_', 1)[0]
        feeder_name = 'screw_feeder_' + screw_type[-2:]

        # [1] 'place_screw' stage:
        with ActionServer.Stage(self, goal_handle, 'place_screw',
                                pick_or_place_cancel) as stage:
            status, result = node.place_at_frame(request.robot_name, screw_id,
                                                 feeder_name + '_inlet_link',
                                                 eef_link=screw_tip_link)
            if status == GoalStatus.STATUS_ABORTED:
                raise ActionServer.Error('Failed to place screw!',
                                         stage=stage.extend_name(result.stage))

        # [Final] Goal succeeded.
        node.com.remove_object(screw_id)
        goal_handle.succeed()
        return PickOrPlaceScrew.Result(stage='', screw_id=screw_id)

#************************************************************************
#  class PickOrPlaceScrewTask                                           *
#************************************************************************
class PickOrPlaceScrewTask(PickOrPlaceScrewTaskClient):
    def __init__(self, node, server_ns='pick_or_place_screw'):
        super().__init__(node, server_ns)
        self._server = PickOrPlaceScrewTaskServer(node, server_ns)
        self.wait_for_server()
