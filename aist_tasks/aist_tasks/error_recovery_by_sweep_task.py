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
from rclpy.node            import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from action_msgs.msg       import GoalStatus
from geometry_msgs.msg     import PoseStamped, Pose, Quaternion, PointStamped
from aist_msgs.msg         import Pointing
from aist_msgs.action      import ErrorRecoveryBySweep
from task_wrappers         import ActionServer, GroupedSimpleActionClient

#*********************************************************************
#  class ErrorRecoveryBySweepTaskClient                              *
#*********************************************************************
class ErrorRecoveryBySweepTaskClient(GroupedSimpleActionClient):
    def __init__(self, node: Node, server_ns: str='error_recovery_by_sweep'):
        super().__init__(node, ErrorRecoveryBySweep, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def send_goal(self, robot_name, pose, part_id, message,
                  *, timeout_sec=None):
        return super().send_goal(
                  ErrorRecoveryBySweep.Goal(robot_name=robot_name,
                                            item_id=part_id,
                                            pose=pose,
                                            message=message),
                  feedback_callback=self.stage_feedback_cb,
                  timeout_sec=timeout_sec)

#*********************************************************************
#  class ErrorRecoveryBySweepTaskServer                              *
#*********************************************************************
class ErrorRecoveryBySweepTaskServer(ActionServer):
    def __init__(self, node, server_ns='error_recovery_by_sweep'):
        super().__init__(node, ErrorRecoveryBySweep, server_ns,
                         self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def _execute_cb(self, goal_handle):
        request = goal_handle.request
        node    = self.node
        stop    = lambda: node.stop(request.robot_name)

        def _fix_sweep_direction(pose: PoseStamped, point: PointStamped):
            p = pose.pose.position
            q = node.transform_point_to_target_frame(point,
                                                     pose.header.frame_id) \
                    .point
            s = np.array((q.x - p.x, q.y - p.y, q.z - p.z))

            R = tfs.quaternion_matrix((pose.pose.orientation.x,
                                       pose.pose.orientation.y,
                                       pose.pose.orientation.z,
                                       pose.pose.orientation.w))
            nz = R[0:3, 2]
            ny = s - nz * np.dot(nz, s)
            R[0:3, 1] = ny/np.linalg.norm(ny)
            R[0:3, 0] = np.cross(R[0:3, 1], nz)
            qR = tfs.quaternion_from_matrix(R)
            return PoseStamped(header=pose.header,
                               pose=Pose(position=pose.pose.position,
                                         orientation=Quaternion(x=qR[0],
                                                                y=qR[1],
                                                                z=qR[2],
                                                                w=qR[3])))

        # [1] 'sweep_ready' stage: Go to sweep ready pose.
        with ActionServer.Stage(self, goal_handle, 'go_to_sweep_ready',
                                stop) as stage:
            success = node.go_to_named_pose(request.robot_name, 'sweep_ready')
            if not success:
                raise ActionServer.Error('Failed to go to sweep ready pose',
                                         stage=stage.name)

        # [2] 'request_help' stage: Get Pointing.msg from the remote operator.
        with ActionServer.Stage(self, goal_handle, 'request_help') as stage:
            message = 'Picking_failed!'
            status, result = node.request_help(request.robot_name,
                                               request.pose, request.item_id,
                                               message)
            if status == GoalStatus.STATUS_ABORTED:
                raise ActionServer.Error('Failed to get sweep direction',
                                          stage=stage.name)

        # [3] 'sweep' stage:
        with ActionServer.Stage(self, goal_handle, 'sweep', stop) as stage:
            pose   = _fix_sweep_direction(request.pose,
                                          PointStamped(
                                              header=result.pointing.header,
                                              point=result.pointing.point))
            params = node.sweep_parameters[request.item_id]
            status, result = node.sweep(request.robot_name, pose,
                                        request.item_id)
            if status == GoalStatus.STATUS_ABORTED:
                raise ActionServer.Error('Failed to sweep',
                                         stage=stage.extend_name(result.stage))

        # [Final] Goal succeeded.
        goal_handle.succeed()
        return ErrorRecoveryBySweep.Result(stage='')


#************************************************************************
#  class ErrorRecoveryBySweepTask                                       *
#************************************************************************
class ErrorRecoveryBySweepTask(ErrorRecoveryBySweepTaskClient):
    def __init__(self, node, server_ns='error_recovery_by_sweep'):
        super().__init__(node, server_ns)
        self._server = ErrorRecoveryBySweepTaskServer(node, server_ns)
        self.wait_for_server()
