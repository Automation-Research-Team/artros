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
from action_msgs.msg             import GoalStatus
from geometry_msgs.msg           import PoseStamped
from aist_msgs.action            import AttemptBin
from task_wrappers.action_client import GroupedSimpleActionClient
from task_wrappers.action_server import ActionServer

from typing                      import Optional

#*********************************************************************
#  class AttemptBinTaskClient                                        *
#*********************************************************************
class AttemptBinTaskClient(GroupedSimpleActionClient):
    def __init__(self, node: Node, server_ns: str='attempt_bin'):
        super().__init__(node, AttemptBin, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def send_goal(self, robot_name: str, bin_id: str, pick_all: bool,
                  max_attempts: int, *, timeout_sec: Optional[float]=0.0):
        return super().send_goal(AttemptBin.Goal(robot_name=robot_name,
                                                 bin_id=bin_id,
                                                 pick_all=pick_all,
                                                 max_attempts=max_attempts),
                                 feedback_callback=self.stage_feedback_cb,
                                 timeout_sec=timeout_sec)

#*********************************************************************
#  class AttemptBinTaskServer                                        *
#*********************************************************************
class AttemptBinTaskServer(ActionServer):
    def __init__(self, node: Node, server_ns: str='attempt_bin'):
        super().__init__(node, AttemptBin, server_ns, self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def _execute_cb(self, goal_handle):
        def _is_eye_on_hand(robot_name, camera_name):
            return camera_name == robot_name + '_camera'

        request              = goal_handle.request
        node                 = self.node
        stop                 = lambda: node.stop(request.robot_name)
        pick_or_place_cancel = lambda: node.pick_or_place_cancel_goal(
                                           request.robot_name)

        bin_props = node.bin_props.get(request.bin_id)
        if not bin_props:
            raise ActionServer.Error('unknown bin_id[%s]' % request.bin_id,
                                     stage='')
        part_id    = bin_props['part_id']
        part_props = node.part_props.get(part_id)
        if not part_props:
            raise ActionServer.Error('unknown part_id[%s]' % part_id,
                                     stage='')

        tool_name = part_props['gripper_name']
        if node.gripper(request.robot_name).name != gripper_name:
            # [1] 'pick_tool' stage: Pick suction tool.
            with ActionServer.Stage(self, goal_handle, 'pick_tool') as stage:
                status, result = node.pick_tool(request.robot_name, tool_name)
                if status is GoalStatus.STATUS_ABORTED:
                    raise ActionServer.Error('Failed to pick tool!',
                                             stage=stage.extend_name(
                                                       result.stage))

        # [2] 'go_home' stage: Go to home pose.
        with ActionServer.Stage(self, goal_handle, 'go_home', stop) as stage:
            success = node.go_to_named_pose(request.robot_name, 'home')
            if not success:
                raise ActionServer.Error('Failed to go home', stage=stage.name)

        fine_parameters = False  # Use default graspability parameters
        pick_poses      = []
        fail_poses      = []
        place_offset    = 0.020

        while True:
            # If no graspability poses available, search for them.
            if not pick_poses:
                if _is_eye_on_hand(request.robot_name,
                                   part_props['camera_name']):
                    # [3] 'move_camera' stage: Go to pose for capturing bin.
                    #     Move to 0.15m above the bin if the camera is mounted
                    #     on the robot.
                    with ActionServer.Stage(self, goal_handle, 'move_camera',
                                            stop) as stage:
                        success = node.go_to_frame(request.robot_name,
                                                   bin_props['name'],
                                                   (0, 0, 0.15))
                        if not success:
                            raise ActionServer.Error('Failed to move camera',
                                                     stage=stage.name)

                # [4] 'search' stage: Search for graspabilities.
                with ActionServer.Stage(self, goal_handle, 'search') as stage:
                    status, result = node.search_bin(request.bin_id,
                                                     fine_parameters)
                    if status is GoalStatus.STATUS_ABORTED:
                        raise ActionServer.Error(
                            'Failed to search graspabilities',
                            stage=stage.name)
                    pick_poses = result.graspabilities.poses

            # [5] 'pick' stage: Pick a part at a pose selected from pick_poses.
            with ActionServer.Stage(self, goal_handle, 'pick',
                                    pick_or_place_cancel) as stage:
                # Attempt only once if fine graspability parameters are used.
                status, result, pose = self._attempt_pick(
                                           request.robot_name, part_id,
                                           pick_poses, fail_poses,
                                           1 if fine_parameters else \
                                           request.max_attempts)
                if not fine_parameters:
                    if status in (GoalStatus.STATUS_ABORTED,
                                  GoalStatus.STATUS_UNKNOWN):
                        self.logger.warn('--- AttemptBin: failed stage[%s], switch to fine parameters'
                                         % stage.name)
                        fine_parameters = True
                        pick_poses      = []
                        fail_poses      = []
                elif status is GoalStatus.STATUS_ABORTED:
                    self.logger.warn('--- AttemptBin: aborted stage[%s] under fine parameters'
                                     % stage.name)
                    fine_parameters = False
                    pick_poses      = []
                    fail_poses      = []
                    raise ActionServer.Error('Failed to pick',
                                             stage=stage.extend_name(
                                                 result.stage),
                                             pose=pose)
                elif status is GoalStatus.STATUS_UNKNOWN:  # no poses remained
                    self.logger.warn('--- AttemptBin: finished stage[%s] with status[%d], break'
                                     % (stage.name, status))
                    break

            if status is GoalStatus.STATUS_SUCCEEDED:
                # [6] 'place' stage: Begin placing and wait until reaching
                #     approach pose.
                with ActionServer.Stage(self, goal_handle, 'place',
                                        pick_or_place_cancel) as stage:
                    # Place the picked part (not wait).
                    node.place_at_frame(request.robot_name, part_id,
                                        part_props['destination'],
                                        offset=(0.0, place_offset, 0.0),
                                        timeout_sec=0.0)
                    place_offset = -place_offset

                    if _is_eye_on_hand(request.robot_name,
                                       part_props['camera_name']):
                        # Wait until placing finished.
                        status, result = node \
                                        .pick_or_place_wait(request.robot_name)
                        if status is GoalStatus.STATUS_ABORTED:
                            raise ActionServer.Error('Failed to place',
                                                     stage=stage.extend_name(
                                                               result.stage))
                        pick_poses = []
                    else:
                        node.pick_or_place_wait(request.robot_name,
                                                target_stage='approach')

                        # Search graspabilities for the next trial.
                        status, result = node.search_bin(request.bin_id,
                                                         fine_parameters)
                        if status is GoalStatus.STATUS_ABORTED:
                            raise ActionServer.Error(
                                'Failed to search graspabilities',
                                stage=stage.name)
                        pick_poses = result.graspabilities.poses

                        # Wait until placing finished.
                        status, result = node \
                                        .pick_or_place_wait(request.robot_name)
                        if status is GoalStatus.STATUS_ABORTED:
                            raise ActionServer.Error('Failed to place',
                                                     stage=stage.extend_name(
                                                               result.stage))

            if not request.pick_all:
                break

        # [7] 'go_back_home' stage: Go back to home pose.
        with ActionServer.Stage(self, goal_handle, 'go_back_home',
                                stop) as stage:
            success = node.go_to_named_pose(request.robot_name, 'home')
            if not success:
                raise ActionServer.Error('Failed to go back home',
                                         stage=stage.name)

        goal_handle.succeed()
        return AttemptBin.Result(stage='')

    def _attempt_pick(self, robot_name, part_id, pick_poses, fail_poses,
                      max_attempts):
        """ Repeat until a part successfully picked.

        Args:
          robot_name:   Robot name.
          part_id:      ID of part to be picked.
          pick_poses:   Poses of candidate picking points.
          fail_poses:   Poses of picking points already failed.
          max_attempts: Maximum number of picking attempts.

        Returns:
          Tuple of GoalStatus and Result of picking,
        """
        def _is_close_to_fail_poses(pose, tolerance=0.005):
            def _is_close_to_fail_pose(pose, fail_pose, tolerance):
                position      = pose.pose.position
                fail_position = fail_pose.pose.position
                return abs(position.x - fail_position.x) < tolerance and \
                       abs(position.y - fail_position.y) < tolerance and \
                       abs(position.z - fail_position.z) < tolerance

            for fail_pose in fail_poses:
                if _is_close_to_fail_pose(pose, fail_pose, tolerance):
                    return True
            return False

        # Attempt to pick the item.
        node = self.node
        pose = None
        nattempts = 0
        for p in pick_poses.poses:
            pose = PoseStamped(header=pick_poses.header, pose=p)
            if _is_close_to_fail_poses(pose):
                continue

            # Perform picking.
            status, result = node.pick(robot_name, part_id, pose)

            # A. Pick succeeded.
            if status is GoalStatus.STATUS_SUCCEEDED:
                return status, result, pose

            # B. Pick failed.
            elif status is GoalStatus.STATUS_ABORTED:
                # B-1. Error in moving to approach or pick pose.
                if result.stage in ('move', 'approach'):
                    fail_poses.append(pose)

                # B-2. Error in departing from pick pose.
                elif result.stage == 'depart':
                    return status, result, pose

                # B-3. Error in grasping.
                elif result.stage == 'verify':
                    fail_poses.append(pose)
                    nattempts += 1
                    if nattempts == max_attempts:
                        return status, result, pose

            # C. Pick canceled.
            elif status is GoalStatus.STATUS_CANCELED:
                return status, result, pose

        # Here, no graspability poses remained or max_attempts attained.
        return GoalStatus.STATUS_UNKNOWN, None, pose


#************************************************************************
#  class AttemptBinTask                                                 *
#************************************************************************
class AttemptBinTask(AttemptBinTaskClient):
    def __init__(self, node, server_ns='attempt_bin'):
        super().__init__(node, server_ns)
        self._server = AttemptBinTaskServer(node, server_ns)
        self.wait_for_server()

    @property
    def server(self):
        return self._server
