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
import time, copy
import numpy as np
from rclpy.callback_groups       import MutuallyExclusiveCallbackGroup
from aist_msgs.action            import HandEyeCalibration
from task_wrappers.action_server import ActionServer
from task_wrappers.action_client import SimpleActionClient

#*********************************************************************
#  class HandEyeCalibrationTaskClient                                *
#*********************************************************************
class HandEyeCalibrationTaskClient(SimpleActionClient):
    def __init__(self, node, server_ns='handeye_calibration'):
        super().__init__(node, HandEyeCalibration, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup())

    def send_goal(self, camera_name, robot_name, eye_on_hand,
                  end_effector_link, initpose, keyposes, *, timeout_sec=None):
        return super().send_goal(
                   HandEyeCalibration.Goal(camera_name=camera_name,
                                           robot_name=robot_name,
                                           eye_on_hand=eye_on_hand,
                                           end_effector_link=end_effector_link,
                                           initpose=initpose,
                                           keyposes=keyposes),
                   feedback_callback=self.stage_feedback_cb,
                   timeout_sec=timeout_sec)

#*********************************************************************
#  class HandEyeCalibrationTaskServer                                *
#*********************************************************************
class HandEyeCalibrationTaskServer(ActionServer):
    def __init__(self, node, server_ns='handeye_calibration'):
        super().__init__(node, HandEyeCalibration, server_ns, self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup())

    def _execute_cb(self, goal_handle):
        request = goal_handle.request
        node    = self.node
        stop    = lambda: node.stop(request.robot_name)

        node.reset()

        # [1] 'go_home' stage: Go to home pose.
        with ActionServer.Stage(self, goal_handle, 'go_home', stop) as stage:
            success = node.go_to_named_pose(request.robot_name, 'home')
            if not success:
                raise ActionServer.Error('Failed to go home', stage=stage.name)

        # [2] 'go_to_initpose' stage: Go to initial pose.
        with ActionServer.Stage(self, goal_handle, 'go_to_initpose',
                                stop) as stage:
            success = node.go_to_pose_goal(
                          request.robot_name,
                          node.pose_from_xyzrpy(request.initpose),
                          end_effector_link=request.end_effector_link)
            if not success:
                raise ActionServer.Error('Failed to go to initial pose',
                                         stage=stage.name)

        # Collect samples over pre-defined poses
        keyposes = np.array(request.keyposes).reshape(-1, 6).tolist()
        for i, keypose in enumerate(keyposes, 1):
            print('\n*** Keypose [%d/%d]: Try! ***' % (i, len(keyposes)))
            if request.eye_on_hand:
                self._move_and_take_sample(goal_handle, keypose)
            else:
                self._visit_subposes_and_take_samples(goal_handle, keypose, i)
                print('*** Keypose [%d/%d]: Completed. ***'
                      % (i, len(keyposes)))

        # [4] 'go_back_home' stage: Go back to home pose.
        with ActionServer.Stage(self, goal_handle, 'go_back_home',
                                stop) as stage:
            success = node.go_to_named_pose(request.robot_name, 'home')
            if not success:
                raise ActionServer.Error('Failed to go back home',
                                         stage=stage.name)

        # [5] 'compute_calibration' stage: Go back to home pose.
        with ActionServer.Stage(self, goal_handle,
                                'compute_calibration') as stage:
            res = node.compute_calibration()
            if not res.success:
                raise ActionServer.Error('Failed to compute calibration',
                                         stage=stage.name)
            node.save_calibration(res)

        # [Final] Goal succeeded.
        goal_handle.succeed()
        return Sweep.Result(stage='')

    def _visit_subposes_and_take_samples(self, goal_handle, keypose,
                                         keypose_num):
        subpose = copy.copy(keypose)
        roll = subpose[3]
        for i in range(3):
            print('\n--- Subpose [%d/5]: Try! ---' % (i + 1))
            if self._move_and_take_sample(goal_handle, subpose):
                self.logger.info('Subpose [%d/5]: Succeeded.' % (i + 1))
            else:
                self.logger.error('Subpose [%d/5]: Failed.' % (i + 1))
            subpose[3] -= 30.0

        subpose[3]  = roll - 30.0
        subpose[4] += 15.0

        for i in range(2):
            print('\n--- Subpose [%d/5]: Try! ---' % (i + 4))
            if self._move_and_take_sample(goal_handle, subpose):
                self.logger.info('Subpose [%d/5]: Succeeded.' % (i + 4))
            else:
                self.logger.error('Subpose [%d/5]: Failed.' % (i + 4))
            subpose[4] -= 30.0

    def _move_and_take_sample(self, goal_handle, xyzrpy):
        self.logger.info('trying to move to %s' % xyzrpy)

        request = goal_handle.request
        node    = self.node
        stop    = lambda: node.stop(request.robot_name)

        # [3] 'move_and_take_sample' stage: Go to sample pose.
        with ActionServer.Stage(self, goal_handle, 'move_and_take_sample',
                                stop) as stage:
            success = node.go_to_pose_goal(
                          request.robot_name, node.pose_from_xyzrpy(xyzrpy),
                          end_effector_link=request.end_effector_link)
            if not success:
                self.logger.error('failed to go to sample pose')
                return False

            time.sleep(node._settling_time)  # Wait the robot to settle.

            try:
                node.take_sample(timeout_sec=0.0)
                node.trigger_frame(request.camera_name)
                res = node.wait_for_sample(timeout_sec=1.0)
            except Exception as ex:
                self.logger.error(str(ex))
                return False

            if not res.success:
                self.logger.error('failed to take sample: %s' % res.message)
                return False
            else:
                self.logger.info('  %d-th sample taken'
                                 % len(node.get_sample_list().transform_cm))
                return True

#************************************************************************
#  class HandEyeCalibrationTask                                         *
#************************************************************************
class HandEyeCalibrationTask(HandEyeCalibrationTaskClient):
    def __init__(self, node, server_ns='handeye_calibration'):
        super().__init__(node, server_ns)
        self._server = HandEyeCalibrationTaskServer(node, server_ns)
        self.wait_for_server()
