#
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
import threading, collections
from rclpy.callback_groups       import MutuallyExclusiveCallbackGroup
from task_wrappers.action_client import SimpleActionClient
from task_wrappers.action_server import ActionServer
from aist_msgs.action            import RequestHelp
from aist_msgs.msg               import RequestHelp as RequestHelpMsg, Pointing
from geometry_msgs.msg           import PoseStamped, Point, Vector3
from visualization_msgs.msg      import Marker
from std_msgs.msg                import ColorRGBA
from builtin_interfaces.msg      import Duration

from rclpy.node                  import Node
from std_msgs.msg                import Header
from typing                      import Optional

#*********************************************************************
#  class RequestHelpTaskClient                                       *
#*********************************************************************
class RequestHelpTaskClient(SimpleActionClient):
    def __init__(self, node: Node, server_ns: str='request_help'):
        """ Create RequestHelpTaskClient.

        Args:
          node:      Node to which the client assigned.
          server_ns: Namespace of the RequestHelpTaskServer.
        """
        super().__init__(node, RequestHelp, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup())
        self._marker_pub = node.create_publisher(Marker, 'pointing_marker', 1)

    def send_goal(self, robot_name: str, pose: PoseStamped, part_id: str,
                  message: str, *, timeout_sec: Optional[float]=0.0):
        """ Request help to the remote operator.
        Request finger direction for the specified graspability point
        and receive response.

        Args:
          robot_name: Name of the robot.
          pose:       Pose of the graspability point to be sweeped.
          part_id:    ID for specifying part.
          message:    Message to be displayed to the operator on VR side.

        Returns:
          tuple:      Tuple of the GoalStatus and Result of the action.
        """
        # Keep graspability pose for the start point of the ARROW marker.
        self._pose = self.node.transform_pose_to_target_frame(
                         pose, target_frame=self.node.planning_frame)

        goal = RequestHelp.Goal()
        goal.request.robot_name = robot_name
        goal.request.item_id    = part_id
        goal.request.pose       = self._pose
        goal.request.request    = RequestHelpMsg.SWEEP_DIR_REQ
        goal.request.message    = message
        return super().send_goal(goal, feedback_callback=self._feedback_cb,
                                 timeout_sec=timeout_sec)

    def delete_markers(self):
        """ Delete all markers published by _marker_pub.
        """
        marker        = Marker()
        marker.action = Marker.DELETEALL
        marker.ns     = 'pointing_marker'
        self._marker_pub.publish(marker)

    def _publish_marker(self, header: Header, pos: Point, lifetime: int=15):
        marker              = Marker()
        marker.header       = header
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns           = 'pointing_marker'
        marker.id           = 0
        marker.type         = Marker.ARROW
        marker.action       = Marker.ADD
        marker.scale        = Vector3(x=0.006, y=0.014, z=0.015)
        marker.color        = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        marker.lifetime     = Duration(sec=lifetime, usec=0)
        marker.points       = [self._pose.pose.position, pos]
        self._marker_pub.publish(marker)

    def _feedback_cb(self, feedback):
        self._publish_marker(feedback.feedback.pointing.header,
                             feedback.feedback.pointing.point)

#*********************************************************************
#  class RequestHelpTaskServer                                       *
#*********************************************************************
class RequestHelpTaskServer(ActionServer):
    _Pointing = ('NO_RES', 'SWEEP_RES', 'RECAPTURE_RES')

    def __init__(self, node, server_ns='request_help'):
        super().__init__(node, RequestHelp, server_ns, self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup())

        # RequestHelp message publishing stuffs: ROS -> Unity
        self._request_help_pub = node.create_publisher(RequestHelpMsg,
                                                       'help', 10)

        # Pointing message subscription stuffs: ROS <- Unity
        self._pointing      = None
        self._pointing_cond = threading.Condition()
        self._pointing_sub  = node.create_subscription(Pointing, 'pointing',
                                                       self._pointing_cb, 3)

    def _pointing_cb(self, pointing: Pointing):
        """ Subscribe error recovery command messages from the remote operator.
        Reception of the message is notified to the execution callback
        of the action server.
        """
        pointing.header.stamp = self.node.get_clock().now().to_msg()
        with self._pointing_cond:
            self._pointing = pointing
            self._pointing_cond.notify_all()

    def _execute_cb(self, goal_handle):
        # Loop until Pointing.msg other than NO_RES received.
        while True:
            # Publish RequestHelp.msg toward the remote operator.
            request = goal_handle.request.request
            request.pose.header.stamp = self.node.get_clock().now().to_msg()
            self._request_help_pub.publish(request)

            # Get Pointing.msg from the remote operator.
            with self._pointing_cond:
                if self._pointing_cond.wait_for(lambda: self._pointing, 1.0):
                    pointing = self._pointing
                    self._pointing = None
                else:
                    pointing = Pointing(pointing_state=Pointing.NO_RES)

            goal_handle.publish_feedback(
                RequestHelp.Feedback(pointing=pointing))

            if pointing.pointing_state != Pointing.NO_RES:
                break

            ActionServer.check_goal_status(goal_handle, 'preempted',
                                           pointing=pointing)

        goal_handle.succeed()
        self.logger.info('goal SUCCEEDED[%s: pos=(%f %f %f)]'
                         % (RequestHelpTaskServer \
                            ._Pointing[pointing.pointing_state],
                            pointing.point.x, pointing.point.y,
                            pointing.point.z))
        return RequestHelp.Result(pointing=pointing)

#************************************************************************
#  class RequestHelpTask                                                *
#************************************************************************
class RequestHelpTask(RequestHelpTaskClient):
    def __init__(self, node, server_ns='request_help'):
        super().__init__(node, server_ns)
        self._server = RequestHelpTaskServer(node, server_ns)
        self.wait_for_server()
