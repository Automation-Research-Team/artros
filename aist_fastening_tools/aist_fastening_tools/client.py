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
from rclpy.callback_groups       import MutuallyExclusiveCallbackGroup
from action_msgs.msg             import GoalStatus
from std_msgs.msg                import Bool
from control_msgs.msg            import GripperCommand as GripperCommandMsg
from control_msgs.action         import GripperCommand
from aist_msgs.action            import ScrewToolCommand, SuctionToolCommand
from task_wrappers.action_client import SimpleActionClient

from rclpy.node                  import Node
from typing                      import Optional

#************************************************************************
#  class SuctionTool                                                    *
#************************************************************************
class SuctionTool(SimpleActionClient):
    """ Suction tool client of aist_msgs.action.SuctionToolCommand type.
    """
    def __init__(self, node: Node, name: str, *,
                 suck_min_period:   float=0.5,
                 blow_min_period:   float=0.2,
                 grasp_timeout_sec: float=1.0):
        """ Create SuctionTool.

        Args:
          node: The ROS node to add the suction tool client to.
          name: Name of the gripper.
          suck_min_period: Minimum period continueing suctioned state for
            success of grasping.
          blow_min_period: Minimum period continueing unsuctioned state for
            success of releasing.
          grasp_timeout_sec: Timeout time waiting for success of grasping
            or releasing command issued asynchronously, that is, zero
            `timeout_sec` value is specified.
        """
        controller_ns    = name + '_controller'
        self._name       = name
        self._client_cbg = MutuallyExclusiveCallbackGroup()
        super().__init__(node, SuctionToolCommand,
                         controller_ns + '/gripper_cmd',
                         callback_group=self._client_cbg)

        # if not self.wait_for_server(timeout_sec=1.0):
        #     raise RuntimeError(
        #         'failed to establish connection to the actionserver[%s]' \
        #         % (controller_ns + '/gripper_cmd'))

        self._suctioned  = None
        self._suctioned_sub \
                         = node.create_subscription(
                               Bool, controller_ns + '/suctioned',
                               self._suctioned_cb, 10,
                               callback_group=MutuallyExclusiveCallbackGroup())
        self._parameters = {'suck_min_period': suck_min_period,
                            'blow_min_period': blow_min_period,
                            'grasp_timeout':   grasp_timeout_sec}

    @property
    def name(self)-> str:
        return self._name

    @property
    def type(self)-> str:
        return 'suction'

    @property
    def base_link(self)-> str:
        return self._name + '/base_link'

    @property
    def tip_link(self)-> str:
        return self._name + '/tip_link'

    @property
    def parameters(self)-> dict:
        return self._parameters

    def set_parameters(self, params: dict)-> None:
        self._parameters |= dict(filter(lambda item: item[0]
                                        in self._parameters,
                                        params.items()))

    def pregrasp(self)-> None:
        # Set goal.min_period to zero so that the goal succeeds immediately.
        self._suck_command(True, min_period=0.0, timeout_sec=0.0)

    def grasp(self, *, timeout_sec=0.0):
        return self._suck_command(
                   True, min_period=self._parameters['suck_min_period'],
                   timeout_sec=timeout_sec)

    def postgrasp(self)-> None:
        self.pregrasp()

    def release(self, *, timeout_sec: Optional[float]=None):
        return self._suck_command(
                   False, min_period=self._parameters['blow_min_period'],
                   timeout_sec=timeout_sec)

    def wait(self, *, timeout_sec: Optional[float]=None):
        """ Wait for the status and the result of command or cancel request.
        Wait until the result of the suction command or a cancel request
        issued by `cancel_goal()` becomes available.

        Args:
          timeout_sec: Timeout time waiting for the result of grasping or
            releasing. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          * A tuple of the goal status and the suction command result,
            if the result becomes available within ``timeout_sec``.
          * A tuple of the current (non-terminal) goal state
            and ``None``. otherwise.
        """
        status, result = super().wait(timeout_sec=timeout_sec)
        if status == GoalStatus.STATUS_UNKNOWN:
            return (status,
                    SuctionToolCommand.Result(suctioned=self._suctioned))
        return (status, result)

    def grasped(self, *, timeout_sec: Optional[float]=None):
        # _, result = self.wait(timeout_sec=timeout_sec)
        # return result.suctioned
        return self._suctioned

    def _suck_command(self, suck: Bool,
                      *, min_period: float, timeout_sec: Optional[float]=None):
        if timeout_sec is None:
            grasp_timeout = 0.0
        elif timeout_sec > 0.0:
            grasp_timeout = timeout_sec  # Wait same duration as action timeout
        else:
            grasp_timeout = self.parameters['grasp_timeout']
        return self.send_goal(SuctionToolCommand.Goal(suck=suck,
                                                      min_period=min_period,
                                                      timeout=grasp_timeout),
                              timeout_sec=timeout_sec)

    def _suctioned_cb(self, msg: Bool)-> None:
        self._suctioned = msg.data

#************************************************************************
#  class SuctionGripper                                                 *
#************************************************************************
class SuctionGripper(SuctionTool):
    """ Suction gripper client of aist_msgs.action.SuctionToolCommand type.
    """
    def __init__(self, node: Node, name: str, *,
                 suck_min_period:   float=0.5,
                 blow_min_period:   float=0.2,
                 grasp_timeout_sec: float=1.0):
        """ Create SuctionGripper.

        Args:
          node: The ROS node to add the suction tool client to.
          name: Name of the gripper.
          suck_min_period: Minimum period continueing suctioned state for
            success of grasping.
          blow_min_period: Minimum period continueing unsuctioned state for
            success of releasing.
          grasp_timeout_sec: Timeout time waiting for success of grasping
            or releasing command issued asynchronously, that is, zero
            `timeout_sec` value is specified.
        """
        super().__init__(node, name, suck_min_period=suck_min_period,
                         blow_min_period=blow_min_period,
                         grasp_timeout_sec=grasp_timeout_sec)

    @property
    def base_link(self)-> str:
        return self._name + '_base_link'

    @property
    def tip_link(self)-> str:
        return self._name + '_tip_link'

#************************************************************************
#  class ScrewTool                                                      *
#************************************************************************
class ScrewTool(SuctionTool):
    """ Screw tool client of aist_msgs.action.ScrewToolCommand type.
    """
    def __init__(self, node: Node, name: str, *,
                 suck_min_period:   float=0.5,
                 blow_min_period:   float=0.2,
                 grasp_timeout_sec: float=1.0,
                 speed:             float=1.0,
                 grasp_speed:       float=0.3,
                 retighten:         bool=True):
        """ Create ScrewTool.

        Args:
          node: The ROS node to add the screw tool client to.
          name: Name of the screw tool
          suck_min_period: Minimum period continueing suctioned state for
            success of grasping.
          blow_min_period: Minimum period continueing unsuctioned state for
            success of releasing.
          grasp_timeout_sec: Timeout time waiting for success of grasping
            or releasing command issued asynchronously, that is, zero
            `timeout_sec` value is specified.
          speed: Rotation speed when tightening or releasing.
          grasp_speed: Rotation speed when picking a screw from screw feeder.
          retighten: Loosen a little and tighten again before completing
            tightening screw, if `True`.
        """
        super().__init__(node, name, suck_min_period=suck_min_period,
                         blow_min_period=blow_min_period,
                         grasp_timeout_sec=grasp_timeout_sec)

        controller_ns = name + '_fastening_controller'
        self._screw_tool = SimpleActionClient(node, ScrewToolCommand,
                                              controller_ns + '/command',
                                              callback_group=self._client_cbg)
        # if not self._screw_tool.wait_for_server(timeout_sec=5.0):
        #     raise RuntimeError(
        #         'failed to establish connection to the action server[%s]' \
        #         % (controller_ns + '/tool_cmd'))
        self._parameters['speed']       = speed
        self._parameters['grasp_speed'] = grasp_speed
        self._parameters['retighten']   = retighten

    def tighten(self, *, timeout_sec: Optional[float]=0.0):
        """ Tighten the screw with the tool.
        Desired speed is specified by the parameter ``speed``.

        Args:
          timeout_sec: Timeout time waiting for the tool to complete
            tightening. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          A tuple of the goal status and the tightening result of
          aist_msgs.action.ScrewToolCommand.Result type.
        """
        return self._screw_command(self.parameters['speed'],
                                   retighten=self.parameters['retighten'],
                                   timeout_sec=timeout_sec)

    def loosen(self, *, timeout_sec: Optional[float]=0.0):
        """ Loosen the screw with the tool.

        Desired speed is specified by the parameter ``speed``.

        Args:
          timeout_sec: Timeout time waiting for the tool to complete
            loosening. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          A tuple of the goal status and the loosening result of
          aist_msgs.action.ScrewToolCommand.Result type.
        """
        return self._screw_command(-self.parameters['speed'],
                                   timeout_sec=timeout_sec)

    def pregrasp(self)-> None:
        self._screw_command(self.parameters['speed'], timeout_sec=0.0)
        super().pregrasp()

    def grasp(self, *, timeout_sec: Optional[float]=0.0):
        self._screw_command(self.parameters['grasp_speed'], timeout_sec=0.0)
        status, result = super().grasp(timeout_sec=timeout_sec)
        if status == GoalStatus.STATUS_SUCCEEDED and result.suctioned:
            self._screw_tool.cancel_goal()
        return status, result

    def postgrasp(self)-> None:
        self._screw_tool.cancel_goal()
        super().postgrasp()

    def release(self, *, timeout_sec: Optional[float]=0.0):
        self._screw_command(0.0, timeout_sec=0.0)
        return super().release(timeout_sec=timeout_sec)

    def wait(self, *, timeout_sec: Optional[float]=None):
        return self._screw_tool.wait(timeout_sec=timeout_sec)

    def grasped(self, *, timeout_sec: Optional[float]=None)-> bool:
        _, result = self.wait(timeout_sec=timeout_sec)
        return result.suctioned

    def cancel_goal(self)-> None:
        """ Cancel the latest motion command sent to the gripper.
        """
        super().cancel_goal()
        self._screw_tool.cancel_goal()

    def _screw_command(self, speed: float, *, retighten: bool=False,
                       timeout_sec: Optional[float]=None):
        return self._screw_tool.send_goal(ScrewToolCommand.Goal(
                                            speed=speed, retighten=retighten),
                                          timeout_sec=timeout_sec)

#************************************************************************
#  class PrecisionTool                                                  *
#************************************************************************
class PrecisionTool(SimpleActionClient):
    """ Precision tool client of control_msgs.action GripperCommand type
    """
    def __init__(self, node: Node, name: str, *,
                 min_position: float=0.0, max_position: float=0.1,
                 max_effort: float=0.0):
        """
        Args:
          node: The ROS node to add the suction tool client to.
          name: Name of the suction tool
          min_position: Finger position when grasping.
          max_position: Finger position when releasing.
          max_effort: Maximum effort to be applied when grasping.
        """
        self._name = name

        # Create action client for gripper command.
        super().__init__(node, GripperCommand,
                         name + '_controller/gripper_cmd',
                         callback_group=MutuallyExclusiveCallbackGroup())

        self._parameters = {'grasp_position':   min_position,
                            'release_position': max_position,
                            'max_effort':       max_effort}

    @property
    def name(self)-> str:
        return self._name

    @property
    def type(self)-> str:
        return 'two_finger'

    @property
    def base_link(self)-> str:
        return self._name + '/base_link'

    @property
    def tip_link(self)-> str:
        return self._name + '/tip_link'

    @property
    def parameters(self)-> dict:
        return self._parameters

    def set_parameters(self, params: dict)-> None:
        self._parameters |= dict(filter(lambda item: item[0]
                                        in self._parameters,
                                        params.items()))

    def pregrasp(self)-> None:
        self.release(timeout_sec=0.0)

    def grasp(self, *, timeout_sec: Optional[float]=None):
        """ Grasp an object with the gripper.
        Desired finger position and applied effort are specified by parameters
        with `grasp_position` and `max_effort` keys, respectively.

        Args:
          timeout_sec: Timeout time waiting for the result of grasping.
            Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          A tuple of the goal status and grasping result of
          control_msgs.action.GripperCommand.Result type
        """
        return self.move(self.parameters['grasp_position'],
                         max_effort=self.parameters['max_effort'],
                         timeout_sec=timeout_sec)

    def postgrasp(self)-> None:
        self.grasp(timeout_sec=0.0)

    def release(self, *, timeout_sec: Optional[float]=None):
        """ Release an object grasped by the gripper.
        Desired finger position is specified by a parameter
        with `release_position` key. No effort is applied.

        Args:
          timeout_sec: Timeout time waiting for the result of releasing.
            Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          A tuple of the goal status and releasing result of
          control_msgs.action.GripperCommand.Result type
        """
        return self.move(self.parameters['release_position'],
                         max_effort=0.0, timeout_sec=timeout_sec)

    def move(self, position: float, *,
             max_effort: float=0.0, timeout_sec: Optional[float]=None):
        """ Move gripper to the desired position.

        Args:
          position: Desired finger position.
          max_effort: Desired maximum effort to be applied.
          timeout_sec: Timeout time waiting for the result of grasping or
            releasing. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          A tuple of the goal status and the movement result of
          control_msgs.action.GripperCommand.Result type.
        """
        return self.send_goal(GripperCommand.Goal(
                                  command=GripperCommandMsg(
                                      position=position,
                                      max_effort=max_effort)),
                              timeout_sec=timeout_sec)

    def grasped(self, *, timeout_sec: Optional[float]=None)-> bool:
        _, result = self.wait(timeout_sec=timeout_sec)
        return result.stalled
