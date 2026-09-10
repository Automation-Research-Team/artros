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
#    Science and Technolog (AIST) nor the names of its contributors
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
from action_msgs.msg      import GoalStatus
from aist_robotiq         import RobotiqGripper, RobotiqSuction
from aist_fastening_tools import (SuctionTool, SuctionGripper,
                                  ScrewTool, PrecisionTool)
from aist_barrett         import BarrettHand


def create_gripper(node, name, gripper_type, client_args):
    gripper_client_class = globals().get(gripper_type)
    if gripper_client_class is None:
        raise RuntimeError('unknown type[%s] of the gripper[%s]'
                               % (gripper_type, name))
    if gripper_client_class != RobotiqGripper and \
       node.get_parameter('use_sim_time').get_parameter_value().bool_value:
        return DummyGripper(name, gripper_type)
    return gripper_client_class(node, name, **client_args)


#************************************************************************
#  class DummyGripper                                                   *
#************************************************************************
class DummyGripper(object):
    def __init__(self, name, gripper_type):
        super().__init__()

        self._name = name
        self._type = gripper_type

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

    @property
    def base_link(self):
        return self._name + '/base_link'

    @property
    def tip_link(self):
        return self._name + '/tip_link'

    def set_parameters(self, params: dict):
        pass

    def pregrasp(self):
        pass

    def grasp(self, *, timeout_sec=0.0):
        if timeout_sec is not None or timeout_sec <= 0.0:
            return (GoalStatus.STATUS_UNKNOWN, None)
        else:
            return wait(self, timeout_sec)

    def postgrasp(self):
        pass

    def release(self, *, timeout_sec=0.0):
        if timeout_sec is not None or timeout_sec <= 0.0:
            return (GoalStatus.STATUS_UNKNOWN, None)
        else:
            return wait(self, timeout_sec)

    def wait(self, *, timeout_sec=None):
        return (GoalStatus.STATUS_SUCCEEDED, None)
