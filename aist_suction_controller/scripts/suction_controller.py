#! /usr/bin/env python
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
import rospy
from actionlib                   import SimpleActionServer
from aist_suction_controller.msg import (SuctionControlAction,
                                         SuctionControlResult)
from ur_msgs.msg                 import IOStates
from ur_msgs.srv                 import SetIO
from std_msgs.msg                import Bool

class SuctionController(object):
    def __init__(self):
        super(SuctionController, self).__init__()

        # Subscribe I/O states
        self._in_state     = dict()
        self._out_state    = dict()
        self._io_state_sub = rospy.Subscriber(
                                'ur_hardware_interface/io_states',
                                IOStates, self._io_state_cb, queue_size=1)
        self._set_io       = rospy.ServiceProxy('ur_hardware_interface/set_io',
                                                SetIO)

        # initialize ur_control table
        self._in_port_names         = dict()
        self._digital_in_port       = dict()
        self._digital_out_port_vac  = dict()
        self._digital_out_port_blow = dict()
        self._tool_suction_pubs     = dict()
        for tool in rospy.get_param('~suction_control'):
            name = tool['name']
            self._in_port_names[tool['digital_in_port']] = name
            self._digital_in_port[name]       = tool['digital_in_port']
            self._digital_out_port_vac[name]  = tool['digital_out_port_vac']
            self._digital_out_port_blow[name] = tool['digital_out_port_blow']
            self._tool_suction_pubs[name]     = rospy.Publisher(
                                                    name + '/suctioned', Bool,
                                                    queue_size=1)

        # for operation_mode
        self._operation_mode_in_port_names = dict()
        self._operation_mode_pubs          = dict()
        for mode in rospy.get_param('~operation_mode'):
            name = mode['name']
            self._operation_mode_in_port_names[mode['digital_in_port']] = name
            self._operation_mode_pubs[name] = rospy.Publisher(name, Bool,
                                                              queue_size=1)

        # action server for suction control
        self._suction_srv = SimpleActionServer('~suction',
                                               SuctionControlAction,
                                               self._suction_control_cb, False)
        self._suction_srv.start()

    def _io_state_cb(self, data):
        for read_in_status in data.digital_in_states:
            self._in_state[read_in_status.pin] = read_in_status.state
            if read_in_status.pin in self._in_port_names:
                self._tool_suction_pubs[self._in_port_names[
                    read_in_status.pin]].publish(Bool(read_in_status.state))
            if read_in_status.pin in self._operation_mode_in_port_names:
                self._operation_mode_pubs[self._operation_mode_in_port_names[
                    read_in_status.pin]].publish(Bool(read_in_status.state))

        for read_out_status in data.digital_out_states:
            self._out_state[read_out_status.pin] = read_out_status.state

    def _suction_control_cb(self, goal):
        res = SuctionControlResult()
        res.success = True

        # yaml file check
        if goal.tool_name not in self._digital_in_port:
            rospy.logerr("tool '%s' does not exist in %s."
                         % (goal.tool_name, self._digital_in_port))
            res.success = False

        if goal.tool_name not in self._digital_out_port_vac:
            rospy.logerr("tool '%s' does not exist in %s."
                         % (goal.tool_name, self._digital_out_port_vac))
            res.success = False

        if goal.tool_name not in self._digital_out_port_blow:
            rospy.logerr("tool '%s' does not exist in %s."
                         % (goal.tool_name, self._digital_out_port_blow))
            res.success = False

        if not res.success:
            self._suction_srv.set_aborted(res)
            return

        vac_port  = self._digital_out_port_vac[goal.tool_name]
        blow_port = self._digital_out_port_blow[goal.tool_name]

        if goal.turn_suction_on:
            if not self._set_out_pin_switch(blow_port, False):
                res.success = False
            if not self._set_out_pin_switch(vac_port, True):
                res.success = False
        elif goal.eject:
            if not self._set_out_pin_switch(vac_port, False):
                res.success = False
            if not self._set_out_pin_switch(blow_port, True):
                res.success = False
        else:
            if not self._set_out_pin_switch(blow_port, False):
                res.success = False
            if not self._set_out_pin_switch(vac_port, False):
                res.success = False

        if not res.success:
            if not self._set_out_pin_switch(blow_port, False):
                res.success = False
            if not self._set_out_pin_switch(vac_port, False):
                res.success = False

        if res.success:
            self._suction_srv.set_succeeded(res)
        else:
            self._suction_srv.set_aborted(res)

    def _set_out_pin_switch(self, port, state):
        success = True

        start_time = rospy.get_rostime()

        while (rospy.get_rostime().secs - start_time.secs) <= 3.0:
            res = self._set_io(1, port, 1 if state else 0)
            if not res.success:
                success = False
                break
            if port in self._out_state:
                if self._out_state[port] == state:
                    break

        if success:
            rospy.loginfo("Success. Digital_out_port(%d) changed %r."
                          % (port, state))
        else:
            rospy.logerr("Error. Digital_out_port(%d) can't be changed."
                         % (port))

        return success


if __name__ == '__main__':
    rospy.init_node('suction_controller')
    server = SuctionController()
    rospy.spin()
