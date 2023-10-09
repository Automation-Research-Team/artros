#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2023, National Institute of Advanced Industrial Science and Technology (AIST)
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
# Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
#
import threading, rospy
from actionlib                import SimpleActionServer
from actionlib_msgs.msg       import GoalStatus
from aist_fastening_tools.msg import (SuctionToolCommandAction,
                                      SuctionToolCommandGoal,
                                      SuctionToolCommandFeedback)
from ur_msgs.msg              import IOStates
from ur_msgs.srv              import SetIO
from std_msgs.msg             import Bool

#########################################################################
#  class SuctionToolController                                          #
#########################################################################
class SuctionToolController(object):
    def __init__(self, driver_ns):
        super(SuctionToolController, self).__init__()

        self._name = rospy.get_name()

        # Initialize ur_control table
        self._in_port   = rospy.get_param('~digital_in_port') \
                          if rospy.has_param('~digital_in_port') else None
        self._suck_port = rospy.get_param('~digital_out_port_suck')
        self._blow_port = rospy.get_param('~digital_out_port_blow') \
                          if rospy.has_param('~digital_out_port_blow') \
                          else None

        # Subscribe and set I/O states.
        if self._in_port is not None:
            self._io_states_sub = rospy.Subscriber(driver_ns + '/io_states',
                                                   IOStates,
                                                   self._io_states_cb,
                                                   queue_size=1)
            self._suction_pub   = rospy.Publisher('~suctioned', Bool,
                                                  queue_size=1)

        # Create a service client for setting digital I/O.
        #rospy.wait_for_service(driver_ns + '/set_io')
        self._set_io = rospy.ServiceProxy(driver_ns + '/set_io', SetIO)

        # Create an action server for processing commands to suction tools.
        self._server = SimpleActionServer('~command', SuctionToolCommandAction,
                                          auto_start=False)
        self._server.register_goal_callback(self._goal_cb)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        rospy.loginfo('(%s) controller started', self._name)

    def _goal_cb(self):
        self._active_goal = self._server.accept_new_goal()
        rospy.loginfo('(%s) new goal ACCEPTED', self._name)

        # Set states of suck and blow ports.
        self._set_out_port(self._suck_port, self._active_goal.suck)
        self._set_out_port(self._blow_port, not self._active_goal.suck)

        # If no IN ports are watched, i.e. open-loop, return success.
        if self._in_port is None:
            self._server.set_succeeded()
            rospy.loginfo('(%s) goal SUCCEEDED: no status checking required',
                          self._name)

        self._start_time = rospy.get_rostime()

    def _preempt_cb(self):
        self._server.set_preempted()
        self._set_out_port(self._suck_port, False)
        self._set_out_port(self._blow_port, False)
        rospy.logwarn('(%s) goal CANCELED by client', self._name)

    def _io_states_cb(self, io_states):
        # Find a state of my IN port.
        in_state = next(filter(lambda in_state: in_state.pin == self._in_port,
                               io_states.digital_in_states), None)
        if in_state is None:
            rospy.logerr('(%s) no digiral IN states found at port[%d]',
                         self._name, self._in_port)
            return
        suctioned = in_state.state

        # Publish suction state.
        self._suntion_pub.publish(Bool(suctioned))

        # Return if no active goal running.
        if not self._server.is_active():
            return

        # Publish feedback.
        self._server.publish_feedback(
            SusctionToolCommandFeedback(suctioned))

        # If the DIN state has not reached goal state, update start time.
        if suctioned != goal.suck:
            self._start_time = rospy.get_rostime()

        # Check if the goal state has been kept for min_time.
        if self._active_goal.min_period > rospy.Duration(0.0) and \
           rospy.get_rostime() - start_time > self._active_goal.min_period:
            # Stop blowing.
            if not self._active_goal.suck:
                self._set_out_port(self._blow_port, False)
            self._server.set_succeeded()
            rospy.loginfo('(%s) goal SUCCEEDED: suctioned', self._name)

    def _set_out_port(self, port, state):
        if port is not None:        # blow_port may be None
            if not self._set_io(1, port, 1 if state else 0).success:
                return False
        return True


#########################################################################
#  Entry point                                                          #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('suction_tool_controller')

    driver_ns  = rospy.get_param('~driver_ns', 'ur_hardware_interface')
    controller = SuctionToolController(driver_ns)
    rospy.spin()
