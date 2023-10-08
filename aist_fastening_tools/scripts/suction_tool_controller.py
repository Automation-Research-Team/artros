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
from actionlib                import ActionServer
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
    def __init__(self):
        super(SuctionToolController, self).__init__()

        self._name = rospy.get_name()
        self._lock = threading.Lock()

        # Initialize ur_control table
        self._goal_threads   = {}
        self._goal_handles   = {}
        self._suction_pubs   = {}
        self._in_ports       = {}
        self._out_ports_suck = {}
        self._out_ports_blow = {}
        for tool_name, tool_props in rospy.get_param('~suction_tools').items():
            if 'digital_in_port' in tool_props:
                in_port = tool_props['digital_in_port']
                self._suction_pubs[in_port] = rospy.Publisher(
                                                  tool_name + '/suctioned',
                                                  Bool, queue_size=1)
                self._in_ports[tool_name]   = in_port
            self._out_ports_suck[tool_name] = tool_props['digital_out_suck']
            if 'digital_out_blow' in tool_props:
                self._out_ports_blow[tool_name] = tool_props['digital_out_blow']
            rospy.loginfo('(%s) Loaded %s', self._name, tool_name)

        # Subscribe and set I/O states.
        self._io_states     = None
        self._io_states_sub = rospy.Subscriber(
                                 'ur_hardware_interface/io_states',
                                 IOStates, self._io_states_cb, queue_size=1)

        # Create a service client for setting digital I/O.
        rospy.wait_for_service('ur_hardware_interface/set_io')
        self._set_io = rospy.ServiceProxy('ur_hardware_interface/set_io',
                                          SetIO)

        # Create an action server for processing commands to suction tools.
        self._server = ActionServer('~command', SuctionToolCommandAction,
                                    self._goal_cb, auto_start=False)
        self._server.register_goal_callback(self._goal_cb)
        self._server.start()
        rospy.loginfo('(%s) controller started', self._name)

    def _io_states_cb(self, io_states):
        # Publish current state for each suction tool.
        for in_state in io_states.digital_in_states:
            if in_state.pin in self._suction_pubs:
                self._suction_pubs[in_state.pin].publish(in_state.state)

        self._io_states = io_states

    def _goal_cb(self, goal_handle):
        goal = goal_handle.get_goal()

        # Check validity of the given tool name.
        if goal.tool_name not in self._out_ports_suck:
            goal_handle.set_rejected()
            rospy.logerr('(%s) goal REJECTED: tool[%s] not found',
                         self._name, goal.tool_name)
            return

        # If any active goal for this tool is currently running, cancel it.
        with self._lock:
            active_goal_thread = self._goal_threads.get(goal.tool_name)
            active_goal_handle = self._goal_handles.get(goal.tool_name)
        if active_goal_thread is not None:
            active_goal_handle = self._goal_handles[goal.tool_name]
            active_goal_handle.set_canceled()
            active_goal_thread.join()
            rospy.logwarn('(%s) current active goal[%s] CANCELED because another goal for tool[%s] is received',
                          self._name,
                          active_goal_handle.get_goal_id().id, goal.tool_name)

        # Accept the new goal.
        goal_handle.set_accepted()
        rospy.loginfo('(%s) new goal ACCEPTED for %s',
                      self._name, goal.tool_name)

        # Launch a control loop for the specified tool with a separate thread.
        goal_thread = threading.Thread(target=self._execute_control,
                                       args=(goal_handle,))
        goal_thread.daemon = True
        with self._lock:
            self._goal_threads[goal.tool_name] = goal_thread
            self._goal_handles[goal.tool_name] = goal_handle
        goal_thread.start()

    def _execute_control(self, goal_handle):
        """
        Execute the suck/blow action.
        """
        goal      = goal_handle.get_goal()
        suck_port = self._out_ports_suck.get(goal.tool_name)
        blow_port = self._out_ports_blow.get(goal.tool_name)

        # Set states of suck and blow ports.
        try:
            self._set_out_state(suck_port, goal.suck)
            self._set_out_state(blow_port, not goal.suck)
        except RuntimeError as err:
            self._set_out_state(suck_port, False)
            self._set_out_state(blow_port, False)
            goal_handle.set_aborted()
            rospy.logerr("(%s) goal ABORTED: %s", self._name, err)

            with self._lock:
                self._goal_handles.pop(goal.tool_name)
                self._goal_threads.pop(goal.tool_name)
            return

        #
        if not goal.suck or goal.tool_name not in self._in_ports:
            goal_handle.set_succeeded()
            rospy.loginfo('(%s) goal SUCCEEDED: no status checking required',
                          self._name)

            with self._lock:
                self._goal_handles.pop(goal.tool_name)
                self._goal_threads.pop(goal.tool_name)
            return

        in_port    = self._in_ports[goal.tool_name]
        start_time = rospy.get_rostime()
        rate       = rospy.Rate(20)

        while not rospy.is_shutdown():
            # Check if the goal is canceled from the client.
            if goal_handle.get_goal_status().status in (GoalStatus.PREEMPTING,
                                                        GoalStatus.RECALLING):
                goal_handle.set_canceled()
                rospy.logwarn('(%s) goal CANCELED by client', self._name)
                break

            # Check if the goal is canceled within this node.
            if goal_handle.get_goal_status().status == GoalStatus.PREEMPTED:
                break

            # Read the state of DIN port for this tool.
            in_state = self._get_in_state(in_port)
            if in_state is None:
                goal_handle.set_aborted()
                rospy.logerr('(%s) goal ABORTED: failed to read DIN port[%d]',
                             self._name, in_port)
                break

            # Publish feedback.
            goal_handle.publish_feedback(SusctionToolCommandFeedback(in_state))

            # If the DIN state has not reached goal state, update start time.
            if in_state != goal.suck:
                start_time = rospy.get_rostime()

            # Check if the goal state has been kept for min_time.
            if goal.min_time > rospy.Duration(0.0) and \
               rospy.get_rostime() - start_time > goal.min_time:
                # Stop blowing.
                if not goal.suck:
                    self._set_out_state(blow_port, False)
                goal_handle.set_succeeded()
                rospy.loginfo('(%s) goal SUCCEEDED: suctioned', self._name)
                break

            rate.sleep()

        # Unregister finished goal handle and this thread.
        with self._lock:
            self._goal_handles.pop(goal.tool_name)
            self._goal_threads.pop(goal.tool_name)

    def _set_out_state(self, port, state):
        if port is None:
            return

        # Wait at most three seconds until the port state is actually set
        # to the specified value.
        start_time = rospy.get_rostime()
        timeout    = rospy.Duration(3.0)
        while rospy.get_rostime() - start_time < timeout:
            res = self._set_io(1, port, 1 if state else 0)
            if not res.success:
                raise RuntimeError('Failed to set DOUT port[%d] to %r'
                                   % (port, state))
            if self._get_out_state(port) == state:
                rospy.loginfo('DOUT port[%d] changed to %r', port, state)
                return

        raise RuntimeError('Failed to set DOUT port[%d] within %f sec'
                           % (port, timeout.to_sec()))

    def _get_out_state(self, port):
        if self._io_states is not None:
            for out_state in self._io_states.digital_out_states:
                if out_state.pin == port:
                    return out_state.state
        return None

    def _get_in_state(self, port):
        if slef._io_states is not None:
            for in_state in self._io_states.digital_in_states:
                if in_state.pin == port:
                    return in_state.state
        return None


#########################################################################
#  Entry point                                                          #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('suction_tool_controller')
    controller = SuctionToolController()
    rospy.spin()
