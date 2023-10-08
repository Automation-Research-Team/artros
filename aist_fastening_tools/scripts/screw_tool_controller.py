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
import numpy as np

from actionlib                    import SimpleActionServer
from actionlib_msgs.msg           import GoalStatus
from aist_fastening_tools.msg     import (ScrewToolCommandAction,
                                          ScrewToolCommandResult,
                                          ScrewToolCommandFeedback)
from collections                  import deque
from dynamixel_workbench_msgs.msg import DynamixelState, DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand
from scipy.signal                 import butter, lfiltic, lfilter

#########################################################################
#  class ButterworthLPF                                                 #
#########################################################################
class ButterworthLPF(object):
    """
    Butterworth lowpass digital filter design.

    Check C{scipy.signal.butter} for further details.
    """
    def __init__(self, cutoff, fs, order=5):
        """
        C{ButterLowPass} constructor

        @type  cutoff: float
        @param cutoff: Cut-off frequency in Hz
        @type  fs:     float
        @param fs:     The sampling frequency (Hz) of the signal to be filtered
        @type  order:  int
        @param order:  The order of the filter.
        """
        super(ButterworthLPF, self).__init__()

        nyq           = 0.5 * fs
        normal_cutoff = cutoff / nyq
        self._b, self._a = butter(order, normal_cutoff,
                                  btype='low', analog=False)

    def __call__(self, x):
        """
        Filters the input array across its C{axis=0} (each column is
        considered as an independent signal). Uses initial conditions (C{zi})
        for the filter delays.

        @type  x: array
        @param x: An N-dimensional input array.
        @rtype:  array
        @return: The output of the digital filter.
        """
        if not hasattr(self, 'zi'):
            cols = x.shape[1]
            zi   = lfiltic(self._b, self._a, []).tolist() * cols
            self._zi = np.array(lfiltic(self._b, self._a, []).tolist() * cols)
            self._zi.shape = (-1, cols)
        filtered, self._zi = lfilter(self._b, self._a, x, zi=self._zi, axis=0)
        return filtered

#########################################################################
#  class ScrewToolController                                            #
#########################################################################
class ScrewToolController(object):
    def __init__(self):
        super(ScrewToolController, self).__init__()

        self._name = rospy.get_name()

        # Set configuration parameters.
        self._history_length    = rospy.get_param('~history_length',     2)
        self._speed_threshold   = rospy.get_param('~speed_threshold',   18)
        self._current_threshold = rospy.get_param('~current_threshold', 10)
        self._motor_id          = rospy.get_param('~motor_id',           1)

        # Create a low-pass filter for current published as feedback.
        cutoff = rospy.get_param('~filter_cutoff', 2.5)
        rate   = rospy.get_param('~filter_rate',   100.0)
        order  = rospy.get_param('~filter_order',  2)
        self._current_filter = ButterworthLPF(cutoff, rate, order)
        self._current_queue  = deque(maxlen=win_size)

        # Create a subscriber for receiving state of Dynamixel driver.
        driver_ns = rospy.get_param("~driver_ns", "screw_tool_driver")
        self._dynamixel_state_list = None
        self._dynamixel_state_sub  = rospy.Subscriber(driver_ns
                                                      + '/dynamixel_state',
                                                      DynamixelStateList,
                                                      self._state_list_cb)

        # Create a service client for sending command to Dynamixel driver.
        rospy.wait_for_service(driver_ns + '/dynamixel_command')
        self._dynamixel_command = rospy.ServiceProxy(driver_ns
                                                     + '/dynamixel_command',
                                                     DynamixelCommand)

        # Create an action server for processing commands to screw tools.
        self._server = SimpleActionServer('~command', ScrewToolCommandAction,
                                          auto_start=False)
        self._server.register_goal_callback(self._goal_cb)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()

        rospy.loginfo('(%s) started', self._name)

    def _state_list_cb(self, state_list):
        # Apply low-pass filter to incoming current signal values.
        original = [state.present_current
                    for state in state_list.dynamixel_state]
        filtered = self._current_filter(original)

        # Store state list with filtered current signal values.
        self._dynamixel_states = [DynamixelState(state.name, state.id,
                                                 state.present_position,
                                                 state.present_velocity,
                                                 current)
                                  for state, current
                                  in zip(state_list.dynamixel_state, filtered)]

    def _goal_cb(self, goal_handle):
        goal = goal_handle.get_goal()

        # Check validity of the given tool name.
        if goal.tool_name not in self._motor_ids:
            goal_handle.set_rejected()
            rospy.logerr('(%s) goal REJECTED: tool[%s] not found',
                         self._name, goal.tool_name)
            return

        # If any active goal for this tool is currently running, cancel it.
        with self._lock:
            active_goal_thread = self._goal_threads.get(goal.tool_name)
            active_goal_handle = self._goal_handles.get(goal.tool_name)
        if active_goal_thread is not None:
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
                                       args=(goal_handle, goal.retighten))
        goal_thread.daemon = True
        with self._lock:
            self._goal_threads[goal.tool_name] = goal_thread
            self._goal_handles[goal.tool_name] = goal_handle
        goal_thread.start()

    def _execute_control(self, goal_handle, retighten):
        """
        Execute the tighten/loosen action.
        If retighten is True, after fastening has finished successfully,
        the screw is loosened once and then fastened again.
        """
        goal = goal_handle.get_goal()

        if not goal.speed:
            goal.speed = 1023  # Maximum speed

        motor_id = self._motor_ids[goal.tool_name]

        # For tightening, i.e. CCW rotation, the maximum speed is 1023.
        # For loosen, i.e. CW rotation, it is 2047.
        # At 1024 the motor is stopped.
        target_speed = np.clip(goal.speed, 0, 1023) \
                     + (0 if goal.tighten else 1024)

        # Set target speed.
        self._set_value(motor_id, 'Moving_Speed', target_speed)

        # Initialize to start the loop
        state_history = deque(maxlen=self._history_length)
        rate          = rospy.Rate(20)

        # Wait for motor to start up (and avoid reading incorrect speed values)
        rospy.sleep(1)
        while not rospy.is_shutdown():
            # Check if the goal is canceled from the client.
            if goal_handle.get_goal_status().status in (GoalStatus.PREEMPTING,
                                                        GoalStatus.RECALLING):
                goal_handle.set_canceled()
                rospy.logwarn('(%s) goal CANCELED by client', self._name)
                break

            # Check if the goal is canceled within the server.
            if goal_handle.get_goal_status().status == GoalStatus.PREEMPTED:
                # Return without setting zero speed, because another goal
                # has already been activated and target speed has been set.
                break
                #return

            # Read the state of Dynamixel and keep it in the queue.
            state = self._get_dynamixel_state(motor_id)
            if state is None:
                goal_handle.set_aborted()
                rospy.logerr('(%s) goal ABORTED: error in motor state readout',
                             self._name)
                break
            state_history.append(state)

            # Publish feedback.
            goal_handle.publish_feedback(ScrewToolCommandFeedback(
                                             state.present_velocity,
                                             state.present_current))

            if goal.tighten:
                # Check if the motor is stalled.
                if len(state_history) == state_history.maxlen and \
                   all(state.present_velocity < self._speed_threshold
                       for state in state_history):
                    if retighten:
                        # Turn into loosening direction for 1sec.
                        self._set_value(motor_id, 'Moving_Speed',
                                        target_speed + 1024)
                        rospy.sleep(1.0)
                        self._set_value(motor_id, 'Moving_Speed', 0)  # Stop

                        # Tighten again.
                        self.execute_control(goal_handle, False)
                        return
                    goal_handle.set_succeeded(ScrewToolCommandResult(True))
                    rospy.loginfo('(%s) goal SUCCEEDED: screw tightened',
                                  self._name)
                    break
            else:
                # Check if the motor is load free.
                if len(state_history) == state_history.maxlen and \
                   all(state.present_current < self._current_threshold
                       for state in state_history):
                    goal_handle.set_succeeded(ScrewToolCommandResult(False))
                    rospy.loginfo('(%s) goal SUCCEEDED: screw released',
                                  self._name)
                    break

            rate.sleep()

        # Stop rotation and disable torque.
        self._set_value(motor_id, 'Moving_Speed',  0)
        self._set_value(motor_id, 'Torque_Enable', 0)

        with self._lock:
            self._goal_handles.pop(goal.tool_name)
            self._goal_threads.pop(goal.tool_name)

    def _set_value(self, id, addr_name, value):
        try:
            res = self._dynamixel_command('', id, addr_name, value)
        except rospy.ServiceException as err:
            rospy.logerr('(%s) failed to set value[%d] to %s: %s',
                         self._name, value, addr_name, err)
            return False

        if res.comm_result:
            rospy.loginfo('(%s) succesfully set value[%d] to %s',
                          self._name, value, addr_name)
        else:
            rospy.logerr('(%s) communication error when setting value[%d] to %s',
                         self._name, value, addr_name)
        return res.comm_result

    def _get_dynamixel_state(self, motor_id):
        if not self._dynamixel_states:
            return None

        # Find states with the motor ID of this tool.
        states = [state for state in self._dynamixel_states
                  if state.id == motor_id]
        if not states:
            rospy.logerr('(%s) dynamixel state with ID=%d not found in the received state list',
                         self._name, motor_id)
            return None
        return states[0]


#########################################################################
#  Entry point                                                          #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('screw_tool_controller')
    controller = ScrewToolController()
    rospy.spin()
