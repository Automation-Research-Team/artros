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
import rospy, os
import numpy as np

from sensor_msgs.msg              import JointState
from control_msgs.msg             import (GripperCommandAction,
                                          GripperCommandGoal,
                                          GripperCommandResult,
                                          GripperCommandFeedback)
from actionlib                    import SimpleActionServer

from dynamixel_workbench_msgs.msg import DynamixelState, DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand


class PrecisionGripperController(object):
    def __init__(self):
        super(PrecisionGripperController, self).__init__()

        self._name = rospy.get_name()

        # Read motor id
        self._motor_id = rospy.get_param('~ID')

        # Read timeout value for checking stalled state
        self._stall_timeout = rospy.Duration.from_sec(
                                rospy.get_param('~stall_timeout', 1.0))

        # Read configuration parameters
        self._min_position = rospy.get_param('~min_position', 0.000)
        self._max_position = rospy.get_param('~max_position', 0.010)
        self._max_effort   = rospy.get_param('~max_effort',   0.5)

        # Read servo parameters
        self._min_pos = rospy.get_param('~min_position_count', 2300)
        self._max_pos = rospy.get_param('~max_position_count', 2050)
        self._min_cur = rospy.get_param('~min_effort_count',   3)
        self._max_cur = rospy.get_param('~max_effort_count',   13)

        # Create a subscriber for receiving state of Dynamixel driver.
        driver_ns = rospy.get_param("~driver_ns")
        self._dynamixel_state_topic = driver_ns + '/dynamixel_state'
        self._dynamixel_state       = None
        self._dynamixel_state_sub = rospy.Subscriber(
                                        self._dynamixel_state_topic,
                                        rospy.AnyMsg,
                                        self._test_dynamixel_state_cb)

        # Create a service client for sending command to Dynamixel driver
        service_name = driver_ns + '/dynamixel_command'
        self._dynamixel_command = rospy.ServiceProxy(service_name,
                                                           DynamixelCommand)
        rospy.wait_for_service(service_name)

        # Publish joint state
        self._joint_state_pub = rospy.Publisher('/joint_states',
                                                JointState, queue_size=1)

        # Define the action
        self._server = SimpleActionServer('~gripper_cmd', GripperCommandAction,
                                          auto_start=False)
        self._server.register_goal_callback(self._goal_cb)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self._goal_pos = 0

        rospy.loginfo('(%s) controller started', self._name)

    def _test_dynamixel_state_cb(self, data):
        '''
        This function is executed when a message is received by _state_sub.
        It determines the actual ROS message type that is used in the topic
        and creates a subscriber with the
        given type, to read the status of the motors.
        '''
        message_type = data._connection_header['type']
        self._dynamixel_state_sub.unregister()
        if message_type == 'dynamixel_workbench_msgs/DynamixelStateList':
            self._dynamixel_state     = DynamixelStateList()
            self._dynamixel_state_sub = rospy.Subscriber(
                                            self._dynamixel_state_topic,
                                            DynamixelStateList,
                                            self._dynamixel_state_list_cb)
        elif message_type == 'dynamixel_workbench_msgs/DynamixelState':
            self._dynamixel_state     = DynamixelState()
            self._dynamixel_state_sub = rospy.Subscriber(
                                            self._dynamixel_state_topic,
                                            DynamixelState,
                                            self._dynamixel_state_cb)
        else:
            rospy.logerr('(%s) unexpected message type[%s]',
                         self._name, message_type)

    def _dynamixel_state_list_cb(self, state_list):
        self._dynamixel_state_cb(state_list.dynamixel_state[0])

    def _dynamixel_state_cb(self, state):
        # Keep new state
        self._dynamixel_state = state

        # Publish joint state
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name         = [state.name + '_finger_joint']
        joint_state.position     = [self._position(state)]
        joint_state.velocity     = [0.0]
        joint_state.effort       = [self._effort(state)]
        self._joint_state_pub.publish(joint_state)

        # Handle active goal
        if self._server.is_active():
            self._server.publish_feedback(
                GripperCommandFeedback(*self._state_values(state)))

            if self._is_moving(state):
                self._last_movement_time = rospy.Time.now()
            elif self._reached_goal(state):
                self._server.set_succeeded(
                    GripperCommandResult(*self._state_values(state)))
                rospy.loginfo('(%s) SUCCEEDED: reached goal position',
                              self._name)
            elif self._stalled(state):
                self._server.set_succeeded(
                    GripperCommandResult(*self._state_values(state)))
                rospy.loginfo('(%s) SUCCEEDED: stalled', self._name)

    def _goal_cb(self):
        goal = self._server.accept_new_goal()  # requested goal
        rospy.loginfo('(%s) ACCEPTED new goal', self._name)

        # Check that preempt has not been requested by the client
        if self._server.is_preempt_requested():
            self._server.set_preempted()
            rospy.logwarn('(%s) PREEMPT REQUESTED', self._name)
            return

        try:
            self._last_movement_time = rospy.Time.now()
            self._goal_pos = self._send_move_command(goal.command.position,
                                                     goal.command.max_effort)
        except Exception as e:
            rospy.logerr('(%s) failed to send move command: %s', self._name, e)
            self._server.set_aborted()
            rospy.logerr('(%s) ABORTED goal', self._name)

    def _preempt_cb(self):
        self._stop()
        rospy.logwarn('(%s) PREEMPTED', self._name)
        self._server.set_preempted()

    def _send_move_command(self, position, effort):
        pos = np.clip(int((position - self._min_position) /
                          self.position_per_tick + self._min_pos),
                      self._max_pos, self._min_pos)
        cur = np.clip(int(effort / self.effort_per_tick),
                      -self._max_cur, self._max_cur)
        if abs(cur) < self._min_cur:
            pos_now = np.int32(self._position(self._dynamixel_state))
            cur = self._min_cur if pos > pos_now else -self._min_cur
        rospy.loginfo('** Cmd(pos=%i, cur=%i) for position=%f, effort=%f',
                      pos, cur, position, effort)
        self._set_value('Torque_Enable', cur)
        self._set_value('Goal_Position', pos)
        return pos

    def _set_value(self, addr_name, value):
        try:
            res = self._dynamixel_command('', self._motor_id, addr_name, value)
        except rospy.ServiceException as err:
            rospy.logerr('(%s) failed to set value[%i] to %s: %s',
                         self._name, value, addr_name, err)
            return False

        if res.comm_result:
            rospy.loginfo("(%s) succesfully set value[%i] to %s",
                          self._name, value, addr_name)
        else:
            rospy.logerr('(%s) communication error when setting value[%i] to %s',
                         self._name, value, addr_name)
        return res.comm_result

    def _position(self, state):
        return (state.present_position - self._min_pos) \
             * self.position_per_tick \
             + self._min_position

    def _effort(self, state):
        return state.present_current * self.effort_per_tick

    def _is_moving(self, state):
        return state.present_velocity != 0

    def _reached_goal(self, state):
        return (not self._is_moving(state)) and \
               abs(state.present_position - self._goal_pos) <= 1

    def _stalled(self, state):
        return (not self._is_moving(state)) and \
               (rospy.Time.now() - self._last_movement_time >
                self._stall_timeout)

    def _state_values(self, state):
        return self._position(state), self._effort(state), \
               self._stalled(state),  self._reached_goal(state)

    @property
    def position_per_tick(self):
        return (self._max_position - self._min_position) \
             / (self._max_pos      - self._min_pos)

    @property
    def effort_per_tick(self):
        return self._max_effort / self._max_cur


if __name__ == '__main__':
    rospy.init_node('precision_gripper_controller')
    controller = PrecisionGripperController()
    rospy.spin()
