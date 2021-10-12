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
import lib_robotis_xm430 as xm430
from sensor_msgs  import msg as smsg
from control_msgs import msg as cmsg
from actionlib    import SimpleActionServer
from collections  import namedtuple

#########################################################################
#  class PrecisionGripperController                                     #
#########################################################################
class PrecisionGripperController(object):
    Status = namedtuple('Status', 'pos vel cur mov')

    def __init__(self):
        super(PrecisionGripperController, self).__init__()

        self._name = rospy.get_name()

        # Hardware initialization
        serial_port = rospy.get_param('~serial_port', '/dev/USB0')
        dynamixel   = xm430.USB2Dynamixel_Device(serial_port, baudrate=57600)
        rospy.loginfo('(%s) Starting up on serial port: %s' % (self._name,
                                                               serial_port))
        self._servo = xm430.Robotis_Servo2(dynamixel, 1, series='XM')
        self._servo.set_operating_mode('currentposition')
        self._servo.set_positive_direction('cw')

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

        # Publish joint state
        self._joint_name      = rospy.get_param('~joint_name', 'finger_joint')
        self._joint_state_pub = rospy.Publisher('/joint_states',
                                                smsg.JointState, queue_size=1)

        # Define the action
        self._server = SimpleActionServer('~gripper_cmd',
                                          cmsg.GripperCommandAction,
                                          auto_start=False)
        self._server.register_goal_callback(self._goal_cb)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self._goal_pos = 0

        # Status timer
        rate = rospy.get_param('~publish_rate', 50)
        self._timer = rospy.Timer(rospy.Duration(1.0/rate), self._timer_cb)

        rospy.loginfo('(%s) Started' % self._name)

    def _timer_cb(self, timer_event):
        # Get current status of gripper
        try:
            status = self._get_status()
        except Exception as e:
            rospy.logerr('(%s) failed to get status: %s' % (self._name, e))
            if self._server.is_active():
                rospy.logerr('(%s) aborted goal' % self._name)
                self._server.set_aborted()
            return

        # Publish joint states
        joint_state = smsg.JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name         = [self._joint_name]
        joint_state.position     = [self._position(status)]
        joint_state.velocity     = [0.0]
        joint_state.effort       = [self._effort(status)]
        self._joint_state_pub.publish(joint_state)

        # Handle active goal
        if self._server.is_active():
            rospy.loginfo(status)

            self._server.publish_feedback(
                cmsg.GripperCommandFeedback(*self._status_values(status)))

            if self._is_moving(status):
                self._last_movement_time = rospy.Time.now()
            elif self._reached_goal(status):
                rospy.loginfo('(%s) reached goal' % self._name)
                self._server.set_succeeded(
                    cmsg.GripperCommandResult(*self._status_values(status)))
            elif self._stalled(status):
                rospy.loginfo('(%s) stalled' % self._name)
                self._server.set_succeeded(
                    cmsg.GripperCommandResult(*self._status_values(status)))

    def _goal_cb(self):
        goal = self._server.accept_new_goal()  # requested goal
        rospy.loginfo('(%s) accepted new goal' % self._name)

        # Check that preempt has not been requested by the client
        if self._server.is_preempt_requested():
            rospy.logwarn('(%s) preempt requested' % self._name)
            self._server.set_preempted()
            return

        try:
            self._last_movement_time = rospy.Time.now()
            self._goal_pos = self._send_move_command(goal.command.position,
                                                     goal.command.max_effort)
        except Exception as e:
            rospy.logerr('(%s) failed to send move command: %s' % (self._name,
                                                                   e))
            rospy.logerr('(%s) aborted goal' % self._name)
            self._server.set_aborted()

    def _preempt_cb(self):
        self._stop()
        rospy.logwarn('(%s) preempted' % self._name)
        self._server.set_preempted()

    def _send_move_command(self, position, effort):
        pos = np.clip(int((position - self._min_position) /
                          self.position_per_tick + self._min_pos),
                      self._max_pos, self._min_pos)
        cur = np.clip(int(effort / self.effort_per_tick),
                      -self._max_cur, self._max_cur)
        if abs(cur) < self._min_cur:
            pos_now = np.int32(self._servo.read_current_position())
            cur = self._min_cur if pos > pos_now else -self._min_cur
        rospy.loginfo('** Cmd(pos={}, cur={}) for position={}, effort={}'
                      .format(pos, cur, position, effort))
        self._servo.set_current(cur)
        self._servo.set_goal_position(pos)
        return pos

    def _stop(self):
        self._servo.set_current(0)

    def _get_status(self):
        return PrecisionGripperController.Status(
                   np.int32(self._servo.read_current_position()),
                   np.int32(self._servo.read_current_velocity()),
                   np.int16(self._servo.read_current()),
                   self._servo.is_moving())

    def _position(self, status):
        return (status.pos - self._min_pos) * self.position_per_tick \
             + self._min_position

    def _effort(self, status):
        return status.cur * self.effort_per_tick

    def _is_moving(self, status):
        return status.mov

    def _reached_goal(self, status):
        return (not status.mov) and abs(status.pos - self._goal_pos) <= 1

    def _stalled(self, status):
        return (not status.mov) and \
               (rospy.Time.now() - self._last_movement_time >
                self._stall_timeout)

    def _status_values(self, status):
        return self._position(status), self._effort(status), \
               self._stalled(status),  self._reached_goal(status)

    def _error(self, status):
        return status.err

    @property
    def position_per_tick(self):
        return (self._max_position - self._min_position) \
             / (self._max_pos      - self._min_pos)

    @property
    def effort_per_tick(self):
        return self._max_effort / self._max_cur


if __name__ == '__main__':
    rospy.init_node('precision_gripper_controller')
    try:
        controller = PrecisionGripperController()
        rospy.spin()
    except Exception as e:
        rospy.logerr(e)
