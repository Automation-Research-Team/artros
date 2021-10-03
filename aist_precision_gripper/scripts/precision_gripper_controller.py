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
from aist_robotiq import msg as amsg
from actionlib    import SimpleActionServer

#########################################################################
#  class Status                                                         #
#########################################################################
class Status:
    def __init__(self, pos, vel, cur, mov, err):
        self.pos = pos
        self.vel = vel
        self.cur = cur
        self.mov = mov  # True if moving
        self.err = err  # True if error

#########################################################################
#  class PrecisionGripperController                                     #
#########################################################################
class PrecisionGripperController(object):
    def __init__(self):
        super(PrecisionGripperController, self).__init__()

        self._name = rospy.get_name()

        # Hardware initialization
        serial_port = rospy.get_param('~serial_port', '/dev/USB0')
        dynamixel   = xm430.USB2Dynamixel_Device(serial_port, baudrate=57600)
        self._servo    = xm430.Robotis_Servo2(dynamixel, 1, series='XM')
        rospy.loginfo('(%s) Starting up on serial port: %s' % (self._name,
                                                               serial_port))

        # Read configuration parameters
        self._min_position = rospy.get_param('~min_position', 0.000)
        self._max_position = rospy.get_param('~max_position', 0.010)
        self._min_velocity = rospy.get_param('~min_velocity', 0.000)
        self._max_velocity = rospy.get_param('~max_velocity', 0.010)
        self._min_effort   = rospy.get_param('~min_effort',   0.1)
        self._max_effort   = rospy.get_param('~max_effort',   1.0)
        self._joint_name   = rospy.get_param('~joint_name',   'finger_joint')

        # Read servo parameters
        self._min_pos = rospy.get_param("~open_motor_position",  2500)
        self._max_pos = rospy.get_param("~close_motor_position", 2222)
        self._min_vel = 0
        self._max_vel = rospy.get_param("~speed_limit", 4096)
        self._min_cur = 0
        self._max_cur = rospy.get_param("~grasping_force", 5)

        # Publish joint state
        self._joint_state_pub = rospy.Publisher('/joint_states',
                                                smsg.JointState, queue_size=1)
        self._goal_pos        = 0

        # Define the action
        self._server = SimpleActionServer('~gripper_cmd',
                                          cmsg.GripperCommandAction,
                                          auto_start=False)
        self._server.register_goal_callback(self._goal_cb)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self._action_server.start()

        # Status timer
        self._timer = rospy.Timer(rospy.Duration(0.05), self._status_cb)

        rospy.loginfo('(%s) Started' % self._name)

    def _status_cb(self, timer_event):
        # Get current status of gripper
        status = self._get_status()

        # Handle active goal
        if self._server.is_active():
            if self._error(status):
                rospy.logwarn('(%s) faulted with code: %x'
                              % (self._name, self._error(status)))
                self._server.set_aborted()
                return
            elif self._reached_goal(status):
                rospy.loginfo('(%s) reached goal' % self._name)
                self._server.set_succeeded(
                    cmsg.GripperCommandResult(*self._status_values()))
            elif self._stalled(status):
                rospy.loginfo('(%s) stalled' % self._name)
                self._server.set_succeeded(
                    cmsg.GripperCommandResult(*self._status_values(status)))
            else:
                self._server.publish_feedback(
                    cmsg.GripperCommandFeedback(*self._status_values(status)))

        # Publish joint states
        joint_state = smsg.JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name         = [self._joint_name]
        joint_state.position     = [self._position(status)]
        joint_state.velocity     = [self._velocity(status)]
        joint_state.effort       = [self._effort(status)]
        self._joint_state_pub.publish(joint_state)

    def _send_move_command(self, position, effort):
        pos = np.clip(int((position - self._min_posion)/self.position_per_tick
                          + self._min_pos),
                      self._min_pos, self._max_pos)
        eff = np.clip(int((effort - self._min_effort)/self.effort_per_tick
                          + self._min_cur),
                      self._min_cur, self._max_cur)
        self._send_raw_move_commnt(pos, eff)
        return pos

    def _send_raw_move_command(self, pos, eff):
        try:
            self._servo.set_operating_mode("currentposition")
            self._servo.set_positive_direction("cw")
            self._servo.set_current(eff)
            self._servo.set_goal_position(pos)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("(%s) failed to run commands." % self._name)
            return False

    def _get_status(self):
        try:
            return Status(self._servo.read_current_position(),
                          self._servo.read_current_velocity(),
                          self._servo.read_current(),
                          self._servo.is_moving(),
                          False)
        except:
            rospy.logerr("(%s) failed to get status." % self._name)
            return Status(0, 0, 0, False, True)

    def _position(self, status):
        return (status.pos - self._min_pos)*self.position_per_tick \
             + self._min_positon

    def _velocity(self, status):
        return (status.vel - self._min_vel)*self.velocity_per_tick \
             + self._min_velocity

    def _effort(self, status):
        return (status.cur - self._min_cur)*self.effort_per_tick \
             + self._min_effort

    def _stalled(self, status):
        return not self.mov

    def _reached_goal(self, status):
        return abs(status.pos - self._goal_pos) <= 1

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
    def velocity_per_tick(self):
        return (self._max_velocity - self._min_velocity) \
             / (self._max_vel      - self._min_vel)

    @property
    def effort_per_tick(self):
        return (self._max_effort  - self._min_effort) \
             / (self._max_current - self._min_current)

    def _disable_torque(self):
        try:
            self._servo.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

    def _signum(self, x):
        if x > 0:
            return 1.0
        elif x < 0:
            return -1.0
        else:
            return x


if __name__ == '__main__':
    rospy.init_node('precision_gripper_controller')
    controller = PrecisionGripperController()
    rospy.spin()
