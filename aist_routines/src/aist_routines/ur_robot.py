#!/usr/bin/env python

# Software License Agreement (BSD License)
n#
# Copyright (c) 2021, OMRON SINIC X
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
#  * Neither the name of OMRON SINIC X nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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
# Author: Cristian C. Beltran-Hernandez, Felix von Drigalski

import rospy
import time
import moveit_commander

from controller_manager_msgs.msg import ListControllers, SwitchController
from std_srvs.srv                import Trigger, TriggerRequest
from ur_control.constants        import TERMINATION_CRITERIA
from ur_control                  import conversions
from ur_dashboard_msgs.srv       import (GetLoadedProgram,
                                         IsProgramRunning, Load, SafetyMode)
from ur_msgs.srv                 import (SetPayload, SetSpeedSliderFraction,
                                         SetIO)
from std_msgs.msg                import Bool

#########################################################################
#  class URRobot                                                        #
#########################################################################
class URRobot(object):
    """ Universal Robots specific implementation of RobotBase
        This class provides access to useful hardware specific services.
        Access to force control and gripper is also defined here.
    """

    def __init__(self, robot_name):
        """
        namespace should be "a_bot" or "b_bot".
        use_real_robot is a boolean
        """
        controller_manager = '/' + robot_name + '/controller_manager/'
        self._list_controllers  = rospy.ServiceProxy(controller_manager
                                                     + 'list_conbtrollers',
                                                     ListControllers)
        self._switch_controller = rospy.ServiceProxy(controller_manager +
                                                     'switch_controller',
                                                     SwitchController)

        hw_interface = '/' + robot_name + '/ur_hardware_interface/'
        self._set_payload       = rospy.ServiceProxy(hw_interface
                                                     + 'set_payload',
                                                     SetPayload)
        self._set_speed_slider  = rospy.ServiceProxy(hw_itnerface
                                                     + 'set_speed_slider',
                                                     SetSpeedSliderFraction)
        self._set_io            = rospy.ServiceProxy(hw_interface + 'set_io',
                                                     SetIO)

        self._safety_mode_sub   = rospy.Subscriber(hw_interface
                                                   + 'safety_mode',
                                                   SafetyMode,
                                                   self._safety_mode_cb)
        self._status_sub        = rospy.Subscriber(hw_interface
                                                   + 'robot_program_running',
                                                   Bool,
                                                   self._ros_control_status_cb)

        dashboard = hw_interface + 'dashboard/'
        self._dashboard_clients = \
        {
            'get_loaded_program':
            rospy.ServiceProxy(dashboard + 'get_loaded_program',
                               GetLoadedProgram),
            'program_running':
            rospy.ServiceProxy(dashboard + 'program_running',
                               IsProgramRunning),
            'load_program':
            rospy.ServiceProxy(dashboard + 'load_program', Load),
            'play':
            rospy.ServiceProxy(dashboard + 'play', Trigger),
            'stop':
            rospy.ServiceProxy(dashboard + 'stop', Trigger),
            'quit':
            rospy.ServiceProxy(dashboard + 'quit', Trigger),
            'connect':
            rospy.ServiceProxy(dashboard + 'connect', Trigger),
            'close_popup':
            rospy.ServiceProxy(dashborad + 'close_popup', Trigger),
            'unlock_protective_stop':
            rospy.ServiceProxy(dashborad + 'unlock_protective_stop', Trigger),
        }

        self._is_ros_control_running = False
        self._safety_mode = None
        self._robot_status = dict()

    ###
    ###  Switching controller stuffs
    ###
    def switch_controller(self, controller):
        for c in self._list_controllers().controller:
            if c.name == controller:
                if c.state == 'stopped':
                    # Force restart
                    rospy.logwarn('Force restart of controller')
                    req = SwitchControllerRequest()
                    req.start_controllers = [controller]
                    req.strictness = 1
                    res = self._switch_controller.call(req)
                    rospy.sleep(1)
                    return res.ok
                else:
                    rospy.loginfo('Controller state is ' + c.state + ', returning True.')
                    return True
        rospy.logerr('Specified controller[%s] not found', controller)
        return False

    ###
    ###  SafetyMode stuffs
    ###
    def _safety_mode_cb(self, msg):
        self._safety_mode = msg.mode

    def is_running_normally(self):
        """
        Returns true if the robot is running (no protective stop, not turned off etc).
        """
        return self._safety_mode == SafetyMode.NORMAL or \
               self._safety_mode == SafetyMode.REDUCED

    def is_protective_stopped(self):
        """
        Returns true if the robot is in protective stop.
        """
        return self._safety_mode == SafetyMode.PROTECTIVE_STOP

    def unlock_protective_stop(self, timeout=rospy.Duration(20.0)):
        rospy.loginfo('Attempting to unlock protective stop')
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and \
              rospy.Time.now() - start_time < timeout:
            res = self._dashboard_clients['unlock_protective_stop']()
            if res.success:
                break
            rospy.sleep(0.2)
        if not res.success:
            rospy.logwarn('Could not unlock protective stop')
        self._dashboard_clients['stop']()
        return res.success

    ###
    ###  Payload stuffs
    ###
    def set_payload(self, mass, center_of_gravity):
        """
            mass float
            center_of_gravity list[3]
        """
        self.activate_ros_control()
        try:
            self._set_payload(mass, conversions.to_vector3(center_of_gravity))
            return True
        except Exception as e:
            rospy.logerr('Exception trying to set payload: %s' % e)
        return False

    ###
    ###  ros_control stuffs
    ###
    def _ros_control_status_cb(self, msg):
        self._is_ros_control_running = msg.data

    def wait_for_control_status_to_turn_on(self, timeout):
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and \
              rospy.Time.now() - start_time < timeout:
            if self._is_ros_control_running:
                return True
            rospy.sleep(.1)
        return False

    def activate_ros_control(self, program_name='ROS_external_control.urp',
                             max_retries=10):
        # Check if URCap is already running on UR
        if self._is_ros_control_running:
            try:
                self._set_speed_slider(1.0)
                return True
            except:
                rospy.logerr('Robot was not found or the robot is not a UR!')
                return False
        rospy.loginfo('ros_control is not running')

        if not self.load_program(program_name, max_retries)
            rospy.logwarn('Failed to load program[%s]', program_name)
            return False

        for i in range(max_retries):
            try:
                rospy.loginfo('Play program[%s]', program_name)
                res = self._dashboard_clients['play']()
                rospy.loginfo('Enter wait_for_control_status_to_turn_on')
                self.wait_for_control_status_to_turn_on(2.0)
                rospy.loginfo('Exited wait_for_control_status_to_turn_on')

                if self.switch_contller('scaled_pos_joint_traj_controller'):
                    rospy.loginfo('Successfully activated ROS control')
                    self._set_speed_slider(1.0)
                    return True
            except:
                pass

            rospy.logwarn('Trying to restart URCap program on UR to restart controllers on ROS side')
            self._dashboard_clients['stop']()

        rospy.logerr()
        return False

    ###
    ###  Load/execute program stuffs
    ###
    def load_and_execute_program(self, program_name='',
                                 skip_ros_activation=False):
        if not skip_ros_activation:
            self.activate_ros_control()
        if not self.load_program(program_name):
            return False
        return self.execute_loaded_program()

    def load_program(self, program_name='', max_retries=10):
        # Try to stop running program
        self._dashboard_clients['stop']()
        rospy.sleep(.5)

        for i in range(max_retries):
            try:
                # Load program if it not loaded already
                res = self._dashboard_clients['get_loaded_program']()
                if res.program_name == '/programs/' + program_name:
                    rospy.loginfo('Specified program[%s] already loaded',
                                  program_name)
                    return True

                res = self._dashboard_clients['load_program'](program_name)
                if res.success:
                    rospy.loginfo('Specified program[%s] successfully loaded',
                                  program_name)
                    return True

                res = self._dashboard_clients['quit']()
                rospy.sleep(.5)
                res = self._dashboard_clients['connect']()
                rospy.sleep(.5)
            except:
                rospy.logwarn('Dashboard service did not respond to load_program/quit/connect!')
            rospy.logwarn('Waiting and trying again')
            rospy.sleep(3)

        rospy.logerr('Failed to load program[%s]. Is the UR in Remote Control mode and program installed with correct name?',
                     program_name)
        return False

    def execute_loaded_program(self):
        # Run the program
        try:
            if self._dashboard_clients['play']().success:
                rospy.loginfo('Successfully started program.')
                return True
            rospy.logerr('Could not start program. Is the UR in Remote Control mode and program installed with correct name?')
            return False
        except Exception as e:
            rospy.logerr(str(e))
            return False

    def close_ur_popup(self):
        # Close a popup on the teach pendant to continue program execution
        if self._dashboard_clients['close_popup']().success:
            rospy.loginfo('Successfully closed popup on teach pendant')
            return True
        else:
            rospy.logerr('Could not close popup.')
            return False
