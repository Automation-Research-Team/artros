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
from aist_routines                   import AISTBaseRoutines
from aist_routines.URScriptPublisher import URScriptPublisher
from controller_manager_msgs.srv     import ListControllers, SwitchController
from std_srvs.srv                    import Trigger, TriggerRequest
from ur_dashboard_msgs.srv           import (GetLoadedProgram,
                                             IsProgramRunning, Load)
from ur_dashboard_msgs.msg           import SafetyMode
from ur_msgs.srv                     import (SetPayload, SetSpeedSliderFraction,
                                             SetIO)
from std_msgs.msg                    import Bool

######################################################################
#  class URRobot                                                     #
######################################################################
class URRobot(object):
    def __init__(self, robot_name):
        super(URRobot, self).__init__()

        controller_manager = '/' + robot_name + '/controller_manager/'
        hw_interface       = '/' + robot_name + '/ur_hardware_interface/'
        dashboard          = hw_interface + 'dashboard/'

        self._urscript_publisher = URScriptPublisher(robot_name)

        self._list_controllers   = rospy.ServiceProxy(controller_manager
                                                      + 'list_conbtrollers',
                                                      ListControllers)
        self._switch_controller  = rospy.ServiceProxy(controller_manager
                                                      + 'switch_controller',
                                                      ListControllers)

        self._set_paylod         = rospy.ServiceProxy(hw_interface
                                                      + 'set_payload',
                                                      SetPayload)
        self._set_speed_slider   = rospy.ServiceProxy(hw_interface
                                                      + 'set_speed_slider',
                                                      SetSpeedSliderFraction),
        self._set_io             = rospy.ServiceProxy(hw_interface + 'set_io',
                                                      SetIO),

        self._get_loaded_program = rospy.ServiceProxy(dashboard
                                                      + 'get_loaded_program',
                                                      GetLoadedProgram)
        self._program_running    = rospy.ServiceProxy(dashboard
                                                      + 'program_running',
                                                      IsProgramRunning)
        self._load_program       = rospy.ServiceProxy(dashboard
                                                      + 'load_program', Load),
        self._play               = rospy.ServiceProxy(dashboard + 'play',
                                                      Trigger)
        self._stop               = rospy.ServiceProxy(dashboard + 'stop',
                                                      Trigger)
        self._quit               = rospy.ServiceProxy(dashboard + 'quit',
                                                      Trigger)
        self._connect            = rospy.ServiceProxy(dashboard + 'connect',
                                                      Trigger)
        self._close_popup        = rospy.ServiceProxy(dashboard + 'close_popup',
                                                      Trigger),
        self._unlock_protective_stop \
            = rospy.ServiceProxy(dashboard + 'unlock_protective_stop', Trigger)

        self._safety_mode_sub \
            = rospy.Subscriber(hw_interface + 'safety_mode',
                               SafetyMode, self._safety_mode_cb)
        self._robot_program_running_sub \
            = rospy.Subscriber(hw_interface + 'robot_program_running',
                               Bool, self._robot_program_running_cb)
        self._safety_mode = None
        self._is_robot_program_running = False

    @property
    def urscript_publisher(self):
        return self._urscript_publisher

    ###
    ###  Switching controller stuffs
    ###
    def switch_controller(self, controller_name):
        for controller in self._list_controllers().controller:
            if controller.name == controller_name:
                if controller.state == 'stopped':
                    # Force restart
                    rospy.logwarn('Force restart of controller')
                    req = SwitchControllerRequest()
                    req.start_controllers = [controller_name]
                    req.strictness = 1
                    res = self._switch_controller.call(req)
                    rospy.sleep(1)
                    return res.ok
                else:
                    rospy.loginfo('Controller state is ' + controller.state + ', returning True.')
                    return True
        rospy.logerr('Specified controller[%s] not found', controller_name)
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
            res = self._unlock_protective_stop()
            if res.success:
                break
            rospy.sleep(0.2)
        if not res.success:
            rospy.logwarn('Could not unlock protective stop')
        self._stop()
        return res.success

    ###
    ###  Payload stuffs
    ###
    def set_payload(self, mass, center_of_gravity):
        """
            mass float
            center_of_gravity list[3]
        """
        self.activate_external_control()
        try:
            self._set_payload(mass, center_of_gravity)
            return True
        except Exception as e:
            rospy.logerr('Exception trying to set payload: %s' % e)
        return False

    ###
    ###  robot_program_running stuffs
    ###
    def _robot_program_running_cb(self, msg):
        self._robot_program_running = msg.data

    def wait_for_control_status_to_turn_on(self, timeout):
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and \
              rospy.Time.now() - start_time < timeout:
            if self._robot_program_running:
                return True
            rospy.sleep(.1)
        return False

    def activate_external_control(self, program_name='ROS_external_control.urp',
                             max_retries=10):
        # Check if URCap is already running on UR
        if self._robot_program_running:
            try:
                self._set_speed_slider(1.0)
                return True
            except:
                rospy.logerr('Robot was not found or the robot is not a UR!')
                return False
        rospy.loginfo('ros_control is not running')

        if not self.load_program(program_name, max_retries):
            rospy.logwarn('Failed to load program[%s]', program_name)
            return False

        for i in range(max_retries):
            try:
                rospy.loginfo('Play program[%s]', program_name)
                res = self._play()
                rospy.loginfo('Enter wait_for_control_status_to_turn_on')
                self.wait_for_control_status_to_turn_on(2.0)
                rospy.loginfo('Exited wait_for_control_status_to_turn_on')

                if self.switch_controller('scaled_pos_joint_traj_controller'):
                    rospy.loginfo('Successfully activated ROS control')
                    self._set_speed_slider(1.0)
                    return True
            except:
                pass

            rospy.logwarn('Trying to restart URCap program on UR to restart controllers on ROS side')
            self._stop()

        rospy.logerr()
        return False

    ###
    ###  Load/execute program stuffs
    ###
    def load_and_execute_program(self, program_name='',
                                 skip_external_activation=False):
        if not skip_external_activation:
            self.activate_external_control()
        if not self.load_program(program_name):
            return False
        return self.execute_program()

    def load_program(self, program_name='', max_retries=10):
        # Try to stop running program
        self._stop()
        rospy.sleep(.5)

        for i in range(max_retries):
            try:
                # Load program if it not loaded already
                res = self._get_loaded_program()
                if res.program_name == '/programs/' + program_name:
                    rospy.loginfo('Specified program[%s] already loaded',
                                  program_name)
                    return True

                res = self._load_program(program_name)
                if res.success:
                    rospy.loginfo('Specified program[%s] successfully loaded',
                                  program_name)
                    return True

                res = self._quit()
                rospy.sleep(.5)
                res = self._connect()
                rospy.sleep(.5)
            except:
                rospy.logwarn('Dashboard service did not respond to load_program/quit/connect!')
            rospy.logwarn('Waiting and trying again')
            rospy.sleep(3)

        rospy.logerr('Failed to load program[%s]. Is the UR in Remote Control mode and program installed with correct name?',
                     program_name)
        return False

    def execute_program(self):
        # Run the program
        try:
            if self._play().success:
                rospy.loginfo('Successfully started program.')
                return True
            rospy.logerr('Could not start program. Is the UR in Remote Control mode and program installed with correct name?')
            return False
        except Exception as e:
            rospy.logerr(str(e))
            return False

    def close_ur_popup(self):
        # Close a popup on the teach pendant to continue program execution
        if self._close_popup().success:
            rospy.loginfo('Successfully closed popup on teach pendant')
            return True
        else:
            rospy.logerr('Could not close popup.')
            return False

######################################################################
#  class URRoutines                                                  #
######################################################################
class URRoutines(AISTBaseRoutines):
    def __init__(self):
        super(URRoutines, self).__init__()

        self._ur_robots = {}
        d = rospy.get_param('~robots', {})
        for robot_name in d:
            self._ur_robots[robot_name] = URRobot(robot_name)

    # Interactive stuffs
    def print_help_messages(self):
        super(URRoutines, self).print_help_messages()
        print('=== UR specific commands ===')
        print('  activate: Activate external control URCap')
        print('  s: Search graspabilities')
        print('  a: Attempt to pick and place')
        print('  A: Repeat attempts to pick and place')
        print('  d: Perform small demo')
        print('  H: Move all robots to home')
        print('  B: Move all robots to back')

    def interactive(self, key, robot_name, axis, speed=1.0):

    def activate_external_control(self):
        for ur_robot in self._ur_robots:
            ur_robot.activate_external_control()

    # UR script motions
    def ur_movej(self, robot_name,
                 joint_positions, acceleration=0.5, velocity=0.5, wait=True):
        pub = self._ur_robots[robot_name].urscript_publisher
        return pub.movej(self, joint_positions, acceleration, velocity, wait)

    def ur_movel(self, robot_name, target_pose, end_effector_link="",
                 acceleration=0.5, velocity=0.03, wait=True):
        pub = self._ur_robots[robot_name].urscript_publisher
        if end_effector_link == "":
            end_effector_link = self._grippers[robot_name].tip_link
        success = pub.movel(target_pose, end_effector_link,
                            acceleration, velocity, wait)
        current_pose = self.get_current_pose(robot_name)
        is_all_close = self._all_close(target_pose, current_pose, 0.01)
        return (success, is_all_close, current_pose)

    def ur_movel_rel(self, robot_name, translation,
                     acceleration=0.5, velocity=0.03, wait=True):
        pub = self._ur_robots[robot_name].urscript_publisher
        return pub.movel_rel(translation, acceleration, velocity, wait)

    def ur_linear_push(self, robot_name, force=10.0, wait=True, direction="Z+",
                       max_approach_distance=0.1, forward_speed=0.02):
        pub = self._ur_robots[robot_name].urscript_publisher
        return pub.linear_push(force, direction,
                               max_approach_distance, forward_speed, wait)

    def ur_spiral_motion(self, robot_name, acceleration=0.1, velocity=0.03,
                         max_radius=0.0065, radius_increment=0.002,
                         theta_increment=30, spiral_axis="Z", wait=True):
        pub = self._ur_robots[robot_name].urscript_publisher
        return pub.spiral_motion(acceleration, velocity,
                                 max_radius, radius_increment,
                                 theta_increment, spiral_axis, wait)

    def ur_insertion(self, robot_name,
                     max_force=10.0, force_direction="Z+",
                     forward_speed=0.02, max_approach_distance=0.1,
                     max_radius=0.004, radius_increment=0.0003,
                     max_insertion_distance=0.035, impedance_mass=10,
                     peck_mode=False, wait=True):
        pub = self._ur_robots[robot_name].urscript_publisher
        return pub.insertion(max_force, force_direction,
                             forward_speed, max_approach_distance,
                             max_radius, radius_increment,
                             max_insertion_distance, impedance_mass,
                             peck_mode, wait)

    def ur_horizontal_insertion(self, robot_name,
                                max_force=10.0, force_direction="Y-",
                                forward_speed=0.02, max_approach_distance=0.1,
                                max_radius=0.007, radius_increment=0.0003,
                                max_insertion_distance=0.035,
                                impedance_mass=10,
                                peck_mode=False, wait=True):
        pub = self._ur_robots[robot_name].urscript_publisher
        return pub.horizontal_insertion(max_force, force_direction,
                                        forward_speed, max_approach_distance,
                                        max_radius, radius_increment,
                                        max_insertion_distance,
                                        impedance_mass, peck_mode, wait)
