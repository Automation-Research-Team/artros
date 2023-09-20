#!/usr/bin/env python

import threading
import rospy

from actionlib                      import ActionServer
from aist_fastening_tools.msg       import (ScrewToolCommandAction,
                                            ScrewToolCommandResult,
                                            ScrewToolCommandFeedback)
from aist_utility.thread_with_trace import ThreadTrace
from collections                    import deque

from dynamixel_workbench_msgs.msg   import DynamixelState, DynamixelStateList
from dynamixel_workbench_msgs.srv   import DynamixelCommand


class ScrewToolController(object):
    def __init__(self):
        # get data from .yaml
        controller_ns = rospy.get_param("~controller_ns")
        service_name  = '/'.join(['', controller_ns, 'dynamixel_command'])

        self._dynamixel_state_topic = '/'.join(['', controller_ns,
                                                'dynamixel_state'])
        self._motor_status_topic_is_list = True
        self._dynamixel_current_state    = []

        # initialize motor id table
        self._screw_tools          = dict()
        self._tool_locks           = dict()  #
        self._tool_being_preempted = dict()
        for name, props in rospy.get_param('~screw_tools').items():
            rospy.loginfo("Loaded " + name + " on motor id " + str(props['ID']))
            self._screw_tools.update({name: props['ID']})
            self._tool_locks.update({name: threading.Lock()})
            self._tool_being_preempted.update({name: False})
        self._listener = rospy.Subscriber(self._dynamixel_state_topic,
                                          rospy.AnyMsg, self._test_listener_cb)

        rospy.wait_for_service(service_name)
        self._motor_write_lock = threading.Lock()
        self._dynamixel_command_write = rospy.ServiceProxy(service_name,
                                                           DynamixelCommand)

        self._screw_srv = ActionServer('~command', ScrewToolCommandAction,
                                       goal_cb=self._goal_cb, auto_start=False)
        self._screw_srv.start()

    def _test_listener_cb(self, data):
        '''
        This function is executed when a message is received by _test_listener.
        It determines the actual ROS message type that is used in the topic
        and creates a subscriber with the
        given type, to read the status of the motors.
        '''
        message_type = data._connection_header['type']
        self._listener.unregister()
        if message_type == 'dynamixel_workbench_msgs/DynamixelState':
            self._motor_status_topic_is_list = False
            self._dynamixel_current_state = DynamixelState()
            self._listener = rospy.Subscriber(self._dynamixel_state_topic,
                                              DynamixelState,
                                              self._listener_cb)
        elif message_type == 'dynamixel_workbench_msgs/DynamixelStateList':
            self._motor_status_topic_is_list = True
            self._dynamixel_current_state = DynamixelStateList()
            self._listener = rospy.Subscriber(self._dynamixel_state_topic,
                                              DynamixelStateList,
                                              self._listener_cb)
        else:
            rospy.logerr('Unexpected message type: ' + message_type)

    def _listener_cb(self, data):
        """
        This function is executed when a message is received by _listener.
        It stores the status of the motors connected to the controller.
        """
        self._dynamixel_current_state = data

    def _goal_cb(self, goal_handle):
        goal_handle.set_accepted()
        goal_thread = ThreadTrace(target=self.acquire_lock_and_execute_control,
                                  args=(goal_handle.get_goal(), goal_handle))
        goal_thread.daemon = True
        goal_thread.start()
        return True

    def acquire_lock_and_execute_control(self, goal, goal_handle,
                                         double_check_after_tighten=True):
        """
        Acquires and releases the tool-specific thread lock.
        This avoids commands sent to a single tool to overlap.
        """
        tool_name = goal.tool_name
        if tool_name:
            r = rospy.Rate(20)
            try:
                while self._tool_locks[tool_name].locked():
                    self._tool_being_preempted[tool_name] = True
                    r.sleep()
            except Exception as e:
                rospy.logerr("Can't find tool with name:" + tool_name)
                print(e)
                return False
            rospy.loginfo("Trying to acquire tool lock for:" + tool_name)
            self._tool_locks[tool_name].acquire()
            res = self.execute_control(goal, goal_handle,
                                       double_check_after_tighten)
            self._tool_being_preempted[tool_name] = False
            rospy.loginfo("Releasing lock for:" + tool_name)
            self._tool_locks[tool_name].release()
            return res
        return True

    def execute_control(self, goal, goal_handle,
                        double_check_after_tighten=True):
        """
        Execute the tighten/loosen action.
        If double_check_after_tighten is True, after fastening has finished
        successfully, the screw is loosened once and then fastened again.
        """
        result   = ScrewToolCommandResult()
        feedback = ScrewToolCommandFeedback()
        motor_id = self._screw_tools[goal.tool_name]
        result.control_result = True
        if not goal.speed:
            goal.speed = 1023
        feedback.motor_speed = goal.speed  # Initial setting to start the loop

        if self._tool_being_preempted[goal.tool_name]:
            rospy.loginfo("Motor command pre-empted before entering loop.")
            result.control_result = False
            goal_handle.set_aborted(result)
            return False

        if goal.skip_final_loosen_and_retighten:
            double_check_after_tighten = False

        # For tightening the maximum speed is 1023. For loosen it is 2047.
        # At 1024 the motor is stopped.
        target_speed = 1024 + goal.speed
        if goal.direction == "loosen":
            if target_speed > 2047:
                target_speed = 2047
        elif goal.direction == "tighten":
            if target_speed > 1023:
                target_speed = 1023

        if not goal.tool_name in self._screw_tools:
            rospy.logerr("'%s' does not exist in %s." %
                         (goal.tool_name,
                          self.conf_gripper_filename))
            result.control_result = False
            goal_handle.set_succeeded(result)
            return False

        if not self.set_moving_speed(motor_id, target_speed):
            self.set_moving_speed(motor_id, 1024)
            self.set_torque_enable(motor_id, 0)
            result.control_result = False
            goal_handle.set_aborted(result)
            return False

        if goal.direction == "loosen" and not goal.duration:
            rospy.logwarn("Loosen command was sent, but without a duration. Setting to 2 seconds.")
            goal.duration = 2
        elif goal.direction == "tighten" and not goal.duration:
            rospy.logwarn("Tighten command was sent, but without a maximum duration. Setting to 10 seconds.")
            goal.duration = 10

        # Turn the motor for the specified number of seconds.
        # Rotate the motor until goal.duration is reached is loaded and stops.
        success_flag  = True
        motor_stalled = False
        # Initialize to start the loop
        speed_readings = deque([goal.speed, goal.speed], maxlen=2)
        start_time     = rospy.get_time()
        rate           = rospy.Rate(20)
        # Wait for motor to start up (and avoid reading incorrect speed values)
        rospy.sleep(1)
        while not rospy.is_shutdown():

            if (rospy.get_time() - start_time) > goal.duration:
                rospy.loginfo("Stopping motor due to timeout. Duration: %s, Time elapsed: %s" %
                              (goal.duration, rospy.get_time() - start_time))
                if goal.direction == "loosen":
                    success_flag = True
                    self.set_moving_speed(motor_id, 1024)
                elif goal.direction == "tighten":  # If motor does not stall before timeout, tightening was not successful
                    success_flag = False
                    self.set_moving_speed(motor_id, 1024)
                break

            if self._tool_being_preempted[goal.tool_name]:
                rospy.loginfo("Motor pre-empted. Breaking out of loop to make room for next thread. Duration: %s, Time elapsed: %s" %
                              (goal.duration, rospy.get_time() - start_time))
                success_flag = False
                break

            if feedback.motor_speed == 0 and goal.direction == 'tighten':
                if double_check_after_tighten:
                    rospy.loginfo("Motor has stalled. Loosening for 1 second and then trying to tighten again to confirm success.")
                    # Turn into loosening direction
                    self.set_moving_speed(motor_id, 2047)
                    rospy.sleep(1.0)
                    self.set_moving_speed(motor_id, 1024)  # Stop
                    return self.execute_control(goal, goal_handle,
                                                double_check_after_tighten=False)
                rospy.loginfo("Stopping motor because it has stalled (the screw is tightened)")
                success_flag  = True
                motor_stalled = True
                break

            # Read motor speed.
            speed_readings.append(self.get_present_speed(motor_id))
            if -1 in speed_readings:
                success_flag = False
                rospy.logwarn("Error in motor readout. Stopping.")
                break

            # If both readings are below an arbitrary threshold,
            # we assume the motor has stalled
            stall_threshold = 18
            if all(speed <= stall_threshold for speed in speed_readings):
                feedback.motor_speed = 0
            else:
                feedback.motor_speed = max(speed_readings)

            goal_handle.publish_feedback(feedback)
            rate.sleep()

        motor_stopped = self.set_moving_speed(motor_id, 1024)
        if motor_stopped and success_flag:
            self.set_torque_enable(motor_id, 0)
            result.control_result = True
            result.motor_stalled = motor_stalled
            if goal.direction == "tighten":
                rospy.loginfo("Screw was fastened successfully.")
            else:
                rospy.loginfo("Motor loosened successfully.")
            goal_handle.set_succeeded(result)
        else:
            self.set_torque_enable(motor_id, 0)
            result.control_result = False
            result.motor_stalled  = motor_stalled
            if goal.direction == "tighten":
                rospy.loginfo("Fastening timed out. Screw not fastened.")
            else:
                rospy.loginfo("Timed out, but this error message should not appear anyway.")
            goal_handle.set_aborted(result)
        return True

    def set_torque_enable(self, motor_id, value):
        try:
            self._motor_write_lock.acquire()
            try:
                res = self._dynamixel_command_write('', motor_id,
                                                    "Torque_Enable", value)
            except BaseException:
                rospy.logerr("Write to Dynamixel failed!")
                pass
            self._motor_write_lock.release()
        except rospy.ServiceException as exc:
            rospy.logwarn('An exception occurred in the Torque_Enable set, but processing continues.')
        else:
            if not res.comm_result:
                rospy.logerr('Can not set torque_enable to XL-320. (ID=%i)' %
                             motor_id)
            return res.comm_result
        return True

    def set_moving_speed(self, motor_id, value):
        try:
            self._motor_write_lock.acquire()
            try:
                res = self._dynamixel_command_write('', motor_id,
                                                    "Moving_Speed", value)
            except BaseException:
                rospy.logerr("Write to Dynamixel failed!")
                pass
            self._motor_write_lock.release()
            rospy.loginfo(res)
            rospy.loginfo("Motor " + str(motor_id) \
                          + " Moving_Speed: " + str(value))
        except rospy.ServiceException as exc:
            rospy.logwarn('An exception occurred in the Moving_Speed set. Processing retry.')
        else:
            if not res.comm_result:
                rospy.logerr('Can not set speed to XL-320. (ID=%i)' % motor_id)
            return res.comm_result
        return True

    def get_present_speed(self, motor_id):
        """Wait for the current state top be available"""
        while '_listener' not in dir(self):
            rospy.sleep(0.1)
        if self._motor_status_topic_is_list:
            while not self._dynamixel_current_state.dynamixel_state:
                rospy.sleep(0.1)
        elif self._motor_status_topic_is_list == False:
            while self._dynamixel_current_state.name == '':
                rospy.sleep(0.1)

        if self._motor_status_topic_is_list:
            return next(
                (motor
                 for motor in self._dynamixel_current_state.dynamixel_state
                 if motor.id == motor_id)).present_velocity
        elif self._motor_status_topic_is_list == False:
            return self._dynamixel_current_state.present_velocity


if __name__ == '__main__':
    rospy.init_node('screw_tool_controller')
    server = ScrewToolController()
    rospy.spin()
