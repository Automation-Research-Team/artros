#!/usr/bin/env python

import threading, rospy
import numpy as np

from actionlib                      import ActionServer
from actionlib_msgs.msg             import GoalStatus
from aist_fastening_tools.msg       import (ScrewToolCommandAction,
                                            ScrewToolCommandResult,
                                            ScrewToolCommandFeedback)
from aist_utility.thread_with_trace import ThreadTrace
from collections                    import deque

from dynamixel_workbench_msgs.msg   import DynamixelState, DynamixelStateList
from dynamixel_workbench_msgs.srv   import DynamixelCommand


class ScrewToolController(object):
    def __init__(self):
        super(ScrewToolController, self).__init__()

        self._name = rospy.get_name()

        # initialize motor id table
        self._motor_ids = {}
        self._locks     = {}
        for tool_name, tool_props in rospy.get_param('~screw_tools').items():
            self._motor_ids[tool_name] = tool_props['ID']
            self._locks[tool_name]     = threading.Lock()
            rospy.loginfo("(%s) Loaded %s with id=%d",
                          self._name, tool_name, self._motor_ids[tool_name])

        # Create a subscriber for receiving state of Dynamixel driver.
        driver_ns = rospy.get_param("~driver_ns")
        self._dynamixel_state_list = None
        self._dynamixel_state_sub  = rospy.Subscriber(driver_ns
                                                      + '/dynamixel_state',
                                                      DynamixelStateList,
                                                      self._state_list_cb)

        # Create a service client for sending command to Dynamixel driver
        self._dynamixel_command = rospy.ServiceProxy(driver_ns
                                                     + '/dynamixel_command',
                                                     DynamixelCommand)
        #rospy.wait_for_service(driver_ns + '/dynamixel_command')

        self._server = ActionServer('~command', ScrewToolCommandAction,
                                    self._goal_cb, auto_start=False)
        self._server.start()

        rospy.loginfo('(%s) controller started', self._name)

    def _state_list_cb(self, state_list):
        """
        This function is executed when a message is received by _listener.
        It stores the status of the motors connected to the controller.
        """
        self._dynamixel_states = state_list.dynamixel_state

    def _goal_cb(self, goal_handle):
        goal_handle.set_accepted()
        rospy.loginfo('(%s) new goal ACCEPTED for %s',
                      self._name, goal_handle.get_goal().tool_name)

        goal_thread = ThreadTrace(target=self.acquire_lock_and_execute_control,
                                  args=[goal_handle])
        goal_thread.daemon = True
        goal_thread.start()
        return True

    def acquire_lock_and_execute_control(self, goal_handle,
                                         double_check_after_tighten=True):
        """
        Acquires and releases the tool-specific thread lock.
        This avoids commands sent to a single tool to overlap.
        """
        goal = goal_handle.get_goal()

        if goal.tool_name not in self._locks:
            goal_handle.set_rejected()
            rospy.logerr('(%s) goal REJECTED: tool named %s not found',
                         self._name, goal.tool_name)
            return False

        rospy.loginfo("(%s) Trying to acquire lock for %s",
                      self._name, goal.tool_name)
        self._locks[goal.tool_name].acquire()
        success = self.execute_control(goal_handle, double_check_after_tighten)
        self._locks[goal.tool_name].release()
        rospy.loginfo("(%s) Lock released for %s", self._name, goal.tool_name)
        return success

    def execute_control(self, goal_handle, double_check_after_tighten=True):
        """
        Execute the tighten/loosen action.
        If double_check_after_tighten is True, after fastening has finished
        successfully, the screw is loosened once and then fastened again.
        """
        goal = goal_handle.get_goal()
        if not goal.speed:
            goal.speed = 1023  # Maximum speed
        if not goal.timeout:
            goal.timeout = rospy.Duration(10.0 if goal.tighten else 2.0)

        motor_id = self._motor_ids[goal.tool_name]
        result   = ScrewToolCommandResult(success=True, stalled=False)
        feedback = ScrewToolCommandFeedback(speed=goal.speed)

        if goal.skip_final_loosen_and_retighten:
            double_check_after_tighten = False

        # For tightening the maximum speed is 1023. For loosen it is 2047.
        # At 1024 the motor is stopped.
        if goal.tighten:  # CCW rotation
            target_speed = np.clip(goal.speed, 0, 1023)
        else:             # CW rotation
            target_speed = np.clip(goal.speed, 0, 1023) + 1024

        if not self._set_value(motor_id, 'Moving_Speed', target_speed):
            self._set_value(motor_id, 'Moving_Speed',  1024)  # Stop
            self._set_value(motor_id, 'Torque_Enable', 0)
            result.success = False
            goal_handle.set_aborted(result)
            rospy.logerr('(%s) goal ABORTED: failed to set speed[%d] for %s',
                         self._name, target_speed, goal.tool_name)
            return False

        # Initialize to start the loop
        speed_readings = deque([goal.speed, goal.speed], maxlen=2)
        start_time     = rospy.Time.now()
        rate           = rospy.Rate(20)
        # Wait for motor to start up (and avoid reading incorrect speed values)
        rospy.sleep(1)
        while not rospy.is_shutdown():

            # Check if the goal is canceled.
            if goal_handle.get_goal_status().status in (GoalStatus.PREEMPTING,
                                                        GoalStatus.RECALLING):
                goal_handle.set_canceled()
                rospy.logwarn('(%s) goal CANCELED: Motor pre-empted. Breaking out of loop to make room for next thread. Timeout: %s, Time elapsed: %s',
                              self._name,
                              goal.timeout.to_sec(), time_elapsed.to_sec())
                break

            # Check if timeout has expired.
            time_elapsed = rospy.Time.now() - start_time
            if time_elapsed > goal.timeout:
                rospy.logwarn("(%s) motor stopped due to timeout[%f sec]. Time elapsed: %f sec",
                              self._name,
                              goal.timeout.to_sec(), time_elapsed.to_sec())
                if goal.tighten:  # If motor does not stall before timeout, tightening was not successful
                    result.success = False
                    goal_handle.set_aborted(result)
                    rospy.logerr('(%s) goal ABORTED: screw not fastened',
                                 self._name)
                else:
                    result.success = True
                    goal_handle.set_succeeded(result)
                    rospy.loginfo('(%s) goal SUCCEDED: screw loosened',
                                  self._name)
                break

            # Check if the motor is stalled.
            if feedback.speed == 0 and goal.tighten:
                if double_check_after_tighten:
                    rospy.loginfo('(%s) Motor has stalled. Loosening for 1 second and then trying to tighten again to confirm success.',
                                  self._name)
                    # Turn into loosening direction for 1sec.
                    self._set_value(motor_id, 'Moving_Speed', 2047)
                    rospy.sleep(1.0)
                    self._set_value(motor_id, 'Moving_Speed', 1024)  # CW Stop

                    return self.execute_control(goal_handle, False)

                result.success = True
                result.stalled = True
                goal_handle.set_succeeded(result)
                rospy.loginfo('(%s) goal SUCCEEDED: motor stalled and the screw  tightened',
                              self._name)
                break

            # Read motor speed.
            speed = self._get_present_speed(motor_id)
            if speed < 0:
                result.success = False
                goal_handle.set_aborted(result)
                rospy.logerr("(%s) goal ABORTED: error in motor readout",
                              self._name)
                break
            speed_readings.append(speed)

            # If both readings are below an arbitrary threshold,
            # we assume the motor has stalled
            stall_threshold = 18
            if all(speed <= stall_threshold for speed in speed_readings):
                feedback.speed = 0
            else:
                feedback.speed = max(speed_readings)

            goal_handle.publish_feedback(feedback)
            rate.sleep()

        # Stop rotation and disable torque.
        self._set_value(motor_id, 'Moving_Speed',  1024)
        self._set_value(motor_id, 'Torque_Enable', 0)

        return result.success

    def _set_value(self, id, addr_name, value):
        try:
            res = self._dynamixel_command('', id, addr_name, value)
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

    def _get_present_speed(self, motor_id):
        # Wait for the current state top be available
        while not self._dynamixel_states:
            rospy.sleep(0.1)

        states = [state for state in self._dynamixel_states
                  if state.id == motor_id]
        if not states:
            rospy.logerr('(%s) dynamixel state with ID=%i not found in state list',
                         self._name, motor_id)
            return -1
        return states[0].present_velocity


if __name__ == '__main__':
    rospy.init_node('screw_tool_controller')
    server = ScrewToolController()
    rospy.spin()
