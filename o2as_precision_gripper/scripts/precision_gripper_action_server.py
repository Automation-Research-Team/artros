#! /usr/bin/env python
import lib_robotis_xm430 as xm430
import sys
import time
import rospy
import actionlib
import o2as_msgs.msg

class PrecisionGripperAction:
    def __init__(self, serial_port = '/dev/ttyUSB1'):
        self.dynamixel = xm430.USB2Dynamixel_Device( serial_port, baudrate = 57600 )
        self.p1 = xm430.Robotis_Servo2( self.dynamixel, 1, series = "XM" )  #inner gripper
        self.p2 = xm430.Robotis_Servo2( self.dynamixel, 2, series = "XM" )  #outer gripper
        #read the parameters
        name = rospy.get_name()
        self._feedback = o2as_msgs.msg.PrecisionGripperCommandFeedback()
        self._result = o2as_msgs.msg.PrecisionGripperCommandResult()
        self.outer_force = rospy.get_param(name + "outer_force", 30)
        self.inner_force = rospy.get_param(name + "inner_force", 7)
        self.grasping_inner_force = rospy.get_param(name + "/grasping_inner_force", 5)
        self.outer_open_position = rospy.get_param(name + "outer_open_position", -50000)
        self.outer_close_position = rospy.get_param(name + "outer_close_position", 50240)
        self.speed_limit = rospy.get_param(name + "speed_limit", 10)
        #define the action
        self._action_name = "precision_gripper_action"
        self._action_server = actionlib.SimpleActionServer(self._action_name, o2as_msgs.msg.PrecisionGripperCommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._action_server.start()
        rospy.loginfo('Action server '+ str(self._action_name)+" started.")
        return

    def execute_cb(self, goal):
        #print goal
        #print self._feedback.motor_speed
        
        # publish info to the console for the user
        rospy.loginfo('Executing'+ str(self._action_name)+"."+"request sent:")
        rospy.loginfo(goal)
        # start executing the action
        command_is_sent = False
        if goal.stop:
            command_is_sent1 = self.inner_gripper_disable_torque()
            command_is_sent2 = self.outer_gripper_disable_torque()
            if command_is_sent1 and command_is_sent2 is True:
                command_is_sent = True
            else:
                command_is_sent = False
        elif goal.open_outer_gripper_fully:        
            command_is_sent = self.outer_gripper_open_fully(self.outer_force)
        elif goal.close_outer_gripper_fully:
            command_is_sent = self.outer_gripper_close_fully(self.outer_force)
        elif goal.open_inner_gripper_fully:
            rospy.loginfo("Opening inner gripper")
            if goal.this_action_grasps_an_object:
                command_is_sent = self.inner_gripper_open_fully(self.grasping_inner_force)
            else:
                command_is_sent = self.inner_gripper_open_fully(self.inner_force)
        elif goal.close_inner_gripper_fully:
            rospy.loginfo("Closing inner gripper")
            if goal.this_action_grasps_an_object:
                command_is_sent = self.inner_gripper_close_fully(self.grasping_inner_force)
            else:
                command_is_sent = self.inner_gripper_close_fully(self.inner_force)
        else:
            rospy.logerr('No command sent to the gripper, service request was empty.')
            command_is_sent = False
        
        success = command_is_sent
        if success:
            if goal.stop:
                self._feedback.motor_speed = -1 #an arbitary number higher than self.speed_limit
            elif goal.open_outer_gripper_fully or goal.close_outer_gripper_fully:  
                self._feedback.motor_speed = self.p2.read_current_velocity()
            elif goal.open_inner_gripper_fully or goal.close_inner_gripper_fully:
                self._feedback.motor_speed = self.p1.read_current_velocity()
            while self._feedback.motor_speed > self.speed_limit:
                rospy.sleep(0.1)
                # check that preempt has not been requested by the client
                if self._action_server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._action_server.set_preempted()
                    success = False
                    break
                if goal.open_outer_gripper_fully or goal.close_outer_gripper_fully:  
                    self._feedback.motor_speed = self.p2.read_current_velocity()
                elif goal.open_inner_gripper_fully or goal.close_inner_gripper_fully:
                    self._feedback.motor_speed = self.p1.read_current_velocity()
                # publish the feedback
                self._action_server.publish_feedback(self._feedback)
            if success:
                self._result.success = True
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._action_server.set_succeeded(self._result)
        else:
            self._action_server.set_preempted()

    #outer gripper related functions
    def outer_gripper_close_new(self, goal_position):
        try:
            self.p2.set_operating_mode("currentposition")
            self.p2.set_current(100) #0.3Nm
            i2 = self.p2.read_current_position()
            i_avg = int(i2)
            while i_avg < goal_position:
                self.p2.set_goal_position(i_avg)
                current2 = int(self.p2.read_current())
                if current2 > 100:
                    break
                i_avg = i_avg + 8
        except:
            rospy.logerr("Failed to run commands.")


    def inner_gripper_read_current_position(self):
        try:
            x=self.p1.read_current_position()
            rospy.loginfo("id1="+str(x))
        except:
            rospy.logerr("Failed to run commands.")

    def outer_gripper_read_current_position(self):
        try:
            x=self.p2.read_current_position()
            rospy.loginfo("id2="+str(x))
        except:
            rospy.logerr("Failed to run commands.")


    def outer_gripper_open_fully(self,current):
        try:
            self.p2.set_operating_mode("currentposition")
            self.p2.set_current(current)
            self.p2.set_goal_position(self.outer_open_position)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False


    def outer_gripper_close_fully(self,current):
        try:
            self.p2.set_operating_mode("currentposition")
            self.p2.set_current(current)
            self.p2.set_goal_position(self.outer_close_position)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

    def outer_gripper_disable_torque(self):
        try:
            self.p2.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False


    def outer_gripper_open_to(self, location):#still editing
        try:
            self.p2.set_operating_mode("currentposition")
            self.p2.set_current(100)
            self.p2.set_goal_position(location)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False


    #inner gripper related functions
    ######################################################
    ######################################################

    def inner_gripper_open_fully(self, current):
        try:
            self.p1.set_operating_mode("currentposition")
            self.p1.set_positive_direction("ccw")
            self.p1.set_current(current)
            self.p1.set_goal_position(3799)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

    def inner_gripper_close_fully(self, current):
        try:
            self.p1.set_operating_mode("current")
            self.p1.set_positive_direction("cw")
            self.p1.set_current(current)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

    def inner_gripper_open_slightly(self, current_position):
        try:
            self.p1.set_operating_mode("currentposition")
            self.p1.set_current(8)
            #current_position=self.p1.read_current_position()
            #current_position = current_position-130
            self.p1.set_goal_position(current_position)
        except:
            rospy.logerr("Failed to run commands.")

    def inner_gripper_disable_torque(self):
        try:
            self.p1.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

        
if __name__ == '__main__':
    rospy.init_node('precision_gripper_action_server')
    serial_port = rospy.get_param("precision_gripper_server/serial_port", "/dev/ttyUSB2")
    rospy.loginfo("Starting up on serial port: " + serial_port)
    server = PrecisionGripperAction(rospy.get_name())
    rospy.spin()