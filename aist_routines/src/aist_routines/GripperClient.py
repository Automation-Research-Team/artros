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

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs       import msg as cmsg
from numpy              import clip

######################################################################
#  class GripperClient                                               #
######################################################################
class GripperClient(object):
    def __init__(self, name, type, base_link, tip_link):
        object.__init__(self)
        self._name             = name
        self._type             = type
        self._base_link        = base_link
        self._default_tip_link = tip_link
        self._tip_link         = tip_link
        self._parameters       = {}

    @staticmethod
    def create(type_name, kwargs):
        ClientClass = globals()[type_name]
        if rospy.get_param('use_real_robot', False):
            return ClientClass(**kwargs)
        else:
            return ClientClass.base(**kwargs)

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

    @property
    def base_link(self):
        return self._base_link

    @property
    def tip_link(self):
        return self._tip_link

    @tip_link.setter
    def tip_link(self, tip_link):
        self._tip_link = tip_link

    @property
    def parameters(self):
        return self._parameters

    @parameters.setter
    def parameters(self, parameters):
        for key, value in parameters.items():
            self._parameters[key] = value

    def reset_tip_link(self):
        self._tip_link = self._default_tip_link

    def pregrasp(self, timeout=0):
        return self.release(timeout)

    def grasp(self, timeout=0):
        return True

    def postgrasp(self, timeout=0):
        return self.grasp(wait)

    def release(self, timeout=0):
        return True

    def wait(self):
        return True

    def cancel(self):
        pass

######################################################################
#  class VoidGripper                                                 #
######################################################################
class VoidGripper(GripperClient):
    def __init__(self, base_link):
        super(VoidGripper, self).__init__('void_gripper', 'void',
                                          base_link, base_link)
        rospy.loginfo('void_gripper initialized.')

    @staticmethod
    def base(base_link):
        return VoidGripper(base_link)

######################################################################
#  class GenericGripper                                              #
######################################################################
class GenericGripper(GripperClient):
    def __init__(self, name, action_ns, base_link, tip_link,
                 min_position=0.0, max_position=0.1, max_effort=5.0):
        super(GenericGripper, self).__init__(name, 'two_finger',
                                             base_link, tip_link)
        self._client = actionlib.SimpleActionClient(action_ns,
                                                    cmsg.GripperCommandAction)
        self._client.wait_for_server()

        self.parameters = {'grasp_position':   min_position,
                           'release_position': max_position,
                           'max_effort':       max_effort}

        rospy.loginfo('%s initialized.', action_ns)

    @staticmethod
    def base(name, action_ns, base_link, tip_link,
             min_position, max_position, max_effort):
        return GenericGripper(name, action_ns, base_link, tip_link,
                              min_position, max_position, max_effort)

    def grasp(self, timeout=0):
        return self.move(self.parameters['grasp_position'],
                         self.parameters['max_effort'],
                         timeout)

    def release(self, timeout=0):
        return self.move(self.parameters['release_position'], 0, timeout)

    def move(self, position, max_effort=0, timeout=0):
        self._client.send_goal(cmsg.GripperCommandGoal(
                                   cmsg.GripperCommand(position, max_effort)))
        return self.wait(timeout)

    def wait(self, timeout=0):
        if timeout < 0:
            return False
        if not self._client.wait_for_result(rospy.Duration(timeout)):
            rospy.logerr('Timeout[%f] has expired before goal finished',
                         timeout)
            return False
        result = self._client.get_result()
        return result.stalled

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

######################################################################
#  class RobotiqGripper                                              #
######################################################################
class RobotiqGripper(GenericGripper):
    def __init__(self, prefix='a_bot_gripper_', effort=0.0, velocity=0.1):
        ns = prefix + 'controller'
        self._min_gap      = rospy.get_param(ns + '/min_gap')
        self._max_gap      = rospy.get_param(ns + '/max_gap')
        self._min_position = rospy.get_param(ns + '/min_position')
        self._max_position = rospy.get_param(ns + '/max_position')

        assert self._min_gap < self._max_gap
        assert self._min_position != self._max_position

        super(RobotiqGripper, self).__init__(prefix.rstrip('_'),
                                             ns + '/gripper_cmd',
                                             prefix + 'base_link',
                                             prefix + 'tip_link',
                                             self._min_gap, self._max_gap,
                                             effort)

    @staticmethod
    def base(prefix, effort, velocity):
        return RobotiqGripper(prefix, effort, velocity)

    def move(self, gap, max_effort=0, timeout=0):
        return super(RobotiqGripper, self).move(self._position(gap),
                                                max_effort, timeout)

    def wait(self, timeout=0):
        result = super(RobotiqGripper, self).wait(timeout)
        return result

    def _position(self, gap):
        return (gap - self._min_gap) * self._position_per_gap \
             + self._min_position

    def _gap(self, position):
        return (position - self._min_position) / self._position_per_gap \
             + self._min_gap

    @property
    def _position_per_gap(self):
        return (self._max_position - self._min_position) \
             / (self._max_gap - self._min_gap)

######################################################################
#  class SuctionGripper                                              #
######################################################################
class SuctionGripper(GripperClient):
    def __init__(self, name, action_ns, state_ns='', eject=False):
        import o2as_msgs.msg
        import std_msgs.msg

        super(SuctionGripper, self).__init__(
            *SuctionGripper._initargs(name, action_ns, state_ns, eject))
        self._client    = actionlib.SimpleActionClient(
                              action_ns,
                              o2as_msgs.msg.SuctionControlAction)
        self._sub       = rospy.Subscriber(state_ns,
                                           std_msgs.msg.Bool,
                                           self._state_callback)
        self._suctioned = False
        self._eject     = eject  # blow when releasing
        self._goal                     = o2as_msgs.msg.SuctionControlGoal()
        self._goal.fastening_tool_name = 'suction_tool'
        self._goal.turn_suction_on     = False
        self._goal.eject_screw         = False

    @staticmethod
    def base(name, action_ns, state_ns, eject):
        return GripperClient(*SuctionGripper._initargs(name, action_ns,
                                                       state_ns, eject))

    @staticmethod
    def _initargs(name, action_ns, state_ns, eject):
        return (name, 'suction',
                name + '_base_link', name + '_tip_link')

    def pregrasp(self, timeout=0):
        return self._send_command(True, timeout)

    def grasp(self, timeout=0):
        if not self._send_command(True, timeout):
            return False
        rospy.sleep(0.5)        # Wait until the state is updated.
        return self._suctioned

    def postgrasp(self, timeout=0):
        return self._suctioned

    def release(self, timeout=0):
        return self._send_command(False, timeout)

    def wait(self, timeout=0):
        return self._suctioned

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _send_command(self, turn_suction_on, timeout=0):
        self._goal.turn_suction_on = turn_suction_on
        self._goal.eject_screw     = not turn_suction_on and self._eject
        self._client.send_goal(self._goal)
        if not self._client.wait_for_result(rospy.Duration(timeout)):
            rospy.logerr('Timeout[%f] has expired before goal finished',
                         timeout)
            return False
        result = self._client.get_result()
        return result.success

    def _state_callback(self, msg):
        self._suctioned = msg.data

######################################################################
#  class PrecisionGripper                                            #
######################################################################
class PrecisionGripper(GripperClient):
    def __init__(self, prefix='a_bot_gripper_'):
        import o2as_msgs.msg

        super(PrecisionGripper, self).__init__(
            *PrecisionGripper._initargs(prefix))
        self._client = actionlib.SimpleActionClient(
                           'precision_gripper_action',
                           # str(prefix) + 'gripper/gripper_action_controller',
                           o2as_msgs.msg.PrecisionGripperCommandAction)
        self._goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
        self._goal.stop                         = False
        self._goal.open_outer_gripper_fully     = False
        self._goal.close_outer_gripper_fully    = False
        self._goal.open_inner_gripper_fully     = False
        self._goal.close_inner_gripper_fully    = False
        self._goal.this_action_grasps_an_object = False
        self._goal.linear_motor_position        = 0.0
        self._goal.outer_gripper_opening_width  = 0.0
        self._goal.inner_gripper_opening_width  = 0.0
        self._goal.slight_opening_width         = 0.0

        self.parameters = {'cmd': ''}

    @staticmethod
    def base(prefix):
        return GripperClient(*PrecisionGripper._initargs(prefix))

    @staticmethod
    def _initargs(prefix):
        return (prefix.rstrip('_'), 'two_finger',
                prefix + 'base_link', prefix + 'tip_link')

    @property
    def linear_motor_position(self):
        return self._goal.linear_motor_position

    def pregrasp(self, timeout=0):
        cmd     = self.parameters['cmd']
        success = False
        if cmd == 'complex_pick_from_inside':
            success = self._inner_command(True, False, timeout)
        elif cmd == 'complex_pick_from_outside':
            self._inner_command(False, False, timeout)
        elif cmd == 'easy_pick_only_inner' or \
             cmd == 'inner_gripper_from_inside' or \
             cmd == '':
            success = self._inner_command(False, False, timeout)
        elif cmd == 'easy_pick_outside_only_inner' or \
             cmd == 'inner_gripper_from_outside':
            success = self._inner_command(True, False, timeout)
        return success

    def grasp(self, timeout=0):
        cmd     = self.parameters['cmd']
        success = False
        if cmd == 'complex_pick_from_inside':
            success = self._inner_command(False, True,  timeout) and \
                      self._outer_command(True,  False, timeout)
        elif cmd == 'complex_pick_from_outside':
            success = self._inner_command(True, True,  timeout) and \
                      self._outer_command(True, False, timeout)
        elif cmd == 'easy_pick_only_inner' or \
             cmd == 'inner_gripper_from_inside' or \
             cmd == '':
            success = self._inner_command(True, True, timeout)
        elif cmd == 'easy_pick_outside_only_inner' or \
             cmd == 'inner_gripper_from_outside':
            success = self._inner_command(False, True, timeout)
        return success

    def release(self, timeout=0):
        cmd     = self.parameters['cmd']
        success = False
        if cmd == 'complex_pick_from_inside':
            success = self._outer_command(False, False, timeout) and \
                      self._inner_command(True,  False, timeout)
        elif cmd == 'complex_pick_from_outside':
            success = self._outer_command(False, False, timeout) and \
                      self._inner_command(False, False, timeout)
        elif cmd == 'easy_pick_only_inner' or \
             cmd == 'inner_gripper_from_inside' or \
             cmd == '':
            success = self._inner_command(False, False, timeout)
        elif cmd == 'easy_pick_outside_only_inner' or \
             cmd == 'inner_gripper_from_outside':
            success = self._inner_command(True, False, timeout)
        return success

    def wait(self, timeout=0):
        if timeout < 0:
            return False
        elif not self._client.wait_for_result(rospy.Duration(timeout)):
            rospy.logerr('Timeout[%f] has expired before goal finished',
                         timeout)
            return False
        return self._client.get_result().success

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _inner_command(self, close, grasps_an_object, timeout):
        self._goal.open_inner_gripper_fully     = not close
        self._goal.close_inner_gripper_fully    = close
        self._goal.open_outer_gripper_fully     = False
        self._goal.close_outer_gripper_fully    = False
        self._goal.this_action_grasps_an_object = grasps_an_object
        self._client.send_goal(self._goal)
        return self.wait(timeout)

    def _outer_command(self, close, grasps_an_object, timeout):
        self._goal.open_inner_gripper_fully     = False
        self._goal.close_inner_gripper_fully    = False
        self._goal.open_outer_gripper_fully     = not close
        self._goal.close_outer_gripper_fully    = close
        self._goal.this_action_grasps_an_object = grasps_an_object
        self._client.send_goal(self._goal)
        return self.wait(timeout)

######################################################################
#  class Lecp6Gripper                                                #
######################################################################
class Lecp6Gripper(GripperClient):
    def __init__(self, prefix='a_bot_gripper_', open_no=1, close_no=2):
        import tranbo_control.msg

        super(Lecp6Gripper, self).__init__(*Lecp6Gripper._initargs(prefix))
        self._client = actionlib.SimpleActionClient(
                           '/arm_driver/lecp6_driver/lecp6',
                           tranbo_control.msg.Lecp6CommandAction)
        self._goal = tranbo_control.msg.Lecp6CommandGoal()

        self.parameters = {'release_stepdata': open_no,
                           'grasp_stepdata':   close_no}

    @staticmethod
    def base(prefix):
        return GripperClient(*Lecp6Gripper._initargs(prefix))

    @staticmethod
    def _initargs(prefix):
        return (prefix.rstrip('_'), 'two_finger',
                prefix + 'base_link', prefix + 'tip_link')

    def grasp(self, timeout=0):
        return self._send_command(True, timeout)

    def release(self, timeout=0):
        return self._send_command(False, timeout)

    def wait(self, timeout=0):
        if timeout < 0:
            return False
        if not self._client.wait_for_result(rospy.Duration(timeout)):
            rospy.logerr('Timeout[%f] has expired before goal finished',
                         timeout)
            return False
        return self._client.get_result().reached_goal

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _send_command(self, close, timeout):
        self._goal.command.stepdata_no \
            = self.parameters['grasp_stepdata' if close else
                              'release_stepdata']
        self._client.send_goal(self._goal)
        return self.wait(timeout)

######################################################################
#  class MagswitchGripper                                            #
######################################################################
class MagswitchGripper(GripperClient):
    def __init__(self, prefix='a_bot_magnet_',
                 sensitivity=0, grasp_position=30, confirm_position=100):
        import tranbo_control.msg

        super(MagswitchGripper, self).__init__(
            *MagswitchGripper._initargs(prefix))
        self._client = actionlib.SimpleActionClient(
                              '/arm_driver/magswitch_driver/magswitch',
                              tranbo_control.msg.MagswitchCommandAction)
        self._goal   = tranbo_control.msg.MagswitchCommandGoal()
        self._calibration_step = 0

        self.parameters = {'sensitivity':      sensitivity,
                           'grasp_position':   grasp_position,
                           'confirm_position': confirm_position}

    @staticmethod
    def base(prefix):
        return GripperClient(*MagswitchGripper._initargs(prefix))

    @staticmethod
    def _initargs(prefix):
        return (prefix.rstrip('_'), 'magnet',
                prefix + 'base_link', prefix + 'tip_link')

    @property
    def calibration_step(self):
        return self._calibration_step

    def calibration(self, calibration_select):
        return self._send_command(0, 0, 1, calibration_select)

    def pregrasp(self, timeout=0):
        return self._send_command(self.parameters['grasp_position'], timeout)

    def grasp(self, timeout=0):
        return True
        # return self._send_command(self.parameters['grasp_position'], timeout)

    def postgrasp(self, timeout=0):
        return self._send_command(self.parameters['confirm_position'], timeout)

    def release(self, timeout=0):
        return self._send_command(0, timeout)

    def wait(self, timeout=0):
        if timeout < 0:
            return False
        elif not self._client.wait_for_result(rospy.Duration(timeout)):
            rospy.logerr('Timeout[%.1f] has expired before goal finished',
                         timeout)
            return False
        result = self._client.get_result()
        if self._goal.command.calibration_trigger == 1:
            self._calibration_step = result.magswitch_out.calibration_step
            return position > 0 and result.magswitch_out.calibration_state != 0
        return result.reached_goal

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _send_command(self, position, timeout,
                      calibration_trigger=0, calibration_select=0):
        self._calibration_step = 0
        self._goal.used_sdo = False
        self._goal.command.calibration_trigger = calibration_trigger
        self._goal.command.calibration_select  = calibration_select
        self._goal.command.sensitivity \
            = clip(self.parameters['sensitivity'], -30, 30)
        self._goal.command.position = clip(position, 0, 100)
        self._client.send_goal(self._goal)
        return self.wait(timeout)
