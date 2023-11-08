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
from actionlib                import SimpleActionClient
from actionlib_msgs.msg       import GoalStatus
from numpy                    import clip
from control_msgs.msg         import (GripperCommand, GripperCommandAction,
                                      GripperCommandGoal)
from aist_fastening_tools.msg import (SuctionToolCommandAction,
                                      SuctionToolCommandGoal)
from std_msgs.msg             import Bool


######################################################################
#  class GripperClient                                               #
######################################################################
class GripperClient(object):
    def __init__(self, name, type, base_link=None, tip_link=None):
        object.__init__(self)
        self._name             = name
        self._type             = type
        self._base_link        = base_link if base_link else name + '_base_link'
        self._tip_link         = tip_link if tip_link else name + '_tip_link'
        self._default_tip_link = self._tip_link
        self._parameters       = {}

    @staticmethod
    def create(name, props):
        type_name = props.pop('type', None)
        if type_name is None:
            raise rospy.ROSException('(GripperClient) no type specified for gripper[%s]' % name)
        ClientClass = globals()[type_name]
        if rospy.get_param('use_real_robot', False):
            return ClientClass(name, **props)
        else:
            return ClientClass.simulated(name, **props)

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

    def pregrasp(self):
        self.release(rospy.Duration(-1))

    def grasp(self, timeout=rospy.Duration()):
        return True

    def postgrasp(self):
        self.grasp(rospy.Duration(-1))

    def release(self, timeout=rospy.Duration()):
        return True

    def move(self, position):
        return True

    def wait(self):
        return True

    def cancel(self):
        pass

######################################################################
#  class VoidGripper                                                 #
######################################################################
class VoidGripper(GripperClient):
    def __init__(self, name, tip_link):
        super(VoidGripper, self).__init__(name, 'void', tip_link, tip_link)
        rospy.loginfo('void_gripper initialized.')

    @staticmethod
    def simulated(name, tip_link):
        return VoidGripper(name, tip_link)

######################################################################
#  class GenericGripper                                              #
######################################################################
class GenericGripper(GripperClient):
    def __init__(self, name, action_ns, base_link=None, tip_link=None,
                 min_position=0.0, max_position=0.1, max_effort=5.0):
        super(GenericGripper, self).__init__(name, 'two_finger',
                                             base_link, tip_link)
        self._client = SimpleActionClient(action_ns, GripperCommandAction)
        self._parameters = {'grasp_position':   min_position,
                            'release_position': max_position,
                            'max_effort':       max_effort}

        if not self._client.wait_for_server(timeout=rospy.Duration(5)):
            self._client = None
            rospy.logerr('(GenericGripper) failed to connect to server[%s]',
                         action_ns)

        rospy.loginfo('%s initialized.', action_ns)

    @staticmethod
    def simulated(name, action_ns, base_link=None, tip_link=None,
                  min_position=0.0, max_position=0.1, max_effort=5.0):
        return GenericGripper(name, action_ns, base_link, tip_link,
                              min_position, max_position, max_effort)

    def grasp(self, timeout=rospy.Duration()):
        return self.move(self._parameters['grasp_position'],
                         self._parameters['max_effort'],
                         timeout)

    def release(self, timeout=rospy.Duration()):
        return self.move(self._parameters['release_position'], 0, timeout)

    def move(self, position, max_effort=0, timeout=rospy.Duration()):
        self._client.send_goal(GripperCommandGoal(GripperCommand(position,
                                                                 max_effort)))
        return self.wait(timeout)

    def wait(self, timeout=rospy.Duration()):
        if timeout < rospy.Duration():
            return False
        if not self._client.wait_for_result(timeout):
            self._client.cancel_goal()
            rospy.logerr('goal CANCELED because timeout[%.1f] has expired.',
                         timeout.to_sec())
            return False
        return self._client.get_result().stalled

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

######################################################################
#  class RobotiqGripper                                              #
######################################################################
class RobotiqGripper(GenericGripper):
    def __init__(self, name, controller_ns, max_effort=0.0, velocity=0.1):
        from aist_robotiq.srv import SetVelocity

        self._min_gap      = rospy.get_param(controller_ns + '/min_gap')
        self._max_gap      = rospy.get_param(controller_ns + '/max_gap')
        self._min_position = rospy.get_param(controller_ns + '/min_position')
        self._max_position = rospy.get_param(controller_ns + '/max_position')
        self._min_velocity = rospy.get_param(controller_ns + '/min_velocity')
        self._max_velocity = rospy.get_param(controller_ns + '/max_velocity')

        self._set_velocity = rospy.ServiceProxy(controller_ns + '/set_velocity',
                                                SetVelocity)
        self.set_velocity(velocity)

        assert self._min_gap < self._max_gap
        assert self._min_position != self._max_position


        super(RobotiqGripper, self).__init__(name,
                                             controller_ns + '/gripper_cmd',
                                             None, None,
                                             self._min_gap, self._max_gap,
                                             max_effort)

    @staticmethod
    def simulated(name, controller_ns, max_effort=0.0, velocity=0.1):
        return RobotiqGripper(name, controller_ns, max_effort, velocity)

    def set_velocity(self, velocity):
        return self._set_velocity(velocity).success

    def move(self, gap, max_effort=0, timeout=rospy.Duration()):
        return super(RobotiqGripper, self).move(self._position(gap),
                                                max_effort, timeout)

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
#  class PrecisionGripper                                            #
######################################################################
class PrecisionGripper(GenericGripper):
    def __init__(self, name, controller_ns):
        min_position = rospy.get_param(controller_ns + '/min_position')
        max_position = rospy.get_param(controller_ns + '/max_position')
        max_effort   = rospy.get_param(controller_ns + '/max_effort')

        assert min_position < max_position

        super(PrecisionGripper, self).__init__(name,
                                               controller_ns + '/gripper_cmd',
                                               None, None,
                                               min_position, max_position,
                                               max_effort)

######################################################################
#  class SuctionGripper                                              #
######################################################################
class SuctionGripper(GripperClient):
    def __init__(self, name, controller_ns,
                 suck_min_period=0.5, blow_min_period=0.2):
        super(SuctionGripper, self).__init__(name, 'suction')

        self._client     = SimpleActionClient(controller_ns + '/command',
                                              SuctionToolCommandAction)
        self._state_sub  = rospy.Subscriber(controller_ns + '/suctioned',
                                            Bool, self._state_cb)
        self._suctioned  = False
        self._parameters = {'suck_min_period': suck_min_period,
                            'blow_min_period': blow_min_period}

        if not self._client.wait_for_server(timeout=rospy.Duration(5)):
            self._client = None
            rospy.logerr('(SuctionGripper) failed to connect to server[%s]',
                         controller_ns + '/command')

    @staticmethod
    def simulated(name, controller_ns,
                  suck_min_period=0.5, blow_min_period=0.2):
        return GripperClient(name, 'suction')

    def pregrasp(self):
        # Set goal.min_period to zero so that the goal succeeds immediately.
        self._send_command(True, rospy.Duration(0), rospy.Duration(-1))

    def grasp(self, timeout=rospy.Duration()):
        rospy.sleep(0.5)
        return self._send_command(True, rospy.Duration(0), rospy.Duration(-1))

    def postgrasp(self):
        self.pregrasp()

    def release(self, timeout=rospy.Duration()):
        return self._send_command(False,
                                  rospy.Duration(
                                      self._parameters['blow_min_period']),
                                  timeout)

    def wait(self, timeout=rospy.Duration()):
        if timeout < rospy.Duration():
            return False
        if not self._client.wait_for_result(timeout):
            self._client.cancel_goal()
            rospy.logerr('goal CANCELED because timeout[%.1f] has expired.',
                         timeout.to_sec())
            return False
        return self._suctioned

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _send_command(self, suck, min_period, timeout=rospy.Duration()):
        self._client.send_goal(SuctionToolCommandGoal(suck, min_period))
        return self.wait(timeout)

    def _state_cb(self, msg):
        self._suctioned = msg.data

######################################################################
#  class Lecp6Gripper                                                #
######################################################################
class Lecp6Gripper(GripperClient):
    def __init__(self, name, controller_ns, open_no=1, close_no=2):
        from tranbo_control.msg import Lecp6CommandAction, Lecp6CommandGoal

        super(Lecp6Gripper, self).__init__(name, 'two_finger')
        self._client     = SimpleActionClient(controller_ns + '/lecp6',
                                              Lecp6CommandAction)
        self._parameters = {'release_stepdata': open_no,
                            'grasp_stepdata':   close_no}

        if not self._client.wait_for_server(timeout=rospy.Duration(5)):
            self._client = None
            rospy.logerr('(Lecp6Gripper) failed to connect to server[%s]',
                         controller_ns + '/lecp6')

    @staticmethod
    def simulated(name, controller_ns, open_no=1, close_no=2):
        return GripperClient(name, 'two_finger')

    def grasp(self, timeout=rospy.Duration()):
        return self._send_command(True, timeout)

    def release(self, timeout=rospy.Duration()):
        return self._send_command(False, timeout)

    def wait(self, timeout=rospy.Duration()):
        if timeout < rospy.Duration():
            return False
        if not self._client.wait_for_result(rospy.Duration(timeout)):
            self._client.cancel_goal()
            rospy.logerr('goal CANCELED because timeout[%.1f] has expired.',
                         timeout.to_sec())
            return False
        return self._client.get_result().reached_goal

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _send_command(self, close, timeout):
        goal = Lecp6CommandGoal()
        goal.command.stepdata_no = self._parameters['grasp_stepdata' \
                                                    if close else \
                                                    'release_stepdata']
        self._client.send_goal(goal)
        return self.wait(timeout)

######################################################################
#  class MagswitchGripper                                            #
######################################################################
class MagswitchGripper(GripperClient):
    def __init__(self, name, controller_ns,
                 sensitivity=0, grasp_position=30, confirm_position=100):
        from tranbo_control.msg import (MagswitchCommandAction,
                                        MagswitchCommandGoal)

        super(MagswitchGripper, self).__init__(name, 'magnet')
        self._client           = SimpleActionClient(controller_ns
                                                    + '/magswitch',
                                                    MagswitchCommandAction)
        self._goal             = MagswitchCommandGoal()
        self._calibration_step = 0
        self._parameters       = {'sensitivity':      sensitivity,
                                  'grasp_position':   grasp_position,
                                  'confirm_position': confirm_position}

        if not self._client.wait_for_server(timeout=rospy.Duration(5)):
            self._client = None
            rospy.logerr('(MagswitchGripper) failed to connect to server[%s]',
                         controller_ns + '/magswitch')

    @staticmethod
    def simulated(name, controller_ns,
                 sensitivity=0, grasp_position=30, confirm_position=100):
        return GripperClient(name, 'magnet')

    @property
    def calibration_step(self):
        return self._calibration_step

    def calibration(self, calibration_select):
        return self._send_command(0, 0, 1, calibration_select)

    def move(self, position, timeout=rospy.Duration()):
        return self._send_command(position, timeout)

    def pregrasp(self):
        self._send_command(self._parameters['grasp_position'],
                           rospy.Duration(-1))

    def grasp(self, timeout=rospy.Duration()):
        return True

    def postgrasp(self):
        self._send_command(self._parameters['confirm_position'],
                           rospy.Duration(-1))

    def release(self, timeout=rospy.Duration()):
        return self._send_command(0, timeout)

    def wait(self, timeout=rospy.Duration()):
        if timeout < rospy.Duration():
            return False
        elif not self._client.wait_for_result(rospy.Duration(timeout)):
            self._client.cancel_goal()
            rospy.logerr('goal CANCELED because timeout[%.1f] has expired.',
                         timeout.to_sec())
            return False
        result = self._client.get_result()
        if self._goal.command.calibration_trigger == 1:
            self._calibration_step = result.magswitch_out.calibration_step
            return position > 0 and result.magswitch_out.calibration_state != 0
        return result.magswitch_out.calibration_state == 3

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
            = clip(self._parameters['sensitivity'], -30, 30)
        self._goal.command.position = clip(position, 0, 100)
        self._client.send_goal(self._goal)
        rospy.loginfo('(MagswitchGripper) send command with position[%d]',
                      position)
        return self.wait(timeout)
