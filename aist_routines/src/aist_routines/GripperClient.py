import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs       import msg as cmsg
from numpy              import clip

######################################################################
#  class GripperClient                                               #
######################################################################
class GripperClient(object):
    def __init__(self, type, base_link, tip_link, timeout):
        self._type             = type
        self._base_link        = base_link
        self._default_tip_link = tip_link
        self._tip_link         = tip_link
        self._timeout          = timeout
        self._parameters       = {}

    @staticmethod
    def create(type_name, kwargs):
        ClientClass = globals()[type_name]
        if rospy.get_param('use_real_robot', False):
            return ClientClass(**kwargs)
        else:
            return ClientClass.base(**kwargs)

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
    def timeout(self):
        return self._timeout

    @property
    def parameters(self):
        return self._parameters

    @parameters.setter
    def parameters(self, parameters):
        for key, value in parameters.items():
            self._parameters[key] = value

    def reset_tip_link(self):
        self._tip_link = self._default_tip_link

    def pregrasp(self, wait=True):
        return self.release(wait)

    def grasp(self, wait=True):
        return True

    def postgrasp(self, wait=True):
        return self.grasp(wait)

    def release(self, wait=True):
        return True

    def wait(self):
        return True

    def cancel(self):
        pass

######################################################################
#  class VoidGripper                                                 #
######################################################################
class VoidGripper(GripperClient):
    def __init__(self, base_link, timeout=10.0):
        super(VoidGripper, self).__init__('void',
                                          base_link, base_link, timeout)
        rospy.loginfo('void_gripper initialized.')

    @staticmethod
    def base(base_link, timeout):
        return VoidGripper(base_link, timeout)

######################################################################
#  class GenericGripper                                              #
######################################################################
class GenericGripper(GripperClient):
    def __init__(self, action_ns, base_link, tip_link,
                 min_position=0.0, max_position=0.1, max_effort=5.0,
                 timeout=10.0):
        super(GenericGripper, self).__init__('two_finger',
                                             base_link, tip_link, timeout)
        self._client = actionlib.SimpleActionClient(action_ns,
                                                    cmsg.GripperCommandAction)
        self._client.wait_for_server()

        self.parameters = {'grasp_position':   min_position,
                           'release_position': max_position,
                           'max_effort':       max_effort}

        rospy.loginfo('%s initialized.', action_ns)

    @staticmethod
    def base(action_ns, base_link, tip_link,
             min_position, max_position, max_effort, timeout):
        return GenericGripper(action_ns, base_link, tip_link,
                              min_position, max_position, max_effort, timeout)

    def grasp(self, wait=True):
        return self.move(self.parameters['grasp_position'],
                         self.parameters['max_effort'])

    def release(self, wait=True):
        return self.move(self.parameters['release_position'], 0)

    def move(self, position, max_effort=0, wait=True):
        self._client.send_goal(cmsg.GripperCommandGoal(
                                   cmsg.GripperCommand(position, max_effort)))
        return self.wait() if wait else (0, False)

    def wait(self):
        if not self._client.wait_for_result(rospy.Duration(self.timeout)):
            rospy.logerr('Timeout[%f] has expired before goal finished',
                         self.timeout)
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
    def __init__(self, prefix='a_bot_gripper_',
                 effort=0.0, velocity=0.1, timeout=6.0):
        ns = prefix + 'controller'
        self._min_gap      = rospy.get_param(ns + '/min_gap')
        self._max_gap      = rospy.get_param(ns + '/max_gap')
        self._min_position = rospy.get_param(ns + '/min_position')
        self._max_position = rospy.get_param(ns + '/max_position')

        super(RobotiqGripper, self).__init__(ns + '/gripper_cmd',
                                             prefix + 'base_link',
                                             prefix + 'tip_link',
                                             self._min_gap, self._max_gap,
                                             effort, timeout)

    @staticmethod
    def base(prefix, effort, velocity, timeout):
        return RobotiqGripper(prefix, effort, velocity, timeout)

    def move(self, gap, max_effort=0, wait=True):
        return super(RobotiqGripper, self).move(self._position(gap),
                                                max_effort, wait)

    # def wait(self):
    #     position, stalled = super(RobotiqGripper, self).wait()
    #     return self._gap(position), stalled

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
    def __init__(self, prefix, eject=False, timeout=2.0):
        import o2as_msgs.msg
        import std_msgs.msg

        super(SuctionGripper, self) \
            .__init__(*SuctionGripper._initargs(prefix, eject, timeout))
        self._client    = actionlib.SimpleActionClient(
                              'o2as_fastening_tools/suction_control',
                              o2as_msgs.msg.SuctionControlAction)
        self._sub       = rospy.Subscriber('suction_tool/screw_suctioned',
                                           std_msgs.msg.Bool,
                                           self._state_callback)
        self._suctioned = False
        self._eject     = eject  # blow when releasing
        self._goal                     = o2as_msgs.msg.SuctionControlGoal()
        self._goal.fastening_tool_name = 'suction_tool'
        self._goal.turn_suction_on     = False
        self._goal.eject_screw         = False

    @staticmethod
    def base(prefix, eject, timeout):
        return GripperClient(*SuctionGripper._initargs(prefix, eject, timeout))

    @staticmethod
    def _initargs(prefix, eject, timeout):
        return ('suction', prefix + 'base_link', prefix + 'pad_link', timeout)

    def pregrasp(self, wait=True):
        return self._send_command(True)

    def grasp(self, wait=True):
        if not self._send_command(True):
            return False
        rospy.sleep(0.5)        # Wait until the state is updated.
        return self._suctioned

    def postgrasp(self, wait=True):
        return self._suctioned

    def release(self, wait=True):
        return self._send_command(False)

    def wait(self, wait=True):
        return self._suctioned

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _send_command(self, turn_suction_on):
        self._goal.turn_suction_on = turn_suction_on
        self._goal.eject_screw     = not turn_suction_on and self._eject
        self._client.send_goal(self._goal)
        if not self._client.wait_for_result(rospy.Duration(self.timeout)):
            rospy.logerr('Timeout[%f] has expired before goal finished',
                         self.timeout)
            return False
        result = self._client.get_result()
        return result.success

    def _state_callback(self, msg):
        self._suctioned = msg.data

######################################################################
#  class PrecisionGripper                                            #
######################################################################
class PrecisionGripper(GripperClient):
    def __init__(self, prefix='a_bot_gripper_', timeout=3.0):
        import o2as_msgs.msg

        super(PrecisionGripper, self) \
            .__init__(*PrecisionGripper._initargs(prefix, timeout))
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
    def base(prefix, timeout):
        return GripperClient(*PrecisionGripper._initargs(prefix, timeout))

    @staticmethod
    def _initargs(prefix, timeout):
        return ('two_finger',
                prefix + 'base_link', prefix + 'tip_link', timeout)

    @property
    def linear_motor_position(self):
        return self._goal.linear_motor_position

    def pregrasp(self, wait=True):
        cmd     = self.parameters['cmd']
        success = False
        if cmd == 'complex_pick_from_inside':
            success = self._inner_command(True, False, wait)
        elif cmd == 'complex_pick_from_outside':
            self._inner_command(False, False, wait)
        elif cmd == 'easy_pick_only_inner' or \
             cmd == 'inner_gripper_from_inside' or \
             cmd == '':
            success = self._inner_command(False, False, wait)
        elif cmd == 'easy_pick_outside_only_inner' or \
             cmd == 'inner_gripper_from_outside':
            success = self._inner_command(True, False, wait)
        return success

    def grasp(self, wait=True):
        cmd     = self.parameters['cmd']
        success = False
        if cmd == 'complex_pick_from_inside':
            success = self._inner_command(False, True,  wait) and \
                      self._outer_command(True,  False, wait)
        elif cmd == 'complex_pick_from_outside':
            success = self._inner_command(True, True,  wait) and \
                      self._outer_command(True, False, wait)
        elif cmd == 'easy_pick_only_inner' or \
             cmd == 'inner_gripper_from_inside' or \
             cmd == '':
            success = self._inner_command(True, True, wait)
        elif cmd == 'easy_pick_outside_only_inner' or \
             cmd == 'inner_gripper_from_outside':
            success = self._inner_command(False, True, wait)
        return success

    def release(self, wait=True):
        cmd     = self.parameters['cmd']
        success = False
        if cmd == 'complex_pick_from_inside':
            success = self._outer_command(False, False, wait) and \
                      self._inner_command(True,  False, wait)
        elif cmd == 'complex_pick_from_outside':
            success = self._outer_command(False, False, wait) and \
                      self._inner_command(False, False, wait)
        elif cmd == 'easy_pick_only_inner' or \
             cmd == 'inner_gripper_from_inside' or \
             cmd == '':
            success = self._inner_command(False, False, wait)
        elif cmd == 'easy_pick_outside_only_inner' or \
             cmd == 'inner_gripper_from_outside':
            success = self._inner_command(True, False, wait)
        return success

    def wait(self):
        if not self._client.wait_for_result(rospy.Duration(self.timeout)):
            rospy.logerr('Timeout[%f] has expired before goal finished',
                         self.timeout)
            return False
        return self._client.get_result().success

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _inner_command(self, close, grasps_an_object, wait):
        self._goal.open_inner_gripper_fully     = not close
        self._goal.close_inner_gripper_fully    = close
        self._goal.open_outer_gripper_fully     = False
        self._goal.close_outer_gripper_fully    = False
        self._goal.this_action_grasps_an_object = grasps_an_object
        self._client.send_goal(self._goal)
        return self.wait() if wait else True

    def _outer_command(self, close, grasps_an_object, wait):
        self._goal.open_inner_gripper_fully     = False
        self._goal.close_inner_gripper_fully    = False
        self._goal.open_outer_gripper_fully     = not close
        self._goal.close_outer_gripper_fully    = close
        self._goal.this_action_grasps_an_object = grasps_an_object
        self._client.send_goal(self._goal)
        return self.wait() if wait else True

######################################################################
#  class Lecp6Gripper                                                #
######################################################################
class Lecp6Gripper(GripperClient):
    def __init__(self, prefix='a_bot_gripper_',
                 timeout=3.0, open_no=1, close_no=2):
        import tranbo_control.msg

        super(Lecp6Gripper, self) \
            .__init__(*Lecp6Gripper._initargs(prefix, timeout))
        self._client = actionlib.SimpleActionClient(
                           '/arm_driver/lecp6_driver/lecp6',
                           tranbo_control.msg.Lecp6CommandAction)
        self._goal = tranbo_control.msg.Lecp6CommandGoal()

        self.parameters = {'release_stepdata': open_no,
                           'grasp_stepdata':   close_no}

    @staticmethod
    def base(prefix, timeout):
        return GripperClient(*Lecp6Gripper._initargs(prefix, timeout))

    @staticmethod
    def _initargs(prefix, timeout):
        return ('two_finger', prefix + 'base_link', prefix + 'link', timeout)

    def grasp(self, wait=True):
        return self._send_command(True, wait)

    def release(self, wait=True):
        return self._send_command(False, wait)

    def wait(self):
        if not self._client.wait_for_result(rospy.Duration(self.timeout)):
            rospy.logerr('Timeout[%f] has expired before goal finished',
                         self.timeout)
            return False
        return self._client.get_result().reached_goal

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _send_command(self, close, wait):
        self._goal.command.stepdata_no \
            = self.parameters['grasp_stepdata' if close else
                              'release_stepdata']
        self._client.send_goal(self._goal)
        return self.wait() if wait else True

######################################################################
#  class MagswitchGripper                                            #
######################################################################
class MagswitchGripper(GripperClient):
    def __init__(self, prefix='a_bot_magnet_', timeout=10.0,
                 sensitivity=0, contact_position=100, grasp_position=100):
        import tranbo_control.msg

        super(MagswitchGripper, self) \
            .__init__(*MagswitchGripper._initargs(prefix, timeout))
        self._client = actionlib.SimpleActionClient(
                              '/arm_driver/magswitch_driver/magswitch',
                              tranbo_control.msg.MagswitchCommandAction)
        self._goal   = tranbo_control.msg.MagswitchCommandGoal()
        self._calibration_step = 0

        self.parameters = {'sensitivity':      sensitivity,
                           'contact_position': contact_position,
                           'grasp_position':   grasp_position}

    @staticmethod
    def base(prefix, timeout):
        return GripperClient(*MagswitchGripper._initargs(prefix, timeout))

    @staticmethod
    def _initargs(prefix, timeout):
        return ('magnet', prefix + 'base_link', prefix + 'link', timeout)

    @property
    def calibration_step(self):
        return self._calibration_step

    def calibration(self, calibration_select):
        return self._send_command(0, True, 1, calibration_select)

    def pregrasp(self, wait=True):
        return self._send_command(self.parameters['contact_position'], wait)

    def grasp(self, wait=True):
        return True
        # return self._send_command(self.parameters['grasp_position'], wait)

    def postgrasp(self, wait=True):
        return self._send_command(self.parameters['grasp_position'], wait)

    def release(self, wait=True):
        return self._send_command(0, wait)

    def wait(self):
        if not self._client.wait_for_result(rospy.Duration(self.timeout)):
            rospy.logerr('Timeout[%.1f] has expired before goal finished',
                         self.timeout)
            return False
        result = self._client.get_result()
        if self._goal.command.calibration_trigger == 1:
            self._calibration_step = result.magswitch_out.calibration_step
            return position > 0 and result.magswitch_out.calibration_state != 0
        return result.reached_goal

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _send_command(self, position, wait,
                      calibration_trigger=0, calibration_select=0):
        self._calibration_step = 0
        self._goal.used_sdo = False
        self._goal.command.calibration_trigger = calibration_trigger
        self._goal.command.calibration_select  = calibration_select
        self._goal.command.sensitivity \
            = clip(self.parameters['sensitivity'], -30, 30)
        self._goal.command.position = clip(position, 0, 100)
        self._client.send_goal(self._goal)
        return self.wait() if wait else True
