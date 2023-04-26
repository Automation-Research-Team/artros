# Copyright (C) 2020-2021, National Institute of Advanced Industrial Science
# and Technology (AIST), TOYOTA MOTOR CORPORATION, Ltd.
#
# Any using, copying, disclosing information regarding the software and
# documentation without permission of the copyright holders are prohibited.
# The software is provided "AS IS", without warranty of any kind, express or
# implied, including all implied warranties of merchantability and fitness.
# In no event shall the authors or copyright holders be liable for any claim,
# damages or other liability, whether in an action of contract, tort or
# otherwise, arising from, out of or in connection with the software or
# the use or other dealings in the software.
import rospy, threading
from actionlib             import SimpleActionClient
from actionlib_msgs.msg    import GoalStatus
from aist_moveit_servo.msg import (PoseTrackingAction, PoseTrackingGoal,
                                   PoseTrackingFeedback)
from std_srvs.srv          import Empty
from moveit_msgs.srv       import ChangeDriftDimensions, ChangeControlDimensions

######################################################################
#  class PoseTrackingClient                                          #
######################################################################
class PoseTrackingClient(object):
    def __init__(self, server="/pose_tracking_servo"):
        super(PoseTrackingClient, self).__init__()

        timeout = rospy.Duration(5)

        self._condition = threading.Condition()
        self._feedback  = None
        rospy.wait_for_service(server + '/reset_servo_status', timeout)
        self._reset_servo_status \
            = rospy.ServiceProxy(server + '/reset_servo_status', Empty)

        rospy.wait_for_service(server + '/change_drift_dimensions', timeout)
        self._change_drift_dimensions \
            = rospy.ServiceProxy(server + '/change_drift_dimensions',
                                 ChangeDriftDimensions)

        rospy.wait_for_service(server + '/change_control_dimensions', timeout)
        self._change_control_dimensions \
            = rospy.ServiceProxy(server + '/change_control_dimensions',
                                 ChangeControlDimensions)

        self._pose_tracking = SimpleActionClient(server + '/pose_tracking',
                                                 PoseTrackingAction)
        if not self._pose_tracking.wait_for_server(timeout=timeout):
            raise rospy.ROSExceptioon('failed to connect to action ')

        rospy.loginfo('(PoseTrackingClient) connected to server[%s]', server)

    @property
    def is_connected(self):
        return self._pose_tracking is not None

    def reset_servo_status(self):
        self._reset_servo_status()

    def change_drift_dimensions(self,
                                drift_x_translation, drift_y_translation,
                                drift_z_translation, drift_x_rotation,
                                drift_y_rotation,    drift_z_rotation):
        return self._change_drift_dimensions(drift_x_translation,
                                             drift_y_translation,
                                             drift_z_translation,
                                             drift_x_rotation,
                                             drift_y_rotation,
                                             drift_z_rotation).success

    def change_control_dimensions(self,
                                  control_x_translation, control_y_translation,
                                  control_z_translation, control_x_rotation,
                                  control_y_rotation,    control_z_rotation):
        return self._change_control_dimensions(control_x_translation,
                                               control_y_translation,
                                               control_z_translation,
                                               control_x_rotation,
                                               control_y_rotation,
                                               control_z_rotation).success

    def send_goal(self, target_offset,
                  positional_tolerance=(0, 0, 0), angular_tolerance=0,
                  terminate_on_success=False, servo_timeout=rospy.Duration(0.5),
                  done_cb=None):
        self._feedback = None
        self._pose_tracking.send_goal(PoseTrackingGoal(target_offset,
                                                       positional_tolerance,
                                                       angular_tolerance,
                                                       terminate_on_success,
                                                       servo_timeout),
                                      done_cb=done_cb,
                                      feedback_cb=self._feedback_cb)

    def cancel_goal(self, wait=False):
        if self.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._pose_tracking.cancel_goal()
            if wait:
                self._pose_tracking.wait_for_result()

    def cancel_all_goals(self):
        self._pose_tracking.cancel_all_goals()

    def wait_for_result(self, timeout=rospy.Duration()):
        if self._pose_tracking.wait_for_result(timeout):
            return self._pose_tracking.get_result().status
        return None

    def wait_for_tolerance_satisfied(self, timeout=rospy.Duration()):
        timeout_time = rospy.get_rostime() + timeout
        loop_period  = rospy.Duration(0.1)
        self._feedback = None
        with self._condition:
            while self._feedback is None:
                if self.get_state() not in (GoalStatus.PENDING,
                                            GoalStatus.ACTIVE):
                    break

                if timeout > rospy.Duration():
                    time_left = timeout_time - rospy.get_rostime()
                    if time_left <= rospy.Duration():
                        break
                    if time_left > loop_period:
                        time_left = loop_period
                else:
                    time_left = loop_period
                self._condition.wait(time_left.to_sec())
        return self._feedback

    def get_state(self):
        return self._pose_tracking.get_state()

    def _feedback_cb(self, feedback):
        if feedback.within_tolerance:
            with self._condition:
                self._feedback = feedback
                self._condition.notifyAll()
