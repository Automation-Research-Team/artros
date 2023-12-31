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
from geometry_msgs.msg     import Transform
from math                  import degrees

######################################################################
#  class PoseTrackingClient                                          #
######################################################################
class PoseTrackingClient(SimpleActionClient):
    def __init__(self, server="/pose_tracking_servo"):
        SimpleActionClient.__init__(self, server + '/pose_tracking',
                                    PoseTrackingAction)

        timeout = rospy.Duration(5)

        self._condition        = threading.Condition()
        self._within_tolerance = False
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

        if not self.wait_for_server(timeout=timeout):
            raise rospy.ROSExceptioon('failed to connect to action ')

        rospy.loginfo('(PoseTrackingClient) connected to server[%s]', server)

    def reset_servo_status(self):
        self._reset_servo_status()

    def change_drift_dimensions(self,
                                drift_x_translation, drift_y_translation,
                                drift_z_translation, drift_x_rotation,
                                drift_y_rotation,    drift_z_rotation):
        transform_jog_frame_to_drift_frame = Transform()
        return self._change_drift_dimensions(
                drift_x_translation, drift_y_translation, drift_z_translation,
                drift_x_rotation, drift_y_rotation, drift_z_rotation,
                transform_jog_frame_to_drift_frame).success

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
                  terminate_on_success=False, reset_input_lpf=True,
                  servo_timeout=rospy.Duration(1.0), done_cb=None):
        SimpleActionClient.send_goal(
            self,
            PoseTrackingGoal(target_offset, positional_tolerance,
                             angular_tolerance, terminate_on_success,
                             reset_input_lpf, servo_timeout),
            done_cb=done_cb, feedback_cb=self._feedback_cb)

    def wait_for_tolerance_state(self, within_tolerance,
                                 timeout=rospy.Duration(0)):
        timeout_time = rospy.Time.now() + timeout
        loop_period  = rospy.Duration(0.1)
        self._within_tolerance = not within_tolerance
        rospy.loginfo('(PoseTrackingClient) Wait for tolerance %s' \
                      % ('SATISFIED' if within_tolerance else 'VIOLATED'))
        with self._condition:
            while self._within_tolerance != within_tolerance:
                if self.get_state() not in (GoalStatus.PENDING,
                                            GoalStatus.ACTIVE):
                    break                       # servo preempted or aborted
                if timeout > rospy.Duration(0):
                    time_left = timeout_time - rospy.Time.now()
                    if time_left <= rospy.Duration(0):
                        break                   # timeout expired
                    if time_left > loop_period:
                        time_left = loop_period
                else:
                    time_left = loop_period
                self._condition.wait(time_left.to_sec())
        return self._within_tolerance == within_tolerance

    def _feedback_cb(self, feedback):
        if feedback.within_tolerance != self._within_tolerance:
            with self._condition:
                rospy.loginfo('(PoseTrackingClient) Notify that tolerance %s, err=[%f,%f,%f; %f]' \
                      % ('SATISFIED' if feedback.within_tolerance else 'VIOLATED',
                         feedback.positional_error[0],
                         feedback.positional_error[1],
                         feedback.positional_error[2],
                         degrees(feedback.angular_error)))
                self._within_tolerance = feedback.within_tolerance
                self._condition.notify()
