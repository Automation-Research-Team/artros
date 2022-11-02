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
import rospy
from actionlib            import SimpleActionClient
from aist_controllers.msg import PoseTrackingAction, PoseTrackingGoal

######################################################################
#  class PoseTrackingClient                                          #
######################################################################
class PoseTrackingClient(object):
    def __init__(self, server="/pose_tracking_servo"):
        super(PoseTrackingClient, self).__init__()

        self._pose_tracking = SimpleActionClient(server + '/pose_tracking',
                                                 PoseTrackingAction)

        if self._pose_tracking.wait_for_server(timeout=rospy.Duration(5)):
            rospy.loginfo('(PoseTrackingClient) connected to server[%s]',
                          server + '/pose_tracking')
        else:
            self._pose_tracking = None
            rospy.logerr('(PoseTrackingClient) failed to connect to server[%s]',
                         server + '/pose_tracking')

    @property
    def is_connected(self):
        return self._pose_tracking is not None

    def send_goal(self, target_offset,
                  positional_tolerance=(0, 0, 0), angular_tolerance=0,
                  feedback_cb=None):
        self._pose_tracking.send_goal(
            PoseTrackingGoal(target_offset=target_offset,
                             positional_tolerance=positional_tolerance,
                             angular_tolerance=angular_tolerance),
            feedback_cb=feedback_cb)

    def cancel_goal(self, wait=False):
        self._pose_tracking.cancel_goal()
        if wait:
            self._track_box.wait_for_result()

    def get_state(self):
        return self._pose_tracking.get_state()
