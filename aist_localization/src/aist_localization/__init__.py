import rospy
import actionlib
from std_srvs           import srv as ssrv
from aist_localization  import msg as lmsg
from operator           import itemgetter

#########################################################################
#  class LocalizationClient                                             #
#########################################################################
class LocalizationClient(object):
    def __init__(self):
        super(LocalizationClient, self).__init__()

        self._load_scene = rospy.ServiceProxy("localization/load_scene",
                                              ssrv.Trigger)
        self._localize = actionlib.SimpleActionClient("localization/localize",
                                                      lmsg.localizeAction)
        self._localize.wait_for_server()

    def load_scene(self):
        return self._load_scene().success

    def send_goal(self, object_name, number_of_poses=1):
        self._poses    = []
        self._overlaps = []
        goal = lmsg.localizeGoal()
        goal.object_name     = object_name
        goal.number_of_poses = number_of_poses
        self._localize.send_goal(goal, feedback_cb=self._feedback_cb)

    def wait_for_result(self, timeout=5):
        if (not self._localize.wait_for_result(timeout)):
            self._localize.cancel_goal()  # Cancel goal if timeout expired.

        # Sort obtained poses in descending order of overlap values.
        pairs = sorted(zip(self._poses, self._overlaps),
                       key=itemgetter(1), reverse=True)
        if len(pairs):
            self._poses, self._overlaps = zip(*pairs)

        result = self._localize.get_result()
        return (self._poses, self._overlaps,
                result.success if result else False)

    def _feedback_cb(self, feedback):
        self._poses.append(feedback.pose)
        self._overlaps.append(feedback.overlap)
