import numpy as np
from numpy.linalg import norm
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import actionlib
import o2as_msgs.msg

class InnerPickDetection(object):

    def __init__(self):
         #Variable
        self._current_image = Image()

        # Config parameters
        # TODO: read values from config file
        self._x = 270
        self._y = 275
        self._w = 64
        self._h = 32

        self._image_topic = "/a_bot_camera/color/image_raw"

        # Subscriber
        rospy.Subscriber(self._image_topic, Image, self.image_callback)

        #define the action
        self._action_name = "inner_pick_detection_action"
        self._action_server = actionlib.SimpleActionServer(self._action_name, o2as_msgs.msg.innerPickDetectionAction, execute_cb=self.action_callback, auto_start = False)
        self._action_server.start()
        rospy.loginfo('Action server '+ str(self._action_name)+" started.")
        self.action_result = o2as_msgs.msg.innerPickDetectionResult()

        img_empty = cv2.imread('/root/catkin_ws/o2as_bg_ratio/images/image2.png')
        img_empty = np.asarray(img_empty)[:, :, ::-1]
        self._empty_bg_ratio = self.compute_red_ratio(img_empty, self._x, self._y, self._w, self._h)
        #rospy.get_param("/empty_bg_ratio", '0.7')
    
    # Action Callback
    def action_callback(self, goal):
        rospy.loginfo('Executing'+ str(self._action_name)+"."+"request sent:")
        rospy.loginfo(goal)
        rospy.loginfo('Executing'+ str(self._action_name)+"."+"request sent:")

        #TODO consider the part to compare the red ratio
        res = self.compute_red_ratio(self._current_image, self._x, self._y, self._w, self._h) < self._empty_bg_ratio

        self.action_result.success = res
        self.action_result.picked = res
        self._action_server.set_succeeded(self.action_result)

    # Image Callback
    def image_callback(self, msg_in):
        self._current_image = CvBridge.imgmsg_to_cv2(msg_in, desired_encoding="passthrough")
        self._current_image = np.asarray(self._current_image)[:, :, ::-1]

    #Compute ratio
    def compute_red_ratio(self, img, x, y, w, h, br_threshold=0.2, red_threshold=0.7, vis=False):
        """Compute the ratio of red area in the image.

        The returned value should be used to check if the precision gripper pick a
        part or not. Assumption here is that the picked part will decrease the
        ratio of red area; low value implies the gripper picks a part.

        Return True if bg_ratio < bg_threshold, False otherwise.

        :param img: RGB image, 8bit (0-255), numpy.ndarray, shape=(w, h, 3)
        :param int x: x-axis of Upper left of ROI.
        :param int y: y-axis of Upper left of ROI.
        :param int w: Width of ROI.
        :param int h: Height of ROI.
        """
        # bright pixels
        img = img / 255.0
        img = img[y:(y + h), x:(x + w), :]

        if vis:
            import matplotlib.pyplot as plt
            plt.imshow(img, interpolation="none")

        # Ignore dark area
        brightness = norm(img, axis=2)
        ixs = np.where(brightness > br_threshold)
        pixels = img[ixs[0], ixs[1], :]
        brightness = brightness[ixs[0], ixs[1]]

        # extract background
        red_ratio = pixels[:, 0] / brightness
        # plt.hist(red_ratio)
        ixs_bg = np.where(red_ratio > red_threshold)[0]

        # compute background ratio
        return float(ixs_bg.shape[0]) / pixels.shape[0]


if __name__ == "__main__":

  # Initialization
  rospy.init_node("o2as_inner_pick_detection")
  node = InnerPickDetection()

  rospy.spin()