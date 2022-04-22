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
import dynamic_reconfigure.client
from std_srvs import srv as ssrv
from aist_depth_filter import DepthFilterClient as DepthFilter

######################################################################
#  class RealSenseCamera                                             #
######################################################################
class RealSenseCamera(object):
    def __init__(self, name="a_bot_camera"):
        super(RealSenseCamera, self).__init__()
        self._name       = name
        self._dyn_camera = dynamic_reconfigure.client.Client(name, timeout=5.0)
        self._dyn_sensor = dynamic_reconfigure.client.Client(
                               name + "/coded_light_depth_sensor", timeout=5.0)
        self.laser_power = 16

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

    @property
    def camera_info_topic(self):
        return self._name + "/color/camera_info"

    @property
    def image_topic(self):
        return self._name + "/color/image_raw"

    @property
    def pointcloud_topic(self):
        return self._name + "/depth/color/points"

    @property
    def depth_topic(self):
        return self._name + "/aligned_depth_to_color/image_raw"

    @property
    def laser_power(self):
        ret = self._dyn_sensor.get_configuration()
        return ret["laser_power"]

    @laser_power.setter
    def laser_power(self, value):
        self._dyn_sensor.update_configuration({"laser_power" : value})

    def enalbe_laser(self, enabled):
        if enabled:
            self._dyn_sensor.update_configuration(
                {"laser_power" : self._recent_laser_power})
        else:
            self._recent_laser_power = self.laser_power
            self._dyn_sensor.update_configuration({"laser_power" : 0})

#########################################################################
#  class RealSenseMultiplexer                                           #
#########################################################################
class RealSenseMultiplexer(object):
    def __init__(self, name="multiplexer", camera_names):
        super(RealSenseMultiplexer, self).__init__()

        self._cameras = {}
        self._filters = {}
        for camera_name in camera_names:
            self._cameras[camera_name] = RealSenseCamera(camera_name)
            self._filters[camera_name] = DepthFilter(camera_name)
            self._cameras[camera_name].enable_laser(False)
        if camera_names:
            self._selected_camera = camera_names[0]
            self._camera.enable_laser(True)
        else:
            self._selected_camera = ""

    # Camera selection stuffs
    @property
    def selected_camera(self):
        return self._selected_camera

    def select_camera(self, camera_name):
        self._camera.enable_laser(False)
        self._selected_camera = camera_name
        self._camera.enable_laser(True)

    # Camera control stuffs
    @property
    def laser_power(self):
        return self._camera.laser_power

    @laser_power.setter
    def laser_power(self, value):
        self._camera.laser_power = value

    def enalbe_laser(self, enabled):
        self._camera.enable_laser(enabled)

    # Filter control stuffs
    def saveBG(self):
        return self._filter.saveBG()

    def capture(self):
        return self._filter.capture()

    @property
    def background_threshold(self):
        return self._filter.background_threshold

    @background_threshold.setter
    def background_threshold(self, value):
        self._filter.background_threshold = value

    @property
    def depth_range(self):
        return self._filter.depth_range

    @depth_range.setter
    def depth_range(self, range):
        self._filter.depth_range = range

    @property
    def roi(self):
        return self._filter.roi

    @roi.setter
    def roi(self, bbox):
        self._filter.roi = bbox

    @property
    def window_radius(self):
        return self._filter.window_radius

    @window_radius.setter
    def window_radius(self, value):
        self._filter.window_radius = value

    @property
    def scale(self):
        return self._filter.scale

    @scale.setter
    def scale(self, value):
        self._filter.scale = value

    @property
    def _camera(self):
        return self._cameras[self._selected_camera]

    @property
    def _filter(self):
        return self._filters[self._selected_camera]
