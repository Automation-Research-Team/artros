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
from rclpy.node import Node
import dynamic_reconfigure.client

######################################################################
#  class CameraMultiplexerClient                                     #
######################################################################
class CameraMultiplexerClient(object):
    def __init__(self, server='camera_multiplexer'):
        super().__init__()
        self._camera_names = rospy.get_param(server + '/camera_names')
        self._ddr_client   = dynamic_reconfigure.client.Client(server,
                                                               timeout=30.0)

    @property
    def camera_names(self):
        return self._camera_names

    @property
    def active_camera(self):
        return self._ddr_client.get_configuration()['active_camera']

    def activate_camera(self, camera_name):
        if camera_name in self.camera_names:
            self._ddr_client.update_configuration({'active_camera':
                                                   camera_name})
            rospy.sleep(0.2)
            return True
        else:
            return False

######################################################################
#  class RealSenseMultiplexerClient                                  #
######################################################################
class RealSenseMultiplexerClient(CameraMultiplexerClient):

    class RealSenseCamera(object):
        def __init__(self, camera_name):
            super().__init__()
            self._ddr_client = dynamic_reconfigure.client.Client(
                                   camera_name + '/coded_light_depth_sensor',
                                   timeout=5.0)

        @property
        def laser_power(self):
            return self._ddr_client.get_configuration()['laser_power']

        @laser_power.setter
        def laser_power(self, value):
            self._ddr_client.update_configuration({'laser_power': value})

    def __init__(self, server='camera_multiplexer', value=16):
        try:
            super().__init__(server)
            self._cameras = dict(zip(self.camera_names,
                                 [RealSenseMultiplexerClient.RealSenseCamera(
                                     camera_name)
                                  for camera_name in self.camera_names]))
        except Exception as e:
            rospy.logerr(str(e))
            rospy.logerr("Cameras failed to initialize. "
            "Are the camera nodes started? Does /camera_multiplexer/camera_names"
            "contain unused cameras? camera_names: " + str(self.camera_names))
        for camera in self._cameras.values():
            camera.laser_power = 0
        self._cameras[self.active_camera].laser_power = value

    def activate_camera(self, camera_name, value=16):
        self._cameras[self.active_camera].laser_power = 0
        super().activate_camera(camera_name)
        self._cameras[self.active_camera].laser_power = value