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
from dynamic_reconfigure.client import Client as DynReconfClient
from std_srvs.srv               import Trigger, SetBool

######################################################################
#  class CameraClient                                                #
######################################################################
class CameraClient(object):
    def __init__(self, name, camera_type):
        self._name = name
        self._type = camera_type

    @staticmethod
    def create(name, type_name):
        ClientClass = globals()[type_name]
        if rospy.get_param('use_real_robot', False):
            return ClientClass(name)
        else:
            return ClientClass.base(name)

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

    def is_continuous_shot(self):
        return True

    def continuous_shot(self, enabled):
        pass

    def trigger_frame(self):
        return True

######################################################################
#  class MonocularCamera                                             #
######################################################################
class MonocularCamera(CameraClient):
    def __init__(self, name='IIDCCamera'):
        super(MonocularCamera, self).__init__(name, 'area')
        self._dyn_reconf = DynReconfClient(name, timeout=None)

    @staticmethod
    def base(name):
        return CameraClient(name, 'area')

    def is_continuous_shot(self):
        return self._dyn_reconf.get_configuration()['continuous_shot']

    def continuous_shot(self, enabled):
        self._dyn_reconf.update_configuration({'continuous_shot' : enabled})

######################################################################
#  class DepthCamera                                                 #
######################################################################
class DepthCamera(CameraClient):
    def __init__(self, name):
        super(DepthCamera, self).__init__(name, 'depth')

    @staticmethod
    def base(name):
        return CameraClient(name, 'depth')

######################################################################
#  class RealSenseCamera                                             #
######################################################################
class RealSenseCamera(DepthCamera):
    def __init__(self, name='a_bot_camera'):
        super(RealSenseCamera, self).__init__(name)
        self._enable     = rospy.ServiceProxy(name + '/enable', SetBool)
        self._dyn_camera = DynReconfClient(name, timeout=5.0)

    def continuous_shot(self, enabled):
        self._enable(enabled)
        return True

    # def trigger_frame(self):
    #     self.continuous_shot(True)
    #     rospy.sleep(0.5)
    #     self.continuous_shot(False)
    #     return True

######################################################################
#  class CodedLightRealSenseCamera                                   #
######################################################################
class CodedLightRealSenseCamera(RealSenseCamera):
    def __init__(self, name='a_bot_camera'):
        super(CodedLightRealSenseCamera, self).__init__(name)
        self._dyn_sensor = DynReconfClient(name + '/coded_light_depth_sensor',
                                           timeout=5.0)
        # Don't change current value of laser power not to activate
        # multiple realsense cameras simultaneously.

    @property
    def laser_power(self):
        return self._dyn_sensor.get_configuration()['laser_power']

    @laser_power.setter
    def laser_power(self, value):
        self._dyn_sensor.update_configuration({'laser_power' : value})

######################################################################
#  class PhoXiCamera                                                 #
######################################################################
class PhoXiCamera(DepthCamera):
    def __init__(self, name='a_phoxi_m_camera'):
        super(PhoXiCamera, self).__init__(name)
        self._dyn_reconf    = DynReconfClient(name, timeout=5.0)
        self._trigger_frame = rospy.ServiceProxy(name + '/trigger_frame',
                                                 Trigger, persistent=True)

    def is_continuous_shot(self):
        self._dyn_reconf.get_configuration()['trigger_mode'] == 0

    def continuous_shot(self, enabled):
        self._dyn_reconf.update_configuration({'trigger_mode' :
                                               0 if enabled else 1})
        return True

    def trigger_frame(self):
        try:
            return self._trigger_frame().success
        except rospy.ServiceException:
            self._trigger_frame \
                = rospy.ServiceProxy(self.name + '/trigger_frame',
                                     Trigger, persistent=True)
            return self._trigger_frame().success
