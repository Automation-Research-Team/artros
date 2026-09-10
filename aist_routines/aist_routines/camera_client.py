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
import rclpy, time
from ddynamic_reconfigure2 import ParameterClient
from std_srvs.srv          import Trigger

######################################################################
#  class CameraClient                                                #
######################################################################
class CameraClient(ParameterClient):
    def __init__(self, node, name, props={}):
        super().__init__(node, name)
        self._logger = node.get_logger()

    @staticmethod
    def create(node, name, type_name, props):
        ClientClass = globals()[type_name]
        if ClientClass is None:
            raise RuntimeError('unknown type[%s] of the camera[%s]'
                               % (type_name, name))
        try:
            return ClientClass(node, name, **props)
        except RuntimeError as e:
            return CameraClient(node, name)

    @property
    def logger(self):
        return self._logger

    def is_continuous_shot(self):
        return True

    def continuous_shot(self, enabled):
        pass

    def trigger_frame(self):
        return True

######################################################################
#  class USBCamera                                                   #
######################################################################
class USBCamera(CameraClient):
    def __init__(self, node, name='live_camera'):
        super().__init__(node, name)

######################################################################
#  class AreaCamera                                                  #
######################################################################
class AreaCamera(CameraClient):
    def __init__(self, node, name='IIDCCamera'):
        super().__init__(node, name)

    def is_continuous_shot(self):
        return self.get_parameters_sync(['continuous_shot'])[0]

    def continuous_shot(self, enabled):
        self.set_parameters_sync([('continuous_shot', enabled)])

######################################################################
#  class CodedLightRealsenseCamera                                   #
######################################################################
class CodedLightRealsenseCamera(CameraClient):
    def __init__(self, node, name='a_bot_camera'):
        super().__init__(node, name)

    @property
    def laser_power(self):
        return self.get_parametrs_sync(
                   ['coded_light_depth_sensor.laser_power'])[0]

    @laser_power.setter
    def laser_power(self, value):
        return self.set_parameters_sync(
                   [('coded_light_depth_sensor.laser_power', value)])[0] \
                   .successful

######################################################################
#  class PhoXiCamera                                                 #
######################################################################
class PhoXiCamera(CameraClient):
    def __init__(self, node, name='a_phoxi_m_camera'):
        super().__init__(node, name)
        self._trigger_frame = node.create_client(Trigger,
                                                 name + '/trigger_frame')
        if not self._trigger_frame.wait_for_service(timeout_sec=1.0):
            txt = 'failed to establish connection to the service[%s]' \
                % (name + '/trigger_frame')
            self.logger.error(txt)
            raise RuntimeError(txt)

    def is_continuous_shot(self):
        return self.get_parameters_sync(['trigger_mode'])[0] == 0

    def continuous_shot(self, enabled):
        return self.set_parameters_sync([('trigger_mode',
                                          0 if enabled else 1)])[0].successful

    def trigger_frame(self):
        future = self._trigger_frame.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.1)
        return future.result().success

######################################################################
#  class ZividCamera                                                 #
######################################################################
class ZividCamera(CameraClient):
    def __init__(self, node, name='a_bot_camera'):
        from zivid_camera.srv import Capture, LoadSettingsFromFile

        super().__init__(node, name)
        # self._dyn_settings = DynReconfClient(name + '/settings', timeout=5.0)
        # self._dyn_acquisition_0 \
        #     = DynReconfClient(name + '/settings/acquisition_0', timeout=5.0)
        # self._dyn_acquisition_0.update_configuration({'enabled': True})
        self._trigger_frame = node.create_client(Capture, name + '/capture')
        if not self._trigger_frame.wait_for_service(timeout_sec=1.0):
            txt = 'failed to establish connection to the service[%s]' \
                % (name + '/capture')
            self.logger.error(txt)
            raise RuntimeError(txt)

    def trigger_frame(self):
        future = self._trigger_frame.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.1)
        return future.result().success
