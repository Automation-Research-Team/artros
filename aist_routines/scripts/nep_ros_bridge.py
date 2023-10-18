#!/usr/bin/env python
#
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

import rospy, rosgraph, rostopic
import nep
import signal, time

from threading          import Thread
from geometry_msgs.msg  import (Accel, Point, Pose, Quaternion, Transform,
                                Twist, Vector3, Wrench, Point32)
from sensor_msgs.msg    import (CameraInfo, JointState, Imu, Temperature,
                                PointCloud)
from std_msgs.msg       import String, Float32, ColorRGBA, Bool, Int32

from nepbridge.ROS2JSON import*

#########################################################################
#  class NepROSBridge                                                   #
#########################################################################
class NepROSBrige(object):
    def __init__(self):
        super(NepROSBridge, self).__init__()

        self._nep_node = nep.node('nep_bridge')
        self._nep_conf = nep_node(rospy.get_param('nep_ip', '163.220.51.108'))
        self._nep_pubs = {}
        self._topics   = []
        self._threads  = {}

    def run(self):
        rospy.on_shutdown(lambda: print("ROS node has been shutdown."))

        update_ros_info_thread = Thread(target=self._update_ros_info)
        update_ros_info_thread.daemon = True
        update_ros_info_thread.start()

        signal.signal(signal.SIGINT, signal_handler)

        prev_topics = []
        while True:
            new_topics = [topic for topic in topics if topic not in prev_topics]
            if new_topics:
                self._spawn_threads_for_new_topics(new_topics)
                prev_topics = topics.copy()
            rospy.sleep(1.0)

    def _update_ros_info(self):
        while not rospy.is_shutdown():
            try:
                master = rosgraph.Master('/rostopic')
                state  = master.getSystemState()
                self._topics = state[0]
            except rospy.ROSException as e:
                print("Error: " + str(e))

            rospy.sleep(1.0)

    def _spawn_threads_for_new_topics(self, new_topics):
        print("New Topics:")
        for topic in new_topics:
            name = topic[0]
            #print(name)
            if name == '/rosout' or name == '/rosout_agg':
                continue

            topic_type = rostopic.get_topic_type(topic[0])[0]
            #print("Topic type:", topic_type)
            self._threads[name] = Thread(target=thread_subs,
                                         args=({"name": name,
                                                "msg_type": topic_type},))
            self._threads[name].start()
            time.sleep(.5)
            #print(th)
            #print("  Message Type: " + str(topic_type.split("/")[-1]))

    def _add_joint_state_subscriber(self, topics_info):
        msg_type = topics_info["msg_type"]
        if msg_type == "sensor_msgs/JointState":
            name = topics_info["name"]
            self._nep_pubs[name] = {"type": msg_type,
                                    "pub" : self._nep_node.new_pub(
                                                name, "json", self._nep_conf)}
            rospy.Subscriber(name, JointState, self._jointstate_cb,
                             [name, msg_type])

    def _bool_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_bool(msg))

    def _int32_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_int32(msg))

    def _float32_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_float32(msg))

    def _string_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_string(msg))

    def _colorrgba_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_colorrgba(msg))

    def _vector3_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_vector3(msg))

    def _quaternion_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_quaternion(msg))

    def _point_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_point(msg))

    def _pose_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_pose(msg))

    def _transform_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_transform(msg))

    def _twist_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_twist(msg))

    def _wrench_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_wrench(msg))

    def _accel_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_accel(msg))

    def _camerainfo_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_camerainfo(msg))

    def _jointstate_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_jointstate(msg))

    def _imu_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_imu(msg))

    def _temperature_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_temperature(msg))

    def _pointcloud_cb(self, msg, topics_info):
        pub = self._nep_pubs[topics_info[0]]['pub']
        pub.publish(create_json_message_from_pointcloud(msg))


#########################################################################
#  entry point                                                          #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('nep_ros_bridge')

    nep_ros_bridge = NepROSBridge()
    nep_ros_bridge.run()


def signal_handler(sig, frame):
    print('Ctrl+C detected. Exiting gracefully...')
    sys.exit(0)

if __name__ == "__main__":
    rospy.on_shutdown(lambda: print("ROS node has been shutdown."))

    print_ros_info_thread = Thread(target=print_ros_info)
    print_ros_info_thread.daemon = True
    print_ros_info_thread.start()

    signal.signal(signal.SIGINT, signal_handler)

    prev_topics = []
    while True:
        new_topics = [topic for topic in topics if topic not in prev_topics]
        if new_topics:
            print_topics(new_topics)
            prev_topics = topics.copy()
        rospy.sleep(1.0)
