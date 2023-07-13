#!/usr/bin/env python
import nep
import time
import threading
import time
import rospy
import rosgraph
import threading
import rostopic
import signal
import sys
import rospy

from geometry_msgs.msg  import (Accel, Point, Pose, Quaternion, Transform,
                                Twist, Vector3, Wrench, Point32)
from sensor_msgs.msg    import (CameraInfo, JointState, Imu, Temperature,
                                PointCloud)
from std_msgs.msg       import String, Float32, ColorRGBA, Bool, Int32

from nepbridge.ROS2JSON import*



pubs = {}
th = {}


def thread_subs(topics_info):
    global pubs, conf
    topic_name = topics_info["name"]
    topic_type = topics_info["msg_type"]
    if topic_type == "sensor_msgs/JointState":
        pubs[topic_name] = {"type": topic_type,
                            "pub" : nepnode.new_pub(topic_name, "json", conf)}
        rospy.Subscriber(topic_name, JointState, callback_JointState,
                         [topic_name, topic_type])

    rospy.spin()


def callback_Transform(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_transform(data))
def callback_Twist(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_twist(data))
def callback_Wrench(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_wrench(data))
def callback_Vector3(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_vector3(data))
def callback_Accel(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_accel(data))
def callback_Point(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_point(data))
def callback_Pose(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_pose(data))
def callback_Quaternion(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_quaternion(data))
def callback_Int32(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_int32(data))
def callback_String(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_string(data))
def callback_Float32(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_float32(data))
def callback_ColorRGBA(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_colorrgba(data))
def callback_Bool(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_bool(data))
def callback_CameraInfo(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_camerainfo(data))
def callback_JointState(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_jointstate(data))
def callback_IMU(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_imu(data))
def callback_Temperature(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_temperature(data))
def callback_PointCloud(data, topics_info):
    global pubs
    pubs[topics_info[0]]['pub'].publish(create_json_message_from_pointcloud(data))


ip_address = "163.220.51.108"
try:
    ip_address = sys.argv[1]
    # Use the argument as needed
    print("IP Address:", ip_address)
except:
    pass

node_name = 'nep_bridge'
nepnode   = nep.node(node_name)        # Create a new node
conf      = nepnode.hybrid(ip_address)
rospy.init_node(node_name, anonymous=True)


topics = []

def print_ros_info():
    global topics
    while not rospy.is_shutdown():
        try:
            master = rosgraph.Master('/rostopic')
            state = master.getSystemState()

            nodes = state[1]
            topics = state[0]

        except rospy.ROSException as e:
            print("Error: " + str(e))

        rospy.sleep(1.0)

def print_topics(new_topics):
    print("New Topics:")
    for topic in new_topics:
        topic_name = topic[0]
        #print(topic_name)
        if topic_name != '/rosout' and topic_name != '/rosout_agg':
            topic_type = rostopic.get_topic_type(topic[0])[0]
            #print("Topic type:", topic_type)
            th[topic_name] = threading.Thread(target=thread_subs,
                                              args=({"name": topic_name,
                                                     "msg_type": topic_type},))
            th[topic_name].start()
            time.sleep(.5)
            #print(th)

            #print("  Message Type: " + str(topic_type.split("/")[-1]))

def signal_handler(sig, frame):
    print('Ctrl+C detected. Exiting gracefully...')
    sys.exit(0)

if __name__ == "__main__":
    rospy.on_shutdown(lambda: print("ROS node has been shutdown."))

    print_ros_info_thread = threading.Thread(target=print_ros_info)
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
