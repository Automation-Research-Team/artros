#!/usr/bin/env python
import nep
import time
import threading
import time
import rospy

from geometry_msgs.msg import Point,Vector3
from std_msgs.msg import String, Int32
from std_msgs.msg import Header
from finger_pointing_msgs.msg import pointing

import nep
import time

# Important: 	You need to change the IP address <'127.0.0.1'> by
# 		the IP address of PC running NEP CLI
IP = '163.220.51.108'
# Create a new nep node
node = nep.node("getPointing")
conf = node.hybrid(IP)
# Create a new nep subscriber with the topic <'test'>
sub = node.new_sub('VR/Pointing', "json", conf)

rospy.init_node("getPointing", anonymous=True)

# topic = ""
# ROSpubPointing = rospy.Publisher(topic, Accel, queue_size=10)

header = Header()


ROSpubPointing = rospy.Publisher('pointing', pointing, queue_size = 10)


while not rospy.is_shutdown():

    # Read data in a non-blocking mode
    s, msg = sub.listen()
    # if s == True, then there is data in the socket
    if s:
        #print(msg)
        msg_pointing = pointing()     #クラスと同様に，インスタンスを作成することで，個々のフィールドに値を設定できる
        header.stamp = rospy.Time.now()  # Set the timestamp to the current time
        header.frame_id = "world"   # Set the frame ID (replace with your desired frame ID)
        finger_pos = Point()
        finger_pos.x =  msg["finger_pos"]["z"]
        finger_pos.y = -msg["finger_pos"]["x"]
        finger_pos.z =  msg["finger_pos"]["y"]
        finger_dir = Vector3()
        finger_dir.x =  msg["finger_dir"]["z"]
        finger_dir.y = -msg["finger_dir"]["x"]
        finger_dir.z =  msg["finger_dir"]["y"]
        pointing_state = msg["pointing_state"]

        msg_pointing.header = header
        msg_pointing.finger_pos = finger_pos
        msg_pointing.finger_dir = finger_dir
        msg_pointing.pointing_state = pointing_state
        ROSpubPointing.publish(msg_pointing)
