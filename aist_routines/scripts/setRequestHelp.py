#!/usr/bin/env python
import nep
import rospy
from finger_pointing_msgs.msg import request_help

# Important: 	You need to change the IP address <'127.0.0.1'> by
# 		the IP address of PC running NEP CLI
IP = '163.220.51.108'
# Create a new nep node
node = nep.node('setRequestHelp')
conf = node.hybrid(IP)
# Create a new nep subscriber with the topic <'test'>
pubRequestHelp = node.new_pub('VR/RequestHelp', 'json', conf)
rospy.init_node('setRequestHelp', anonymous=True)


def callback_RequestHelp(data):
    global pubRequestHelp
    robot_name = data.robot_name
    item_id = data.item_id
    pose = data.pose.pose

    msg_pose = {'position':    {'x': -pose.position.y,
                                'y':  pose.position.z,
                                'z':  pose.position.x},
                'orientation': {'x': -pose.orientation.y,
                                'y':  pose.orientation.z,
                                'z':  pose.orientation.x,
                                'w':  pose.orientation.w }}
    request = data.request
    message = data.message
    pubRequestHelp.publish({'robot_name': robot_name,
                            'item_id':    item_id,
                            'pose':       msg_pose,
                            'request':    request,
                            'message':    message})

 ## Here write the ROS subscriber code ...
topic_name = 'help'
rospy.Subscriber(topic_name, request_help, callback_RequestHelp)
rospy.spin()
