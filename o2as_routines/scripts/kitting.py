#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi
import numpy as np
from collections import namedtuple

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg
from o2as_usb_relay.srv import *
from o2as_graspability_estimation.srv import *

from o2as_routines.base import O2ASBaseRoutines

from PIL import Image, ImageDraw

LOG_LEVEL = log_level = rospy.DEBUG
# LOG_LEVEL = log_level = rospy.INFO

CroppedArea = namedtuple("CroppedArea", ["min_row", "max_row", "min_col", "max_col"])

# Temporary variable for integration of graspability estimation in AIST
bin_id_for_graspability_estimation = {
  "part_7": 1,
  "part_13": 2,
  "part_11": 3,
  "part_8": 4,
  "part_4": 5,
  "part_14": 6,
  "part_17": 7,
  "part_5": 8,
  "part_12": 9,
  "part_9": 10
}

part_poses_demo = {
  "part_4":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.07),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_5":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_6":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_7":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_8":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.024),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_9":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_10":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_11":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.037),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_12":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.009),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
    
  },
  "part_13":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.024),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_14":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_15":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_16":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_17":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_18":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  }
}

class KittingClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the kitting task. See base.py for shared convenience functions.
  """
  def __init__(self):

    super(KittingClass, self).__init__()
    
    # params
    self.bin_id = rospy.get_param('part_bin_list')
    self.gripper_id = rospy.get_param('gripper_id')
    self.tray_id = rospy.get_param('tray_id')
    
    # services
    self._suction = rospy.ServiceProxy("o2as_usb_relay/set_power", SetPower)
    self._search_grasp = rospy.ServiceProxy("search_grasp", SearchGrasp)

    self.set_up_item_parameters()
    rospy.sleep(.5)

  def set_up_item_parameters(self):
    self.item_names = []
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # 

  ################ ----- Routines  
  ################ 
  ################ 

  def switch_suction(self, on=False):
    # Judge success or fail using pressure status.
    
    return self._suction(1, on)

  def pick_using_dual_suction_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link=""):
    rospy.loginfo("Go to the target.")
    res = self.move_lin(group_name, pose_goal_stamped, speed_slow, end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to the target.")
      return False
    res = self.switch_suction(True)
    rospy.sleep(1)
    if not res:
      rospy.logdebug("Couldn't pick the target using suction.")
      return False

  def naive_pick(self, group_name, bin_id, speed_fast = 1.0, speed_slow = 1.0, approach_height = 0.05,bin_eff_height = 0.07, bin_eff_xoff = 0, bin_eff_deg_angle = 0,end_effector_link = ""):

    #Place gripper above bin
    goal_pose_above = geometry_msgs.msg.PoseStamped()
    goal_pose_above.header.frame_id = bin_id
    goal_pose_above.pose.position.x = bin_eff_xoff
    goal_pose_above.pose.position.y = 0
    goal_pose_above.pose.position.z = bin_eff_height
    goal_pose_above.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2 , 0))
    res, move_group_res = self.move_lin(group_name, goal_pose_above, speed_slow, "")
    print("res")
    print(res)
    print(move_group_res)
    if not move_group_res:
      rospy.loginfo("Couldn't go to the target.")
    #TODO Problem with gazebo controller while controlling the robot with movelin
      return False

    #Open the gripper 
    self.send_gripper_command(gripper= "precision_gripper_inner", command="open")
    #self.precision_gripper_inner_open()

    #Descend
    rospy.loginfo("Descend on target.")
    goal_pose_descend = copy.deepcopy(goal_pose_above)
    goal_pose_descend.pose.position.z = goal_pose_descend.pose.position.z-approach_height
    res, move_group_res  = self.move_lin(group_name, goal_pose_descend, speed_fast, end_effector_link)
    if not move_group_res:
      rospy.loginfo("Couldn't go to above the target bin.")
      return False

    #Close the gripper 
    self.send_gripper_command(gripper= "precision_gripper_inner", command="close")
    #self.precision_gripper_inner_close()

    #Ascend
    rospy.loginfo("Ascend with target.")
    goal_pose_ascend = copy.deepcopy(goal_pose_descend)
    goal_pose_ascend.pose.position.z = goal_pose_ascend.pose.position.z+approach_height
    res, move_group_res  = self.move_lin(group_name, goal_pose_ascend, speed_fast, end_effector_link)
    if not move_group_res:
      rospy.loginfo("Couldn't go to above the target bin.")
      return False


  
  def view_bin(self, group_name, bin_id, speed_fast = 1.0, speed_slow = 1.0, bin_eff_height = 0.2, bin_eff_xoff = 0, bin_eff_deg_angle = 20,end_effector_link = ""):
    # TODO: adjust the x,z and end effector orientatio for optimal view of the bin to use with the  \search_grasp service
    goal_pose = geometry_msgs.msg.PoseStamped()
    goal_pose.header.frame_id = bin_id
    goal_pose.pose.position.x = bin_eff_xoff
    goal_pose.pose.position.y = 0
    goal_pose.pose.position.z = bin_eff_height
    #goal orientation for a_bot_camera_depth_frame 
    #goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2 + 20*pi/180, 0))
    #res = self.go_to_pose_goal(group_name, goal_pose, speed_slow, "a_bot_camera_depth_frame")
    #goal orientation for gripper
    goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2 + 20*pi/180, 0))
    res = self.move_lin(group_name, goal_pose, speed_slow, "")
    if not res:
      rospy.loginfo("Couldn't go to the target.")
    #TODO Problem with gazebo controller while controlling the robot with movelin
    #  return False
 
    #TODO Esure that the the motion is finished before generating the mask.

    #TODO not sure if the sleep is necessary is move_lin wait for the motion to be finished?
    rospy.sleep(1)

    point_top1 = geometry_msgs.msg.PointStamped()
    point_top1.header.frame_id = str(bin_id)+"_bottom_front_right_corner"
    point_top1.point = geometry_msgs.msg.Point(0.0, 0.0, 0.0)

    point_top2 = geometry_msgs.msg.PointStamped()
    point_top2.header.frame_id = str(bin_id)+"_bottom_back_right_corner"
    point_top2.point = geometry_msgs.msg.Point(0.0, 0.0, 0.0)

    point_top3 = geometry_msgs.msg.PointStamped()
    point_top3.header.frame_id = str(bin_id)+"_bottom_back_left_corner"
    point_top3.point = geometry_msgs.msg.Point(0.0, 0.0, 0.0)

    point_top4 = geometry_msgs.msg.PointStamped()
    point_top4.header.frame_id = str(bin_id)+"_bottom_front_left_corner"
    point_top4.point = geometry_msgs.msg.Point(0.0, 0.0, 0.0)

    #TODO change the fisheye from to the depth frame in casse of offset. but fisheye should e ok since the two images are aligned (depth and rgb) after the real sense node
    point_top1_cam = self.listener.transformPoint("a_bot_camera_depth_optical_frame", point_top1).point
    point_top2_cam = self.listener.transformPoint("a_bot_camera_depth_optical_frame", point_top2).point
    point_top3_cam = self.listener.transformPoint("a_bot_camera_depth_optical_frame", point_top3).point
    point_top4_cam = self.listener.transformPoint("a_bot_camera_depth_optical_frame", point_top4).point


    #print("point_top1_cam")
    #print(point_top1_cam)
    #print("point_top2_cam")
    #print(point_top2_cam)
    #print("point_top3_cam")
    #print(point_top3_cam)
    #print("point_top4_cam")
    #print(point_top4_cam)

    #point_test_center_cam = geometry_msgs.msg.Point(0, 0, 0.4)
    #print(point_test_center_cam)

#TODO project the 4 points in the depth plane (which projection matrix to use from the camera)
    #projection matrix
    #554.3827128226441, 0.0, 320.5, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 1.0    
    #
    #P: [554.3827128226441, 0.0, 320.5, -38.80678989758509, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    #TODO take the parameters from the /camera_info topic instead


#for gazebo
    cameraMatK = np.array([[554.3827128226441, 0.0, 320.5],
                           [0.0, 554.3827128226441, 240.5],
                           [0.0, 0.0, 1.0]])

#for ID Realsense on robot ID61*41   width 640 height 360
#    cameraMatK = np.array([[461.605774, 0.0, 318.471497],
#                           [0.0, 461.605804, 180.336258],
#                           [0.0, 0.0, 1.0]])



    point_top1_cam_np = np.array([point_top1_cam.x, point_top1_cam.y, point_top1_cam.z])   
    point_top2_cam_np = np.array([point_top2_cam.x, point_top2_cam.y, point_top2_cam.z])   
    point_top3_cam_np = np.array([point_top3_cam.x, point_top3_cam.y, point_top3_cam.z])   
    point_top4_cam_np = np.array([point_top4_cam.x, point_top4_cam.y, point_top4_cam.z])   
    #used to test the projection
    #point_test_center_cam_np = np.array([point_test_center_cam.x, point_test_center_cam.y, point_test_center_cam.z])   
      
    point_top1_img_np = cameraMatK.dot(point_top1_cam_np)
    point_top2_img_np = cameraMatK.dot(point_top2_cam_np)
    point_top3_img_np = cameraMatK.dot(point_top3_cam_np)
    point_top4_img_np = cameraMatK.dot(point_top4_cam_np)
    #point_test_center_img_np = cameraMatK.dot(point_test_center_cam_np) 

    #print(point_top1_img_np)
    #print(point_top2_img_np)
    #print(point_top3_img_np)
    #print(point_top4_img_np)
    #print(point_test_center_img_np)

    polygon = [(point_top1_img_np[0]/point_top1_img_np[2],point_top1_img_np[1]/point_top1_img_np[2]),
               (point_top2_img_np[0]/point_top2_img_np[2],point_top2_img_np[1]/point_top2_img_np[2]),
               (point_top3_img_np[0]/point_top3_img_np[2],point_top3_img_np[1]/point_top3_img_np[2]),
               (point_top4_img_np[0]/point_top4_img_np[2],point_top4_img_np[1]/point_top4_img_np[2])]

    mask_img = Image.new('L', (640,480), 0)
    ImageDraw.Draw(mask_img).polygon(polygon, outline = 1, fill = 255)
    mask_image_np = np.array(mask_img)
    namefile = "mask_"+str(bin_id)+".png"
    mask_img.save(namefile,'PNG')


    #used to do the bitwise comparison between the mask and the original image
    #mask = Image.new('RGB', (640,480), 255)
    #ImageDraw.Draw(mask).polygon(polygon, outline = 1, fill = 1)
    #mask_np = np.array(mask)
 
    #img_original = Image.open("bin_origin.png")
    #img_original_np = np.array(img_original)
 
    #img_res_np = np.bitwise_and(mask_np, img_original_np)
    #img_res = Image.fromarray(np.uint8(img_res_np)) 
    #img_res.save('mask_original_bin.png','PNG')


#    point_top1_cam = t.transformPoint("a_bot_camera_depth_frame", point_top1)
#    point_top1_cam = t.transformPoint(point_top4.header.frame_id, point_top1)


  def pick_using_precision_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link = ""):
    # TODO: here is for Osaka Univ.
    pass

  def place_using_dual_suction_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link = ""):
    rospy.loginfo("Go to the target.")
    res = self.move_lin(group_name, pose_goal_stamped, speed, end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to the target.")
      return False
    res = self.switch_suction(False)
    rospy.sleep(1)
    if not res:
      return False

  def place_using_precision_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link =""):
    # TODO: here is for Osaka Univ.
    pass

  def pick(self, group_name, pose_goal_stamped, speed_fast = 1.0, speed_slow = 1.0, end_effector_link = "", approach_height = 0.15):

    rospy.loginfo("Go to above the target bin.")
    pose_goal_above = copy.deepcopy(pose_goal_stamped)
    pose_goal_above.pose.position.z = approach_height
    res = self.move_lin(group_name, pose_goal_above, speed_fast, end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to above the target bin.")
      return False

    if end_effector_link == "dual_suction_gripper_pad_link":
      self.pick_using_dual_suction_gripper(group_name, pose_goal_stamped, speed_slow, end_effector_link, approach_height)
    elif end_effector_link == "gripper":
      self.pick_using_precision_gripper(group_name, pose_goal_stamped, speed_slow, end_effector_link, approach_height)

    rospy.loginfo("Go to above the target bin.")
    res = self.move_lin(group_name, pose_goal_above, speed_slow, end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to above the target bin.")
      return False

    return True
    
  def place(self, group_name, pose_goal_stamped, speed_fast = 1.0, speed_slow = 1.0, end_effector_link = "", approach_height = 0.15):
    rospy.loginfo("Go to above the target tray partition.")
    pose_goal_above = copy.deepcopy(pose_goal_stamped)
    pose_goal_above.pose.position.z = approach_height
    res = self.move_lin(group_name, pose_goal_above, speed_slow, end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to above the target bin.")
      return False



    rospy.loginfo("Go to above the target tray partition.")
    res = self.move_lin(group_name, pose_goal_above, speed_fast, end_effector_link)
    # if not res:
    #   rospy.logdebug("Couldn't go to above the target bin.")
    #   return False

    rospy.loginfo("Go back to home.")
    res = self.go_to_named_pose("home", group_name, speed_fast)
    # if not res:
    #   rospy.logdebug("Couldn't go back to home.")
    #   return False

    return True

  ################ ----- Demos  
  ################ 
  ################ 

  def pick_and_place_demo(self):

    speed_fast = 1.0
    speed_slow = 1.0

    for set_num in range(1,4):
      item_list = rospy.get_param("/set_"+str(set_num))
      rospy.loginfo(item_list)
      rospy.loginfo("kitintg set_"+str(set_num) +" started!")

      for item in item_list:
        req_search_grasp = SearchGraspRequest()
        req_search_grasp.part_id = int(str(item).strip("part_"))
        req_search_grasp.bin_id = int(bin_id_for_graspability_estimation[item])
        req_search_grasp.filename = str(rospy.Time.now()) + ".tiff"
        req_search_grasp.gripper = "suction"
        resp_search_grasp = self._search_grasp(req_search_grasp)

        if self.gripper_id[item] == "suction":
          # NOTE: should be given object_position from vision package (o2as_graspability_estimation)
          object_position = geometry_msgs.msg.PointStamped()
          object_position.header.frame_id = "a_phoxi_m_sensor"
          object_position.point = geometry_msgs.msg.Point(
            resp_search_grasp.pos3D[0].x, 
            resp_search_grasp.pos3D[0].y, 
            resp_search_grasp.pos3D[0].z)
          rospy.logdebug("\nGrasp point in %s: (x, y, z) = (%f, %f, %f)", 
            object_position.header.frame_id, 
            object_position.point.x, 
            object_position.point.y, 
            object_position.point.z)

          # Transform object_position to goal_pose in bin
          goal_pose = geometry_msgs.msg.PoseStamped()
          goal_pose.header.frame_id = "world"
          goal_pose.pose.position = self.listener.transformPoint(goal_pose.header.frame_id, object_position).point
          goal_pose.pose.orientation = copy.deepcopy(self.downward_orientation)
          rospy.logdebug("\nGrasp point in %s: (x, y, z) = (%f, %f, %f)", 
            goal_pose.header.frame_id, 
            goal_pose.pose.position.x, 
            goal_pose.pose.position.y, 
            goal_pose.pose.position.z)
          # FIXME: if finished check errors between scene and real, you must remove code about world_pose and robot_pose
          world_pose = geometry_msgs.msg.PoseStamped()
          world_pose.header.frame_id = "world"
          world_pose.pose.position = self.listener.transformPoint(world_pose.header.frame_id, object_position).point
          world_pose.pose.orientation = copy.deepcopy(self.downward_orientation)
          rospy.logdebug("\nGrasp point in %s: (x, y, z) = (%f, %f, %f)", 
            world_pose.header.frame_id, 
            world_pose.pose.position.x, 
            world_pose.pose.position.y, 
            world_pose.pose.position.z)
          
          res = self.pick("b_bot", goal_pose, speed_fast, speed_slow, "b_bot_dual_suction_gripper_pad_link", 0.1)
          robot_pose = self.groups["b_bot"].get_current_pose("b_bot_dual_suction_gripper_pad_link")
          rospy.logdebug("\nrobot ee position in %s: (x, y, z) = (%s, %s, %s)", 
            robot_pose.header.frame_id, 
            robot_pose.pose.position.x, 
            robot_pose.pose.position.y, 
            robot_pose.pose.position.z)
          if not res:
            rospy.logerr("Failed to pick target object.")
            return
          raw_input()

          self.go_to_named_pose("home", "b_bot")

          # place_pose = geometry_msgs.msg.PoseStamped()
          # place_pose.header.frame_id = self.tray_id[item]
          # place_pose.pose.position = copy.deepcopy(part_poses_demo[item]["goal_position"])
          # place_pose.pose.orientation = copy.deepcopy(part_poses_demo[item]["goal_orientation"])
          # res = self.place("b_bot", place_pose, speed_fast, speed_slow, "b_bot_dual_suction_gripper_pad_link")
          # if not res:
          #   rospy.logerr("Failed to place target object.")
          #   return
          # raw_input()

      print(" ____  _   _ ____  ____  _____ _   _ ____  ")
      print("/ ___|| | | / ___||  _ \| ____| \ | |  _ \ ")
      print("\___ \| | | \___ \| |_) |  _| |  \| | | | |")
      print(" ___) | |_| |___) |  __/| |___| |\  | |_| |")
      print("|____/ \___/|____/|_|   |_____|_| \_|____/ ")
      print("                                           ")

      raw_input()
  
  def kitting_task(self):
    # self.go_to_named_pose("home", "c_bot")
    # self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

    #self.pick_and_place_demo()
    self.naive_pick("a_bot", "set2_bin1_3")

    # self.go_to_named_pose("home", "c_bot")
    # self.go_to_named_pose("home", "b_bot")
    # self.go_to_named_pose("home", "a_bot")

    # TODO


if __name__ == '__main__':

  try:
    kit = KittingClass()
    kit.set_up_item_parameters()
    
    kit.kitting_task()
    # kit.check_point_test()
    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
