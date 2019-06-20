#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
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
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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
# Author: Toshio UESHIBA

import sys
import copy
import rospy
from math import pi, radians, degrees

from tf import TransformListener, transformations as tfs
import moveit_commander
from moveit_commander.conversions import pose_to_list

from geometry_msgs import msg as gmsg

from GripperClient      import GripperClient, Robotiq85Gripper, \
                               SuctionGripper, PrecisionGripper
from CameraClient       import CameraClient, PhoXiCamera, RealsenseCamera
from GraspabilityClient import GraspabilityClient
from MarkerPublisher    import MarkerPublisher
from PickOrPlaceAction  import PickOrPlaceAction
from URScriptPublisher  import URScriptPublisher

######################################################################
#  global fucntions                                                  #
######################################################################
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is gmsg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is gmsg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

def clamp(x, min_x, max_x):
    return min(max(min_x, x), max_x)

######################################################################
#  class AISTBaseRoutines                                            #
######################################################################
class AISTBaseRoutines(object):
    def __init__(self):
        super(AISTBaseRoutines, self).__init__()
        rospy.init_node("aist_routines", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.listener = TransformListener()
        rospy.sleep(1.0)        # Necessary for listner spinning up

        # Grippers and cameras
        if rospy.get_param("use_real_robot", False):
            self._grippers = {
                # "a_bot": Robotiq85Gripper("a_bot_"),
                "a_bot": PrecisionGripper("a_bot_"),
                "b_bot": SuctionGripper("b_bot_single_"),
                "c_bot": Robotiq85Gripper("c_bot_"),
                "d_bot": SuctionGripper("d_bot_dual_")
            }
            self._cameras = {
                "a_phoxi_m_camera": PhoXiCamera("a_phoxi_m_camera"),
                "a_bot_camera":     RealsenseCamera("a_bot_camera"),
            }
            self._urscript_publishers = {
                "a_bot": URScriptPublisher("a_bot"),
                "b_bot": URScriptPublisher("b_bot"),
                "c_bot": URScriptPublisher("c_bot"),
                "d_bot": URScriptPublisher("d_bot"),
            }
        else:
            self._grippers = {
                # "a_bot": GripperClient("a_bot_robotiq_85_gripper",
                #                        "two-finger",
                #                        "a_bot_robotiq_85_base_link",
                #                        "a_bot_robotiq_85_tip_link"),
                "a_bot": GripperClient("a_bot_gripper",
                                       "two-finger",
                                       "a_bot_gripper_base_link",
                                       "a_bot_gripper_tip_link"),
                "b_bot": GripperClient("b_bot_single_suction_gripper",
                                       "suction",
                                       "b_bot_single_suction_gripper_base_link",
                                       "b_bot_single_suction_gripper_pad_link"),
                "c_bot": GripperClient("c_bot_robotiq_85_gripper",
                                       "two-finer",
                                       "c_bot_robotiq_85_base_link",
                                       "c_bot_robotiq_85_tip_link"),
                "d_bot": GripperClient("d_bot_dual_suction_gripper",
                                       "suction",
                                       "d_bot_dual_suction_gripper_base_link",
                                       "d_bot_dual_suction_gripper_pad_link")
            }
            self._cameras = {
                "a_phoxi_m_camera": CameraClient(
                                        "a_phoxi_m_camera",
                                        "depth",
                                        "/a_phoxi_m_camera/camera_info",
                                        "/a_phoxi_m_camera/depth_map"),
                "a_bot_camera":     CameraClient(
                                        "a_bot_camera",
                                        "depth",
                                        "/a_bot_camera/rgb/camera_info",
                                        "/a_bot_camera/depth/points"),
            }

        self._markerPublisher    = MarkerPublisher()
        self._graspabilityClient = GraspabilityClient()
        self._pickOrPlaceAction  = PickOrPlaceAction(self)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        rospy.signal_shutdown("AISTBaseRoutines() completed.")
        # rospy.spin()  # Wait for Ctrl-C pressed to shutdown the routines
        return False  # Do not forward exceptions

    # Basic motion stuffs
    def go_to_named_pose(self, named_pose, group_name):
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_named_target(named_pose)
        group.set_max_velocity_scaling_factor(1.0)
        success = group.go(wait=True)
        group.clear_pose_targets()
        return success

    def go_to_frame(self, robot_name, target_frame, offset=(0, 0, 0),
                    speed=1.0, high_precision=False, end_effector_link="",
                    move_lin=False):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose            = gmsg.Pose(gmsg.Point(0, 0, 0),
                                                gmsg.Quaternion(0, 0, 0, 1))
        return self.go_to_pose_goal(robot_name,
                                    self.effector_target_pose(target_pose,
                                                              offset),
                                    speed, high_precision, end_effector_link,
                                    move_lin)

    def go_to_pose_goal(self, robot_name, target_pose, speed=1.0,
                        high_precision=False, end_effector_link="",
                        move_lin=False):
        # rospy.loginfo("move to " + self.format_pose(target_pose))
        self.publish_marker(target_pose, "pose")

        if end_effector_link == "":
            end_effector_link = self._grippers[robot_name].tip_link

        group = moveit_commander.MoveGroupCommander(robot_name)
        group.set_end_effector_link(end_effector_link)
        group.set_pose_target(target_pose)
        group.set_max_velocity_scaling_factor(clamp(speed, 0.0, 1.0))

        if move_lin:
            pose_world = self.listener.transformPose(
                                group.get_planning_frame(), target_pose).pose
            waypoints  = []
            waypoints.append(pose_world)
            (plan, fraction) = group.compute_cartesian_path(waypoints,
                                                            0.0005,  # eef_step
                                                            0.0) # jump_threshold
            # rospy.loginfo("Compute cartesian path succeeded with " +
            #               str(fraction*100) + "%")
            robots      = moveit_commander.RobotCommander()
            plan        = group.retime_trajectory(robots.get_current_state(),
                                                  plan, speed)
            success = group.execute(plan, wait=True)
        else:
            goal_tolerance = group.get_goal_tolerance()
            planning_time  = group.get_planning_time()
            if high_precision:
                group.set_goal_tolerance(.000001)
                group.set_planning_time(10)
            success = group.go(wait=True)
            if high_precision:
                group.set_goal_tolerance(goal_tolerance)
                group.set_planning_time(planning_time)

        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        current_pose = group.get_current_pose()
        is_all_close = all_close(target_pose.pose, current_pose.pose, 0.01)
        # rospy.loginfo("reached " + self.format_pose(current_pose))
        return (success, is_all_close, current_pose)

    # UR script motions
    def do_move_j(self, robot_name,
                  joint_positions, acceleration=0.5, velocity=0.5, wait=False):
        pub = self._urscript_publishers[robot_name]
        return pub.move_j(self, joint_positions, acceleration, velocity, wait)

    def do_lin_move(self, robot_name,
                    target_pose, acceleration=0.5, velocity=0.03, wait=False):
        pub = self._urscript_publishers[robot_name]
        return pub.lin_move(target_pose, acceleration, velocity, wait)

    def do_lin_mov_rel(self, robot_name, translation,
                       acceleration=0.5, velocity=0.03, wait=False):
        pub = self._urscript_publishers[robot_name]
        return pub.lin_move_rel(translation, acceleration, velocity, wait)

    def do_linear_push(self, robot_name, force=10.0, wait=True, direction="Z+",
                       max_approach_distance=0.1, forward_speed=0.02):
        pub = self._urscript_publishers[robot_name]
        return pub.linear_push(force, direction,
                               max_approach_distance, forward_speed, wait)

    def do_spiral_motion(self, robot_name, acceleration=0.1, velocity=0.03,
                         max_radius=0.0065, radius_increment=0.002,
                         theta_increment=30, spiral_axis="Z", wait=False):
        pub = self._urscript_publishers[robot_name]
        return pub.spiral_motion(acceleration, velocity,
                                 max_radius, radius_increment,
                                 theta_increment, spiral_axis, wait)

    def do_insertion(self, robot_name,
                     max_force=10.0, force_direction="Z+",
                     forward_speed=0.02, max_approach_distance=0.1,
                     max_radius=0.004, radius_increment=0.0003,
                     max_insertion_distance=0.035, impedance_mass=10,
                     peck_mode=False, wait=False):
        pub = self._urscript_publishers[robot_name]
        return pub.insertion(max_force, force_direction,
                             forward_speed, max_approach_distance,
                             max_radius, radius_increment,
                             max_insertion_distance, impedance_mass,
                             peck_mode, wait)

    def do_horizontal_insertion(self, robot_name,
                                max_force=10.0, force_direction="Y-",
                                forward_speed=0.02, max_approach_distance=0.1,
                                max_radius=0.007, radius_increment=0.0003,
                                max_insertion_distance=0.035,
                                impedance_mass=10,
                                peck_mode=False, wait=False):
        pub = self._urscript_publishers[robot_name]
        return pub.horizontal_insertion(max_force, force_direction,
                                        forward_speed, max_approach_distance,
                                        max_radius, radius_increment,
                                        max_insertion_distance,
                                        impedance_mass, peck_mode, wait)

    # Gripper stuffs
    def gripper(self, robot_name):
        return self._grippers[robot_name]

    def pregrasp(self, robot_name, command=""):
        return self._grippers[robot_name].pregrasp(command)

    def grasp(self, robot_name, command=""):
        return self._grippers[robot_name].grasp(command)

    def release(self, robot_name, command=""):
        return self._grippers[robot_name].release(command)

    # Camera stuffs
    def camera(self, camera_name):
        return self._cameras[camera_name]

    def start_acquisition(self, camera_name):
        return self._cameras[camera_name].start_acquisition()

    def stop_acquisition(self, camera_name):
        return self._cameras[camera_name].stop_acquisition()

    # Marker stuffs
    def delete_all_markers(self):
        self._markerPublisher.delete_all()

    def publish_marker(self, pose_stamped, marker_type, text="", lifetime=15):
        return self._markerPublisher.add(pose_stamped, marker_type,
                                         text, lifetime)

    # Graspability stuffs
    def create_background_image(self, camera_name):
        camera = self._cameras[camera_name]
        camera.start_acquisition()
        success = self._graspabilityClient.create_background_image(
                        camera.image_topic)
        camera.stop_acquisition()
        return success

    def create_mask_image(self, camera_name, nbins):
        camera = self._cameras[camera_name]
        camera.start_acquisition()
        success = self._graspabilityClient.create_mask_image(
                        camera.image_topic, nbins)
        camera.stop_acquisition()
        return success

    def search_graspability(self, robot_name, camera_name, part_id, bin_id,
                            marker_lifetime=60):
        gripper = self._grippers[robot_name]
        camera  = self._cameras[camera_name]
        camera.start_acquisition()
        (poses, rotipz, gscore, success) = \
            self._graspabilityClient.search(camera.camera_info_topic,
                                            camera.depth_topic,
                                            camera.normal_topic,
                                            gripper.type, part_id, bin_id)
        camera.stop_acquisition()
        if success:
            for i, pose in enumerate(poses):
                self.publish_marker(pose, "graspability",
                                    "{}[{:.3f}]".format(i, gscore[i]),
                                    lifetime=marker_lifetime)
        return (poses, rotipz, gscore, success)

    # Pick and place action stuffs
    def pick(self, robot_name, target_pose,
             grasp_offset=0.0, gripper_command="",
             speed_fast=1.0, speed_slow=0.1, approach_offset=0.10,
             liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        return self._pickOrPlaceAction.execute(
            robot_name, target_pose, True, gripper_command,
            grasp_offset, approach_offset, liftup_after,
            speed_fast, speed_slow, acc_fast, acc_slow)

    def place(self, robot_name, target_pose,
              grasp_offset=0.0, gripper_command="",
              speed_fast=1.0, speed_slow=0.1, approach_offset=0.05,
              liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        return self._pickOrPlaceAction.execute(
            robot_name, target_pose, False, gripper_command,
            grasp_offset, approach_offset, liftup_after,
            speed_fast, speed_slow, acc_fast, acc_slow)

    def pick_at_frame(self, robot_name, target_frame, offset=(0, 0, 0),
                      grasp_offset=0.0, gripper_command="",
                      speed_fast=1.0, speed_slow=0.1, approach_offset=0.05,
                      liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose            = gmsg.Pose(gmsg.Point(0, 0, 0),
                                                gmsg.Quaternion(0, 0, 0, 1))
        return self.pick(robot_name, target_pose,
                         grasp_offset, gripper_command,
                         speed_fast, speed_slow, approach_offset,
                         liftup_after, acc_fast, acc_slow)

    def place_at_frame(self, robot_name, target_frame, offset=(0, 0, 0),
                       grasp_offset=0.0, gripper_command="",
                       speed_fast=1.0, speed_slow=0.1, approach_offset=0.05,
                       liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose            = gmsg.Pose(gmsg.Point(0, 0, 0),
                                                gmsg.Quaternion(0, 0, 0, 1))
        return self.place(robot_name, target_pose,
                          grasp_offset, gripper_command,
                          speed_fast, speed_slow, approach_offset,
                          liftup_after, acc_fast, acc_slow)

    # Utility functions
    def format_pose(self, poseStamped):
        pose = self.listener.transformPose("workspace_center",
                                            poseStamped).pose
        rpy  = map(degrees, tfs.euler_from_quaternion([pose.orientation.w,
                                                       pose.orientation.x,
                                                       pose.orientation.y,
                                                       pose.orientation.z]))
        return "[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]".format(
            pose.position.x, pose.position.y, pose.position.z, *rpy)

    def effector_target_pose(self, target_pose, offset):
        T = tfs.concatenate_matrices(
                self.listener.fromTranslationRotation(
                    (target_pose.pose.position.x,
                     target_pose.pose.position.y,
                     target_pose.pose.position.z),
                    (target_pose.pose.orientation.x,
                     target_pose.pose.orientation.y,
                     target_pose.pose.orientation.z,
                     target_pose.pose.orientation.w)),
                self.listener.fromTranslationRotation(
                    offset,
                    tfs.quaternion_from_euler(0, radians(90), 0)))
        pose = gmsg.PoseStamped()
        pose.header.frame_id = target_pose.header.frame_id
        pose.pose = gmsg.Pose(gmsg.Point(*tfs.translation_from_matrix(T)),
                              gmsg.Quaternion(*tfs.quaternion_from_matrix(T)))
        return pose
