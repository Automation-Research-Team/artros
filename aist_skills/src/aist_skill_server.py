#!/usr/bin/env python

from gripper import GripperBase, Robotiq85Gripper, SuctionGripper
import sys
from math import pi, radians
import copy

import numpy as np
import rospy
from geometry_msgs import msg as msg
import visualization_msgs.msg
import tf
from tf import transformations as tfs
import tf_conversions
import actionlib
import moveit_commander
from moveit_commander.conversions import pose_to_list

import o2as_msgs.srv
import aist_graspability.msg


def quaternion_msg_to_tf(orientation):
    """Convert quaternion geometry_msgs.msg.Quaternion to tf quaternion (as numpy.ndarray).
    """

    return np.array([[orientation.x],[orientation.y],[orientation.z],[orientation.w]])

def quaternion_tf_to_msg(quaternion):
    """Convert quaternion tf quaternion (as numpy.ndarray) to geometry_msgs.msg.Quaternion.
    """

    return msg.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

def rotatePoseByRPY(roll, pitch, yaw, inpose):
    """Return pose rotated by rpy.

    This function is converted of o2as_helper_function.h.
    """

    q_rotate = tfs.quaternion_from_euler(roll, pitch, yaw)
    q = quaternion_msg_to_tf(inpose.orientation)
    q = tfs.quaternion_multiply(q, q_rotate)
    inpose.orientation = quaternion_tf_to_msg(q)

    return inpose

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

    elif type(goal) is msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

def clamp(x, min_x, max_x):
    return min(max(min_x, x), max_x)


class SkillServer(object):
    def __init__(self):
        super(SkillServer, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)

        if rospy.get_param('use_real_robot', False):
            self.grippers = {
                'a_bot': Robotiq85Gripper('a_bot_'),
                'b_bot': SuctionGripper('b_bot_single_'),
                'c_bot': Robotiq85Gripper('c_bot_'),
                'd_bot': SuctionGripper('d_bot_dual_')
            }
        else:
            self.grippers = {
                'a_bot': GripperBase('a_bot_robotiq_85_gripper',
                                     'a_bot_robotiq_85_tip_link'),
                'b_bot': GripperBase('b_bot_single_suction_gripper',
                                     'b_bot_single_suction_gripper_pad_link'),
                'c_bot': GripperBase('c_bot_robotiq_85_gripper',
                                     'c_bot_robotiq_85_tip_link'),
                'd_bot': GripperBase('d_bot_dual_suction_gripper',
                                     'd_bot_dual_suction_gripper_pad_link')
            }

        # Topics to publish
        self.pubMarker = rospy.Publisher("visualization_marker",
                                         visualization_msgs.msg.Marker,
                                         queue_size=10)
        # Services
        self.publishMarkerService = rospy.Service("aist_skills/publishMarker",
                                                  o2as_msgs.srv.publishMarker,
                                                  self.publishMarkerCallback)
        self.goToNamedPoseService = rospy.Service("aist_skills/goToNamedPose",
                                                  o2as_msgs.srv.goToNamedPose,
                                                  self.goToNamedPoseCallback)
        self.goToPoseGoalService  = rospy.Service("aist_skills/goToPoseGoal",
                                                  o2as_msgs.srv.goToPoseGoal,
                                                  self.goToPoseGoalCallback)
        self.gripperCommandService = rospy.Service("aist_skills/gripperCommand",
                                                   o2as_msgs.srv.gripperCommand,
                                                   self.gripperCommandCallback)

        # Action clients
        # self.fge_action_client = actionlib.SimpleActionClient('aist_graspability/search_grasp_from_phoxi', aist_graspability.msg.SearchGraspFromPhoxiAction)
        # self.fge_action_client.wait_for_server()

        # Action servers
        self.pickOrPlaceAction = actionlib.SimpleActionServer(
                                        'aist_skills/pickOrPlace',
                                        o2as_msgs.msg.pickOrPlaceAction,
                                        execute_cb=self.pickOrPlaceCallback,
                                        auto_start=False)
        self.pickOrPlaceAction.start()

        self.listener = tf.TransformListener()
        self.marker_id_count = 0

        rospy.loginfo("aist_skills server starting up!")

    # publishMarker stuffs
    def publishMarkerCallback(self, req):
        rospy.loginfo("Received publishMarker callback.")
        return self.publishMarker(req.marker_pose, req.marker_type)

    def publishMarker(self, marker_pose, marker_type):
        marker = visualization_msgs.msg.Marker()
        marker.header = copy.deepcopy(marker_pose.header)
        marker.header.stamp = rospy.Time.now()
        marker.pose = copy.deepcopy(marker_pose.pose)

        marker.ns = "markers"
        marker.id = self.marker_id_count
        self.marker_id_count += 1
        marker.lifetime = rospy.Duration(10.0)
        marker.action = visualization_msgs.msg.Marker.ADD

        if marker_type == "pose":
            self.publishPoseMarker(marker_pose)
            # Add a flat sphere
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.scale.x = .01
            marker.scale.y = .05
            marker.scale.z = .05
            marker.color.g = 1.0
            marker.color.a = 0.8
        elif marker_type == "pick_pose":
            self.publishPoseMarker(marker_pose)
            # Add a flat sphere
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.scale.x = .01
            marker.scale.y = .05
            marker.scale.z = .05
            marker.color.r = 0.8
            marker.color.g = 0.4
            marker.color.a = 0.8
        elif marker_type == "place_pose":
            self.publishPoseMarker(marker_pose)
            # Add a flat sphere
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.scale.x = .01
            marker.scale.y = .05
            marker.scale.z = .05
            marker.color.g = 1.0
            marker.color.a = 0.8
        elif marker_type == "aist_vision_result":
            # Add a sphere
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.scale.x = .01
            marker.scale.y = .01
            marker.scale.z = .01
            marker.color.r = 0.8
            marker.color.g = 0.4
            marker.color.b = 0.0
            marker.color.a = 0.8
        elif marker_type == "":
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.scale.x = .02
            marker.scale.y = .1
            marker.scale.z = .1
            marker.color.g = 1.0
            marker.color.a = 0.8
        else:
            rospy.logwarn("No useful marker message received.")
            return False

        self.pubMarker_.publish(marker)
        if self.marker_id_count > 50:
            self.marker_id_count = 0
        return True

    def publishPoseMarker(self, marker_pose):
        marker = visualization_msgs.msg.Marker()
        marker.header = copy.deepcopy(marker_pose.header)
        marker.header.stamp = rospy.Time.now()
        marker.pose = copy.deepcopy(marker_pose.pose)

        marker.ns = "markers"
        marker.id = self.marker_id_count
        self.marker_id_count += 1
        marker.lifetime = rospy.Duration(10)
        marker.action = visualization_msgs.msg.Marker.ADD

        # This draws a TF-like frame.
        marker.type = visualization_msgs.msg.Marker.ARROW
        marker.scale.x = .1
        marker.scale.y = .01
        marker.scale.z = .01
        marker.color.a = .8

        arrow_x = visualization_msgs.msg.Marker()
        arrow_y = visualization_msgs.msg.Marker()
        arrow_z = visualization_msgs.msg.Marker()
        arrow_x = copy.deepcopy(marker)
        arrow_y = copy.deepcopy(marker)
        arrow_z = copy.deepcopy(marker)
        arrow_x.id = self.marker_id_count
        self.marker_id_count += 1
        arrow_y.id = self.marker_id_count
        self.marker_id_count += 1
        arrow_z.id = self.marker_id_count
        self.marker_id_count += 1
        arrow_x.color.r = 1.0
        arrow_y.color.g = 1.0
        arrow_z.color.b = 1.0

        arrow_y.pose = rotatePoseByRPY(0, 0, pi/2, arrow_y.pose)
        arrow_z.pose = rotatePoseByRPY(0, -pi/2, 0, arrow_z.pose)

        self.pubMarker_.publish(arrow_x)
        self.pubMarker_.publish(arrow_y)
        self.pubMarker_.publish(arrow_z)

        return True

    def goToNamedPoseCallback(self, req):
        group = moveit_commander.MoveGroupCommander(req.planning_group)
        group.set_named_target(req.named_pose)
        group.set_max_velocity_scaling_factor(1.0)
        success = group.go(wait=True)
        group.clear_pose_targets()
        return success

    def goToPoseGoalCallback(self, req):
        return self.goToPoseGoal(req.planning_group, req.target_pose,
                                 req.speed, req.high_precision,
                                 req.end_effector_link, req.move_lin)

    def goToPoseGoal(self, group_name, target_pose,
                     speed=1.0, high_precision=False, end_effector_link="",
                     move_lin=True):
        # self.publish_marker(target_pose, "pose")

        if end_effector_link == "":
            end_effector_link = self.grippers[group_name].tip_link
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_end_effector_link(end_effector_link)
        group.set_pose_target(target_pose)
        group.set_max_velocity_scaling_factor(clamp(speed, 0.0, 1.0))

        res = o2as_msgs.srv.goToPoseGoalResponse()

        if move_lin:
            pose_world = self.listener.transformPose(group.get_planning_frame(),
                                                     target_pose).pose
            waypoints  = []
            waypoints.append(pose_world)
            (plan, fraction) = group.compute_cartesian_path(
                                waypoints,  # waypoints to follow
                                0.0005,     # eef_step
                                0.0)        # jump_threshold
            rospy.loginfo("Compute cartesian path succeeded with " +
                          str(fraction*100) + "%")
            robots      = moveit_commander.RobotCommander()
            plan        = group.retime_trajectory(robots.get_current_state(),
                                                  plan, speed)
            res.success = group.execute(plan, wait=True)
        else:
            goal_tolerance = group.get_goal_tolerance()
            planning_time  = group.get_planning_time()
            if high_precision:
                group.set_goal_tolerance(.000001)
                group.set_planning_time(10)
            res.success = group.go(wait=True)
            if high_precision:
                group.set_goal_tolerance(goal_tolerance)
                group.set_planning_time(planning_time)

        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        res.current_pose = group.get_current_pose()
        res.all_close    = all_close(target_pose.pose, res.current_pose.pose,
                                     0.01)
        rospy.loginfo("Target position: ")
        rospy.loginfo(target_pose)
        rospy.loginfo("Actual position: ")
        rospy.loginfo(res.current_pose)

        return res

    def gripperCommandCallback(self, req):
        gripper = self.grippers[req.group_name]
        res     = o2as_msgs.srv.gripperCommandResponse()
        if req.command == "pregrasp":
            gripper.pregrasp()
        elif req.command == "grasp":
            gripper.grasp()
        elif req.command == "release":
            gripper.release()
        res.success = True
        return res

    def pickOrPlaceCallback(self, goal):
        gripper  = self.grippers[goal.group_name]
        feedback = o2as_msgs.msg.pickOrPlaceFeedback()

        # Go to approach pose.
        res = self.goToPoseGoal(
                       goal.group_name,
                       self.gripperPose(goal.pose, goal.approach_offset),
                       goal.speed_fast if goal.pick else goal.speed_slow)
        feedback.current_pose = res.current_pose
        self.pickOrPlaceAction.publish_feedback(feedback)

        # Pregrasp
        if goal.pick:
            gripper.pregrasp()

        # Go to pick/place pose.
        res = self.goToPoseGoal(goal.group_name,
                                self.gripperPose(goal.pose,
                                                 gripper.grasp_offset),
                                goal.speed_slow)
        feedback.current_pose = res.current_pose
        self.pickOrPlaceAction.publish_feedback(feedback)

        # Grasp or release
        res_gripper = gripper.grasp() if goal.pick else gripper.release()

        # Go back to approach pose.
        result = o2as_msgs.msg.pickOrPlaceResult()
        res = self.goToPoseGoal(
                       goal.group_name,
                       self.gripperPose(goal.pose, goal.approach_offset),
                       goal.speed_slow if goal.pick else goal.speed_fast)
        result.current_pose = res.current_pose
        result.success      = result.success and res_gripper
        self.pickOrPlaceAction.set_succeeded(result)

    def gripperPose(self, target_pose, offset):
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
                    (0, 0, offset),
                    tfs.quaternion_from_euler(0, radians(90), 0)))

        gripper_pose = msg.PoseStamped()
        gripper_pose.header.frame_id  = target_pose.header.frame_id
        gripper_pose.pose = msg.Pose(msg.Point(
                                         *tfs.translation_from_matrix(T)),
                                     msg.Quaternion(
                                         *tfs.quaternion_from_matrix(T)))
        return gripper_pose


if __name__ == '__main__':
    rospy.init_node("aist_skills", anonymous=True)
    try:
        node = SkillServer()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
