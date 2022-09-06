/*
 *  \file	pose_head_group_tracker.cpp
 *  \brief	ROS tracker of aist_controllers::PoseHeadAction type
 */
#include <aist_controllers/joint_group_tracker.h>
#include <aist_controllers/PoseHeadAction.h>
#include <kdl/solveri.hpp>

namespace aist_controllers
{
/************************************************************************
*  class JointGroupTracker<PoseHeadAction>::Tracker		*
************************************************************************/
template <> bool
JointGroupTracker<aist_controllers::PoseHeadAction>
    ::Tracker::update(const goal_cp& goal)
{
  // Convert target pose to base_link.
    auto	original_pose = goal->target;
    original_pose.header.stamp = ros::Time::now();
    _listener.waitForTransform(_base_link,
			       original_pose.header.frame_id,
			       original_pose.header.stamp,
			       ros::Duration(1.0));
    geometry_msgs::PoseStamped	transformed_pose;
    _listener.transformPose(_base_link, original_pose, transformed_pose);
    KDL::Frame	target;
    tf::poseMsgToKDL(transformed_pose.pose, target);

  // Get current joint positions.
    KDL::JntArray	current_pos(njoints());
    jointsToKDL(_command.data, current_pos);

  // Compute target joint positions.
    KDL::JntArray	target_pos(njoints());
    const auto		error = _pos_iksolver->CartToJnt(current_pos,
							 target, target_pos);
    if (error < 0)
	ROS_ERROR_STREAM("(JointGroupTracker) IkSolver failed["
			 << solver_error_message(error) << ']');
    clamp(target_pos);
    ROS_DEBUG_STREAM("target_pos  = " << target_pos);

  // Set desired positions of joint group command.
    jointsFromKDL(target_pos, _command.data);

    return false;
}

}	// namepsace aist_controllers

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "pose_head_tracker");

    aist_controllers::JointGroupTracker<aist_controllers::PoseHeadAction>
	tracker("pose_head");
    ros::spin();

    return 0;
}
