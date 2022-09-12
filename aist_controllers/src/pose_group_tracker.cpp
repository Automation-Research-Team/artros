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
  // Get current pose of pointing frame.
    const auto	now = ros::Time::now();
    _listener.waitForTransform(_base_link, goal->pointing_frame,
			       now, ros::Duration(1.0));
    tf::StampedTransform	current_pose;
    _listener.lookupTransform(_base_link, goal->pointing_frame,
			      now, current_pose);

  // Convert target pose to base_link.
    auto	original_pose = goal->target;
    original_pose.header.stamp = now;
    _listener.waitForTransform(_base_link, original_pose.header.frame_id,
			       original_pose.header.stamp, ros::Duration(1.0));
    geometry_msgs::PoseStamped	transformed_pose;
    _listener.transformPose(_base_link, original_pose, transformed_pose);
    tf::Stamped<tf::Pose>	target_pose;
    tf::poseStampedMsgToTF(transformed_pose, target_pose);

  // Get current joint positions.
    KDL::JntArray	current_pos(njoints());
    jointsToKDL(_command.data, current_pos);

  // Compute target joint positions.
    KDL::JntArray	target_pos(njoints());
    for (;;)
    {
	KDL::Frame	target;
	tf::poseTFToKDL(target_pose, target);

	const auto	error = _pos_iksolver->CartToJnt(current_pos, target,
							 target_pos);
	if (error >= 0)
	    break;

      // If no solutions found, update target pose to the middle of current
      // and target poses.
	target_pose.setOrigin(0.5*(current_pose.getOrigin() +
				   target_pose.getOrigin()));
	target_pose.setRotation(current_pose.getRotation().slerp(
				    target_pose.getRotation(), 0.5));
    }

  //clamp(target_pos);
    ROS_DEBUG_STREAM("target_pos = " << target_pos);

  // Duration of the time step
    const auto	dt = std::max(_duration, goal->min_duration);

  // Correct time_from_start in order to enforce maximum joint velocity.
    if (goal->max_velocity > 0)
    {
      // Compute the largest required rotation among all the joints
	double	rot_max = 0;
	for (size_t i = 0; i < current_pos.rows(); ++i)
	{
	    const auto	rot = std::abs(current_pos(i) - target_pos(i));
	    if (rot > rot_max)
		rot_max = rot;
	}

	const auto	dp_max = goal->max_velocity * dt.toSec();
	const auto	k = dp_max / std::max(dp_max, rot_max);

	for (size_t i = 0; i < current_pos.rows(); ++i)
	    target_pos(i) = current_pos(i)
			  + k * (target_pos(i) - current_pos(i));
    }

  // Set desired positions of trajectory command.
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
