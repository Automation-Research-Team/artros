/*
 *  \file	pose_head_tracker.cpp
 *  \brief	ROS tracker of aist_controllers::PoseHeadAction type
 */
#include <aist_controllers/joint_trajectory_tracker.h>
#include <aist_controllers/PoseHeadAction.h>
#include <kdl/solveri.hpp>

namespace aist_controllers
{
/************************************************************************
*  class JointTrajectoryTracker<PoseHeadAction>::Tracker		*
************************************************************************/
template <> bool
JointTrajectoryTracker<aist_controllers::PoseHeadAction>
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

  // Compute target joint positions.
    KDL::JntArray	target_pos(_jnt_pos.rows());
    const auto		error = _pos_iksolver->CartToJnt(_jnt_pos,
							 target, target_pos);
    if (error != KDL::SolverI::E_NOERROR)
	ROS_ERROR_STREAM("(JointTrajectoryTracker) IkSolver failed["
			 << solver_error_message(error) << ']');
    clamp(target_pos);
    ROS_DEBUG_STREAM("target_pos  = " << target_pos);

  // Set desired positions of trajectory command.
    auto&	point = _trajectory.points[0];
    jointsFromKDL(target_pos, point.positions);

  // Set desired time of the pointing_frame reaching at the target.
    point.time_from_start = std::max(goal->min_duration, ros::Duration(0.01));

  // Correct time_from_start in order to enforce maximum joint velocity.
    if (goal->max_velocity > 0)
    {
      // Compute the largest required rotation among all the joints
	double	rot_max = 0;
	for (size_t i = 0; i < _jnt_pos.rows(); ++i)
	{
	    const auto	rot = std::abs(_jnt_pos(i) - target_pos(i));
	    if (rot > rot_max)
		rot_max = rot;
	}

    	ros::Duration	required_duration(rot_max / goal->max_velocity);
    	if (required_duration > point.time_from_start)
    	    point.time_from_start = required_duration;
    }

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

    aist_controllers::JointTrajectoryTracker<aist_controllers::PoseHeadAction>
	tracker("pose_head");
    ros::spin();

    return 0;
}
