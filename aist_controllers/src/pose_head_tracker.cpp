/*
 *  \file	pose_head_tracker.cpp
 *  \brief	ROS tracker of aist_controllers::PoseHeadAction type
 */
#include <aist_controllers/joint_trajectory_tracker.h>
#include <aist_controllers/PoseHeadAction.h>

namespace aist_controllers
{
/************************************************************************
*  class JointTrajectoryTracker<PoseHeadAction>::Tracker		*
************************************************************************/
template <> bool
JointTrajectoryTracker<aist_controllers::PoseHeadAction>
    ::Tracker::update(const goal_cp& goal)
{
    constexpr int	MAX_ITERATIONS = 15;

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
    std::cerr << "--- target ---\n" << target << std::endl;
    
    KDL::JntArray	jnt_pos(_jnt_pos);
    _pos_iksolver->CartToJnt(jnt_pos, target, _jnt_pos);

  // Compute the largest required rotation among all the joints
    auto&	point   = _trajectory.points[0];
    double	rot_max = 0;
    for (size_t i = 0; i < point.positions.size(); ++i)
    {
	const auto	rot = std::abs(_jnt_pos(i) - point.positions[i]);
	if (rot > rot_max)
	    rot_max = rot;

	point.positions[i] = _jnt_pos(i);
    }

    point.time_from_start = std::max(goal->min_duration, ros::Duration(0.01));
    if (goal->max_velocity > 0)
    {
	ros::Duration	required_duration(rot_max / goal->max_velocity);
	if (required_duration > point.time_from_start)
	    point.time_from_start = required_duration;
    }

  //return success;
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
