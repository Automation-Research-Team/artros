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
    pose_t	target;
    tf::poseMsgToTF(transformed_pose.pose, target);

  // Iteratively compute trajectory.
    double	err_p   = 2*M_PI;	// angular error in preveous step
    bool	success = false;

    for (int n = 0; n < MAX_ITERATIONS; ++n)
    {
      // Get transform from effector_link to base_link for current _jnt_pos
	const auto	Tbe = get_chain_transform();

      // Target pose w.r.t. effector_frame
	const auto	error_pose = Tbe.inverse() * target;
      //std::cerr << "error_pose: " << error_pose << std::endl;

      // We apply a "wrench" proportional to the desired correction
	KDL::Frame	correction_kdl;
	tf::transformTFToKDL(error_pose, correction_kdl);
	const auto	twist = diff(correction_kdl, KDL::Frame());

      // Compute jacobian for current _jnt_pos
	_jac_solver->JntToJac(_jnt_pos, _jacobian);

      // Converts the "wrench" into "joint corrections"
      // with a jacbobian-transpose
	for (size_t i = 0; i < _jnt_pos.rows(); ++i)
	{
	    double	jnt_eff = 0;
	    for (size_t j = 0; j < 6; ++j)
		jnt_eff -= (_jacobian(j, i) * twist(j));
	    _jnt_pos(i) = clamp(_jnt_pos(i) + jnt_eff,
				_jnt_pos_min(i), _jnt_pos_max(i));
	}

	if (err < _goal_error || std::abs(err - err_p) < 0.001)
	{
	    err_p = err;
	    success = true;
	    break;
	}

	err_p = err;
    }

    ROS_DEBUG_STREAM("Expected error: " << err_p*180.0/M_PI << "(deg)");

    _feedback.pointing_angle_error = err_p;

  //the goal will end when the angular error of the pointing axis
  //is lower than _goal_error. This variable is assigned with the maximum
  //between the ros param _goal_error and the estimated error
  //from the iterative solver last iteration, i.e. err_p
    err_p = std::max(err_p, _goal_error);

    ROS_DEBUG_STREAM("the goal will terminate when error is: "
		     << err_p*180.0/M_PI << " degrees => "
		     << err_p << " radians");

  // Determines if we need to increase the duration of the movement
  // in order to enforce a maximum velocity.

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
