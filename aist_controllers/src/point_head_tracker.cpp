/*
 *  \file	point_head_tracker.cpp
 *  \brief	ROS tracking controller of control_msgs::PointHeadAction type
 */
#include <aist_controllers/joint_trajectory_tracker.h>
#include <control_msgs/PointHeadAction.h>

namespace aist_controllers
{
/************************************************************************
*  class JointTrajectoryTracker<control_msgs::PointHeadAction>::Tracker	*
************************************************************************/
template <> bool
JointTrajectoryTracker<control_msgs::PointHeadAction>
    ::Tracker::update(const goal_cp& goal)
{
    constexpr int	MAX_ITERATIONS = 15;

  // Extract pointing_axis
    KDL::Vector		pointing_axis(goal->pointing_axis.x,
				      goal->pointing_axis.y,
				      goal->pointing_axis.z);
    if (pointing_axis.Norm() < KDL::epsilon)
	throw std::runtime_error("pointing_axis must not be zero");
    pointing_axis.Normalize();

  // Convert target point to base_link.
    auto	original_point = goal->target;
    original_point.header.stamp = ros::Time::now();
    _listener.waitForTransform(_base_link,
			       original_point.header.frame_id,
			       original_point.header.stamp,
			       ros::Duration(1.0));
    geometry_msgs::PointStamped	transformed_point;
    _listener.transformPoint(_base_link, original_point, transformed_point);
    KDL::Vector		target;
    tf::pointMsgToKDL(transformed_point.point, target);

  // Get current joint positions.
    auto&		point = _trajectory.points[0];
    KDL::JntArray	current_pos(njoints());
    jointsToKDL(point.positions, current_pos);

  // Initialize target positions with current positions.
    KDL::JntArray	target_pos(current_pos);

  // Iteratively compute target positions.
    KDL::Jacobian	jacobian(njoints());
    double		err_p   = 2*M_PI;  // angular error in preveous step
    bool		success = false;
    for (int n = 0; n < MAX_ITERATIONS; ++n)
    {
      // Get transform from _pointing_frame to base_link for current _jnt_pos
	KDL::Frame	Tbe;
	_pos_fksolver->JntToCart(target_pos, Tbe);

      // Compute vector from the origin of _pointing_frame to the target
	auto		view_vector = target - Tbe.p;
	view_vector.Normalize();

      // Angular error and its direction between pointing_axis and view_vector.
	auto		axis = (Tbe * pointing_axis) * view_vector;
	const auto	err  = axis.Normalize();

	ROS_DEBUG_STREAM("Step[" << n << "]: joint_position = " << target_pos
			 << ", anglular error = " << err*180.0/M_PI
			 << "(deg)");

	if (std::abs(err - err_p) < 0.001)
	{
	    err_p = err;
	    break;
	}
	err_p = err;

      // We apply a "wrench" proportional to the desired correction
	const KDL::Frame	correction(KDL::Rotation::Rot2(axis, err));
	const auto		twist = diff(correction, KDL::Frame());

      // Compute jacobian for current target positions.
	_jac_solver->JntToJac(target_pos, jacobian);

      // Converts the "wrench" into "joint corrections"
      // with a jacbobian-transpose
	for (size_t i = 0; i < target_pos.rows(); ++i)
	    for (size_t j = 0; j < 6; ++j)
		target_pos(i) -= jacobian(j, i) * twist(j);
	clamp(target_pos);
    }

    ROS_DEBUG_STREAM("Expected error: " << err_p*180.0/M_PI << "(deg)");

    _feedback.pointing_angle_error = err_p;

  //the goal will end when the angular error of the pointing axis
  //is lower than _goal_error. This variable is assigned with the maximum
  //between the ros param _goal_error and the estimated error
  //from the iterative solver last iteration, i.e. err_p
    ROS_DEBUG_STREAM("the goal will terminate when error is: "
		     << err_p*180.0/M_PI << " degrees => "
		     << err_p << " radians");

  // Set desired positions of trajectory command.
    jointsFromKDL(target_pos, point.positions);

  // Correct time_from_start in order to enforce maximum joint velocity.
    if (goal->max_velocity > 0)
    {
      // Compute the largest required rotation among all the joints
	double	rot_max = 0;
	for (size_t i = 0; i < target_pos.rows(); ++i)
	{
	    const auto	rot = std::abs(current_pos(i) - target_pos(i));
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
    ros::init(argc, argv, "point_head_tracker");

    aist_controllers::JointTrajectoryTracker<control_msgs::PointHeadAction>
	tracker("point_head");
    ros::spin();

    return 0;
}
