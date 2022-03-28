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
    const vector3_t	pointing_axis(goal->pointing_axis.x,
				      goal->pointing_axis.y,
				      goal->pointing_axis.z);
    if (pointing_axis.isZero())
	throw std::runtime_error("pointing_axis must not be zero");

  // Convert target point to base_link.
    auto	original_point = goal->target;
    original_point.header.stamp = ros::Time::now();
    _listener.waitForTransform(_base_link,
			       original_point.header.frame_id,
			       original_point.header.stamp,
			       ros::Duration(1.0));
    geometry_msgs::PointStamped	transformed_point;
    _listener.transformPoint(_base_link, original_point, transformed_point);
    point_t	target;
    tf::pointMsgToTF(transformed_point.point, target);

  // Iteratively compute trajectory.
    double	err_p   = 2*M_PI;	// angular error in preveous step
    bool	success = false;

    for (int n = 0; n < MAX_ITERATIONS; ++n)
    {
      // Get transform from effector_link to base_link for current _jnt_pos
	const auto	Tbe = get_chain_transform();

      // Vector from the origin of effector_frame to the target w.r.t. itself
	const auto	view_vector = Tbe.getBasis().inverse()
				    * (target - Tbe.getOrigin()).normalized();

      // Angular error and its direction between pointing_axis and view_vector.
	const auto	axis = Tbe.getBasis()
			     * pointing_axis.cross(view_vector);
	const auto	err  = view_vector.angle(pointing_axis);

	ROS_DEBUG_STREAM("Step[" << n << "]: jnt_pos = (" << _jnt_pos
			 << "), anglular error = " << err*180.0/M_PI
			 << "(deg)");

      // We apply a "wrench" proportional to the desired correction
	KDL::Frame	correction_kdl;
	tf::transformTFToKDL(tf::Transform(tf::Quaternion(axis, err),
					   vector3_t(0, 0, 0)),
			     correction_kdl);
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
    ros::init(argc, argv, "point_head_tracker");

    aist_controllers::JointTrajectoryTracker<control_msgs::PointHeadAction>
	tracker("point_head");
    ros::spin();

    return 0;
}
