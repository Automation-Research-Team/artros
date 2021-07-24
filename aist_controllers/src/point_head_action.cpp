/*
 *  $Id
 */
#include <aist_controllers/joint_position_tracker.h>
#include <control_msgs/PointHeadAction.h>

namespace aist_controllers
{
/************************************************************************
*  class point_head::commander						*
************************************************************************/
template <> trajectory_msgs::JointTrajectory
joint_position_tracker<control_msgs::PointHeadAction>::commander
	::compute_trajectory(const goal_cp& goal)
{
  // Convert target point to base_link.
    const auto	now = ros::Time::now();
    _listener.waitForTransform(_base_link, goal->target.header.frame_id,
			       now, ros::Duration(1.0));
    geometry_msgs::PointStamped	original_point = goal->target;
    original_point.header.stamp = now;
    geometry_msgs::PointStamped	transformed_point;
    _listener.transformPoint(_base_link, original_point, transformed_point);
    point_t	target;
    tf::pointMsgToTF(transformed_point.point, target);


    constexpr int	MAX_ITERATIONS = 15;

    int		limit_flips = 0;
    double	correction_angle = 2*M_PI;
    double	correction_delta = 2*M_PI;

    for (int n = 0; n < MAX_ITERATIONS && fabs(correction_delta) > 0.001; ++n)
    {
      // Transform from effector_link to base_link
	const auto	Tbe = get_transform();

      // Vector from the origin of leaf to the target w.r.t. itself
	const auto	view_vector = Tbe.getBasis().inverse()
				    * (target - Tbe.getOrigin())
				      .normalized();

      // Compute angular difference between pointing_axis and view_vector.
	const auto	prev_correction = correction_angle;
	correction_angle = view_vector.angle(goal->pointing_axis);
	correction_delta = correction_angle - prev_correction;

	ROS_DEBUG_STREAM("Step[" << n << "]: jnt_pos = (" << _jnt_pos
			 << "), angle error = " << correction_angle << "(rad)");

	_goal_error = correction_angle; //expected error after this iteration
	
	if (correction_angle < 0.5*_success_angle_threshold)
	{
	    ROS_DEBUG_STREAM("Accepting solution as estimated error is: "
			     << correction_angle*180.0/M_PI
			     << "degrees and stopping condition is half of "
			     << _success_angle_threshold*180.0/M_PI);
	    return;
	}


	const auto	correction_axis = Tbe.getBasis()
					* (pointing_axis.cross(view_vector)
					   .normalized());
	KDL::Frame	correction_kdl;
	tf::transformTFToKDL({tf::Quaternion(correction_axis,
					     0.5*correction_angle).
			      vector3_t(0,0,0)}, correction_kdl);

      // We apply a "wrench" proportional to the desired correction
	KDL::Frame	identity_kdl;
	KDL::Twist	twist = diff(correction_kdl, identity_kdl);
	KDL::Wrench	wrench;
	for (size_t i = 0; i < 6; ++i)
	    wrench(i) = -1.0*twist(i);
	const auto	wrench = compute_wrench(goal);

	_jac_solver->JntToJac(_jnt_pos, _jacobian);

      // Converts the "wrench" into "joint corrections"
      // with a jacbobian-transpose
	for (size_t i = 0; i < _jnt_pos.rows(); ++i)
	{
	    double	jnt_eff = 0;
	    for (size_t j = 0; j < 6; ++j)
		jnt_eff += (_jacobian(j, i) * wrench(j));
	    _jnt_pos(i) = clamp(_jnt_pos(i) + jnt_eff,
				_limits[i].lower, _limits[i].upper);
	}

	if (limit_flips > 1)
	{
	    ROS_ERROR("Goal is out of joint limits, trying to point there anyway... \n");
	    break;
	}
    }
    ROS_DEBUG_STREAM("Iterative solver took "
		     << niter << " steps. Expected error: "
		     << correction_angle << " radians");

  //the goal will end when the angular error of the pointing axis
  //is lower than _goal_error. This variable is assigned with the maximum
  //between the ros param _success_angle_threshold and the estimated error
  //from the iterative solver last iteration, i.e. _goal_error current value
    _goal_error = std::max(_goal_error, _success_angle_threshold);

    ROS_DEBUG_STREAM("the goal will terminate when error is: "
		     << _goal_error*180.0/M_PI << " degrees => "
		     << _goal_error << " radians");

  // Computes the duration of the movement.
    ros::Duration min_duration(0.01);

    if (goal->min_duration > min_duration)
        min_duration = goal->min_duration;

  // Determines if we need to increase the duration of the movement in order to enforce a maximum velocity.
    if (goal->max_velocity > 0)
    {
      // compute the largest required rotation among all the joints
	double	largest_rotation = 0;
	for (size_t i = 0; i < positions.size(); ++i)
	{
	    const auto required_rotation = fabs(positions[i] -
					    traj_state.response.position[i]);
	    if (required_rotation > largest_rotation)
		largest_rotation = required_rotation;
	}

	ros::Duration limit_from_velocity(largest_rotation / gh.getGoal()->max_velocity);
	if (limit_from_velocity > min_duration)
	    min_duration = limit_from_velocity;
    }

  // Computes the command to send to the trajectory controller.
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = traj_state.request.time;

    traj.joint_names.push_back(traj_state.response.name[0]);
    traj.joint_names.push_back(traj_state.response.name[1]);

    traj.points.resize(1);
    traj.points[0].positions = positions;
    traj.points[0].velocities.push_back(0);
    traj.points[0].velocities.push_back(0);
    traj.points[0].time_from_start = ros::Duration(min_duration);

    return traj;
}

}	// namepsace aist_controllers

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "point_head_action");

    ros::NodeHandle	node;
    point_head		ch(node);
    ros::spin();

    return 0;
}
