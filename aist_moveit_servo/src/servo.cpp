/******************************************************************************
 *      Title     : servo.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 *      Modified  : 10/3/2023
 *      Modifier  : Toshio Ueshiba
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * Copyright (c) 2023, National Institute of Advanced Industrial Science
 *		       and Technology(AIST)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include <aist_moveit_servo/servo.h>
#include <aist_moveit_servo/make_shared_from_pool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static constexpr double	 ROBOT_STATE_WAIT_TIME	 = 10.0;  // seconds
static constexpr size_t	 ROS_LOG_THROTTLE_PERIOD = 30;	  // Seconds to throttle logs inside loops

namespace aist_moveit_servo
{
static void
my_update_cb(const sensor_msgs::JointStateConstPtr& joint_state)
{
    ROS_INFO_STREAM("my_update_cb: [" << joint_state->header.stamp.sec
		    << '.' << std::setw(9) << std::setfill('0')
		    << joint_state->header.stamp.nsec
		    << "] " << joint_state->position[0]);
}

/************************************************************************
*  global functions							*
************************************************************************/
planning_scene_monitor::PlanningSceneMonitorPtr
createPlanningSceneMonitor(const std::string& robot_description,
			   const std::string& move_group_name,
			   const std::string& joint_states_topic,
			   double update_period,
			   const std::string& logname)
{
    using	namespace planning_scene_monitor;

    const auto	monitor = std::make_shared<PlanningSceneMonitor>(
				robot_description);
    if (!monitor->getPlanningScene())
    {
	ROS_ERROR_STREAM_NAMED(logname, "Failed to get PlanningSceneMonitor");
	exit(EXIT_FAILURE);
    }

    monitor->startSceneMonitor();
    monitor->startWorldGeometryMonitor(
	PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
	PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
	false /* skip octomap monitor */);
    monitor->startStateMonitor(joint_states_topic);
  //monitor->getStateMonitor()->addUpdateCallback(my_update_cb); // For debug
    monitor->getStateMonitor()->enableCopyDynamics(true);  // Copy velocity also

    if (!monitor->getStateMonitor()
		->waitForCompleteState(move_group_name,
				       ROBOT_STATE_WAIT_TIME))
    {
	ROS_FATAL_NAMED(logname, "Timeout waiting for current state");
	exit(EXIT_FAILURE);
    }

    ROS_INFO_STREAM_NAMED(
	logname,
	"PlanningSceneMonitor started: RobotState is updated from topic["
	<< joint_states_topic << ']');

    return monitor;
}

/************************************************************************
*  class Servo								*
************************************************************************/
/*
 *  public member functions
 */
// Constructor for the class that handles servoing calculations
Servo::Servo(ros::NodeHandle& nh, const std::string& logname)
    :nh_(nh),
     internal_nh_(nh_, "internal"),
     logname_(logname),
     parameters_(nh_, logname_),
     planning_scene_monitor_(createPlanningSceneMonitor(
				 nh_.param<std::string>("robot_description",
							"robot_description"),
				 parameters_.move_group_name,
				 parameters_.joint_topic,
				 parameters_.publish_period, logname_)),
     collision_checker_(nh_, parameters_, planning_scene_monitor_),

     collision_velocity_scale_sub_(internal_nh_.subscribe(
				       "collision_velocity_scale",
				       ROS_QUEUE_SIZE,
				       &Servo::collisionVelocityScaleCB, this)),
     servo_status_pub_(nh_.advertise<std_msgs::Int8>(parameters_.status_topic,
						     ROS_QUEUE_SIZE)),
     worst_case_stop_time_pub_(internal_nh_.advertise<flt64_t>(
				   "worst_case_stop_time", ROS_QUEUE_SIZE)),
     outgoing_cmd_pub_(
	 parameters_.command_out_type == "trajectory_msgs/JointTrajectory" ?
	 nh_.advertise<trajectory_t>(parameters_.command_out_topic,
				     ROS_QUEUE_SIZE) :
	 nh_.advertise<multi_array_t>(parameters_.command_out_topic,
				      ROS_QUEUE_SIZE)),
     outgoing_cmd_debug_pub_(
	 nh_.advertise<trajectory_t>(parameters_.command_out_topic + "_debug",
				     ROS_QUEUE_SIZE)),
     actual_joint_state_debug_pub_(
	 nh_.advertise<joint_state_t>("actual_joint_states", ROS_QUEUE_SIZE)),
     durations_pub_(nh_.advertise<DurationArray>("durations", 1)),
     drift_dimensions_srv_(
	 nh_.advertiseService(ros::names::append(nh_.getNamespace(),
						 "change_drift_dimensions"),
			      &Servo::changeDriftDimensionsCB, this)),
     control_dimensions_srv_(
	 nh_.advertiseService(ros::names::append(nh_.getNamespace(),
						 "change_control_dimensions"),
			      &Servo::changeControlDimensionsCB, this)),
     reset_servo_status_srv_(
	 nh_.advertiseService(ros::names::append(nh_.getNamespace(),
						 "reset_servo_status"),
			      &Servo::resetServoStatusCB, this)),
     durations_(),
     ddr_(nh_),

     robot_state_(planning_scene_monitor_->getStateMonitor()
					 ->getCurrentState()),
     actual_positions_(),
     actual_velocities_(),
     ff_positions_(),

     invalid_command_count_(0),

     drift_dimensions_({false, false, false, false, false, false}),
     control_dimensions_({true, true, true, true, true, true}),

     position_filters_(),

     joint_trajectory_(),
     joint_indices_(),

     servo_status_(StatusCode::NO_WARNING),
     collision_velocity_scale_(nullptr),
     input_mtx_()
{
  // Setup joint_trajectory command to be published.
    joint_trajectory_.header.frame_id = parameters_.planning_frame;
    joint_trajectory_.header.stamp    = ros::Time(0);
    joint_trajectory_.joint_names = jointGroup()->getActiveJointModelNames();

  // Setup a map from joint names to there indices for buffers of actual state.
    for (size_t i = 0; i < numJoints(); ++i)
	joint_indices_[joint_trajectory_.joint_names[i]] = i;

  // Low-pass filters for the joint positions
    for (size_t i = 0; i < numJoints(); ++i)
    {
	position_filters_.emplace_back(
	    parameters_.low_pass_filter_half_order,
	    parameters_.low_pass_filter_cutoff_frequency *
	    parameters_.publish_period);
    }

  // Initialize position buffer so that low-pass filters can be reset anytime.
    updateJoints();

  // Setup dynamic reconfigure server
    ddr_.registerVariable<int>("lowpass_filter_half_order",
			       parameters_.low_pass_filter_half_order,
			       boost::bind(
				   &Servo::initializeLowPassFilters,
				   this, _1,
				   parameters_.low_pass_filter_cutoff_frequency),
			       "Half order of low pass filter", 1, 5);
    ddr_.registerVariable<double>("lowpass_filter_cutoff_frequency",
				  parameters_.low_pass_filter_cutoff_frequency,
				  boost::bind(
				      &Servo::initializeLowPassFilters,
				      this,
				      parameters_.low_pass_filter_half_order,
				      _1),
				  "Cutoff frequency of low pass filter",
				  0.5, 100.0);
    ddr_.publishServicesTopicsAndUpdateConfigData();

  // Print robot model information.
    robot_state_->getRobotModel()->printModelInfo(std::cerr);

  // Show names of model frame and variables of the robot model.
    ROS_INFO_STREAM_NAMED(logname_, "model_frame: "
			  << robot_state_->getRobotModel()->getModelFrame());
    for (const auto& name : robot_state_->getRobotModel()->getVariableNames())
	ROS_INFO_STREAM_NAMED(logname_, "  variable_name: " << name);
}

Servo::~Servo()
{
    stop();
}

//! Pause or unpause processing servo commands while keeping the timers alive
void
Servo::start()
{
  // Set current positions and zero velocities to trajectory
  // so that the robot does not move.
    updateJoints();
    setPointsToTrajectory(actual_positions_, vector_t::Zero(numJoints()));

  // Reset output low-pass filters with current positions.
    resetLowPassFilters();

    if (parameters_.check_collisions)
    {
	collision_checker_.start();	// Check collisions in this timer
	collision_checker_.setPaused(false);
    }
}

void
Servo::stop()
{
    if (parameters_.check_collisions)
	collision_checker_.setPaused(true);
}

void
Servo::updateRobot()
{
    publishServoStatus();
    updateJoints();			// Read robot status.
    publishWorstCaseStopTime();
}

bool
Servo::publishTrajectory(const twist_t& twist_cmd, const pose_t& ff_pose)
{
  // Transform given pose to model reference frame.
    auto robot_state = *robot_state_;
    auto Trt = tf2::eigenToTransform(robot_state.getGlobalLinkTransform(
					 ff_pose.header.frame_id));
    Trt.header.stamp	= ff_pose.header.stamp;
    Trt.header.frame_id = robot_state.getRobotModel()->getModelFrame();
    Trt.child_frame_id	= ff_pose.header.frame_id;
    pose_t	ff_pose_in_reference;
    tf2::doTransform(ff_pose, ff_pose_in_reference, Trt);

  // Solve IK for robot_state.
    if (!robot_state.setFromIK(jointGroup(), ff_pose_in_reference.pose,
			       parameters_.ee_frame_name))
    {
	ROS_WARN_STREAM_THROTTLE_NAMED(
	    ROS_LOG_THROTTLE_PERIOD, logname_,
	    "Failed to solve IK from incoming feedforward pose.");

	return publishTrajectory(twist_cmd, nullptr);
    }

    ff_positions_.resize(numJoints());
    robot_state.copyJointGroupPositions(jointGroup(), ff_positions_.data());

    return publishTrajectory(twist_cmd, ff_positions_);
}

bool
Servo::publishTrajectory(const twist_t& twist_cmd, const twist_t& twist_ff)
{
  // Transform given forwarding twist to ee_frame.
    auto		robot_state = *robot_state_;
    const auto		Tef = getFrameTransform(parameters_.ee_frame_name,
						twist_ff.header.frame_id);
    const matrix33_t	R   = Tef.matrix().block<3, 3>(0, 0);
    const vector3_t	t   = Tef.matrix().block<3, 1>(0, 3);
    const vector3_t	ang = R * vector3_t(twist_ff.twist.angular.x,
					    twist_ff.twist.angular.y,
					    twist_ff.twist.angular.z);
    const vector3_t	lin = R * vector3_t(twist_ff.twist.linear.x,
					    twist_ff.twist.linear.y,
					    twist_ff.twist.linear.z)
			    + t.cross(ang);
    vector_t		twist_in_ee_frame(6);
    twist_in_ee_frame.head(3) = lin;
    twist_in_ee_frame.tail(3) = ang;

  // Solve IK for robot_state.
    if (!robot_state.setFromDiffIK(jointGroup(), twist_in_ee_frame,
				   parameters_.ee_frame_name,
				   parameters_.publish_period))
    {
	ROS_WARN_STREAM_THROTTLE_NAMED(
	    ROS_LOG_THROTTLE_PERIOD, logname_,
	    "Failed to solve IK from incoming feedforward twist.");

	return publishTrajectory(twist_cmd, nullptr);
    }

    ff_positions_.resize(numJoints());
    robot_state.copyJointGroupPositions(jointGroup(), ff_positions_.data());

    return publishTrajectory(twist_cmd, ff_positions_);
}

/*
 *  private member functions
 */
bool
Servo::isValid(const twist_t& twist_cmd) const
{
    if (std::isnan(twist_cmd.twist.linear.x)  ||
	std::isnan(twist_cmd.twist.linear.y)  ||
	std::isnan(twist_cmd.twist.linear.z)  ||
	std::isnan(twist_cmd.twist.angular.x) ||
	std::isnan(twist_cmd.twist.angular.y) ||
	std::isnan(twist_cmd.twist.angular.z))
    {
	ROS_WARN_STREAM_THROTTLE_NAMED(
	    ROS_LOG_THROTTLE_PERIOD, logname_,
	    "nan in incoming twist command. Skipping this datapoint.");

	return false;
    }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
    if (parameters_.command_in_type == "unitless" &&
	(std::abs(twist_cmd.twist.linear.x)  > 1 ||
	 std::abs(twist_cmd.twist.linear.y)  > 1 ||
	 std::abs(twist_cmd.twist.linear.z)  > 1 ||
	 std::abs(twist_cmd.twist.angular.x) > 1 ||
	 std::abs(twist_cmd.twist.angular.y) > 1 ||
	 std::abs(twist_cmd.twist.angular.z) > 1))
    {
	ROS_WARN_STREAM_THROTTLE_NAMED(
	    ROS_LOG_THROTTLE_PERIOD, logname_,
	    "Component of incoming command is >1. Skipping this datapoint.");

	return false;
    }

    return (twist_cmd.twist.linear.x  != 0.0 ||
	    twist_cmd.twist.linear.y  != 0.0 ||
	    twist_cmd.twist.linear.z  != 0.0 ||
	    twist_cmd.twist.angular.x != 0.0 ||
	    twist_cmd.twist.angular.y != 0.0 ||
	    twist_cmd.twist.angular.z != 0.0);
}

bool
Servo::isValid(const joint_jog_t& joint_cmd) const
{
    for (auto velocity : joint_cmd.velocities)
	if (std::isnan(velocity))
	{
	    ROS_WARN_STREAM_THROTTLE_NAMED(
		ROS_LOG_THROTTLE_PERIOD, logname_,
		"nan in incoming command. Skipping this datapoint.");
	    return false;
	}

    for (auto velocity : joint_cmd.velocities)
	if (velocity != 0.0)
	    return true;

    return false;
}

//! Do calculations for a single iteration and publish one outgoing command
template <class CMD> bool
Servo::publishTrajectory(const CMD& cmd, const vector_t& positions)
{
    if (isValid(cmd))
    {
	setTrajectory(cmd, positions);

	invalid_command_count_ = 0;
    }
    else
    {
	setPointsToTrajectory(actual_positions_,
			      vector_t::Zero(numJoints()), true);
	resetLowPassFilters();

      // Skip the servoing publication if all inputs have been zero
      // for several cycles in a row.
      // num_outgoing_halt_msgs_to_publish == 0 signifies that we should keep
      // republishing forever.
	if (invalid_command_count_ < std::numeric_limits<int>::max())
	    ++invalid_command_count_;	// Avoid overflow

	if ((parameters_.num_outgoing_halt_msgs_to_publish != 0) &&
	    (invalid_command_count_ >
	     parameters_.num_outgoing_halt_msgs_to_publish))
	{
	    ROS_DEBUG_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, logname_,
					    "Invalid command. Do nothing.");
	    return false;
	}
    }

  // Put the outgoing msg in the right format
  // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
    if (parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    {
      // When a joint_trajectory_controller receives a new command,
      // a stamp of 0 indicates "begin immediately"
      // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
	joint_trajectory_.header.stamp = ros::Time(0);
	outgoing_cmd_pub_.publish(joint_trajectory_);

	joint_trajectory_.header.stamp = ros::Time::now();
	outgoing_cmd_debug_pub_.publish(joint_trajectory_);
    }
    else if (parameters_.command_out_type == "std_msgs/Float64MultiArray")
    {
	auto	joints = moveit::util::make_shared_from_pool<multi_array_t>();
	if (parameters_.publish_joint_positions &&
	    !joint_trajectory_.points.empty())
	    joints->data = joint_trajectory_.points[0].positions;
	else if (parameters_.publish_joint_velocities &&
		 !joint_trajectory_.points.empty())
	    joints->data = joint_trajectory_.points[0].velocities;
	outgoing_cmd_pub_.publish(joints);
    }

  // [For debug] Publish durations between events.
    const auto	now = ros::Time::now();
    durations_.cmd_out = (now - durations_.header.stamp).toSec();
    auto	durations_tmp = durations_;
    durations_tmp.header.stamp = now;
    durations_pub_.publish(durations_tmp);

  // [For debug] Publish joint states subscribed
  //		 by planning_scene_monitor::CurrentStateMonitor.
    auto joint_state = moveit::util::make_shared_from_pool<joint_state_t>();
    joint_state->header.frame_id = joint_trajectory_.header.frame_id;
    joint_state->header.stamp    = stamp_;
    joint_state->name = joint_trajectory_.joint_names;
    joint_state->position.resize(numJoints());
    joint_state->velocity.resize(numJoints());
    joint_state->effort.resize(numJoints());
    for (size_t i = 0; i < numJoints(); ++i)
    {
    	joint_state->position[i] = actual_positions_(i);
    	joint_state->velocity[i] = actual_velocities_(i);
	joint_state->effort[i]   = 0;
    }
    actual_joint_state_debug_pub_.publish(joint_state);

    return true;
}

//! Parse the incoming joint msg for the joints of our MoveGroup
void
Servo::updateJoints()
{

    std::tie(robot_state_, stamp_)
	= planning_scene_monitor_->getStateMonitor()->getCurrentStateAndTime();

  // Keep original joint positions and velocities.
    actual_positions_.resize(numJoints());
    robot_state_->copyJointGroupPositions(jointGroup(),
					  actual_positions_.data());
    actual_velocities_.resize(numJoints());
    robot_state_->copyJointGroupVelocities(jointGroup(),
					   actual_velocities_.data());

    ROS_DEBUG_STREAM_NAMED(logname(), "joint updated@["
			   << stamp_.sec
			   << '.' << std::setw(9) << std::setfill('0')
			   << stamp_.nsec
			   << "] " << actual_positions_(0));
}

//! Do servoing calculations for Cartesian twist commands
void
Servo::setTrajectory(const twist_t& twist_cmd, const vector_t& positions)
{
    auto	cmd = twist_cmd;

  // Set uncontrolled dimensions to 0 in command frame
    {
      // Guard control_dimensions_
	const std::lock_guard<std::mutex>	lock(input_mtx_);

	if (!control_dimensions_[0])
	    cmd.twist.linear.x = 0;
	if (!control_dimensions_[1])
	    cmd.twist.linear.y = 0;
	if (!control_dimensions_[2])
	    cmd.twist.linear.z = 0;
	if (!control_dimensions_[3])
	    cmd.twist.angular.x = 0;
	if (!control_dimensions_[4])
	    cmd.twist.angular.y = 0;
	if (!control_dimensions_[5])
	    cmd.twist.angular.z = 0;
    }

  // Transform the command to the MoveGroup planning frame.
    if (cmd.header.frame_id != parameters_.planning_frame)
    {
	if (cmd.header.frame_id.empty())
	    cmd.header.frame_id = parameters_.robot_link_command_frame;

	const auto	Tpc = getFrameTransform(cmd.header.frame_id);
	Eigen::Vector3d	linear(cmd.twist.linear.x,
			       cmd.twist.linear.y,
			       cmd.twist.linear.z);
	Eigen::Vector3d angular(cmd.twist.angular.x,
				cmd.twist.angular.y,
				cmd.twist.angular.z);

	linear  = Tpc.linear() * linear;
	angular = Tpc.linear() * angular;

      // Put these components back into a TwistStamped
	cmd.header.frame_id = parameters_.planning_frame;
	cmd.twist.linear.x  = linear(0);
	cmd.twist.linear.y  = linear(1);
	cmd.twist.linear.z  = linear(2);
	cmd.twist.angular.x = angular(0);
	cmd.twist.angular.y = angular(1);
	cmd.twist.angular.z = angular(2);
    }

    auto	jacobian = robot_state_->getJacobian(jointGroup());
    auto	delta_x  = scaleCommand(cmd);

  // May allow some dimensions to drift, based on drift_dimensions
  // i.e. take advantage of task redundancy.
  // Remove the Jacobian rows corresponding to True
  // in the vector drift_dimensions
  // Work backwards through the 6-vector so indices don't get out of order
    {
      // Guard drift_dimensions_
	const std::lock_guard<std::mutex>	lock(input_mtx_);

	for (auto dimension = jacobian.rows() - 1; dimension >= 0; --dimension)
	    if (drift_dimensions_[dimension] && jacobian.rows() > 1)
		removeDimension(jacobian, delta_x, dimension);
    }

    const auto	svd = Eigen::JacobiSVD<matrix_t>(jacobian,
						 Eigen::ComputeThinU |
						 Eigen::ComputeThinV);
    const auto	matrix_s	= svd.singularValues().asDiagonal();
    const auto	pseudo_inverse	= svd.matrixV() * matrix_s.inverse()
				* svd.matrixU().transpose();
    vector_t	delta_theta	= pseudo_inverse * delta_x;

    applyVelocityScaling(delta_theta,
			 velocityScalingFactorForSingularity(delta_x, svd,
							     pseudo_inverse));
    convertDeltasToTrajectory(positions, delta_theta);
}

//! Do servoing calculations for direct commands to a joint
void
Servo::setTrajectory(const joint_jog_t& joint_cmd, const vector_t& positions)
{
    auto	delta_theta = scaleCommand(joint_cmd);

    applyVelocityScaling(delta_theta, 1.0);
    convertDeltasToTrajectory(positions, delta_theta);
}

//! Scale them to physical units.
/*!
  Do it if incoming velocity commands are from a unitless joystick.
  Also, multiply by timestep to calculate a position change.
*/
Servo::vector_t
Servo::scaleCommand(const twist_t& twist_cmd) const
{
  // Apply user-defined scaling if inputs are unitless [-1:1]
    const auto	linear_scale	 = (parameters_.command_in_type == "unitless" ?
				    parameters_.linear_scale : 1.0)
				 * parameters_.publish_period;
    const auto	rotational_scale = (parameters_.command_in_type == "unitless" ?
				    parameters_.rotational_scale : 1.0)
				 * parameters_.publish_period;
    vector_t	delta_x(6);
    delta_x[0] = linear_scale	  * twist_cmd.twist.linear.x;
    delta_x[1] = linear_scale	  * twist_cmd.twist.linear.y;
    delta_x[2] = linear_scale	  * twist_cmd.twist.linear.z;
    delta_x[3] = rotational_scale * twist_cmd.twist.angular.x;
    delta_x[4] = rotational_scale * twist_cmd.twist.angular.y;
    delta_x[5] = rotational_scale * twist_cmd.twist.angular.z;

    return delta_x;
}

//! Scale them to physical units.
/*!
  Do it if incoming velocity commands are from a unitless joystick,
  Also, multiply by timestep to calculate a position change.
*/
Servo::vector_t
Servo::scaleCommand(const joint_jog_t& joint_cmd) const
{
    vector_t	delta_theta(numJoints());
    delta_theta.setZero();

    const auto joint_scale = (parameters_.command_in_type == "unitless" ?
			      parameters_.joint_scale : 1.0)
			   * parameters_.publish_period;

    for (size_t i = 0; i < joint_cmd.joint_names.size(); ++i)
	try
	{
	    const auto	j = joint_indices_.at(joint_cmd.joint_names[i]);

	    delta_theta[j] = joint_scale * joint_cmd.velocities[i];
	}
	catch (const std::out_of_range& e)
	{
	    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, logname_,
					   "Ignoring joint "
					   << joint_cmd.joint_names[i]);
	}

    return delta_theta;
}

//! Calculate a velocity scaling factor
/*!
  Possibly due to proximity of singularity and direction of motion
*/
double
Servo::velocityScalingFactorForSingularity(
		const vector_t& commanded_velocity,
		const Eigen::JacobiSVD<matrix_t>& svd,
		const matrix_t& pseudo_inverse)
{
    double	velocity_scale = 1;
    size_t	num_dimensions = commanded_velocity.size();

  // Find the direction away from nearest singularity.
  // The last column of U from the SVD of the Jacobian points directly
  // toward or away from the singularity.
  // The sign can flip at any time, so we have to do some extra checking.
  // Look ahead to see if the Jacobian's condition will decrease.
    vector_t	vector_toward_singularity = svd.matrixU().col(
						num_dimensions - 1);

    const auto	ini_condition = svd.singularValues()(0)
			      / svd.singularValues()(
				  svd.singularValues().size() - 1);

  // This singular vector tends to flip direction unpredictably. See R. Bro,
  // "Resolving the Sign Ambiguity in the Singular Value Decomposition".
  // Look ahead to see if the Jacobian's condition will decrease in this
  // direction. Start with a scaled version of the singular vector
    vector_t	delta_x(num_dimensions);
    double	scale = 100;
    delta_x = vector_toward_singularity / scale;

  // Calculate a small change in joints
    auto	new_robot_state = *robot_state_;
    vector_t	new_theta;
    new_robot_state.copyJointGroupPositions(jointGroup(), new_theta);
    new_theta += pseudo_inverse * delta_x;
    new_robot_state.setJointGroupPositions(jointGroup(), new_theta);
    matrix_t	new_jacobian = new_robot_state.getJacobian(jointGroup());

    Eigen::JacobiSVD<matrix_t>	new_svd(new_jacobian);
    const auto	new_condition = new_svd.singularValues()(0)
			      / new_svd.singularValues()(
				  new_svd.singularValues().size() - 1);

  // If new_condition < ini_condition, the singular vector does point
  // towards a singularity. Otherwise, flip its direction.
    if (ini_condition >= new_condition)
	vector_toward_singularity *= -1;

  // If this dot product is positive, we're moving toward singularity
  // ==> decelerate
    const auto	dot = vector_toward_singularity.dot(commanded_velocity);
    if (dot > 0)
    {
      // Ramp velocity down linearly when the Jacobian condition is between
      // lower_singularity_threshold and hard_stop_singularity_threshold,
      // and we're moving towards the singularity
	if ((ini_condition > parameters_.lower_singularity_threshold) &&
	    (ini_condition < parameters_.hard_stop_singularity_threshold))
	{
	    velocity_scale = 1 - (ini_condition -
				  parameters_.lower_singularity_threshold)
			       / (parameters_.hard_stop_singularity_threshold -
				  parameters_.lower_singularity_threshold);
	    servo_status_ = StatusCode::DECELERATE_FOR_SINGULARITY;
	    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, logname_,
					   SERVO_STATUS_CODE_MAP.at(servo_status_));
	}

      // Very close to singularity, so halt.
	else if (ini_condition > parameters_.hard_stop_singularity_threshold)
	{
	    velocity_scale = 0;
	    servo_status_ = StatusCode::HALT_FOR_SINGULARITY;
	    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, logname_,
					   SERVO_STATUS_CODE_MAP.at(servo_status_));
	}
    }

    return velocity_scale;
}

//! Apply velocity scaling for proximity of collisions and singularities
/*!
 Slow motion down if close to singularity or collision.

 \param delta_theta	  motion command, used in calculating new_joint_tray
 \param singularity_scale tells how close we are to a singularity
*/
void
Servo::applyVelocityScaling(vector_t& delta_theta, double singularity_scale)
{
  // Convert to joint angle velocities for checking and applying joint
  // specific velocity limits.
    double	bounding_scale = 1.0;

    for (const auto joint : jointGroup()->getActiveJointModels())
    {
	const auto&	bounds	 = joint->getVariableBounds(joint->getName());
	const auto	i	 = joint_indices_.at(joint->getName());
	const auto	velocity = delta_theta[i] / parameters_.publish_period;

	if (bounds.velocity_bounded_ && velocity != 0.0)
	{
	  // Clamp each joint velocity to a joint specific
	  // [min_velocity, max_velocity] range.
	    // const auto	bounded_velocity = std::clamp(velocity,
	    // 					      8*bounds.min_velocity_,
	    // 					      8*bounds.max_velocity_);
	    const auto	bounded_velocity = std::clamp(velocity,
	    					      bounds.min_velocity_,
	    					      bounds.max_velocity_);
	    bounding_scale = std::min(bounding_scale,
				      bounded_velocity / velocity);
	}
    }

  // Convert back to joint angle increments.
    const auto	collision_velocity_scale = collisionVelocityScale();

    delta_theta *= (bounding_scale *
		    collision_velocity_scale * singularity_scale);

    if (bounding_scale < 1)
	ROS_WARN_STREAM_THROTTLE_NAMED(3, logname_,
				       "Velocity bounded by scale value="
				       << bounding_scale);

    if (collision_velocity_scale <= 0)
    {
	servo_status_ = StatusCode::HALT_FOR_COLLISION;

	ROS_WARN_STREAM_THROTTLE_NAMED(3, logname_, "Halting for collision!");
    }
    else if (collision_velocity_scale < 1)
    {
	servo_status_ = StatusCode::DECELERATE_FOR_COLLISION;

	ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, logname_,
				       SERVO_STATUS_CODE_MAP.at(servo_status_));
    }
}

//! Convert joint deltas to an outgoing JointTrajectory command
void
Servo::convertDeltasToTrajectory(const vector_t& positions,
				 const vector_t& delta_theta)
{
    vector_t	desired_positions = positions + delta_theta;
    applyLowPassFilters(desired_positions);

    if (checkPositionLimits(desired_positions, delta_theta))
	setPointsToTrajectory(desired_positions, delta_theta);
    else
    {
	setPointsToTrajectory(actual_positions_,
			      vector_t::Zero(delta_theta.size()), true);
	servo_status_ = StatusCode::JOINT_BOUND;
    }
}

//! Avoid overshooting joint limits
bool
Servo::checkPositionLimits(const vector_t& positions,
			   const vector_t& delta_theta) const
{
    for (const auto joint : jointGroup()->getActiveJointModels())
    {
	using JointType	= moveit::core::JointModel::JointType;

	const auto&	limits = joint->getVariableBoundsMsg();
	if (limits.empty())
	    continue;

	const auto	margin = (joint->getType() == JointType::PRISMATIC ?
				  parameters_.prismatic_joint_limit_margin :
				  parameters_.revolute_joint_limit_margin);
	if (robot_state_->satisfiesPositionBounds(joint, -margin))
	    continue;

      // Check if pending velocity command is moving in the right direction
	const auto	i	 = joint_indices_.at(joint->getName());
	const auto	position = positions[i];
	const auto	delta	 = delta_theta[i];
	if ((delta < 0 && position < (limits[0].min_position + margin)) ||
	    (delta > 0 && position > (limits[0].max_position - margin)))
	{
	    ROS_WARN_STREAM_THROTTLE_NAMED(
		ROS_LOG_THROTTLE_PERIOD, logname_,
		ros::this_node::getName() << " " << joint->getName()
		<< " close to a position limit. Halting.");
	    return false;
	}
    }

    return true;
}

//! Satisfy Gazebo by stuffing multiple messages into one
/*!
*/
void
Servo::setPointsToTrajectory(const vector_t& positions,
			     const vector_t& delta_theta, bool sudden)
{
    auto&	points = joint_trajectory_.points;
    points.resize(parameters_.use_gazebo ? 30 : 1);

    auto&	front = points.front();
    if (parameters_.publish_joint_positions)
    {
	front.positions.resize(positions.size());
	for (size_t i = 0; i < front.positions.size(); ++i)
	    front.positions[i] = positions[i];
    }
    if (parameters_.publish_joint_velocities)
    {
	front.velocities.resize(delta_theta.size());
	for (size_t i = 0; i < front.velocities.size(); ++i)
	    front.velocities[i] = delta_theta[i] / parameters_.publish_period;
    }
    if (parameters_.publish_joint_accelerations)
    {
      // I do not know of a robot that takes acceleration commands.
      // However, some controllers check that this data is non-empty.
      // Send all zeros, for now.
	front.accelerations.resize(delta_theta.size(), 0.0);
    }

  // When sending out trajectory_msgs/JointTrajectory type messages,
  // the "trajectory" is just a single point.
  // That point cannot have the same timestamp as the start
  // of trajectory execution since that would mean the
  // arm has to reach the first trajectory point the moment execution begins.
  // To prevent errors about points being 0 seconds in the past,
  // the smallest supported timestep is added as time from start
  // to the trajectory point.
    if (sudden)
	front.time_from_start.fromNSec(1);
    else
	front.time_from_start = ros::Duration(parameters_.publish_period);

  // [Required for gazebo simulation]
  // Gazebo simulations have very strict message timestamp requirements.
  // Spam several redundant points into the trajectory. The first few may be
  // skipped if the time stamp is in the past when it reaches the client.
  // Start from 2nd point (i = 1) because we already have the first point.
  // The timestamps are shifted up one period since first point is at
  // 1 * publish_period, not 0.
    for (size_t i = 1; i < points.size(); ++i)
    {
	points[i] = points[i-1];
	points[i].time_from_start += ros::Duration(parameters_.publish_period);
    }
}

//! Set zero velocities to all points in the trajectory
void
Servo::setZeroVelocitiesToTrajectory()
{
    for (auto& point : joint_trajectory_.points)
	point.velocities.assign(point.velocities.size(), 0);
}

//! Remove the Jacobian row and the delta-x element of one Cartesian dimension
/*!
  Take advantage of task redundancy

  \param matrix		The Jacobian matrix.
  \param delta_x	Vector of Cartesian delta commands,
			should be the same size as matrix.rows()
  \param row_to_remove	Dimension that will be allowed to drift,
			e.g. row_to_remove = 2 allows z-translation drift.
*/
void
Servo::removeDimension(matrix_t& jacobian,
		       vector_t& delta_x, uint row_to_remove) const
{
    const auto	nrows = jacobian.rows() - 1;
    const auto	ncols = jacobian.cols();

    if (row_to_remove < nrows)
    {
	jacobian.block(row_to_remove, 0, nrows - row_to_remove, ncols)
	    = jacobian.block(row_to_remove + 1, 0,
			     nrows - row_to_remove, ncols);
	delta_x.segment(row_to_remove, nrows - row_to_remove)
	    = delta_x.segment(row_to_remove + 1, nrows - row_to_remove);
    }

    jacobian.conservativeResize(nrows, ncols);
    delta_x.conservativeResize(nrows);
}

/*
 *  private member functions: low-pass filter stuffs
 */
//! Change order and/or cutoff of filters
void
Servo::initializeLowPassFilters(int half_order, double cutoff_frequency)
{
    {
      // Guard position_filters_
	const std::lock_guard<std::mutex> lock(input_mtx_);

	parameters_.low_pass_filter_half_order	 = half_order;
	parameters_.low_pass_filter_cutoff_frequency = cutoff_frequency;

	for (auto&& position_filter : position_filters_)
	    position_filter.initialize(
		parameters_.low_pass_filter_half_order,
		parameters_.low_pass_filter_cutoff_frequency *
		parameters_.publish_period);
    }

    resetLowPassFilters();
}

//! Smooth position commands with a lowpass filter
void
Servo::applyLowPassFilters(vector_t& positions)
{
  // Guard position_filters_
    const std::lock_guard<std::mutex> lock(input_mtx_);

    for (size_t i = 0; i < position_filters_.size(); ++i)
	positions[i] = position_filters_[i].filter(positions[i]);
}

//! Set the filters to the specified values
void
Servo::resetLowPassFilters()
{
  // Guard position_filters_
    const std::lock_guard<std::mutex> lock(input_mtx_);

    for (size_t i = 0; i < position_filters_.size(); ++i)
	position_filters_[i].reset(actual_positions_[i]);
}

/*
 *  private member functions: callbacks
 */
double
Servo::collisionVelocityScale() const
{
  // Guard collision_velocity_scale_
    const std::lock_guard<std::mutex> lock(input_mtx_);

    return (collision_velocity_scale_ == nullptr ?
	    1.0 : collision_velocity_scale_->data);
}

void
Servo::collisionVelocityScaleCB(const flt64_cp& velocity_scale)
{
  // Guard collision_velocity_scale_
    const std::lock_guard<std::mutex> lock(input_mtx_);

    collision_velocity_scale_ = velocity_scale;
}

//! Allow drift in certain dimensions
/*!
  For example, may allow the wrist to rotate freely.
  This can help avoid singularities.

  \param request	the service request
  \param response	the service response
  \return		true if the adjustment was made
*/
bool
Servo::changeDriftDimensionsCB(
		moveit_msgs::ChangeDriftDimensions::Request&  req,
		moveit_msgs::ChangeDriftDimensions::Response& res)
{
  // Guard drift_dimensions_
    const std::lock_guard<std::mutex> lock(input_mtx_);

    drift_dimensions_[0] = req.drift_x_translation;
    drift_dimensions_[1] = req.drift_y_translation;
    drift_dimensions_[2] = req.drift_z_translation;
    drift_dimensions_[3] = req.drift_x_rotation;
    drift_dimensions_[4] = req.drift_y_rotation;
    drift_dimensions_[5] = req.drift_z_rotation;

    res.success = true;

    return true;
}

//! Service callback for changing servoing dimensions
/*!
  e.g. ignore rotation about X
*/
bool
Servo::changeControlDimensionsCB(
		moveit_msgs::ChangeControlDimensions::Request&  req,
		moveit_msgs::ChangeControlDimensions::Response& res)
{
  // Guard control_dimensions_
    const std::lock_guard<std::mutex> lock(input_mtx_);

    control_dimensions_[0] = req.control_x_translation;
    control_dimensions_[1] = req.control_y_translation;
    control_dimensions_[2] = req.control_z_translation;
    control_dimensions_[3] = req.control_x_rotation;
    control_dimensions_[4] = req.control_y_rotation;
    control_dimensions_[5] = req.control_z_rotation;

    res.success = true;

    return true;
}

/*
 *  Servo status stuffs
 */
void
Servo::publishServoStatus() const
{
    const auto	msg = moveit::util::make_shared_from_pool<std_msgs::Int8>();
    msg->data = static_cast<int8_t>(servo_status_);
    servo_status_pub_.publish(msg);
}

bool
Servo::resetServoStatusCB(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    resetServoStatus();

    return true;
}

/*
 *  Worst case stop time stuffs
 */
void
Servo::publishWorstCaseStopTime() const
{
  // Calculate worst case joint stop time, for collision checking
    double	worst_case_stop_time = 0;
    for (const auto joint : jointGroup()->getActiveJointModels())
    {
	const auto&	bound = joint->getVariableBounds()[0];

      // Some joints do not have acceleration limits
	if (bound.acceleration_bounded_)
	{
	  // Be conservative when calculating overall acceleration
	  // limit from min and max limits
	    const auto	accel_limit = std::min(fabs(bound.min_acceleration_),
					       fabs(bound.max_acceleration_));
	    const auto	i = joint_indices_.at(joint->getName());

	    worst_case_stop_time = std::max(worst_case_stop_time,
					    fabs(actual_velocities_[i]
						 / accel_limit));
	}
	else
	    ROS_WARN_STREAM_THROTTLE_NAMED(
		ROS_LOG_THROTTLE_PERIOD, logname_,
		"An acceleration limit is not defined for this joint; minimum"
		"stop distance should not be used for collision checking");
    }

  // publish message
    auto	msg = moveit::util::make_shared_from_pool<flt64_t>();
    msg->data = worst_case_stop_time;
    worst_case_stop_time_pub_.publish(msg);
}

template bool	Servo::publishTrajectory(const twist_t& twist_cmd,
					 const vector_t& positions)	;
template bool	Servo::publishTrajectory(const joint_jog_t& joint_cmd,
					 const vector_t& positions)	;

}  // namespace aist_moveit_servo
