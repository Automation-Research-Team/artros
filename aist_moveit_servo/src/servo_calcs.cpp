/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
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
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/
/*      Title     : servo_calcs.cpp
 *      Project   : aist_moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson, Toshio Ueshiba
 */
#include <cassert>
#include <std_msgs/Float64MultiArray.h>
#include <aist_moveit_servo/make_shared_from_pool.h>
#include <aist_moveit_servo/servo_calcs.h>

static const std::string LOGNAME = "servo_calcs";
constexpr size_t	 ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops

namespace aist_moveit_servo
{
namespace
{
/************************************************************************
*  static functions							*
************************************************************************/
std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Pose& pose)
{
    return out << pose.position.x    << ' '
	       << pose.position.y    << ' '
	       << pose.position.z    << ';'
	       << pose.orientation.x << ' '
	       << pose.orientation.y << ' '
	       << pose.orientation.z << ' '
	       << pose.orientation.w;
}

std::ostream&
operator <<(std::ostream& out, const Eigen::Isometry3d& isometry)
{
    const Eigen::Quaterniond	q(isometry.rotation());

    return out << isometry.translation()(0) << ' '
	       << isometry.translation()(1) << ' '
	       << isometry.translation()(2) << ';'
	       << q.x() << ' '
	       << q.y() << ' '
	       << q.z() << ' '
	       << q.w();
}

std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Twist& twist)
{
    return out << twist.linear.x  << ' '
	       << twist.linear.y  << ' '
	       << twist.linear.z  << ';'
	       << twist.angular.x << ' '
	       << twist.angular.y << ' '
	       << twist.angular.z;
}

}  // namespace

/************************************************************************
*  class ServoCalcs							*
************************************************************************/
/*
 *  public member functions
 */
// Constructor for the class that handles servoing calculations
ServoCalcs::ServoCalcs(const ros::NodeHandle& nh, ServoParameters& parameters,
                       const planning_scene_monitor_p& planning_scene_monitor)
  :parameters_(parameters),
   planning_scene_monitor_(planning_scene_monitor),

   nh_(nh),
   internal_nh_(nh, "internal"),
   twist_cmd_sub_(
       nh_.subscribe(parameters_.cartesian_command_in_topic, ROS_QUEUE_SIZE,
		     &ServoCalcs::twistCmdCB, this,
		     ros::TransportHints().reliable().tcpNoDelay(true))),
   joint_cmd_sub_(
       nh_.subscribe(parameters_.joint_command_in_topic, ROS_QUEUE_SIZE,
		     &ServoCalcs::jointCmdCB, this,
		     ros::TransportHints().reliable().tcpNoDelay(true))),
   collision_velocity_scale_sub_(
       internal_nh_.subscribe(
	   "collision_velocity_scale", ROS_QUEUE_SIZE,
	   &ServoCalcs::collisionVelocityScaleCB, this,
	   ros::TransportHints().reliable().tcpNoDelay(true))),
   status_pub_(nh_.advertise<std_msgs::Int8>(parameters_.status_topic,
					     ROS_QUEUE_SIZE)),
   worst_case_stop_time_pub_(internal_nh_.advertise<flt64_t>(
				 "worst_case_stop_time", ROS_QUEUE_SIZE)),
   outgoing_cmd_pub_(
       parameters_.command_out_type == "trajectory_msgs/JointTrajectory" ?
       nh_.advertise<trajectory_t>(
	   parameters_.command_out_topic, ROS_QUEUE_SIZE) :
       nh_.advertise<std_msgs::Float64MultiArray>(
	   parameters_.command_out_topic, ROS_QUEUE_SIZE)),
   outgoing_cmd_debug_pub_(
       parameters_.command_out_type == "trajectory_msgs/JointTrajectory" ?
       nh_.advertise<trajectory_t>(
	   parameters_.command_out_topic + "_debug", ROS_QUEUE_SIZE) :
       nh_.advertise<std_msgs::Float64MultiArray>(
	   parameters_.command_out_topic + "_debug", ROS_QUEUE_SIZE)),
   durations_pub_(
       nh_.advertise<aist_moveit_servo::DurationArray>("durations", 1)),
   drift_dimensions_srv_(
       nh_.advertiseService(ros::names::append(nh_.getNamespace(),
					       "change_drift_dimensions"),
			    &ServoCalcs::changeDriftDimensions, this)),
   control_dimensions_srv_(
       nh_.advertiseService(ros::names::append(nh_.getNamespace(),
					       "change_control_dimensions"),
			    &ServoCalcs::changeControlDimensions, this)),
   reset_status_srv_(
       nh_.advertiseService(ros::names::append(nh_.getNamespace(),
					       "reset_servo_status"),
			    &ServoCalcs::resetStatus, this)),
   ddr_(nh_),

   invalid_command_count_(0),
   robot_state_(planning_scene_monitor_->getStateMonitor()->getCurrentState()),
   robot_state_stamp_(ros::Time(0)),
   actual_positions_(),
   actual_velocities_(),

   joint_trajectory_(),
   joint_indices_(),

   position_filters_(),

   thread_(),
   stop_requested_(true),

   status_(StatusCode::NO_WARNING),
   paused_(false),
   collision_velocity_scale_(1.0),

   drift_dimensions_({false, false, false, false, false, false}),
   control_dimensions_({true, true, true, true, true, true}),

   input_mutex_(),
   twist_cmd_(),
   joint_cmd_(),

   input_cv_(),
   new_input_cmd_(false)
{
  // Setup joint_trajectory command to be published.
    joint_trajectory_.header.frame_id = parameters_.planning_frame;
    joint_trajectory_.header.stamp    = ros::Time(0);
    joint_trajectory_.joint_names = joint_group()->getActiveJointModelNames();

  // Setup a map from joint names to there indices for buffers of actual state.
    for (size_t i = 0; i < num_joints(); ++i)
	joint_indices_[joint_trajectory_.joint_names[i]] = i;

  // Low-pass filters for the joint positions
    for (size_t i = 0; i < num_joints(); ++i)
    {
	position_filters_.emplace_back(
	    parameters_.low_pass_filter_half_order,
	    parameters_.low_pass_filter_cutoff_frequency *
	    parameters_.publish_period);
    }

  // Initialize position buffer so that low-pass filters can be reset anytime.
    updateJoints();

  // Initialize buffer for incoming twist command.
    twist_cmd_.header.stamp    = ros::Time(0);
    twist_cmd_.twist.linear.x  = 0.0;
    twist_cmd_.twist.linear.y  = 0.0;
    twist_cmd_.twist.linear.z  = 0.0;
    twist_cmd_.twist.angular.x = 0.0;
    twist_cmd_.twist.angular.y = 0.0;
    twist_cmd_.twist.angular.z = 0.0;

  // Initialize buffer for incoming joint command.
    joint_cmd_.header.stamp = ros::Time(0);
    joint_cmd_.velocities.resize(num_joints(), 0.0);

  // Setup dynamic reconfigure server
    ddr_.registerVariable<int>("lowpass_filter_half_order",
			       parameters_.low_pass_filter_half_order,
			       boost::bind(
				   &ServoCalcs::initializeLowPassFilters,
				   this, _1,
				   parameters_.low_pass_filter_cutoff_frequency),
			       "Half order of low pass filter", 1, 5);
    ddr_.registerVariable<double>("lowpass_filter_cutoff_frequency",
				  parameters_.low_pass_filter_cutoff_frequency,
				  boost::bind(
				      &ServoCalcs::initializeLowPassFilters,
				      this,
				      parameters_.low_pass_filter_half_order,
				      _1),
				  "Cutoff frequency of low pass filter",
				  0.5, 100.0);
    ddr_.publishServicesTopics();
}

ServoCalcs::~ServoCalcs()
{
    stop();
}

//! Start the timer where we do work and publish outputs
void
ServoCalcs::start()
{
  // Stop the thread if we are currently running
    stop();

    updateJoints();
    setPointsToTrajectory(actual_positions_, vector_t::Zero(num_joints()));

    stop_requested_ = false;
    thread_	    = std::thread([this] { mainCalcLoop(); });
    new_input_cmd_  = false;
}

//! Pause or unpause processing servo commands while keeping the timers alive
void
ServoCalcs::setPaused(bool paused)
{
    paused_ = paused;
}

//! Change the controlled link. Often, this is the end effector
/*!
  This must be a link on the robot since MoveIt tracks the transform (not tf)
*/
void
ServoCalcs::changeRobotLinkCommandFrame(const std::string& new_command_frame)
{
    parameters_.robot_link_command_frame = new_command_frame;
}

/*
 *  private member functions
 */
template <class MSG> bool
ServoCalcs::isStale(const MSG& msg) const
{
    return (ros::Time::now() - msg.header.stamp >
	    ros::Duration(parameters_.incoming_command_timeout));
}

bool
ServoCalcs::isValid(const geometry_msgs::TwistStamped& msg) const
{
    if (std::isnan(msg.twist.linear.x)  ||
	std::isnan(msg.twist.linear.y)  ||
	std::isnan(msg.twist.linear.z)  ||
	std::isnan(msg.twist.angular.x) ||
	std::isnan(msg.twist.angular.y) ||
	std::isnan(msg.twist.angular.z))
    {
	ROS_WARN_STREAM_THROTTLE_NAMED(
	    ROS_LOG_THROTTLE_PERIOD, LOGNAME,
	    "nan in incoming twist command. Skipping this datapoint.");

	return false;
    }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
    if ((parameters_.command_in_type == "unitless") &&
	((std::abs(msg.twist.linear.x)  > 1) ||
	 (std::abs(msg.twist.linear.y)  > 1) ||
	 (std::abs(msg.twist.linear.z)  > 1) ||
	 (std::abs(msg.twist.angular.x) > 1) ||
	 (std::abs(msg.twist.angular.y) > 1) ||
	 (std::abs(msg.twist.angular.z) > 1)))
    {
	ROS_WARN_STREAM_THROTTLE_NAMED(
	    ROS_LOG_THROTTLE_PERIOD, LOGNAME,
	    "Component of incoming command is >1. Skipping this datapoint.");

	return false;
    }

    return msg.twist.linear.x  != 0.0 ||
	   msg.twist.linear.y  != 0.0 ||
	   msg.twist.linear.z  != 0.0 ||
           msg.twist.angular.x != 0.0 ||
	   msg.twist.angular.y != 0.0 ||
	   msg.twist.angular.z != 0.0;
}

bool
ServoCalcs::isValid(const control_msgs::JointJog& msg)
{
    for (auto velocity : msg.velocities)
	if (std::isnan(velocity))
	{
	    ROS_WARN_STREAM_THROTTLE_NAMED(
		ROS_LOG_THROTTLE_PERIOD, LOGNAME,
		"nan in incoming command. Skipping this datapoint.");
	    return false;
	}

    for (auto velocity : msg.velocities)
	if (velocity != 0.0)
	    return true;

    return false;
}

//! Stop the currently running thread
void
ServoCalcs::stop()
{
  // Request stop
    stop_requested_ = true;

  // Notify condition variable in case the thread is blocked on it
    {
      // scope so the mutex is unlocked after so the thread can continue
      // and therefore be joinable
	const std::lock_guard<std::mutex> lock(input_mutex_);

	new_input_cmd_ = false;
	input_cv_.notify_all();
    }

  // Join the thread
    if (thread_.joinable())
	thread_.join();
}

//! Run the main calculation loop
void
ServoCalcs::mainCalcLoop()
{
    ros::Rate rate(1.0 / parameters_.publish_period);

    while (ros::ok() && !stop_requested_)
    {
      // lock the input state mutex
	std::unique_lock<std::mutex> input_lock(input_mutex_);

      // low latency mode -- begin calculations as soon as a new command is
      // received.
	if (parameters_.low_latency_mode)
	    input_cv_.wait(input_lock, [this]
			   { return (new_input_cmd_ || stop_requested_); });

      // reset new_input_cmd_ flag
	new_input_cmd_ = false;

      // run servo calcs
	const auto start_time = ros::Time::now();
	calculateSingleIteration();
	const auto run_duration = ros::Time::now() - start_time;

      // Log warning when the run duration was longer than the period
	if (run_duration.toSec() > parameters_.publish_period)
	    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
					   "run_duration: "
					   << run_duration.toSec()
					   << " ("
					   << parameters_.publish_period
					   << ")");

      // normal mode, unlock input mutex and wait for the period of the loop
	if (!parameters_.low_latency_mode)
	{
	    input_lock.unlock();
	    rate.sleep();
	}
    }
}

//! Do calculations for a single iteration and publish one outgoing command
void
ServoCalcs::calculateSingleIteration()
{
    publishStatus();			// Publish servo status.
    updateJoints();			// Read robot status.
    publishWorstCaseStopTime();

  // If paused or while waiting for initial servo commands,
  // just keep the low-pass filters up to date with current
  // joints so a jump doesn't occur when restarting
    if (wait_for_servo_commands_ || paused_)
    {
	resetLowPassFilters();

      // Check if there are any new commands with valid timestamp
	wait_for_servo_commands_ = twist_cmd_.header.stamp == ros::Time(0) &&
				   joint_cmd_.header.stamp == ros::Time(0);

      // Early exit
	return;
    }

  // Prioritize cartesian servoing above joint servoing.
    if (isValid(twist_cmd_))
    {
	invalid_command_count_ = 0;

	if (isStale(twist_cmd_))
	{
	    zeroVelocitiesInTrajectory();
	    resetLowPassFilters();

	    ROS_WARN_STREAM_THROTTLE_NAMED(
		10, LOGNAME,
		"Stale twist command. "
		"Try a larger 'incoming_command_timeout' parameter?");
	}
	else
	    setCartesianServoTrajectory(twist_cmd_);
    }
    else if (isValid(joint_cmd_))
    {
	invalid_command_count_ = 0;

	if (isStale(joint_cmd_))
	{
	    zeroVelocitiesInTrajectory();
	    resetLowPassFilters();

	    ROS_WARN_STREAM_THROTTLE_NAMED(
		10, LOGNAME,
		"Stale joint command. "
		"Try a larger 'incoming_command_timeout' parameter?");
	}
	else
	    setJointServoTrajectory(joint_cmd_);
    }
    else	// Both twist and joint commands are invalid.
    {
	if (invalid_command_count_ < std::numeric_limits<int>::max())
	    ++invalid_command_count_;	// Avoid overflow

	setPointsToTrajectory(actual_positions_,
			      vector_t::Zero(num_joints()), true);
	resetLowPassFilters();

      // Skip the servoing publication if all inputs have been zero
      // for several cycles in a row.
      // num_outgoing_halt_msgs_to_publish == 0 signifies that we should keep
      // republishing forever.
	if ((parameters_.num_outgoing_halt_msgs_to_publish != 0) &&
	    (invalid_command_count_ >
	     parameters_.num_outgoing_halt_msgs_to_publish))
	{
	    ROS_DEBUG_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
					    "Invalid command. Do nothing.");
	    return;
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
	auto	joints = moveit::util::make_shared_from_pool<
				std_msgs::Float64MultiArray>();
	if (parameters_.publish_joint_positions &&
	    !joint_trajectory_.points.empty())
	    joints->data = joint_trajectory_.points[0].positions;
	else if (parameters_.publish_joint_velocities &&
		 !joint_trajectory_.points.empty())
	    joints->data = joint_trajectory_.points[0].velocities;
	outgoing_cmd_pub_.publish(joints);
    }

  // For debug
    const auto	now = ros::Time::now();
    durations_.cmd_out = (now - durations_.header.stamp).toSec();
    auto	durations_tmp = durations_;
    durations_tmp.header.stamp = now;
    durations_pub_.publish(durations_tmp);
}

//! Parse the incoming joint msg for the joints of our MoveGroup
void
ServoCalcs::updateJoints()
{
    robot_state_       = planning_scene_monitor_->getStateMonitor()
						->getCurrentState();
    robot_state_stamp_ = ros::Time::now();

  // Keep original joint positions and velocities.
    actual_positions_.resize(num_joints());
    robot_state_->copyJointGroupPositions(joint_group(),
					  actual_positions_.data());
    actual_velocities_.resize(num_joints());
    robot_state_->copyJointGroupVelocities(joint_group(),
					   actual_velocities_.data());
}

//! Do servoing calculations for Cartesian twist commands
void
ServoCalcs::setCartesianServoTrajectory(twist_t& cmd)
{
  // Set uncontrolled dimensions to 0 in command frame
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

  // Transform the command to the MoveGroup planning frame.
    if (cmd.header.frame_id != parameters_.planning_frame)
    {
	if (cmd.header.frame_id.empty())
	    cmd.header.frame_id = parameters_.robot_link_command_frame;

	const auto	Tpc = getFrameTransformUnlocked(cmd.header.frame_id);
	Eigen::Vector3d	translation(cmd.twist.linear.x,
				    cmd.twist.linear.y,
				    cmd.twist.linear.z);
	Eigen::Vector3d angular(cmd.twist.angular.x,
				cmd.twist.angular.y,
				cmd.twist.angular.z);

	translation = Tpc.linear() * translation;
	angular	    = Tpc.linear() * angular;

      // Put these components back into a TwistStamped
	cmd.header.frame_id = parameters_.planning_frame;
	cmd.twist.linear.x  = translation(0);
	cmd.twist.linear.y  = translation(1);
	cmd.twist.linear.z  = translation(2);
	cmd.twist.angular.x = angular(0);
	cmd.twist.angular.y = angular(1);
	cmd.twist.angular.z = angular(2);
    }

    auto	jacobian = robot_state_->getJacobian(joint_group());
    auto	delta_x  = scaleCartesianCommand(cmd);

  // May allow some dimensions to drift, based on drift_dimensions
  // i.e. take advantage of task redundancy.
  // Remove the Jacobian rows corresponding to True
  // in the vector drift_dimensions
  // Work backwards through the 6-vector so indices don't get out of order
    for (auto dimension = jacobian.rows() - 1; dimension >= 0; --dimension)
	if (drift_dimensions_[dimension] && jacobian.rows() > 1)
	    removeDimension(jacobian, delta_x, dimension);

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
    convertDeltasToTrajectory(delta_theta);
}

//! Do servoing calculations for direct commands to a joint
void
ServoCalcs::setJointServoTrajectory(const joint_jog_t& cmd)
{
    auto	delta_theta = scaleJointCommand(cmd);

    applyVelocityScaling(delta_theta, 1.0);
    convertDeltasToTrajectory(delta_theta);
}

//! Scale them to physical units.
/*!
  Do it if incoming velocity commands are from a unitless joystick.
  Also, multiply by timestep to calculate a position change.
*/
ServoCalcs::vector_t
ServoCalcs::scaleCartesianCommand(const twist_t& cmd) const
{
  // Apply user-defined scaling if inputs are unitless [-1:1]
    const auto	linear_scale	 = (parameters_.command_in_type == "unitless" ?
				    parameters_.linear_scale : 1.0)
				 * parameters_.publish_period;
    const auto	rotational_scale = (parameters_.command_in_type == "unitless" ?
				    parameters_.rotational_scale : 1.0)
				 * parameters_.publish_period;
    vector_t	delta_x(6);
    delta_x[0] = linear_scale	  * cmd.twist.linear.x;
    delta_x[1] = linear_scale	  * cmd.twist.linear.y;
    delta_x[2] = linear_scale	  * cmd.twist.linear.z;
    delta_x[3] = rotational_scale * cmd.twist.angular.x;
    delta_x[4] = rotational_scale * cmd.twist.angular.y;
    delta_x[5] = rotational_scale * cmd.twist.angular.z;

    return delta_x;
}

//! Scale them to physical units.
/*!
  Do it if incoming velocity commands are from a unitless joystick,
  Also, multiply by timestep to calculate a position change.
*/
ServoCalcs::vector_t
ServoCalcs::scaleJointCommand(const joint_jog_t& cmd) const
{
    vector_t	delta_theta(num_joints());
    delta_theta.setZero();

    const auto joint_scale = (parameters_.command_in_type == "unitless" ?
			      parameters_.joint_scale : 1.0)
			   * parameters_.publish_period;

    for (size_t i = 0; i < cmd.joint_names.size(); ++i)
	try
	{
	    const auto	j = joint_indices_.at(cmd.joint_names[i]);

	    delta_theta[j] = joint_scale * cmd.velocities[i];
	}
	catch (const std::out_of_range& e)
	{
	    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
					   "Ignoring joint "
					   << cmd.joint_names[i]);
	}

    return delta_theta;
}

//! Calculate a velocity scaling factor
/*!
  Possibly due to proximity of singularity and direction of motion
*/
double
ServoCalcs::velocityScalingFactorForSingularity(
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
    vector_t	new_theta;
    robot_state_->copyJointGroupPositions(joint_group(), new_theta);
    new_theta += pseudo_inverse * delta_x;
    robot_state_->setJointGroupPositions(joint_group(), new_theta);
    matrix_t	new_jacobian = robot_state_->getJacobian(joint_group());

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
	    status_ = StatusCode::DECELERATE_FOR_SINGULARITY;
	    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
					   SERVO_STATUS_CODE_MAP.at(status_));
	}

      // Very close to singularity, so halt.
	else if (ini_condition > parameters_.hard_stop_singularity_threshold)
	{
	    velocity_scale = 0;
	    status_ = StatusCode::HALT_FOR_SINGULARITY;
	    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
					   SERVO_STATUS_CODE_MAP.at(status_));
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
ServoCalcs::applyVelocityScaling(vector_t& delta_theta,
				 double singularity_scale)
{
  // Convert to joint angle velocities for checking and applying joint
  // specific velocity limits.
    double	bounding_scale = 1.0;

    for (const auto joint : joint_group()->getActiveJointModels())
    {
	const auto&	bounds	 = joint->getVariableBounds(joint->getName());
	const auto	i	 = joint_indices_.at(joint->getName());
	const auto	velocity = delta_theta[i] / parameters_.publish_period;

	if (bounds.velocity_bounded_ && velocity != 0.0)
	{
	  // Clamp each joint velocity to a joint specific
	  // [min_velocity, max_velocity] range.
	    const auto	bounded_velocity = std::clamp(velocity,
						      16*bounds.min_velocity_,
						      16*bounds.max_velocity_);
	    bounding_scale = std::min(bounding_scale,
				      bounded_velocity / velocity);
	}
    }

  // Convert back to joint angle increments.
    // if (bound_scaling < 1.0)
    //   std::cerr << "*** bound_scaling="
    // 		<< bound_scaling << std::endl;
    delta_theta *= (bounding_scale *
		    collision_velocity_scale_ * singularity_scale);

    if (collision_velocity_scale_ <= 0)
    {
	status_ = StatusCode::HALT_FOR_COLLISION;

	ROS_WARN_STREAM_THROTTLE_NAMED(3, LOGNAME, "Halting for collision!");
    }
    else if (collision_velocity_scale_ < 1)
    {
	status_ = StatusCode::DECELERATE_FOR_COLLISION;

	ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
				       SERVO_STATUS_CODE_MAP.at(status_));
    }
}

//! Convert joint deltas to an outgoing JointTrajectory command
void
ServoCalcs::convertDeltasToTrajectory(const vector_t& delta_theta)
{
    vector_t	desired_positions = actual_positions_ + delta_theta;
    lowPassFilterPositions(desired_positions);

    if (checkPositionLimits(desired_positions, delta_theta))
	setPointsToTrajectory(desired_positions, delta_theta);
    else
    {
	setPointsToTrajectory(actual_positions_,
			      vector_t::Zero(delta_theta.size()), true);
	status_ = StatusCode::JOINT_BOUND;
    }
}

//! Avoid overshooting joint limits
bool
ServoCalcs::checkPositionLimits(const vector_t& positions,
				const vector_t& delta_theta) const
{
    for (const auto joint : joint_group()->getActiveJointModels())
	if (const auto&	limits = joint->getVariableBoundsMsg();
	    !limits.empty() &&
	    !robot_state_->satisfiesPositionBounds(
		joint, -parameters_.joint_limit_margin))
	{
	    const auto	i	 = joint_indices_.at(joint->getName());
	    const auto	position = positions[i];
	    const auto	delta	 = delta_theta[i];

	  // Check if pending velocity command is moving in the right direction
	    if ((delta < 0 && position < (limits[0].min_position +
					  parameters_.joint_limit_margin)) ||
		(delta > 0 && position > (limits[0].max_position -
					  parameters_.joint_limit_margin)))
	    {
		ROS_WARN_STREAM_THROTTLE_NAMED(
		    ROS_LOG_THROTTLE_PERIOD, LOGNAME,
		    ros::this_node::getName()
		    << " " << joint->getName()
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
ServoCalcs::setPointsToTrajectory(const vector_t& positions,
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
	points[i] = points[i - 1];
	points[i].time_from_start += ros::Duration(parameters_.publish_period);
    }
}

//! Set zero velocities to all points in the trajectory
void
ServoCalcs::zeroVelocitiesInTrajectory()
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
ServoCalcs::removeDimension(matrix_t& jacobian,
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
ServoCalcs::initializeLowPassFilters(int half_order, double cutoff_frequency)
{
    const std::lock_guard<std::mutex> lock(input_mutex_);

    parameters_.low_pass_filter_half_order	 = half_order;
    parameters_.low_pass_filter_cutoff_frequency = cutoff_frequency;

    for (auto&& position_filter : position_filters_)
	position_filter.initialize(
	    parameters_.low_pass_filter_half_order,
	    parameters_.low_pass_filter_cutoff_frequency *
	    parameters_.publish_period);

    resetLowPassFilters();
}

//! Smooth position commands with a lowpass filter
void
ServoCalcs::lowPassFilterPositions(vector_t& positions)
{
    for (size_t i = 0; i < position_filters_.size(); ++i)
	positions[i] = position_filters_[i].filter(positions[i]);
}

//! Set the filters to the specified values
void
ServoCalcs::resetLowPassFilters()
{
    for (size_t i = 0; i < position_filters_.size(); ++i)
	position_filters_[i].reset(actual_positions_[i]);
}

/*
 *  private member functions: callbacks
 */
void
ServoCalcs::twistCmdCB(const twist_cp& msg)
{
    const std::lock_guard<std::mutex> lock(input_mutex_);

    twist_cmd_ = *msg;

  // notify that we have a new input
    new_input_cmd_ = true;
    input_cv_.notify_all();
}

void
ServoCalcs::jointCmdCB(const joint_jog_cp& msg)
{
    const std::lock_guard<std::mutex> lock(input_mutex_);

    joint_cmd_ = *msg;

  // notify that we have a new input
    new_input_cmd_ = true;
    input_cv_.notify_all();
}

void
ServoCalcs::collisionVelocityScaleCB(const flt64_cp& msg)
{
    const std::lock_guard<std::mutex> lock(input_mutex_);

    collision_velocity_scale_ = msg->data;
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
ServoCalcs::changeDriftDimensions(
		moveit_msgs::ChangeDriftDimensions::Request&  req,
		moveit_msgs::ChangeDriftDimensions::Response& res)
{
    const std::lock_guard<std::mutex> lock(input_mutex_);

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
ServoCalcs::changeControlDimensions(
		moveit_msgs::ChangeControlDimensions::Request&  req,
		moveit_msgs::ChangeControlDimensions::Response& res)
{
    const std::lock_guard<std::mutex> lock(input_mutex_);

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
ServoCalcs::publishStatus() const
{
    auto status_msg = moveit::util::make_shared_from_pool<std_msgs::Int8>();
    status_msg->data = static_cast<int8_t>(status_);
    status_pub_.publish(status_msg);
}

bool
ServoCalcs::resetStatus(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    status_ = StatusCode::NO_WARNING;
    return true;
}

/*
 *  Worst case stop time stuffs
 */
void
ServoCalcs::publishWorstCaseStopTime() const
{
  // Calculate worst case joint stop time, for collision checking
    double	worst_case_stop_time = 0;
    for (const auto joint : joint_group()->getActiveJointModels())
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
		ROS_LOG_THROTTLE_PERIOD, LOGNAME,
		"An acceleration limit is not defined for this joint; minimum"
		"stop distance should not be used for collision checking");
    }

  // publish message
    auto	msg = moveit::util::make_shared_from_pool<flt64_t>();
    msg->data = worst_case_stop_time;
    worst_case_stop_time_pub_.publish(msg);
}

}  // namespace aist_moveit_servo
