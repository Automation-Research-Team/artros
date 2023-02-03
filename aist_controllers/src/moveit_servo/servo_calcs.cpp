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
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <cassert>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo_calcs.h>

static const std::string LOGNAME = "servo_calcs";
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops

namespace moveit_servo
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
operator <<(std::ostream& out, const Eigen::Isometry3d& transform)
{
    const Eigen::Quaterniond	q(transform.rotation());

    return out << transform.translation()(0) << ' '
	       << transform.translation()(1) << ' '
	       << transform.translation()(2) << ';'
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

// Helper function for detecting zeroed message
bool
isNonZero(const geometry_msgs::TwistStamped& msg)
{
    return msg.twist.linear.x  != 0.0 ||
	   msg.twist.linear.y  != 0.0 ||
	   msg.twist.linear.z  != 0.0 ||
           msg.twist.angular.x != 0.0 ||
	   msg.twist.angular.y != 0.0 ||
	   msg.twist.angular.z != 0.0;
}

// Helper function for detecting zeroed message
bool
isNonZero(const control_msgs::JointJog& msg)
{
    bool all_zeros = true;
    for (double delta : msg.velocities)
	all_zeros &= (delta == 0.0);

    return !all_zeros;
}

// Helper function for converting Eigen::Isometry3d to geometry_msgs/TransformStamped
geometry_msgs::TransformStamped
convertIsometryToTransform(const Eigen::Isometry3d& isometry,
			   const ros::Time& stamp,
			   const std::string& parent_frame,
			   const std::string& child_frame)
{
    auto	transform = tf2::eigenToTransform(isometry);
    transform.header.stamp    = stamp;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id  = child_frame;

    return transform;
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
  :nh_(nh),
   internal_nh_(nh, "internal"),
   parameters_(parameters),
   planning_scene_monitor_(planning_scene_monitor),
   zero_velocity_count_(0),
   wait_for_servo_commands_(true),
   updated_filters_(false),
   current_state_(planning_scene_monitor_->getStateMonitor()
					 ->getCurrentState()),
   joint_model_group_(current_state_->getJointModelGroup(
			  parameters_.move_group_name)),

   joint_state_(),
   original_joint_state_(),
   joint_state_name_map_(),
   
   position_filters_(),
   last_sent_command_(),
   
   twist_stamped_sub_(
       nh_.subscribe(parameters_.cartesian_command_in_topic, ROS_QUEUE_SIZE,
		     &ServoCalcs::twistStampedCB, this,
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
   worst_case_stop_time_pub_(internal_nh_.advertise<std_msgs::Float64>(
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
   durations_pub_(nh_.advertise<aist_controllers::DurationArray>("durations",
								 1)),
   drift_dimensions_srv_(
       nh_.advertiseService(ros::names::append(nh_.getNamespace(),
					       "change_drift_dimensions"),
			    &ServoCalcs::changeDriftDimensions, this)),
   control_dimensions_srv_(
       nh_.advertiseService(ros::names::append(nh_.getNamespace(),
					       "change_control_dimensions"),
			    &ServoCalcs::changeControlDimensions, this)),
   reset_servo_status_srv_(nh_.advertiseService(
			       ros::names::append(nh_.getNamespace(),
						  "reset_servo_status"),
			       &ServoCalcs::resetServoStatus, this)),
   ddr_(nh_),

   thread_(),
   stop_requested_(true),

   status_(StatusCode::NO_WARNING),
   paused_(false),
   collision_velocity_scale_(1.0),
   gazebo_redundant_message_count_(30),
   
   drift_dimensions_({false, false, false, false, false, false}),
   control_dimensions_({true, true, true, true, true, true}),
    
   input_mutex_(),
   twist_stamped_cmd_(),
   joint_servo_cmd_(),
   
   input_cv_(),
   new_input_cmd_(false)
{
    joint_state_.name = joint_model_group_->getActiveJointModelNames();
    joint_state_.position.resize(num_joints());
    joint_state_.velocity.resize(num_joints());

  // A map for the indices of incoming joint commands
    for (std::size_t i = 0; i < num_joints(); ++i)
	joint_state_name_map_[joint_state_.name[i]] = i;

  // Low-pass filters for the joint positions
    for (size_t i = 0; i < num_joints(); ++i)
    {
#if defined(BUTTERWORTH)
	position_filters_.emplace_back(
	    parameters_.low_pass_filter_half_order,
	    parameters_.low_pass_filter_cutoff_frequency *
	    parameters_.publish_period);
#else
	position_filters_.emplace_back(parameters_.low_pass_filter_coeff);
#endif
    }

  // Setup dynamic reconfigure server
#if defined(BUTTERWORTH)
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
#else
    ddr_.registerVariable<double>("lowpass_filter_coeff",
				  parameters_.low_pass_filter_coeff,
				  boost::bind(
				      &ServoCalcs::initializeLowPassFilters,
				      this, _1),
				  "Cutoff frequency of low pass filter",
				  1.0, 100.0);
#endif
    ddr_.publishServicesTopics();
}

ServoCalcs::~ServoCalcs()
{
    stop();
}

//!  Get the MoveIt planning link transform.
/*!
  The transform from the MoveIt planning frame to robot_link_command_frame

  \param	transform that will be calculated
  \return	true if a valid transform was available
*/
bool
ServoCalcs::getCommandFrameTransform(isometry3_t& transform) const
{
    const std::lock_guard<std::mutex>	lock(input_mutex_);

    transform = getFrameTransform(parameters_.robot_link_command_frame);

  // All zeros means the transform wasn't initialized, so return false
    return !transform.matrix().isZero(0);
}

bool
ServoCalcs::getCommandFrameTransform(transform_t& transform) const
{
    isometry3_t	isometry;
    if (!getCommandFrameTransform(isometry))
	return false;

    transform = convertIsometryToTransform(isometry,
					   joint_state_.header.stamp,
					   parameters_.planning_frame,
					   parameters_.robot_link_command_frame);
    return true;
}

//! Get the End Effector link transform.
/*!
  The transform from the MoveIt planning frame to EE link

  \param	transform that will be calculated
  \return	true if a valid transform was available
*/
bool
ServoCalcs::getEEFrameTransform(isometry3_t& transform) const
{
    const std::lock_guard<std::mutex>	lock(input_mutex_);

    transform = getFrameTransform(parameters_.ee_frame_name);

  // All zeros means the transform wasn't initialized, so return false
    return !transform.matrix().isZero(0);
}

bool
ServoCalcs::getEEFrameTransform(transform_t& transform) const
{
    isometry3_t	isometry;
    if (!getEEFrameTransform(isometry))
	return false;

    transform = convertIsometryToTransform(isometry,
					   joint_state_.header.stamp,
					   parameters_.planning_frame,
					   parameters_.ee_frame_name);
    return true;
}

ServoCalcs::isometry3_t
ServoCalcs::getFrameTransform(const std::string& frame) const
{
    return current_state_->getGlobalLinkTransform(parameters_.planning_frame)
	  .inverse()
	 * current_state_->getGlobalLinkTransform(frame);
}

//! Start the timer where we do work and publish outputs
void
ServoCalcs::start()
{
  // Stop the thread if we are currently running
    stop();

  // We will update last_sent_command_ every time we start servo
    auto initial_joint_trajectory = moveit::util::make_shared_from_pool<trajectory_t>();

  // When a joint_trajectory_controller receives a new command,
  // a stamp of 0 indicates "begin immediately"
  // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
    initial_joint_trajectory->header.stamp = ros::Time(0);
    initial_joint_trajectory->header.frame_id = parameters_.planning_frame;
    initial_joint_trajectory->joint_names = joint_state_.name;
    trajectory_point_t point;
    point.time_from_start = ros::Duration(parameters_.publish_period);

    if (parameters_.publish_joint_positions)
	planning_scene_monitor_->getStateMonitor()->getCurrentState()
			       ->copyJointGroupPositions(joint_model_group_,
							 point.positions);
    if (parameters_.publish_joint_velocities)
    {
	std::vector<double> velocity(num_joints());
	point.velocities = velocity;
    }
    if (parameters_.publish_joint_accelerations)
    {
      // I do not know of a robot that takes acceleration commands.
      // However, some controllers check that this data is non-empty.
      // Send all zeros, for now.
	point.accelerations.resize(num_joints());
    }
    initial_joint_trajectory->points.push_back(point);
    last_sent_command_ = initial_joint_trajectory;

    current_state_ = planning_scene_monitor_->getStateMonitor()
					    ->getCurrentState();

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
uint
ServoCalcs::num_joints() const
{
    return joint_state_.name.size();
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
  // Publish status each loop iteration
    auto status_msg = moveit::util::make_shared_from_pool<std_msgs::Int8>();
    status_msg->data = static_cast<int8_t>(status_);
    status_pub_.publish(status_msg);

  // Always update the joints and end-effector transform for 2 reasons:
  // 1) in case the getCommandFrameTransform() method is being used
  // 2) so the low-pass filters are up to date and don't cause a jump
    updateJoints();

  // Update from latest state
    // current_state_ = planning_scene_monitor_->getStateMonitor()
    // 					    ->getCurrentState();

  // Don't end this function without updating the filters
    updated_filters_ = false;

  // If paused or while waiting for initial servo commands,
  // just keep the low-pass filters up to date with current
  // joints so a jump doesn't occur when restarting
    if (wait_for_servo_commands_ || paused_)
    {
	resetLowPassFilters(original_joint_state_);

      // Check if there are any new commands with valid timestamp
	wait_for_servo_commands_ =
	    twist_stamped_cmd_.header.stamp == ros::Time(0.) &&
	    joint_servo_cmd_.header.stamp == ros::Time(0.);

      // Early exit
	return;
    }

  // Check for stale and/or zero cmds
    const auto	twist_command_is_stale =
		    (ros::Time::now() - twist_stamped_cmd_.header.stamp >=
		     ros::Duration(parameters_.incoming_command_timeout));
    const auto	joint_command_is_stale =
		    (ros::Time::now() - joint_servo_cmd_.header.stamp >=
		     ros::Duration(parameters_.incoming_command_timeout));
    const auto	have_nonzero_twist_stamped = isNonZero(twist_stamped_cmd_);
    const auto	have_nonzero_joint_command = isNonZero(joint_servo_cmd_);
    const auto	have_nonzero_command =  have_nonzero_twist_stamped
				     || have_nonzero_joint_command;

  // If not waiting for initial command, and not paused.
  // Do servoing calculations only if the robot should move, for efficiency
  // Create new outgoing joint trajectory command message
    auto joint_trajectory = moveit::util::make_shared_from_pool<
				trajectory_t>();

  // Prioritize cartesian servoing above joint servoing
  // Only run commands if not stale and nonzero
    if (have_nonzero_twist_stamped && !twist_command_is_stale)
    {
	if (!cartesianServoCalcs(twist_stamped_cmd_, *joint_trajectory))
	{
	    resetLowPassFilters(original_joint_state_);
	    return;
	}
    }
    else if (have_nonzero_joint_command && !joint_command_is_stale)
    {
	if (!jointServoCalcs(joint_servo_cmd_, *joint_trajectory))
	{
	    resetLowPassFilters(original_joint_state_);
	    return;
	}
    }
    else
    {
      // Joint trajectory is not populated with anything,
      // so set it to the last positions and 0 velocity
	*joint_trajectory = *last_sent_command_;
	for (auto& point : joint_trajectory->points)
	    point.velocities.assign(point.velocities.size(), 0);
    }

  // Print a warning to the user if both are stale
    if (twist_command_is_stale && joint_command_is_stale)
	ROS_WARN_STREAM_THROTTLE_NAMED(10, LOGNAME,
				       "Stale command. "
				       "Try a larger 'incoming_command_timeout' parameter?");
    
  // If we should halt
    if (!have_nonzero_command)
	suddenHalt(*joint_trajectory);

  // Skip the servoing publication if all inputs have been zero
  // for several cycles in a row.
  // num_outgoing_halt_msgs_to_publish == 0 signifies that we should keep
  // republishing forever.
    bool	ok_to_publish = true;
    if (!have_nonzero_command &&
	(parameters_.num_outgoing_halt_msgs_to_publish != 0) &&
	(zero_velocity_count_ > parameters_.num_outgoing_halt_msgs_to_publish))
    {
	ok_to_publish = false;
	ROS_DEBUG_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
					"All-zero command. Doing nothing.");
    }

  // Store last zero-velocity message flag to prevent superfluous warnings.
  // Cartesian and joint commands must both be zero.
    if (have_nonzero_command)
	zero_velocity_count_ = 0;
    else if (zero_velocity_count_ < std::numeric_limits<int>::max())
	++zero_velocity_count_;	// Avoid overflow

    const auto	now = ros::Time::now();

    if (ok_to_publish && !paused_)
    {
      // Put the outgoing msg in the right format
      // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
	if (parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
	{
	  // When a joint_trajectory_controller receives a new command,
	  // a stamp of 0 indicates "begin immediately"
	  // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
	    joint_trajectory->header.stamp = ros::Time(0);
	    outgoing_cmd_pub_.publish(joint_trajectory);

	    auto	joint_trajectory_debug = *joint_trajectory;
	    joint_trajectory_debug.header.stamp = now;
	    outgoing_cmd_debug_pub_.publish(joint_trajectory_debug);
	}
	else if (parameters_.command_out_type == "std_msgs/Float64MultiArray")
	{
	    auto joints = moveit::util::make_shared_from_pool<
				std_msgs::Float64MultiArray>();
	    if (parameters_.publish_joint_positions &&
		!joint_trajectory->points.empty())
		joints->data = joint_trajectory->points[0].positions;
	    else if (parameters_.publish_joint_velocities &&
		     !joint_trajectory->points.empty())
		joints->data = joint_trajectory->points[0].velocities;
	    outgoing_cmd_pub_.publish(joints);
	}

	durations_.cmd_out = (now - durations_.header.stamp).toSec();
	auto	durations_tmp = durations_;
	durations_tmp.header.stamp = now;
	durations_pub_.publish(durations_tmp);

	last_sent_command_ = joint_trajectory;
    }

  // Update the filters if we haven't yet
    if (!updated_filters_)
	resetLowPassFilters(original_joint_state_);
}

//! Parse the incoming joint msg for the joints of our MoveGroup
void
ServoCalcs::updateJoints()
{
  // Get the latest joint group positions
    current_state_ = planning_scene_monitor_->getStateMonitor()
					    ->getCurrentState();
    current_state_->copyJointGroupPositions(joint_model_group_,
					    joint_state_.position);
    current_state_->copyJointGroupVelocities(joint_model_group_,
					     joint_state_.velocity);
    joint_state_.header.stamp = ros::Time::now();
    
  // Cache the original joints in case they need to be reset
    original_joint_state_ = joint_state_;

  // Calculate worst case joint stop time, for collision checking
    double	worst_case_stop_time = 0;
    for (const auto joint : joint_model_group_->getActiveJointModels())
    {
	const auto&	bound = joint->getVariableBounds()[0];

      // Some joints do not have acceleration limits
	if (bound.acceleration_bounded_)
	{
	  // Be conservative when calculating overall acceleration
	  // limit from min and max limits
	    const auto	accel_limit = std::min(fabs(bound.min_acceleration_),
					       fabs(bound.max_acceleration_));
	    const auto	i = joint_state_name_map_[joint->getName()];

	    worst_case_stop_time
		= std::max(worst_case_stop_time,
			   fabs(joint_state_.velocity[i]
				/ accel_limit));
	}
	else
	    ROS_WARN_STREAM_THROTTLE_NAMED(
		ROS_LOG_THROTTLE_PERIOD, LOGNAME,
		"An acceleration limit is not defined for this joint; minimum"
		"stop distance should not be used for collision checking");
    }

  // publish message
    auto msg = moveit::util::make_shared_from_pool<flt64_t>();
    msg->data = worst_case_stop_time;
    worst_case_stop_time_pub_.publish(msg);
}

//! Do servoing calculations for Cartesian twist commands
bool
ServoCalcs::cartesianServoCalcs(twist_t& cmd, trajectory_t& joint_trajectory)
{
  // Check for nan's in the incoming command
    if (std::isnan(cmd.twist.linear.x)  ||
	std::isnan(cmd.twist.linear.y)  ||
	std::isnan(cmd.twist.linear.z)  ||
	std::isnan(cmd.twist.angular.x) ||
	std::isnan(cmd.twist.angular.y) ||
	std::isnan(cmd.twist.angular.z))
    {
	ROS_WARN_STREAM_THROTTLE_NAMED(
	    ROS_LOG_THROTTLE_PERIOD, LOGNAME,
	    "nan in incoming command. Skipping this datapoint.");
	return false;
    }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
    if (parameters_.command_in_type == "unitless")
    {
	if ((fabs(cmd.twist.linear.x)  > 1) ||
	    (fabs(cmd.twist.linear.y)  > 1) ||
	    (fabs(cmd.twist.linear.z)  > 1) ||
	    (fabs(cmd.twist.angular.x) > 1) ||
	    (fabs(cmd.twist.angular.y) > 1) ||
	    (fabs(cmd.twist.angular.z) > 1))
	{
	    ROS_WARN_STREAM_THROTTLE_NAMED(
		ROS_LOG_THROTTLE_PERIOD, LOGNAME,
		"Component of incoming command is >1. Skipping this datapoint.");
	    return false;
	}
    }

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

  // Transform the command to the MoveGroup planning frame
    if (cmd.header.frame_id != parameters_.planning_frame)
    {
	if (cmd.header.frame_id.empty())
	    cmd.header.frame_id = parameters_.robot_link_command_frame;

	const auto	Tpc = getFrameTransform(cmd.header.frame_id);
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

    auto	delta_x = scaleCartesianCommand(cmd);

  // Convert from cartesian commands to joint commands
    auto	jacobian = current_state_->getJacobian(joint_model_group_);

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

    vector_t	delta_theta = pseudo_inverse * delta_x;

    enforceVelLimits(delta_theta);

  // If close to a collision or a singularity, decelerate
    applyVelocityScaling(delta_theta,
			 velocityScalingFactorForSingularity(delta_x, svd,
							     pseudo_inverse));

    return convertDeltasToOutgoingCmd(delta_theta, joint_trajectory);
}

//! Do servoing calculations for direct commands to a joint
bool
ServoCalcs::jointServoCalcs(const joint_jog_t& cmd,
			    trajectory_t& joint_trajectory)
{
  // Check for nan's
    for (auto velocity : cmd.velocities)
	if (std::isnan(velocity))
	{
	    ROS_WARN_STREAM_THROTTLE_NAMED(
		ROS_LOG_THROTTLE_PERIOD, LOGNAME,
		"nan in incoming command. Skipping this datapoint.");
	    return false;
	}

  // Apply user-defined scaling
    auto	delta_theta = scaleJointCommand(cmd);

    enforceVelLimits(delta_theta);

  // If close to a collision, decelerate
    applyVelocityScaling(delta_theta, 1.0 /* scaling for singularities -- ignore for joint motions */);

    return convertDeltasToOutgoingCmd(delta_theta, joint_trajectory);
}

//! Scale the delta theta to match joint velocity/acceleration limits
void
ServoCalcs::enforceVelLimits(vector_t& delta_theta) const
{
  // Convert to joint angle velocities for checking and applying joint
  // specific velocity limits.
    const auto	velocity = delta_theta / parameters_.publish_period;
    double	velocity_scaling_factor = 1.0;

    size_t	i = 0;
    for (const auto joint : joint_model_group_->getActiveJointModels())
    {
	const auto&	bounds = joint->getVariableBounds(joint->getName());
	const auto	unbounded_velocity = velocity(i);

	if (bounds.velocity_bounded_ && unbounded_velocity != 0.0)
	{
	  // Clamp each joint velocity to a joint specific
	  // [min_velocity, max_velocity] range.
	    const auto bounded_velocity
		= std::min(std::max(unbounded_velocity,
				    16*bounds.min_velocity_),
			   16*bounds.max_velocity_);
	    velocity_scaling_factor
	    	= std::min(velocity_scaling_factor,
	    		   bounded_velocity / unbounded_velocity);
	}
	++i;
    }

  // Convert back to joint angle increments.
    // if (velocity_scaling_factor < 1.0)
    //   std::cerr << "*** velocity_scaling_factor="
    // 		<< velocity_scaling_factor << std::endl;
    delta_theta = velocity_scaling_factor * velocity
    		* parameters_.publish_period;
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
    std::size_t	num_dimensions = commanded_velocity.size();

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
    current_state_->copyJointGroupPositions(joint_model_group_, new_theta);
    new_theta += pseudo_inverse * delta_x;
    current_state_->setJointGroupPositions(joint_model_group_, new_theta);
    matrix_t	new_jacobian = current_state_->getJacobian(
					joint_model_group_);

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
    if (collision_velocity_scale_ == 0)
    {
	status_ = StatusCode::HALT_FOR_COLLISION;
	ROS_WARN_STREAM_THROTTLE_NAMED(3, LOGNAME, "Halting for collision!");
    }
    else if (0 < collision_velocity_scale_ && collision_velocity_scale_ < 1)
    {
	status_ = StatusCode::DECELERATE_FOR_COLLISION;
	ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
				       SERVO_STATUS_CODE_MAP.at(status_));
    }

    delta_theta *= (collision_velocity_scale_ * singularity_scale);
}


//! Convert joint deltas to an outgoing JointTrajectory command
bool
ServoCalcs::convertDeltasToOutgoingCmd(const vector_t& delta_theta,
				       trajectory_t& joint_trajectory)
{
    joint_state_ = original_joint_state_;
    if (!addJointIncrements(joint_state_, delta_theta))
	return false;

    lowPassFilterPositions(joint_state_);

  // Calculate joint velocities here so that positions are filtered
  // and SRDF bounds still get checked
    calculateJointVelocities(joint_state_, delta_theta);

    composeJointTrajMessage(joint_state_, joint_trajectory);

    if (!enforcePositionLimits(joint_state_))
    {
	suddenHalt(joint_trajectory);
	status_ = StatusCode::JOINT_BOUND;
    }

  // done with calculations
    if (parameters_.use_gazebo)
	insertRedundantPointsIntoTrajectory(joint_trajectory,
					    gazebo_redundant_message_count_);
    
    return true;
}

//! Add the deltas to each joint
bool
ServoCalcs::addJointIncrements(joint_state_t& joint_state,
			       const vector_t& delta_theta) const
{
    if (joint_state.position.size() != delta_theta.size())
    {
	ROS_ERROR_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
					ros::this_node::getName()
					<< " Lengths of output and "
					"increments do not match.");
	return false;
    }
	
    for (int i = 0; i < delta_theta.size(); ++i)
	joint_state.position[i] += delta_theta[i];
    
    return true;
}

//! Convert an incremental position command to joint velocity message
void
ServoCalcs::calculateJointVelocities(joint_state_t& joint_state,
				     const vector_t& delta_theta)
{
    for (int i = 0; i < delta_theta.size(); ++i)
	joint_state.velocity[i] = delta_theta[i] / parameters_.publish_period;
}

//! Compose the outgoing JointTrajectory message
void
ServoCalcs::composeJointTrajMessage(const joint_state_t& joint_state,
				    trajectory_t& joint_trajectory) const
{
  // When a joint_trajectory_controller receives a new command,
  // a stamp of 0 indicates "begin immediately"
  // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
    joint_trajectory.header.stamp    = ros::Time(0);
    joint_trajectory.header.frame_id = parameters_.planning_frame;
    joint_trajectory.joint_names     = joint_state.name;

    trajectory_point_t	point;
    point.time_from_start = ros::Duration(parameters_.publish_period);
    if (parameters_.publish_joint_positions)
	point.positions = joint_state.position;
    if (parameters_.publish_joint_velocities)
	point.velocities = joint_state.velocity;
    if (parameters_.publish_joint_accelerations)
    {
      // I do not know of a robot that takes acceleration commands.
      // However, some controllers check that this data is non-empty.
      // Send all zeros, for now.
	std::vector<double> acceleration(num_joints());
	point.accelerations = acceleration;
    }
    joint_trajectory.points.push_back(point);
}

//! Avoid overshooting joint limits
bool
ServoCalcs::enforcePositionLimits(joint_state_t& joint_state) const
{
    bool	halting = false;

    for (const auto joint : joint_model_group_->getActiveJointModels())
	if (const auto&	limits = joint->getVariableBoundsMsg();
	    !current_state_->satisfiesPositionBounds(
		joint, -parameters_.joint_limit_margin) &&
	    !limits.empty())
	{
	    double	joint_angle = 0;
	    for (std::size_t i = 0; i < original_joint_state_.name.size(); ++i)
		if (original_joint_state_.name[i] == joint->getName())
		{
		    joint_angle = original_joint_state_.position[i];
		    break;
		}

	  // Check if pending velocity command is moving in the right
	  // direction
	    const auto	velocity = joint_state.velocity[joint_state_name_map_
							.at(joint->getName())];

	    if ((velocity < 0 &&
		 joint_angle < (limits[0].min_position +
				parameters_.joint_limit_margin)) ||
		(velocity > 0 &&
		 joint_angle > (limits[0].max_position -
				parameters_.joint_limit_margin)))
	    {
		ROS_WARN_STREAM_THROTTLE_NAMED(
		    ROS_LOG_THROTTLE_PERIOD, LOGNAME,
		    ros::this_node::getName()
		    << " " << joint->getName()
		    << " close to a position limit. Halting.");
		halting = true;
	    }
	}

    return !halting;
}

//! Satisfy Gazebo by stuffing multiple messages into one
/*!
  Gazebo simulations have very strict message timestamp requirements
  Spam several redundant points into the trajectory. The first few may be
  skipped if the time stamp is in the past when it reaches the client.
  Needed for gazebo simulation.
*/
void
ServoCalcs::insertRedundantPointsIntoTrajectory(
    trajectory_t& joint_trajectory, int count) const
{
    joint_trajectory.points.resize(count);
    auto point = joint_trajectory.points[0];
  // Start from 2nd point (i = 1) because we already have the first point.
  // The timestamps are shifted up one period since first point is at
  // 1 * publish_period, not 0.
    for (int i = 1; i < count; ++i)
    {
	point.time_from_start = ros::Duration((i + 1)
					      * parameters_.publish_period);
	joint_trajectory.points[i] = point;
    }
}

//! Suddenly halt for a joint limit or other critical issue.
/*!
  Is handled differently for position vs. velocity control.
  Suddenly halt for a joint limit or other critical issue.
  Is handled differently for position vs. velocity control.
*/
void
ServoCalcs::suddenHalt(trajectory_t& joint_trajectory)
{
  // Prepare the joint trajectory message to stop the robot
    joint_trajectory.points.clear();
    joint_trajectory.points.emplace_back();
    auto&	point = joint_trajectory.points.front();

  // When sending out trajectory_msgs/JointTrajectory type messages,
  // the "trajectory" is just a single point.
  // That point cannot have the same timestamp as the start
  // of trajectory execution since that would mean the
  // arm has to reach the first trajectory point the moment execution begins.
  // To prevent errors about points being 0 seconds in the past,
  // the smallest supported timestep is added as time from start
  // to the trajectory point.
    point.time_from_start.fromNSec(1);

    if (parameters_.publish_joint_positions)
	point.positions.resize(num_joints());
    if (parameters_.publish_joint_velocities)
	point.velocities.resize(num_joints());

  // Assert the following loop is safe to execute
    assert(original_joint_state_.position.size() >= num_joints());

  // Set the positions and velocities vectors
    for (std::size_t i = 0; i < num_joints(); ++i)
    {
      // For position-controlled robots, can reset the joints to a known,
      // good state
	if (parameters_.publish_joint_positions)
	    point.positions[i] = original_joint_state_.position[i];

      // For velocity-controlled robots, stop
	if (parameters_.publish_joint_velocities)
	    point.velocities[i] = 0;
    }
}

//! Remove the Jacobian row and the delta-x element of one Cartesian dimension
/*!
  Take advantage of task redundancy

  \param matrix		The Jacobian matrix.
  \param delta_x	Vector of Cartesian delta commands, should be the same size as matrix.rows()
  \param row_to_remove	Dimension that will be allowed to drift, e.g. row_to_remove = 2 allows z-translation drift.
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
 *  private member functions: incoming command scaling stuffs
 */
//! Scale them to physical units.
/*!
  Do it if incoming velocity commands are from a unitless joystick.
  Also, multiply by timestep to calculate a position change.
*/
ServoCalcs::vector_t
ServoCalcs::scaleCartesianCommand(const twist_t& command) const
{
    vector_t result(6);

  // Apply user-defined scaling if inputs are unitless [-1:1]
    if (parameters_.command_in_type == "unitless")
    {
	result[0] = parameters_.linear_scale * parameters_.publish_period
		  * command.twist.linear.x;
	result[1] = parameters_.linear_scale * parameters_.publish_period
		  * command.twist.linear.y;
	result[2] = parameters_.linear_scale * parameters_.publish_period
		  * command.twist.linear.z;
	result[3] = parameters_.rotational_scale * parameters_.publish_period
		  * command.twist.angular.x;
	result[4] = parameters_.rotational_scale * parameters_.publish_period
		  * command.twist.angular.y;
	result[5] = parameters_.rotational_scale * parameters_.publish_period
		  * command.twist.angular.z;
    }
  // Otherwise, commands are in m/s and rad/s
    else
    {
	result[0] = command.twist.linear.x  * parameters_.publish_period;
	result[1] = command.twist.linear.y  * parameters_.publish_period;
	result[2] = command.twist.linear.z  * parameters_.publish_period;
	result[3] = command.twist.angular.x * parameters_.publish_period;
	result[4] = command.twist.angular.y * parameters_.publish_period;
	result[5] = command.twist.angular.z * parameters_.publish_period;
    }

    return result;
}

//! Scale them to physical units.
/*!
  Do it if incoming velocity commands are from a unitless joystick,
  Also, multiply by timestep to calculate a position change.
*/
ServoCalcs::vector_t
ServoCalcs::scaleJointCommand(const joint_jog_t& command) const
{
    vector_t result(num_joints());
    result.setZero();

    const auto	k = (parameters_.command_in_type == "unitless" ?
		     parameters_.joint_scale : 1.0)
		  * parameters_.publish_period;

    for (size_t m = 0; m < command.joint_names.size(); ++m)
	try
	{
	    const auto	i = joint_state_name_map_.at(command.joint_names[m]);

	    result[i] = k * command.velocities[m];
	}
	catch (const std::out_of_range& e)
	{
	    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
					   "Ignoring joint "
					   << command.joint_names[m]);
	}
    
    return result;
}

/*
 *  private member functions: low-pass filter stuffs
 */
//! Change order and/or cutoff of filters
#if defined(BUTTERWORTH)
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
    
    resetLowPassFilters(original_joint_state_);
}
#else
void
ServoCalcs::initializeLowPassFilters(double coeff)
{
    const std::lock_guard<std::mutex> lock(input_mutex_);

    parameters_.low_pass_filter_coeff = coeff;

    for (auto&& position_filter: position_filters_)
	position_filter.initialize(parameters_.low_pass_filter_coeff);

    resetLowPassFilters(original_joint_state_);
}
#endif

//! Smooth position commands with a lowpass filter
void
ServoCalcs::lowPassFilterPositions(joint_state_t& joint_state)
{
    for (size_t i = 0; i < position_filters_.size(); ++i)
	joint_state.position[i] = position_filters_[i].filter(
					joint_state.position[i]);

    updated_filters_ = true;
}

//! Set the filters to the specified values
void
ServoCalcs::resetLowPassFilters(const joint_state_t& joint_state)
{
    for (size_t i = 0; i < position_filters_.size(); ++i)
	position_filters_[i].reset(joint_state.position[i]);

    updated_filters_ = true;
}

/*
 *  private member functions: callbacks
 */
void
ServoCalcs::twistStampedCB(const twist_cp& msg)
{
    const std::lock_guard<std::mutex> lock(input_mutex_);

    twist_stamped_cmd_ = *msg;

  // notify that we have a new input
    new_input_cmd_ = true;
    input_cv_.notify_all();
}

void
ServoCalcs::jointCmdCB(const joint_jog_cp& msg)
{
    const std::lock_guard<std::mutex> lock(input_mutex_);

    joint_servo_cmd_ = *msg;

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

//! Service callback to reset servo status
/*!
  e.g. so the arm can move again after a collision
*/
bool
ServoCalcs::resetServoStatus(std_srvs::Empty::Request& /*req*/,
			     std_srvs::Empty::Response& /*res*/)
{
    status_ = StatusCode::NO_WARNING;
    return true;
}

}  // namespace moveit_servo
