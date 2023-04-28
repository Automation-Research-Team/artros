/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, National Institute of Industrial Scienece
 *  and Technology (AIST)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 *  \file	pose_tracking_servo.h
 *  \brief	ROS pose tracker of aist_moveit_servo::PoseTracking type
 *  \author	Toshio UESHIBA
 */
#pragma once

#include <actionlib/server/simple_action_server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <control_toolbox/pid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aist_moveit_servo/servo.h>
#include <aist_moveit_servo/status_codes.h>
#include <aist_moveit_servo/PoseTrackingAction.h>
#include <aist_moveit_servo/make_shared_from_pool.h>
#include <aist_utility/geometry_msgs.h>
#if defined(USE_BUTTERWORTH_LPF)
#  include <aist_utility/butterworth_lpf.h>
#else
#  include <aist_utility/decay_lpf.h>
#endif

// Conventions:
// Calculations are done in the planning_frame unless otherwise noted.

namespace aist_moveit_servo
{
/************************************************************************
*  class PoseTrackingServo<FF>						*
************************************************************************/
template <class FF>
class PoseTrackingServo : public Servo
{
  private:
    using server_t	 = actionlib::SimpleActionServer<PoseTrackingAction>;
    using goal_cp	 = boost::shared_ptr<const server_t::Goal>;
    using servo_status_t = aist_moveit_servo::StatusCode;
    using int8_cp	 = std_msgs::Int8ConstPtr;
    using raw_pose_t	 = geometry_msgs::Pose;
    using vector3_t	 = Eigen::Vector3d;
    using angle_axis_t	 = Eigen::AngleAxisd;
    using pid_t		 = control_toolbox::Pid;
#if defined(USE_BUTTERWORTH_LPF)
    using lpf_t		 = aist_utility::ButterworthLPF<double, raw_pose_t>;
#else
    using lpf_t		 = aist_utility::DecayLPF<double, raw_pose_t>;
#endif
    struct PIDConfig
    {
	double	k_p	     = 1;
	double	k_i	     = 0;
	double	k_d	     = 0;
	double	windup_limit = 0.1;
    };

  public:
		PoseTrackingServo(ros::NodeHandle& nh,
				  const std::string& logname)		;
		~PoseTrackingServo()					;

    void	run()							;

  private:
    void	readROSParams(const ros::NodeHandle& parameter_nh)	;
    void	tick()							;
    pose_t	correctTargetPose(const raw_pose_t& offset)	const	;
    void	calculatePoseError(const pose_t& target_pose,
				   const pose_t& actual_pose,
				   vector3_t& positional_error,
				   angle_axis_t& angular_error)	const	;
    twist_t	calculateTwistCommand(const vector3_t& positional_error,
				      const angle_axis_t& angular_error);
    void	stopMotion()						;
    void	doPostMotionReset()					;

  // Input low-pass filter stuffs
#if defined(USE_BUTTERWORTH_LPF)
    void	updateInputLowPassFilter(int half_order,
					 double cutoff_frequency)	;
#else
    void	updateInputLowPassFilter(double half_life)		;
#endif

  // PID stuffs
    void	updateLinearPIDs(double PIDConfig::* field,
				 double value)				;
    void	updateAngularPID(double PIDConfig::* field,
				 double value)				;
    void	updatePID(const PIDConfig& pid_config, pid_t& pid)	;

  // PostTrackingAction stuffs
    void	goalCB()						;
    void	preemptCB()						;

  // Target pose stuffs
    void	targetPoseCB(const pose_cp& target_pose)		;
    pose_t	targetPose()					const	;
    bool	haveRecentTargetPose(const ros::Duration& timeout) const;
    void	resetTargetPose()					;

  // Current pose stuffs
    pose_t	actualPose()					const	;

  // FeedForward pose stuffs
    void	publishFeedForwardPose(const pose_t& ff_pose)	const	;
    void	publishFeedForwardPose(std::nullptr_t)		const	;

  private:
  // ROS
    const ros::NodeHandle	parameter_nh_;
    ros::ServiceClient		reset_servo_status_;
    const ros::Subscriber	target_pose_sub_;
    const ros::Publisher	target_pose_debug_pub_;
    const ros::Publisher	actual_pose_debug_pub_;
    const ros::Publisher	ff_pose_debug_pub_;
    DurationArray&		durations_;
    ddr_t			ddr_;

  // Action server stuffs
    server_t			pose_tracking_srv_;
    goal_cp			current_goal_;
    ros::Time			stamp_goal_accepted_;
    int				nframes_within_tolerance_;
    int				min_nframes_within_tolerance_;

  // Target pose stuffs
    pose_cp			target_pose_;
    mutable std::mutex		target_pose_mtx_;

  // Filters for input target pose
#if defined(USE_BUTTERWORTH_LPF)
    int				input_low_pass_filter_half_order_;
    double			input_low_pass_filter_cutoff_frequency_;
#endif
    lpf_t			input_low_pass_filter_;

  // PIDs
    PIDConfig			linear_pid_config_;
    PIDConfig			angular_pid_config_;
    std::array<pid_t, 4>	pids_;
};

template <class FF>
PoseTrackingServo<FF>::PoseTrackingServo(ros::NodeHandle& nh,
					 const std::string& logname)
    :Servo(nh, logname),

     parameter_nh_(nh, "pose_tracking"),
     reset_servo_status_(nh.serviceClient<std_srvs::Empty>(
			     "reset_servo_status")),
     target_pose_sub_(nh.subscribe(
			  "/target_pose", 1,
			  &PoseTrackingServo::targetPoseCB, this,
			  ros::TransportHints().reliable().tcpNoDelay(true))),
     target_pose_debug_pub_(nh.advertise<pose_t>("desired_pose", 1)),
     actual_pose_debug_pub_(nh.advertise<pose_t>("actual_pose", 1)),
     ff_pose_debug_pub_(nh.advertise<pose_t>("ff_pose", 1)),
     durations_(durations()),
     ddr_(parameter_nh_),

     pose_tracking_srv_(nh, "pose_tracking", false),
     current_goal_(nullptr),
     stamp_goal_accepted_(ros::Time(0)),
     nframes_within_tolerance_(0),
     min_nframes_within_tolerance_(5),

     target_pose_(nullptr),
     target_pose_mtx_(),
#if defined(USE_BUTTERWORTH_LPF)
     input_low_pass_filter_half_order_(3),
     input_low_pass_filter_cutoff_frequency_(7.0),
     input_low_pass_filter_(input_low_pass_filter_half_order_,
			    input_low_pass_filter_cutoff_frequency_ *
			    servoParameters().publish_period),
#else
     input_low_pass_filter_(servoParameters().publish_period/0.08),
#endif
     linear_pid_config_(),
     angular_pid_config_(),
     pids_()
{
    readROSParams(parameter_nh_);

  // Setup action server
    pose_tracking_srv_.registerGoalCallback(boost::bind(
						&PoseTrackingServo::goalCB,
						this));
    pose_tracking_srv_.registerPreemptCallback(boost::bind(
						 &PoseTrackingServo::preemptCB,
						 this));
    pose_tracking_srv_.start();

  // Setup dynamic reconfigure server
#if defined(USE_BUTTERWORTH_LPF)
    ddr_.registerVariable<int>("input_lowpass_filter_half_order",
			       input_low_pass_filter_half_order_,
			       boost::bind(
				   &PoseTrackingServo
				   ::updateInputLowPassFilter,
				   this, _1,
				   input_low_pass_filter_cutoff_frequency_),
			       "Half order of input low-pass filter", 1, 5);
    ddr_.registerVariable<double>("input_lowpass_filter_cutoff_frequency",
				  input_low_pass_filter_cutoff_frequency_,
				  boost::bind(
				      &PoseTrackingServo
				      ::updateInputLowPassFilter,
				      this, input_low_pass_filter_half_order_,
				      _1),
				  "Cutoff frequency of input low-pass filter",
				  0.5, 100.0);
#else
    ddr_.registerVariable<double>("input_low_pass_filter_half_life",
				  servoParameters().publish_period
				  /input_low_pass_filter_.decay(),
				  boost::bind(&PoseTrackingServo
					      ::updateInputLowPassFilter,
					      this, _1),
				  "Half-life of input low-pass filter",
				  0.01, 0.5);
#endif
    ddr_.registerVariable<double>("linear_proportional_gain",
				  linear_pid_config_.k_p,
				  boost::bind(&PoseTrackingServo
					      ::updateLinearPIDs,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for translation",
				  0.5, 300.0);
    ddr_.registerVariable<double>("linear_integral_gain",
				  linear_pid_config_.k_i,
				  boost::bind(&PoseTrackingServo
					      ::updateLinearPIDs,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for translation",
				  0.0, 20.0);
    ddr_.registerVariable<double>("linear_derivative_gain",
				  linear_pid_config_.k_d,
				  boost::bind(&PoseTrackingServo
					      ::updateLinearPIDs,
					      this, &PIDConfig::k_d, _1),
				  "Derivative gain for translation",
				  0.0, 20.0);

    ddr_.registerVariable<double>("angular_proportinal_gain",
				  angular_pid_config_.k_p,
				  boost::bind(&PoseTrackingServo
					      ::updateAngularPID,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for rotation",
				  0.5, 300.0);
    ddr_.registerVariable<double>("angular_integral_gain",
				  angular_pid_config_.k_i,
				  boost::bind(&PoseTrackingServo
					      ::updateAngularPID,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for rotation",
				  0.0, 20.0);
    ddr_.registerVariable<double>("angular_derivative_gain",
				  angular_pid_config_.k_d,
				  boost::bind(&PoseTrackingServo
					      ::updateAngularPID,
					      this, &PIDConfig::k_d, _1),
				  "Derivative gain for rotation",
				  0.0, 20.0);

    ddr_.registerVariable<int>("min_nframes_within_tolerance",
			       &min_nframes_within_tolerance_,
			       "Minimum #frames within tolerance",
			       1, 50);

    ddr_.publishServicesTopicsAndUpdateConfigData();

    ROS_INFO_STREAM_NAMED(logname, "(PoseTrackingServo) server started");
}

template <class FF>
PoseTrackingServo<FF>::~PoseTrackingServo()
{
    stopMotion();
}

template <class FF> void
PoseTrackingServo<FF>::run()
{
    ros::AsyncSpinner	spinner(0);
    spinner.start();

    for (ros::Rate rate(1.0/servoParameters().publish_period);
	 ros::ok(); rate.sleep())
    {
	tick();
    }

    spinner.stop();
    ros::waitForShutdown();
}

/*
 *  private member functions
 */
template <class FF> void
PoseTrackingServo<FF>::readROSParams(const ros::NodeHandle& parameter_nh)
{
  // Setup input low-pass filter
    std::size_t error = 0;
#if defined(USE_BUTTERWORTH_LPF)
    error += !rosparam_shortcuts::get(logname(), parameter_nh,
				      "input_low_pass_filter_half_order",
				      input_low_pass_filter_half_order_);
    error += !rosparam_shortcuts::get(logname(), parameter_nh,
				      "input_low_pass_filter_cutoff_frequency",
				      input_low_pass_filter_cutoff_frequency_);
#endif

  // Setup PID configurations
    double	windup_limit;
    error += !rosparam_shortcuts::get(logname(), parameter_nh,
				      "windup_limit", windup_limit);
    linear_pid_config_.windup_limit  = windup_limit;
    angular_pid_config_.windup_limit = windup_limit;

    error += !rosparam_shortcuts::get(logname(), parameter_nh,
				      "linear_proportional_gain",
				      linear_pid_config_.k_p);
    error += !rosparam_shortcuts::get(logname(), parameter_nh,
				      "linear_integral_gain",
				      linear_pid_config_.k_i);
    error += !rosparam_shortcuts::get(logname(), parameter_nh,
				      "linear_derivative_gain",
				      linear_pid_config_.k_d);
    error += !rosparam_shortcuts::get(logname(), parameter_nh,
				      "angular_proportional_gain",
				      angular_pid_config_.k_p);
    error += !rosparam_shortcuts::get(logname(), parameter_nh,
				      "angular_integral_gain",
				      angular_pid_config_.k_i);
    error += !rosparam_shortcuts::get(logname(), parameter_nh,
				      "angular_derivative_gain",
				      angular_pid_config_.k_d);

    error += !rosparam_shortcuts::get(logname(), parameter_nh,
				      "min_nframes_within_tolerance",
				      min_nframes_within_tolerance_);

    rosparam_shortcuts::shutdownIfError(ros::this_node::getName(), error);
}

template <class FF> void
PoseTrackingServo<FF>::tick()
{
    updateRobot();

    const auto	actual_pose = actualPose();
    actual_pose_debug_pub_.publish(actual_pose);

    if (!pose_tracking_srv_.isActive())
	return;

    durations_.tick_begin = (ros::Time::now() -
			     durations_.header.stamp).toSec();

  // Check that servo status is not emergency.
    const auto	servo_status = servoStatus();

    switch (servo_status)
    {
      case servo_status_t::HALT_FOR_SINGULARITY:
      case servo_status_t::HALT_FOR_COLLISION:
      case servo_status_t::JOINT_BOUND:
      {
	doPostMotionReset();

	PoseTrackingResult	result;
	result.status = static_cast<int8_t>(servo_status);
	pose_tracking_srv_.setAborted(result);
	ROS_ERROR_STREAM_NAMED(logname(), "(PoseTrackingServo) goal ABORTED["
			       << SERVO_STATUS_CODE_MAP.at(servo_status)
			       << ']');

	return;
      }

      default:
	break;
    }

  // Check that target pose is recent enough.
    if (!haveRecentTargetPose(current_goal_->timeout) ||
	!static_cast<const FF&>(*this)
	 .haveRecentFeedForwardInput(current_goal_->timeout))
    {
	if (ros::Time::now() - stamp_goal_accepted_ > current_goal_->timeout)
	{
	    doPostMotionReset();

	    PoseTrackingResult	result;
	    result.status = PoseTrackingFeedback::INPUT_TIMEOUT;
	    pose_tracking_srv_.setAborted(result);
	    ROS_ERROR_STREAM_NAMED(logname(),
				   "(PoseTrackingServo) goal ABORTED[The target pose was not updated recently.]");
	}

	return;
    }

  // Correct target pose by offset specified by the goal.
    const auto	target_pose = correctTargetPose(current_goal_->target_offset);

  // Compute positional and angular errors.
    vector3_t		positional_error;
    angle_axis_t	angular_error;
    calculatePoseError(target_pose, actual_pose,
		       positional_error, angular_error);

  // Check if goal tolerance is satisfied.
    const auto	within_tolerance = (std::abs(positional_error(0)) <
				    current_goal_->positional_tolerance[0] &&
				    std::abs(positional_error(1)) <
				    current_goal_->positional_tolerance[1] &&
				    std::abs(positional_error(2)) <
				    current_goal_->positional_tolerance[2] &&
				    std::abs(angular_error.angle()) <
				    current_goal_->angular_tolerance);
    if (within_tolerance)
	++nframes_within_tolerance_;
    else
	nframes_within_tolerance_ = 0;

    if (current_goal_->terminate_on_success &&
	nframes_within_tolerance_ > min_nframes_within_tolerance_)
    {
	doPostMotionReset();

	PoseTrackingResult	result;
	result.status = PoseTrackingFeedback::NO_WARNING;
	pose_tracking_srv_.setSucceeded(result);
	ROS_INFO_STREAM_NAMED(logname(), "(PoseTrackingServo) goal SUCCEEDED");

	return;
    }

  // Publish tracking result as feedback.
    PoseTrackingFeedback	feedback;
    feedback.positional_error[0] = positional_error(0);
    feedback.positional_error[1] = positional_error(1);
    feedback.positional_error[2] = positional_error(2);
    feedback.angular_error	 = angular_error.angle();
    feedback.within_tolerance	 = within_tolerance;
    feedback.status		 = static_cast<int8_t>(servo_status);
    pose_tracking_srv_.publishFeedback(feedback);

  // Compute servo command from PID controller output and send it
  // to the Servo object, for execution
    const auto	twist_cmd = calculateTwistCommand(positional_error,
						  angular_error);
    const auto	ff_pose   = static_cast<const FF&>(*this).ff_pose(
			      target_pose,
			      ros::Duration(servoParameters().publish_period));

    durations_.twist_out = (ros::Time::now() -
			    durations_.header.stamp).toSec();

  // Publish target pose, actual pose and feedforwad pose for debugging.
    target_pose_debug_pub_.publish(target_pose);
    publishFeedForwardPose(ff_pose);

    publishTrajectory(twist_cmd, ff_pose);
}

template <class FF> typename PoseTrackingServo<FF>::pose_t
PoseTrackingServo<FF>::correctTargetPose(const raw_pose_t& offset) const
{
    auto	target_pose = targetPose();

  // Transform the target pose to planning frame unless defined in it.
    if (target_pose.header.frame_id != servoParameters().planning_frame)
    {
	auto	Tpt = tf2::eigenToTransform(getFrameTransform(
						target_pose.header.frame_id));
	Tpt.header.stamp    = target_pose.header.stamp;
	Tpt.header.frame_id = servoParameters().planning_frame;
	Tpt.child_frame_id  = target_pose.header.frame_id;
	tf2::doTransform(target_pose, target_pose, Tpt);
    }

    durations_.header	      = target_pose.header;
    durations_.target_pose_in = (ros::Time::now() -
				 durations_.header.stamp).toSec();

  // Correct target_pose by offset
    tf2::Transform	target_transform;
    tf2::fromMsg(target_pose.pose, target_transform);
    tf2::Transform	offset_transform;
    tf2::fromMsg(offset, offset_transform);
    tf2::toMsg(target_transform * offset_transform, target_pose.pose);

  // Apply input low-pass filter
    target_pose.pose = input_low_pass_filter_.filter(target_pose.pose);
    aist_utility::normalize(target_pose.pose.orientation);

    return target_pose;
}

template <class FF> void
PoseTrackingServo<FF>::calculatePoseError(const pose_t& target_pose,
					  const pose_t& actual_pose,
					  vector3_t& positional_error,
					  angle_axis_t& angular_error) const
{
    positional_error(0) = target_pose.pose.position.x
			- actual_pose.pose.position.x;
    positional_error(1) = target_pose.pose.position.y
			- actual_pose.pose.position.y;
    positional_error(2) = target_pose.pose.position.z
			- actual_pose.pose.position.z;

    Eigen::Quaterniond	q_desired;
    tf2::fromMsg(target_pose.pose.orientation, q_desired);
    Eigen::Quaterniond	q_actual;
    tf2::fromMsg(actual_pose.pose.orientation, q_actual);
    angular_error = q_desired * q_actual.inverse();
}

template <class FF> typename PoseTrackingServo<FF>::twist_t
PoseTrackingServo<FF>::calculateTwistCommand(const vector3_t& positional_error,
					     const angle_axis_t& angular_error)
{
  // use the shared pool to create a message more efficiently
    twist_t		twist_cmd;
    auto&		twist = twist_cmd.twist;
    const ros::Duration	dt(servoParameters().publish_period);

  // Get linear components of twist from PID controllers
    twist.linear.x = pids_[0].computeCommand(positional_error(0), dt);
    twist.linear.y = pids_[1].computeCommand(positional_error(1), dt);
    twist.linear.z = pids_[2].computeCommand(positional_error(2), dt);

  // Get angula components of twist from PID controllers
    const auto	ang_vel_magnitude = pids_[3].computeCommand(
						angular_error.angle(), dt);
    twist.angular.x = ang_vel_magnitude * angular_error.axis()[0];
    twist.angular.y = ang_vel_magnitude * angular_error.axis()[1];
    twist.angular.z = ang_vel_magnitude * angular_error.axis()[2];

    twist_cmd.header.frame_id = servoParameters().planning_frame;
    twist_cmd.header.stamp    = ros::Time::now();

    return twist_cmd;
}

template <class FF> void
PoseTrackingServo<FF>::doPostMotionReset()
{
    stopMotion();

  // Reset error integrals and previous errors of PID controllers
    for (auto&& pid : pids_)
	pid.reset();
}

template <class FF> void
PoseTrackingServo<FF>::stopMotion()
{
  // Send a 0 command to Servo to halt arm motion
    twist_t	twist_cmd;
    twist_cmd.header.frame_id = "";
    twist_cmd.header.stamp    = ros::Time::now();
    twist_cmd.twist.linear.x  = 0;
    twist_cmd.twist.linear.y  = 0;
    twist_cmd.twist.linear.z  = 0;
    twist_cmd.twist.angular.x = 0;
    twist_cmd.twist.angular.y = 0;
    twist_cmd.twist.angular.z = 0;

    publishTrajectory(twist_cmd, nullptr);
}

// Low-pass filter stuffs
#if defined(USE_BUTTERWORTH_LPF)
template <class FF> void
PoseTrackingServo<FF>::updateInputLowPassFilter(int half_order,
						double cutoff_frequency)
{
    input_low_pass_filter_half_order_	    = half_order;
    input_low_pass_filter_cutoff_frequency_ = cutoff_frequency;

    input_low_pass_filter_.initialize(input_low_pass_filter_half_order_,
				      input_low_pass_filter_cutoff_frequency_
				      *servoParameters().publish_period);
    input_low_pass_filter_.reset(actualPose().pose);
}
#else
template <class FF> void
PoseTrackingServo<FF>::updateInputLowPassFilter(double half_life)
{
    std::cerr << "*** Update: " << half_life << std::endl;

    input_low_pass_filter_.initialize(servoParameters().publish_period
				      /half_life);
    input_low_pass_filter_.reset(actualPose().pose);
}
#endif

// PID controller stuffs
template <class FF> void
PoseTrackingServo<FF>::updateLinearPIDs(double PIDConfig::* field,
					double value)
{
    linear_pid_config_.*field = value;
    for (size_t i = 0; i < 3; ++i)
	updatePID(linear_pid_config_, pids_[i]);
}

template <class FF> void
PoseTrackingServo<FF>::updateAngularPID(double PIDConfig::* field,
					double value)
{
    angular_pid_config_.*field = value;
    updatePID(angular_pid_config_, pids_[3]);
}

template <class FF> void
PoseTrackingServo<FF>::updatePID(const PIDConfig& pid_config, pid_t& pid)
{
    std::cerr << "*** updatePID: " << pid_config.k_p
	      << ',' << pid_config.k_i << ',' << pid_config.k_d << std::endl;

    pid.initPid(pid_config.k_p, pid_config.k_i, pid_config.k_d,
		pid_config.windup_limit, -pid_config.windup_limit, true);
}

// PoseTrackingAction stuffs
template <class FF> void
PoseTrackingServo<FF>::goalCB()
{
    resetServoStatus();
    resetTargetPose();
    static_cast<FF&>(*this).resetFeedForwardInput();

    stamp_goal_accepted_      = ros::Time::now();
    nframes_within_tolerance_ = 0;
    current_goal_	      = pose_tracking_srv_.acceptNewGoal();

  // If input low-pass filter was initialized with target pose,
  // the initial feed-forwarded pose calculated from the filter output
  // would be very different from the actual pose, which will cause
  // sudden jump of outgoing joint positions. Therefore we initialize
  // the filter with actual pose.
    if (current_goal_->reset_input_lpf)
	input_low_pass_filter_.reset(actualPose().pose);
    start();

    ROS_INFO_STREAM_NAMED(logname(), "(PoseTrackingServo) goal ACCEPTED");
}

template <class FF> void
PoseTrackingServo<FF>::preemptCB()
{
    doPostMotionReset();

    PoseTrackingResult	result;
    result.status = static_cast<int8_t>(servoStatus());
    pose_tracking_srv_.setPreempted(result);
    ROS_WARN_STREAM_NAMED(logname(), "(PoseTrackingServo) goal CANCELED");
}

// Target pose stuffs
template <class FF> void
PoseTrackingServo<FF>::targetPoseCB(const pose_cp& target_pose)
{
    const std::lock_guard<std::mutex>	lock(target_pose_mtx_);

    target_pose_ = target_pose;

    ROS_DEBUG_STREAM_NAMED(logname(),
			   "(PoseTrackingServo) target_pose received");
}

template <class FF> typename PoseTrackingServo<FF>::pose_t
PoseTrackingServo<FF>::targetPose() const
{
    const std::lock_guard<std::mutex>	lock(target_pose_mtx_);

    return *target_pose_;
}

template <class FF> bool
PoseTrackingServo<FF>::haveRecentTargetPose(const ros::Duration& timeout) const
{
    const std::lock_guard<std::mutex>	lock(target_pose_mtx_);

    return (target_pose_ != nullptr &&
	    ros::Time::now() - target_pose_->header.stamp < timeout);
}

template <class FF> void
PoseTrackingServo<FF>::resetTargetPose()
{
    const std::lock_guard<std::mutex>	lock(target_pose_mtx_);

    target_pose_ = nullptr;
}

// Current pose stuffs
template <class FF> typename PoseTrackingServo<FF>::pose_t
PoseTrackingServo<FF>::actualPose() const
{
    return tf2::toMsg(tf2::Stamped<Eigen::Isometry3d>(
			  getFrameTransform(servoParameters().ee_frame_name),
			  getRobotStateStamp(),
			  servoParameters().planning_frame));
}

// FeedForward pose stuffs
template <class FF> void
PoseTrackingServo<FF>::publishFeedForwardPose(const pose_t& ff_pose) const
{
    ff_pose_debug_pub_.publish(ff_pose);
}

template <class FF> void
PoseTrackingServo<FF>::publishFeedForwardPose(std::nullptr_t) const
{
}

}	// namespace aist_moveit_servo
