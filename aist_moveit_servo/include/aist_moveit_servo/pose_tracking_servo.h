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
#include <aist_utility/butterworth_lpf.h>
#include <aist_utility/geometry_msgs.h>
#include <aist_moveit_servo/servo.h>
#include <aist_moveit_servo/status_codes.h>
#include <aist_moveit_servo/PoseTrackingAction.h>
#include <aist_moveit_servo/make_shared_from_pool.h>

// Conventions:
// Calculations are done in the planning_frame unless otherwise noted.

namespace aist_moveit_servo
{
/************************************************************************
*  class PoseTrackingServo<FF>						*
************************************************************************/
template <class FF>
class PoseTrackingServo
{
  private:
    using server_t	 = actionlib::SimpleActionServer<PoseTrackingAction>;
    using goal_cp	 = boost::shared_ptr<const server_t::Goal>;
    using ddr_t		 = ddynamic_reconfigure::DDynamicReconfigure;
    using servo_status_t = aist_moveit_servo::StatusCode;
    using int8_cp	 = std_msgs::Int8ConstPtr;
    using twist_t	 = geometry_msgs::TwistStamped;
    using twist_cp	 = geometry_msgs::TwistStampedConstPtr;
    using pose_t	 = geometry_msgs::PoseStamped;
    using pose_cp	 = geometry_msgs::PoseStampedConstPtr;
    using raw_pose_t	 = geometry_msgs::Pose;
    using vector3_t	 = Eigen::Vector3d;
    using angle_axis_t	 = Eigen::AngleAxisd;
    using pid_t		 = control_toolbox::Pid;
    using lpf_t		 = aist_utility::ButterworthLPF<double, raw_pose_t>;
    using feed_forward_t = FF;
    
    struct PIDConfig
    {
	double	k_p	     = 1;
	double	k_i	     = 0;
	double	k_d	     = 0;
	double	windup_limit = 0.1;
    };

  public:
		PoseTrackingServo(const ros::NodeHandle& nh,
				  const std::string& logname)		;
		~PoseTrackingServo()					;

    void	run()							;

  private:
    void	readROSParams()						;
    const ServoParameters&
		getServoParameters() const
		{
		    return servo_.getParameters();
		}
    ros::Duration
		expectedCycleTime() const
		{
		    return ros::Duration(getServoParameters().publish_period);
		}

    void	tick()							;
    pose_t	correctTargetPose(const raw_pose_t& offset)	const	;
    void	calculatePoseError(const pose_t& target_pose,
				   vector3_t& positional_error,
				   angle_axis_t& angular_error)	const	;

    twist_cp	calculateTwistCommand(const vector3_t& positional_error,
				      const angle_axis_t& angular_error);
    void	stopMotion()						;
    void	doPostMotionReset()					;

  // Input low-pass filter stuffs
    void	updateInputLowPassFilter(int half_order,
					 double cutoff_frequency)	;

  // PID stuffs
    void	updatePositionPIDs(double PIDConfig::* field,
				   double value)			;
    void	updateOrientationPID(double PIDConfig::* field,
				     double value)			;
    void	updatePID(const PIDConfig& pid_config, pid_t& pid)	;

  // PostTrackingAction stuffs
    void	goalCB()						;
    void	preemptCB()						;

  // Servo status stuffs
    void	servoStatusCB(const int8_cp& servo_status)		;

  // Target pose stuffs
    void	targetPoseCB(const pose_cp& target_pose)		;
    bool	haveRecentTargetPose(const ros::Duration& timeout) const;
    void	resetTargetPose()					;

  private:
    ros::NodeHandle		nh_;
    const std::string		logname_;
    
    Servo			servo_;
    servo_status_t		servo_status_;

    ros::ServiceClient		reset_servo_status_;
    const ros::Subscriber	servo_status_sub_;
    const ros::Subscriber	target_pose_sub_;
    const ros::Publisher	twist_pub_;
    const ros::Publisher	predictive_pose_pub_;
    const ros::Publisher	target_pose_debug_pub_;
    const ros::Publisher	ee_pose_debug_pub_;
    DurationArray&		durations_;

    feed_forward_t		ff_;

  // Action server stuffs
    server_t			pose_tracking_srv_;
    goal_cp			current_goal_;

  // Dynamic reconfigure server
    ddr_t			ddr_;

  // Filters for input target pose
    int				input_low_pass_filter_half_order_;
    double			input_low_pass_filter_cutoff_frequency_;
    lpf_t			input_low_pass_filter_;

  // PIDs
    PIDConfig			linear_pid_config_;
    PIDConfig			angular_pid_config_;
    std::array<pid_t, 4>	pids_;

  // Servo inputs
    pose_t			target_pose_;
    mutable std::mutex		input_mtx_;
};

template <class FF>
PoseTrackingServo<FF>::PoseTrackingServo(const ros::NodeHandle& nh,
					 const std::string& logname)
    :nh_(nh),
     logname_(logname),
     servo_(nh_, createPlanningSceneMonitor("robot_description")),
     servo_status_(servo_status_t::INVALID),

     reset_servo_status_(nh_.serviceClient<std_srvs::Empty>(
			     "reset_servo_status")),
     servo_status_sub_(nh_.subscribe(getServoParameters().status_topic, 1,
				     &PoseTrackingServo::servoStatusCB, this)),
     target_pose_sub_(nh_.subscribe(
			  "/target_pose", 1,
			  &PoseTrackingServo::targetPoseCB, this,
			  ros::TransportHints().reliable().tcpNoDelay(true))),
     twist_pub_(nh_.advertise<twist_t>(
		    getServoParameters().cartesian_command_in_topic, 1)),
     predictive_pose_pub_(getServoParameters().predictive_pose_topic.empty() ?
			  ros::Publisher() :
			  nh_.advertise<pose_t>(
			      getServoParameters().predictive_pose_topic, 1)),
     target_pose_debug_pub_(nh_.advertise<pose_t>("desired_pose", 1)),
     ee_pose_debug_pub_(nh_.advertise<pose_t>("actual_pose", 1)),
     durations_(servo_.durations()),

     ff_(nh_),

     pose_tracking_srv_(nh_, "pose_tracking", false),
     current_goal_(nullptr),
     ddr_(ros::NodeHandle(nh_, "pose_tracking")),

     input_low_pass_filter_half_order_(3),
     input_low_pass_filter_cutoff_frequency_(7.0),
     input_low_pass_filter_(input_low_pass_filter_half_order_,
			    input_low_pass_filter_cutoff_frequency_ *
			    expectedCycleTime().toSec()),

     linear_pid_config_(),
     angular_pid_config_(),
     pids_(),

     target_pose_(),
     input_mtx_()
{
    readROSParams();

  // Initialize input lowpass-filter
    input_low_pass_filter_.initialize(input_low_pass_filter_half_order_,
				      input_low_pass_filter_cutoff_frequency_ *
				      expectedCycleTime().toSec());

  // Initialize PID controllers
    for (size_t i = 0; i < 3; ++i)
	updatePID(linear_pid_config_, pids_[i]);
    updatePID(angular_pid_config_, pids_[3]);

  // Setup action server
    pose_tracking_srv_.registerGoalCallback(boost::bind(
						&PoseTrackingServo::goalCB,
						this));
    pose_tracking_srv_.registerPreemptCallback(boost::bind(
						 &PoseTrackingServo::preemptCB,
						 this));
    pose_tracking_srv_.start();

  // Setup dynamic reconfigure server
    ddr_.registerVariable<int>("input_lowpass_filter_half_order",
			       input_low_pass_filter_half_order_,
			       boost::bind(
				   &PoseTrackingServo
				   ::updateInputLowPassFilter,
				   this, _1,
				   input_low_pass_filter_cutoff_frequency_),
			       "Half order of input low pass filter", 1, 5);
    ddr_.registerVariable<double>("input_lowpass_filter_cutoff_frequency",
				  input_low_pass_filter_cutoff_frequency_,
				  boost::bind(
				      &PoseTrackingServo
				      ::updateInputLowPassFilter,
				      this, input_low_pass_filter_half_order_,
				      _1),
				  "Cutoff frequency of input low pass filter",
				  0.5, 100.0);
    ddr_.registerVariable<double>("linear_proportional_gain",
				  linear_pid_config_.k_p,
				  boost::bind(&PoseTrackingServo
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for translation",
				  0.5, 300.0);
    ddr_.registerVariable<double>("linear_integral_gain",
				  linear_pid_config_.k_i,
				  boost::bind(&PoseTrackingServo
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for translation",
				  0.0, 20.0);
    ddr_.registerVariable<double>("linear_derivative_gain",
				  linear_pid_config_.k_d,
				  boost::bind(&PoseTrackingServo
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_d, _1),
				  "Derivative gain for translation",
				  0.0, 20.0);

    ddr_.registerVariable<double>("angular_proportinal_gain",
				  angular_pid_config_.k_p,
				  boost::bind(&PoseTrackingServo
					      ::updateOrientationPID,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for rotation",
				  0.5, 300.0);
    ddr_.registerVariable<double>("angular_integral_gain",
				  angular_pid_config_.k_i,
				  boost::bind(&PoseTrackingServo
					      ::updateOrientationPID,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for rotation",
				  0.0, 20.0);
    ddr_.registerVariable<double>("angular_derivative_gain",
				  angular_pid_config_.k_d,
				  boost::bind(&PoseTrackingServo
					      ::updateOrientationPID,
					      this, &PIDConfig::k_d, _1),
				  "Derivative gain for rotation",
				  0.0, 20.0);
    ddr_.publishServicesTopics();

    ROS_INFO_STREAM_NAMED(logname_, "(PoseTrackingServo) server started");
}

template <class FF>
PoseTrackingServo<FF>::~PoseTrackingServo()
{
    stopMotion();
}

template <class FF> void
PoseTrackingServo<FF>::run()
{
    ros::AsyncSpinner	spinner(8);
    spinner.start();

    for (auto cycle_time = expectedCycleTime(); ros::ok(); cycle_time.sleep())
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
PoseTrackingServo<FF>::readROSParams()
{
  // Optional parameter sub-namespace specified in the launch file.
  // All other parameters will be read from this namespace.
  // If parameters have been loaded into sub-namespace
  // within the node namespace, append the parameter namespace
  // to load the parameters correctly.
    std::string	parameter_ns;
    auto	nh = (nh_.getParam("parameter_ns", parameter_ns) ?
		      ros::NodeHandle(nh_, parameter_ns) : nh_);

  // Setup input low-pass filter
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(logname_, nh,
				      "input_low_pass_filter_half_order",
				      input_low_pass_filter_half_order_);
    error += !rosparam_shortcuts::get(logname_, nh,
				      "input_low_pass_filter_cutoff_frequency",
				      input_low_pass_filter_cutoff_frequency_);

  // Setup PID configurations
    double	windup_limit;
    error += !rosparam_shortcuts::get(logname_, nh, "windup_limit",
				      windup_limit);
    linear_pid_config_.windup_limit  = windup_limit;
    angular_pid_config_.windup_limit = windup_limit;

    error += !rosparam_shortcuts::get(logname_, nh, "linear_proportional_gain",
				      linear_pid_config_.k_p);
    error += !rosparam_shortcuts::get(logname_, nh, "linear_integral_gain",
				      linear_pid_config_.k_i);
    error += !rosparam_shortcuts::get(logname_, nh, "linear_derivative_gain",
				      linear_pid_config_.k_d);
    error += !rosparam_shortcuts::get(logname_, nh, "angular_proportional_gain",
				      angular_pid_config_.k_p);
    error += !rosparam_shortcuts::get(logname_, nh, "angular_integral_gain",
				      angular_pid_config_.k_i);
    error += !rosparam_shortcuts::get(logname_, nh, "angular_derivative_gain",
				      angular_pid_config_.k_d);

    rosparam_shortcuts::shutdownIfError(ros::this_node::getName(), error);
}

template <class FF> void
PoseTrackingServo<FF>::tick()
{
    durations_.tick_begin = (ros::Time::now() -
			     durations_.header.stamp).toSec();

    if (!pose_tracking_srv_.isActive())
	return;

  // Check that servo status is not emergency.
    switch (servo_status_)
    {
      case servo_status_t::HALT_FOR_SINGULARITY:
      case servo_status_t::HALT_FOR_COLLISION:
      case servo_status_t::JOINT_BOUND:
      {
	doPostMotionReset();
	PoseTrackingResult	result;
	result.status = static_cast<int8_t>(servo_status_);
	pose_tracking_srv_.setAborted(result);
	ROS_ERROR_STREAM_NAMED(logname_, "(PoseTrackingServo) goal ABORTED["
			       << SERVO_STATUS_CODE_MAP.at(servo_status_)
			       << ']');
	return;
      }

      default:
	break;
    }

  // Check that target pose is recent enough.
    if (!haveRecentTargetPose(current_goal_->timeout) ||
	!ff_.haveRecentInput(current_goal_->timeout))
    {
    	doPostMotionReset();
	PoseTrackingResult	result;
	result.status = PoseTrackingResult::INPUT_TIMEOUT;
    	pose_tracking_srv_.setAborted(result);
        ROS_ERROR_STREAM_NAMED(logname_, "(PoseTrackingServo) goal ABORTED["
    			       << "The target pose was not updated recently."
			       << ']');

    	return;
    }

  // Correct target pose by offset specified by the goal.
    const auto	target_pose = correctTargetPose(current_goal_->target_offset);
    
  // Add predictive term to target pose and publish as feed-forward command.
    ff_.publishPrediction(target_pose, expectedCycleTime());
    
  // Compute positional and angular errors.
    vector3_t		positional_error;
    angle_axis_t	angular_error;
    calculatePoseError(target_pose, positional_error, angular_error);

  // Check if goal tolerance is satisfied.
    if (std::abs(positional_error(0)) <
	current_goal_->positional_tolerance[0] &&
	std::abs(positional_error(1)) <
	current_goal_->positional_tolerance[1] &&
	std::abs(positional_error(2)) <
	current_goal_->positional_tolerance[2] &&
	std::abs(angular_error.angle()) < current_goal_->angular_tolerance)
    {
	doPostMotionReset();
	PoseTrackingResult	result;
	result.status = PoseTrackingResult::NO_ERROR;
	pose_tracking_srv_.setSucceeded(result);
	ROS_INFO_STREAM_NAMED(logname_, "(PoseTrackingServo) goal SUCCEEDED");

	return;
    }

  // Publish tracking result as feedback.
    PoseTrackingFeedback	feedback;
    feedback.positional_error[0] = positional_error(0);
    feedback.positional_error[1] = positional_error(1);
    feedback.positional_error[2] = positional_error(2);
    feedback.angular_error	 = angular_error.angle();
    feedback.status		 = static_cast<int8_t>(servo_status_);
    pose_tracking_srv_.publishFeedback(feedback);

  // Compute servo command from PID controller output and send it
  // to the Servo object, for execution
    twist_pub_.publish(calculateTwistCommand(positional_error, angular_error));

    durations_.twist_out = (ros::Time::now() -
			    durations_.header.stamp).toSec();
}

template <class FF> typename PoseTrackingServo<FF>::pose_t
PoseTrackingServo<FF>::correctTargetPose(const raw_pose_t& offset) const
{
    pose_t	target_pose;
    {
	const std::lock_guard<std::mutex>	lock(input_mtx_);

	target_pose = target_pose_;
    }

  // Apply input low-pass filter
    target_pose.pose = input_low_pass_filter_.filter(target_pose.pose);
    aist_utility::normalize(target_pose.pose.orientation);

  // Correct target_pose by offset
    tf2::Transform	target_transform;
    tf2::fromMsg(target_pose.pose, target_transform);
    tf2::Transform	offset_transform;
    tf2::fromMsg(offset, offset_transform);
    tf2::toMsg(target_transform * offset_transform, target_pose.pose);

    return target_pose;
}

template <class FF> void
PoseTrackingServo<FF>::calculatePoseError(const pose_t& target_pose,
					  vector3_t& positional_error,
					  angle_axis_t& angular_error) const
{
    const auto	Tpe = servo_.getEEFrameTransform();
    positional_error(0) = target_pose.pose.position.x - Tpe.translation()(0);
    positional_error(1) = target_pose.pose.position.y - Tpe.translation()(1);
    positional_error(2) = target_pose.pose.position.z - Tpe.translation()(2);

    Eigen::Quaterniond	q_desired;
    tf2::convert(target_pose.pose.orientation, q_desired);
    angular_error = q_desired * Eigen::Quaterniond(Tpe.rotation()).inverse();

  // For debugging
    target_pose_debug_pub_.publish(target_pose);
    const auto	ee_pose = tf2::toMsg(tf2::Stamped<Eigen::Isometry3d>(
					 Tpe, ros::Time::now(),
					 getServoParameters().planning_frame));
    ee_pose_debug_pub_.publish(ee_pose);
}

template <class FF> typename PoseTrackingServo<FF>::twist_cp
PoseTrackingServo<FF>::calculateTwistCommand(const vector3_t& positional_error,
					     const angle_axis_t& angular_error)
{
  // use the shared pool to create a message more efficiently
    const auto	msg = moveit::util::make_shared_from_pool<twist_t>();
    {
	const std::lock_guard<std::mutex>	lock(input_mtx_);

	msg->header.frame_id = target_pose_.header.frame_id;
    }

  // Get twist components from PID controllers
    auto&	twist = msg->twist;
    const auto	dt = expectedCycleTime();
    twist.linear.x = pids_[0].computeCommand(positional_error(0), dt);
    twist.linear.y = pids_[1].computeCommand(positional_error(1), dt);
    twist.linear.z = pids_[2].computeCommand(positional_error(2), dt);

  // Anglular components
    const auto	ang_vel_magnitude = pids_[3].computeCommand(
						angular_error.angle(), dt);
    twist.angular.x = ang_vel_magnitude * angular_error.axis()[0];
    twist.angular.y = ang_vel_magnitude * angular_error.axis()[1];
    twist.angular.z = ang_vel_magnitude * angular_error.axis()[2];

    msg->header.stamp = ros::Time::now();

    return msg;
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
    const auto	msg = moveit::util::make_shared_from_pool<twist_t>();
    {
	const std::lock_guard<std::mutex>	lock(input_mtx_);

	msg->header.frame_id = target_pose_.header.frame_id;
    }
    msg->header.stamp = ros::Time::now();
    twist_pub_.publish(msg);
}

// Low-pass filter stuffs
template <class FF> void
PoseTrackingServo<FF>::updateInputLowPassFilter(int half_order,
						double cutoff_frequency)
{
    input_low_pass_filter_half_order_	    = half_order;
    input_low_pass_filter_cutoff_frequency_ = cutoff_frequency;

    input_low_pass_filter_.initialize(input_low_pass_filter_half_order_,
				      input_low_pass_filter_cutoff_frequency_*
				      expectedCycleTime().toSec());

    input_low_pass_filter_.reset(target_pose_.pose);
}

// PID controller stuffs
template <class FF> void
PoseTrackingServo<FF>::updatePositionPIDs(double PIDConfig::* field,
					  double value)
{
    const std::lock_guard<std::mutex>	lock(input_mtx_);

    linear_pid_config_.*field = value;
    for (size_t i = 0; i < 3; ++i)
	updatePID(linear_pid_config_, pids_[i]);
}

template <class FF> void
PoseTrackingServo<FF>::updateOrientationPID(double PIDConfig::* field,
					    double value)
{
    const std::lock_guard<std::mutex>	lock(input_mtx_);

    angular_pid_config_.*field = value;
    updatePID(angular_pid_config_, pids_[3]);
}

template <class FF> void
PoseTrackingServo<FF>::updatePID(const PIDConfig& pid_config, pid_t& pid)
{
    pid.initPid(pid_config.k_p, pid_config.k_i, pid_config.k_d,
		pid_config.windup_limit, -pid_config.windup_limit, true);
}

// PoseTrackingAction stuffs
template <class FF> void
PoseTrackingServo<FF>::goalCB()
{
    resetTargetPose();
    ff_.resetInput();

  // Wait a bit for a target pose message to arrive.
  // The target pose may get updated by new messages as the robot moves
  // (in a callback function).
    const ros::Duration	DEFAULT_INPUT_TIMEOUT(0.5);
    
    for (const auto start_time = ros::Time::now();
	 ros::Time::now() - start_time < DEFAULT_INPUT_TIMEOUT;
	 ros::Duration(0.001).sleep())
    {
	if (haveRecentTargetPose(DEFAULT_INPUT_TIMEOUT) &&
	    ff_.haveRecentInput(DEFAULT_INPUT_TIMEOUT))
	{
	    input_low_pass_filter_.reset(target_pose_.pose);

	    std_srvs::Empty	empty;
	    reset_servo_status_.call(empty);
	    servo_.start();

	    current_goal_ = pose_tracking_srv_.acceptNewGoal();
	    ROS_INFO_STREAM_NAMED(logname_,
				  "(PoseTrackingServo) goal ACCEPTED");

	    if (pose_tracking_srv_.isPreemptRequested())
		preemptCB();

	    return;
	}
    }

  // No target pose available recently.
  // Once accept the pending goal and then abort it immediately.
    current_goal_ = pose_tracking_srv_.acceptNewGoal();
    PoseTrackingResult	result;
    result.status = static_cast<int8_t>(servo_status_);
    pose_tracking_srv_.setAborted(result);

    ROS_ERROR_STREAM_NAMED(logname_,
			   "(PoseTrackingServo) Cannot accept goal because no target pose available recently.");
}

template <class FF> void
PoseTrackingServo<FF>::preemptCB()
{
    doPostMotionReset();
    PoseTrackingResult	result;
    result.status = static_cast<int8_t>(servo_status_);
    pose_tracking_srv_.setPreempted(result);
    ROS_WARN_STREAM_NAMED(logname_, "(PoseTrackingServo) goal CANCELED");
}

// Servo status stuffs
template <class FF> void
PoseTrackingServo<FF>::servoStatusCB(const int8_cp& servo_status)
{
    servo_status_ = static_cast<servo_status_t>(servo_status->data);
}

// Target pose stuffs
template <class FF> void
PoseTrackingServo<FF>::targetPoseCB(const pose_cp& target_pose)
{
    const std::lock_guard<std::mutex>	lock(input_mtx_);

    target_pose_ = *target_pose;

  // Assure non-zero stamp which will cause the haveRecentTargetPose check
  // to fail servo motions.
    if (target_pose_.header.stamp == ros::Time(0))
	target_pose_.header.stamp = ros::Time::now();

  // If the target pose is not defined in planning frame, transform it.
    if (target_pose_.header.frame_id != getServoParameters().planning_frame)
    {
	auto Tpt = tf2::eigenToTransform(servo_.getFrameTransform(
					     target_pose_.header.frame_id));
	Tpt.header.stamp    = target_pose_.header.stamp;
	Tpt.header.frame_id = getServoParameters().planning_frame;
	Tpt.child_frame_id  = target_pose_.header.frame_id;
	tf2::doTransform(target_pose_, target_pose_, Tpt);
    }

    durations_.header	      = target_pose_.header;
    durations_.target_pose_in = (ros::Time::now() -
				 durations_.header.stamp).toSec();
}

template <class FF> bool
PoseTrackingServo<FF>::haveRecentTargetPose(const ros::Duration& timeout) const
{
    const std::lock_guard<std::mutex>	lock(input_mtx_);

    return (ros::Time::now() - target_pose_.header.stamp < timeout);
}

template <class FF> void
PoseTrackingServo<FF>::resetTargetPose()
{
    const std::lock_guard<std::mutex>	lock(input_mtx_);

    target_pose_	      = pose_t();
    target_pose_.header.stamp = ros::Time(0);
}

}	// namespace aist_moveit_servo
