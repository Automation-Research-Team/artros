/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *  \file	joint_trajectory_servo.cpp
 *  \brief	ROS pose tracker of aist_controllers::PoseTracking type
 */
#include <atomic>
#include <boost/optional.hpp>
#include <control_toolbox/pid.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <actionlib/server/simple_action_server.h>
#include <aist_controllers/PoseTrackingAction.h>
#include <aist_utility/butterworth_lpf.h>

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.

namespace
{
constexpr char		LOGNAME[]	  = "pose_tracking_servo";
constexpr double	DEFAULT_LOOP_RATE = 100;	// Hz
constexpr double	ROS_STARTUP_WAIT  = 10;		// sec
const	  ros::Duration	DEFAULT_INPUT_TIMEOUT{0.5};	// sec
}	// anonymous namespace

namespace geometry_msgs
{
Pose
operator +(const Pose& a, const Pose& b)
{
    Pose	ret;
    ret.position.x    = a.position.x	+ b.position.x;
    ret.position.y    = a.position.y	+ b.position.y;
    ret.position.z    = a.position.z	+ b.position.z;
    ret.orientation.x = a.orientation.x	+ b.orientation.x;
    ret.orientation.y = a.orientation.y	+ b.orientation.y;
    ret.orientation.z = a.orientation.z	+ b.orientation.z;
    ret.orientation.w = a.orientation.w	+ b.orientation.w;

    return ret;
}

Pose
operator *(double c, const Pose& a)
{
    Pose	ret;
    ret.position.x    = c * a.position.x;
    ret.position.y    = c * a.position.y;
    ret.position.z    = c * a.position.z;
    ret.orientation.x = c * a.orientation.x;
    ret.orientation.y = c * a.orientation.y;
    ret.orientation.z = c * a.orientation.z;
    ret.orientation.w = c * a.orientation.w;

    return ret;
}
}	// namespace geometry_msgs

namespace aist_controllers
{
namespace
{
/************************************************************************
*  static functions							*
************************************************************************/
std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Pose& pose)
{
    return out << pose.position.x << ' '
	       << pose.position.y << ' '
	       << pose.position.z << ';'
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
    return out << twist.linear.x << ' '
	       << twist.linear.y << ' '
	       << twist.linear.z << ';'
	       << twist.angular.x << ' '
	       << twist.angular.y << ' '
	       << twist.angular.z;
}

void
normalize(geometry_msgs::Quaternion& q)
{
    const auto	norm1 = 1/std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    q.x *= norm1;
    q.y *= norm1;
    q.z *= norm1;
    q.w *= norm1;
}

}	// anonymous namespace

/************************************************************************
*  class JointTrajectoryServo						*
************************************************************************/
class JointTrajectoryServo
{
  private:
    using planning_scene_monitor_t
			 = planning_scene_monitor::PlanningSceneMonitor;
    using planning_scene_monitor_p
			 = planning_scene_monitor::PlanningSceneMonitorPtr;
    using server_t	 = actionlib::SimpleActionServer<PoseTrackingAction>;

    struct PIDConfig
    {
      // Default values
	double dt	    = 0.001;
	double k_p	    = 1;
	double k_i	    = 0;
	double k_d	    = 0;
	double windup_limit = 0.1;
    };

  public:
		JointTrajectoryServo(const ros::NodeHandle& nh)		;
		~JointTrajectoryServo()					;

    void	run()							;
    void	tick()							;
    const ros::Rate&
		loop_rate()					const	;

  private:
    static planning_scene_monitor_p
		createPlanningSceneMonitor(
		    const std::string& robot_description)		;
    void	readROSParams()						;
    void	robotStateCB(const std_msgs::Int8ConstPtr& msg)	;
    void	targetPoseCB(const geometry_msgs::PoseStampedConstPtr& msg);
    void	goalCB()						;
    void	preemptCB()						;
    void	calculatePoseError(const geometry_msgs::Pose& offset,
				   Eigen::Vector3d& positional_error,
				   Eigen::AngleAxisd& angular_error) const;

    geometry_msgs::TwistStampedConstPtr
		calculateTwistCommand(const Eigen::Vector3d& positional_error,
				      const Eigen::AngleAxisd& angular_error);
    void	stopMotion()						;
    void	doPostMotionReset()					;

  // Input low-pass filter stuffs
    void	updateInputLowPassFilter(int half_order,
					 double cutoff_frequency)	;

  // PID stuffs
    void	initializePID(const PIDConfig& pid_config,
			      std::vector<control_toolbox::Pid>& pid_vector);
    void	updatePositionPIDs(double PIDConfig::* field,
				   double value)			;
    void	updateOrientationPID(double PIDConfig::* field,
				     double value)			;
    void	getPIDErrors(double& x_error,
			     double& y_error,
			     double& z_error,
			     double& orientation_error)			;

  // Target pose stuffs
    void	resetTargetPose()					;
    bool	haveRecentTargetPose(const ros::Duration& timeout) const;

  // End-effector pose stuffs
    bool	haveRecentEndEffectorPose(const ros::Duration& timeout)	const;

  private:
    ros::NodeHandle				_nh;

    planning_scene_monitor_p			_planning_scene_monitor;
    std::unique_ptr<moveit_servo::Servo>	servo_;
    servo_status_t				servo_status_;

    std::string					_move_group_name;
    const moveit::core::JointModelGroup*	_joint_model_group;

    ros::ServiceClient				reset_servo_status_;
    ros::Subscriber				servo_status_sub_;
    ros::Subscriber				_target_pose_sub;
    ros::Publisher				twist_stamped_pub_;
    ros::Publisher				target_pose_pub_;  // for debug
    ros::Publisher				ee_pose_pub_;	   // for debug
    ros::Rate					_loop_rate;
    DurationArray&				durations_;

  // Action server stuffs
    server_t					_tracker_srv;
    boost::shared_ptr<const server_t::Goal>	_current_goal;

  // Dynamic reconfigure server
    ddynamic_reconfigure::DDynamicReconfigure	_ddr;

    tf2_ros::Buffer				_transform_buffer;
    tf2_ros::TransformListener			_transform_listener;

  // Filters for input target pose
    int				_input_low_pass_filter_half_order;
    double			_input_low_pass_filter_cutoff_frequency;
    aist_utility::ButterworthLPF<double, geometry_msgs::Pose>
				_input_low_pass_filter;

  // PIDs
    std::vector<control_toolbox::Pid>		cartesian_position_pids_;
    std::vector<control_toolbox::Pid>		cartesian_orientation_pids_;
    PIDConfig					x_pid_config_,
						y_pid_config_,
						z_pid_config_,
						angular_pid_config_;

  // Transforms w.r.t. planning_frame_
    std::string					_planning_frame;
    Eigen::Isometry3d				_ee_frame_transform;
    ros::Time					_ee_frame_transform_stamp;
    geometry_msgs::PoseStamped			_target_pose;
    mutable std::mutex				_target_pose_mtx;
};

JointTrajectoryServo::JointTrajectoryServo(const ros::NodeHandle& nh)
    :_nh(nh),
     _planning_scene_monitor(createPlanningSceneMonitor("robot_description")),
     servo_(new moveit_servo::Servo(_nh, _planning_scene_monitor)),
     servo_status_(servo_status_t::INVALID),

     _move_group_name(),
     _joint_model_group(nullptr),

     reset_servo_status_(_nh.serviceClient<std_srvs::Empty>(
			     "reset_servo_status")),
     servo_status_sub_(_nh.subscribe(servo_->getParameters().status_topic, 1,
				     &JointTrajectoryServo::servoStatusCB, this)),
     _target_pose_sub(),
     twist_stamped_pub_(),
     target_pose_pub_(_nh.advertise<geometry_msgs::PoseStamped>(
			  "target_pose_debug", 1)),
     ee_pose_pub_(_nh.advertise<geometry_msgs::PoseStamped>(
		      "ee_pose_debug", 1)),
     _loop_rate(DEFAULT_LOOP_RATE),
     durations_(servo_->durations()),

     _tracker_srv(_nh, "pose_tracking", false),
     _current_goal(nullptr),
     _ddr(ros::NodeHandle(_nh, "pose_tracking")),

     _transform_buffer(),
     _transform_listener(_transform_buffer),

     _input_low_pass_filter_half_order(3),
     _input_low_pass_filter_cutoff_frequency(7.0),
     _input_low_pass_filter(_input_low_pass_filter_half_order,
			    _input_low_pass_filter_cutoff_frequency *
			    _loop_rate.expectedCycleTime().toSec()),

     cartesian_position_pids_(),
     cartesian_orientation_pids_(),
     x_pid_config_(),
     y_pid_config_(),
     z_pid_config_(),

     _planning_frame(),
     _ee_frame_transform(),
     _ee_frame_transform_stamp(),
     _target_pose(),
     _target_pose_mtx()
{
    readROSParams();

  // Initialize input lowpass-filter
    _input_low_pass_filter.initialize(_input_low_pass_filter_half_order,
				      _input_low_pass_filter_cutoff_frequency *
				      _loop_rate.expectedCycleTime().toSec());

  // Initialize PID controllers
    initializePID(x_pid_config_,       cartesian_position_pids_);
    initializePID(y_pid_config_,       cartesian_position_pids_);
    initializePID(z_pid_config_,       cartesian_position_pids_);
    initializePID(angular_pid_config_, cartesian_orientation_pids_);

  // Connect to Servo ROS interfaces
    _target_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
    			"/target_pose", 1,
    			&JointTrajectoryServo::targetPoseCB, this,
    			ros::TransportHints().reliable().tcpNoDelay(true));

  // Publish outgoing twist commands to the Servo object
    twist_stamped_pub_ = _nh.advertise<geometry_msgs::TwistStamped>(
    			   servo_->getParameters().cartesian_command_in_topic,
    			   1);

  // Setup action server
    _tracker_srv.registerGoalCallback(boost::bind(&JointTrajectoryServo::goalCB,
						  this));
    _tracker_srv.registerPreemptCallback(boost::bind(
					     &JointTrajectoryServo::preemptCB,
					     this));
    _tracker_srv.start();

  // Setup dynamic reconfigure server
    _ddr.registerVariable<int>("input_lowpass_filter_half_order",
			       _input_low_pass_filter_half_order,
			       boost::bind(
				   &JointTrajectoryServo
				   ::updateInputLowPassFilter,
				   this, _1,
				   _input_low_pass_filter_cutoff_frequency),
			       "Half order of input low pass filter", 1, 5);
    _ddr.registerVariable<double>("input_lowpass_filter_cutoff_frequency",
				  _input_low_pass_filter_cutoff_frequency,
				  boost::bind(
				      &JointTrajectoryServo
				      ::updateInputLowPassFilter,
				      this, _input_low_pass_filter_half_order,
				      _1),
				  "Cutoff frequency of input low pass filter",
				  0.5, 100.0);
    _ddr.registerVariable<double>("linear_proportional_gain",
				  x_pid_config_.k_p,
				  boost::bind(&JointTrajectoryServo
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for translation",
				  0.5, 300.0);
    _ddr.registerVariable<double>("linear_integral_gain",
				  x_pid_config_.k_i,
				  boost::bind(&JointTrajectoryServo
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for translation",
				  0.0, 20.0);
    _ddr.registerVariable<double>("linear_derivative_gain",
				  x_pid_config_.k_d,
				  boost::bind(&JointTrajectoryServo
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_d, _1),
				  "Derivative gain for translation",
				  0.0, 20.0);

    _ddr.registerVariable<double>("angular_proportinal_gain",
				  angular_pid_config_.k_p,
				  boost::bind(&JointTrajectoryServo
					      ::updateOrientationPID,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for rotation",
				  0.5, 300.0);
    _ddr.registerVariable<double>("angular_integral_gain",
				  angular_pid_config_.k_i,
				  boost::bind(&JointTrajectoryServo
					      ::updateOrientationPID,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for rotation",
				  0.0, 20.0);
    _ddr.registerVariable<double>("angular_derivative_gain",
				  angular_pid_config_.k_d,
				  boost::bind(&JointTrajectoryServo
					      ::updateOrientationPID,
					      this, &PIDConfig::k_d, _1),
				  "Derivative gain for rotation",
				  0.0, 20.0);
    _ddr.publishServicesTopics();

    ROS_INFO_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) server started");
}

JointTrajectoryServo::~JointTrajectoryServo()
{
    stopMotion();
}

void
JointTrajectoryServo::run()
{
    ros::AsyncSpinner	spinner(8);
    spinner.start();

    while (ros::ok())
    {
	tick();
	_loop_rate.sleep();
    }

    spinner.stop();
    ros::waitForShutdown();
}

void
JointTrajectoryServo::tick()
{
    durations_.tick_begin = (ros::Time::now() -
			     durations_.header.stamp).toSec();

    if (!_tracker_srv.isActive())
	return;

  // Continue sending PID controller output to Servo
  // until one of the following conditions is met:
  // - Servo status is not in emergency
  // - Target pose becomes outdated
  // - Command frame transform becomes outdated
  // - Goal tolerance is satisfied

  // Check servo status
    switch (servo_status_)
    {
      case servo_status_t::HALT_FOR_SINGULARITY:
      case servo_status_t::HALT_FOR_COLLISION:
      case servo_status_t::JOINT_BOUND:
      {
	doPostMotionReset();
	PoseTrackingResult	result;
	result.status = static_cast<int8_t>(servo_status_);
	_tracker_srv.setAborted(result);
	ROS_ERROR_STREAM_NAMED(
	    LOGNAME, "(JointTrajectoryServo) goal ABORTED["
	    << moveit_servo::SERVO_STATUS_CODE_MAP.at(servo_status_)
	    << ']');
	return;
      }

      default:
	break;
    }

  // Check that target pose is recent enough.
    if (!haveRecentTargetPose(_current_goal->timeout))
    {
    	doPostMotionReset();
	PoseTrackingResult	result;
	result.status = PoseTrackingResult::INPUT_TIMEOUT;
    	_tracker_srv.setAborted(result);
        ROS_ERROR_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal ABORTED["
    			       << "The target pose was not updated recently."
			       << ']');

    	return;
    }

  // Attempt to update robot pose.
    if (servo_->getEEFrameTransform(_ee_frame_transform))
	_ee_frame_transform_stamp = ros::Time::now();

    durations_.ee_frame_in = (_ee_frame_transform_stamp -
			      durations_.header.stamp).toSec();

  // Check that end-effector pose (command frame transform) is recent enough.
    if (!haveRecentEndEffectorPose(_current_goal->timeout))
    {
	doPostMotionReset();
	PoseTrackingResult	result;
	result.status = PoseTrackingResult::INPUT_TIMEOUT;
    	_tracker_srv.setAborted(result);
	ROS_ERROR_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal ABORTED["
			       << "The end effector pose was not updated in time.]");

	return;
    }

  // Compute positional and angular errors.
    Eigen::Vector3d	positional_error;
    Eigen::AngleAxisd	angular_error;
    calculatePoseError(_current_goal->target_offset,
		       positional_error, angular_error);

  // Check if goal tolerance is satisfied.
    if (std::abs(positional_error(0)) <
	_current_goal->positional_tolerance[0] &&
	std::abs(positional_error(1)) <
	_current_goal->positional_tolerance[1] &&
	std::abs(positional_error(2)) <
	_current_goal->positional_tolerance[2] &&
	std::abs(angular_error.angle()) < _current_goal->angular_tolerance)
    {
	doPostMotionReset();
	PoseTrackingResult	result;
	result.status = PoseTrackingResult::NO_ERROR;
	_tracker_srv.setSucceeded(result);
	ROS_INFO_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal SUCCEEDED");

	return;
    }

  // Publish tracking result as feedback.
    PoseTrackingFeedback	feedback;
    feedback.positional_error[0] = positional_error(0);
    feedback.positional_error[1] = positional_error(1);
    feedback.positional_error[2] = positional_error(2);
    feedback.angular_error	 = angular_error.angle();
    feedback.status		 = static_cast<int8_t>(servo_status_);
    _tracker_srv.publishFeedback(feedback);

  // Compute servo command from PID controller output and send it
  // to the Servo object, for execution
    twist_stamped_pub_.publish(calculateTwistCommand(positional_error,
						     angular_error));

    durations_.twist_out = (ros::Time::now() -
			    durations_.header.stamp).toSec();

  // For debugging
    ee_pose_pub_.publish(tf2::toMsg(tf2::Stamped<Eigen::Isometry3d>(
					_ee_frame_transform,
					_ee_frame_transform_stamp,
					_planning_frame)));
}

const ros::Rate&
JointTrajectoryServo::loop_rate() const
{
    return _loop_rate;
}

/*
 *  private member functions
 */
planning_scene_monitor::PlanningSceneMonitorPtr
JointTrajectoryServo::createPlanningSceneMonitor(
    const std::string& robot_description)
{
    using	namespace planning_scene_monitor;

    const auto	monitor = std::make_shared<planning_scene_monitor_t>(
				robot_description);
    if (!monitor->getPlanningScene())
    {
	ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to get PlanningSceneMonitor");
	exit(EXIT_FAILURE);
    }

    monitor->startSceneMonitor();
    monitor->startWorldGeometryMonitor(
	PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
	PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
	false /* skip octomap monitor */);
    monitor->startStateMonitor();

    ROS_INFO_STREAM_NAMED(LOGNAME, "PlanningSceneMonitor started");

    return monitor;
}

void
JointTrajectoryServo::readROSParams()
{
  // Optional parameter sub-namespace specified in the launch file.
  // All other parameters will be read from this namespace.
    std::string	parameter_ns;
    ros::param::get("~parameter_ns", parameter_ns);

  // If parameters have been loaded into sub-namespace
  // within the node namespace, append the parameter namespace
  // to load the parameters correctly.
    auto	nh = (parameter_ns.empty() ?
		      _nh : ros::NodeHandle(_nh, parameter_ns));

  // Wait for ROS parameters to load
    const auto	begin = ros::Time::now();
    while (ros::ok() && !nh.hasParam("planning_frame") &&
	   (ros::Time::now() - begin).toSec() < ROS_STARTUP_WAIT)
    {
	ROS_WARN_STREAM_NAMED(LOGNAME,
			      "Waiting for parameter: planning_frame");
	ros::Duration(0.1).sleep();
    }

    std::size_t error = 0;

    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "planning_frame", _planning_frame);
    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "move_group_name", _move_group_name);
    if (!_planning_scene_monitor->getRobotModel()
				->hasJointModelGroup(_move_group_name))
    {
	++error;
	ROS_ERROR_STREAM_NAMED(
	    LOGNAME, "Unable to find the specified joint model group: "
	    << _move_group_name);
    }

  // Setup _loop_rate
    double	publish_period;
    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "publish_period", publish_period);
    _loop_rate = ros::Rate(1 / publish_period);

    x_pid_config_.dt	   = publish_period;
    y_pid_config_.dt	   = publish_period;
    z_pid_config_.dt	   = publish_period;
    angular_pid_config_.dt = publish_period;

  // Setup input low-pass filter
    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "input_low_pass_filter_half_order",
				      _input_low_pass_filter_half_order);
    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "input_low_pass_filter_cutoff_frequency",
				      _input_low_pass_filter_cutoff_frequency);

  // Setup PID configurations
    double	windup_limit;
    error += !rosparam_shortcuts::get(LOGNAME, nh, "windup_limit",
				      windup_limit);
    x_pid_config_.windup_limit = windup_limit;
    y_pid_config_.windup_limit = windup_limit;
    z_pid_config_.windup_limit = windup_limit;
    angular_pid_config_.windup_limit = windup_limit;

    error += !rosparam_shortcuts::get(LOGNAME, nh, "x_proportional_gain",
				      x_pid_config_.k_p);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "y_proportional_gain",
				      y_pid_config_.k_p);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "z_proportional_gain",
				      z_pid_config_.k_p);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "x_integral_gain",
				      x_pid_config_.k_i);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "y_integral_gain",
				      y_pid_config_.k_i);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "z_integral_gain",
				      z_pid_config_.k_i);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "x_derivative_gain",
				      x_pid_config_.k_d);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "y_derivative_gain",
				      y_pid_config_.k_d);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "z_derivative_gain",
				      z_pid_config_.k_d);

    error += !rosparam_shortcuts::get(LOGNAME, nh, "angular_proportional_gain",
				      angular_pid_config_.k_p);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "angular_integral_gain",
				      angular_pid_config_.k_i);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "angular_derivative_gain",
				      angular_pid_config_.k_d);

    rosparam_shortcuts::shutdownIfError(ros::this_node::getName(), error);
}

void
JointTrajectoryServo::servoStatusCB(const std_msgs::Int8ConstPtr& msg)
{
    const auto	servo_status = static_cast<servo_status_t>(msg->data);

    if (servo_status == servo_status_)
	return;

    servo_status_ = servo_status;

    switch (servo_status)
    {
      case servo_status_t::DECELERATE_FOR_SINGULARITY:
      case servo_status_t::DECELERATE_FOR_COLLISION:
	ROS_WARN_STREAM_NAMED(
	    LOGNAME, "(JointTrajectoryServo) Servo status["
	    << moveit_servo::SERVO_STATUS_CODE_MAP.at(servo_status)
	    << ']');
	break;
      case servo_status_t::HALT_FOR_SINGULARITY:
      case servo_status_t::HALT_FOR_COLLISION:
      case servo_status_t::JOINT_BOUND:
	ROS_ERROR_STREAM_NAMED(
	    LOGNAME, "(JointTrajectoryServo) Servo status["
	    << moveit_servo::SERVO_STATUS_CODE_MAP.at(servo_status)
	    << ']');
	break;
      default:
	ROS_INFO_STREAM_NAMED(
	    LOGNAME, "(JointTrajectoryServo) Servo status["
	    << moveit_servo::SERVO_STATUS_CODE_MAP.at(servo_status)
	    << ']');
	break;
    }
}

void
JointTrajectoryServo::targetPoseCB(const geometry_msgs::PoseStampedConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_target_pose_mtx);

    _target_pose = *msg;

  // Prevent doTransform from copying a stamp of 0,
  // which will cause the haveRecentTargetPose check to fail servo motions
    if (_target_pose.header.stamp == ros::Time(0))
	_target_pose.header.stamp = ros::Time::now();

    durations_.header	      = _target_pose.header;
    durations_.target_pose_in = (ros::Time::now() -
				 durations_.header.stamp).toSec();

  // If the target pose is defined in planning frame, it's OK as is.
    if (_target_pose.header.frame_id == _planning_frame)
	return;

  // Otherwise, transform it to planning frame.
    auto Tpt = tf2::eigenToTransform(servo_->getFrameTransform(
					 _target_pose.header.frame_id));
    Tpt.header.stamp    = _target_pose.header.stamp;
    Tpt.header.frame_id = _planning_frame;
    Tpt.child_frame_id  = _target_pose.header.frame_id;
    tf2::doTransform(_target_pose, _target_pose, Tpt);
  /*
    try
    {
	tf2::doTransform(_target_pose, _target_pose,
			 _transform_buffer.lookupTransform(
			     _planning_frame, _target_pose.header.frame_id,
			     ros::Time(0), ros::Duration(0.1)));
    }
    catch (const tf2::TransformException& err)
    {
	ROS_WARN_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) " << err.what());
    }
  */
}

void
JointTrajectoryServo::goalCB()
{
    resetTargetPose();

  // Wait a bit for a target pose message to arrive.
  // The target pose may get updated by new messages as the robot moves
  // (in a callback function).
    for (const auto start_time = ros::Time::now();
	 ros::Time::now() - start_time < DEFAULT_INPUT_TIMEOUT;
	 ros::Duration(0.001).sleep())
    {
	if (haveRecentTargetPose(DEFAULT_INPUT_TIMEOUT) &&
	    haveRecentEndEffectorPose(DEFAULT_INPUT_TIMEOUT))
	{
	    _input_low_pass_filter.reset(_target_pose.pose);

	    std_srvs::Empty	empty;
	    reset_servo_status_.call(empty);
	    servo_->start();

	    _current_goal = _tracker_srv.acceptNewGoal();
	    ROS_INFO_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal ACCEPTED["
				  << _current_goal->target_offset << ']');

	    if (_tracker_srv.isPreemptRequested())
		preemptCB();

	    return;
	}

	if (servo_->getEEFrameTransform(_ee_frame_transform))
	    _ee_frame_transform_stamp = ros::Time::now();
    }

  // No target pose available recently.
  // Once accept the pending goal and then abort it immediately.
    _current_goal = _tracker_srv.acceptNewGoal();
    PoseTrackingResult	result;
    result.status = static_cast<int8_t>(servo_status_);
    _tracker_srv.setAborted(result);

    ROS_ERROR_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) Cannot accept goal because no target pose available recently.");
}

void
JointTrajectoryServo::preemptCB()
{
    doPostMotionReset();
    PoseTrackingResult	result;
    result.status = static_cast<int8_t>(servo_status_);
    _tracker_srv.setPreempted(result);
    ROS_WARN_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal CANCELED");
}

void
JointTrajectoryServo::calculatePoseError(const geometry_msgs::Pose& offset,
				      Eigen::Vector3d& positional_error,
				      Eigen::AngleAxisd& angular_error) const
{
    geometry_msgs::PoseStamped	target_pose;
    {
	std::lock_guard<std::mutex> lock(_target_pose_mtx);

	target_pose = _target_pose;
    }

  // Applly input low-pass filter
    target_pose.pose = _input_low_pass_filter.filter(target_pose.pose);
    normalize(target_pose.pose.orientation);

  // Correct target_pose by offset
    tf2::Transform	target_transform;
    tf2::fromMsg(target_pose.pose, target_transform);
    tf2::Transform	offset_transform;
    tf2::fromMsg(offset, offset_transform);
    tf2::toMsg(target_transform * offset_transform, target_pose.pose);

  // Publish corrected pose for debugging
    target_pose_pub_.publish(target_pose);

  // Compute errors
    positional_error(0) = target_pose.pose.position.x
			- _ee_frame_transform.translation()(0);
    positional_error(1) = target_pose.pose.position.y
			- _ee_frame_transform.translation()(1);
    positional_error(2) = target_pose.pose.position.z
			- _ee_frame_transform.translation()(2);

    Eigen::Quaterniond	q_desired;
    tf2::convert(target_pose.pose.orientation, q_desired);
    angular_error = q_desired
		  * Eigen::Quaterniond(_ee_frame_transform.rotation())
			.inverse();
}

geometry_msgs::TwistStampedConstPtr
JointTrajectoryServo::calculateTwistCommand(
			const Eigen::Vector3d& positional_error,
			const Eigen::AngleAxisd& angular_error)
{
  // use the shared pool to create a message more efficiently
    const auto	msg = moveit::util::make_shared_from_pool<
			geometry_msgs::TwistStamped>();
    {
	std::lock_guard<std::mutex> lock(_target_pose_mtx);

	msg->header.frame_id = _target_pose.header.frame_id;
    }

  // Get twist components from PID controllers
    auto&	twist = msg->twist;
    twist.linear.x = cartesian_position_pids_[0]
		    .computeCommand(positional_error(0),
				    _loop_rate.expectedCycleTime());
    twist.linear.y = cartesian_position_pids_[1]
		    .computeCommand(positional_error(1),
				    _loop_rate.expectedCycleTime());
    twist.linear.z = cartesian_position_pids_[2]
		    .computeCommand(positional_error(2),
				    _loop_rate.expectedCycleTime());

  // Anglular components
    const auto	ang_vel_magnitude = cartesian_orientation_pids_[0]
				   .computeCommand(angular_error.angle(),
						   _loop_rate
						   .expectedCycleTime());
    twist.angular.x = ang_vel_magnitude * angular_error.axis()[0];
    twist.angular.y = ang_vel_magnitude * angular_error.axis()[1];
    twist.angular.z = ang_vel_magnitude * angular_error.axis()[2];

    msg->header.stamp = ros::Time::now();

    return msg;
}

void
JointTrajectoryServo::doPostMotionReset()
{
    stopMotion();

  // Reset error integrals and previous errors of PID controllers
    cartesian_position_pids_[0].reset();
    cartesian_position_pids_[1].reset();
    cartesian_position_pids_[2].reset();
    cartesian_orientation_pids_[0].reset();
}

void
JointTrajectoryServo::stopMotion()
{
  // Send a 0 command to Servo to halt arm motion
    const auto	msg = moveit::util::make_shared_from_pool<
			geometry_msgs::TwistStamped>();
    {
	std::lock_guard<std::mutex> lock(_target_pose_mtx);

	msg->header.frame_id = _target_pose.header.frame_id;
    }
    msg->header.stamp = ros::Time::now();
    twist_stamped_pub_.publish(msg);
}

// Low-pass filter stuffs
void
JointTrajectoryServo::updateInputLowPassFilter(int half_order,
					    double cutoff_frequency)
{
    _input_low_pass_filter_half_order	    = half_order;
    _input_low_pass_filter_cutoff_frequency = cutoff_frequency;

    _input_low_pass_filter.initialize(_input_low_pass_filter_half_order,
				      _input_low_pass_filter_cutoff_frequency
				      *_loop_rate.expectedCycleTime().toSec());

    _input_low_pass_filter.reset(_target_pose.pose);
}

// PID controller stuffs
void
JointTrajectoryServo::updatePositionPIDs(double PIDConfig::* field, double value)
{
    std::lock_guard<std::mutex> lock(_target_pose_mtx);

    x_pid_config_.*field = value;
    y_pid_config_.*field = value;
    z_pid_config_.*field = value;

    cartesian_position_pids_.clear();
    initializePID(x_pid_config_, cartesian_position_pids_);
    initializePID(y_pid_config_, cartesian_position_pids_);
    initializePID(z_pid_config_, cartesian_position_pids_);
}

void
JointTrajectoryServo::updateOrientationPID(double PIDConfig::* field,
					double value)
{
    std::lock_guard<std::mutex> lock(_target_pose_mtx);

    angular_pid_config_.*field = value;

    cartesian_orientation_pids_.clear();
    initializePID(angular_pid_config_, cartesian_orientation_pids_);
}

void
JointTrajectoryServo::initializePID(const PIDConfig& pid_config,
				 std::vector<control_toolbox::Pid>& pid_vector)
{
    bool use_anti_windup = true;
    pid_vector.push_back(control_toolbox::Pid(pid_config.k_p,
					      pid_config.k_i,
					      pid_config.k_d,
					      pid_config.windup_limit,
					      -pid_config.windup_limit,
					      use_anti_windup));
}

void
JointTrajectoryServo::getPIDErrors(double& x_error, double& y_error,
				double& z_error, double& orientation_error)
{
    double dummy1, dummy2;
    cartesian_position_pids_.at(0).getCurrentPIDErrors(&x_error,
						       &dummy1, &dummy2);
    cartesian_position_pids_.at(1).getCurrentPIDErrors(&y_error,
						       &dummy1, &dummy2);
    cartesian_position_pids_.at(2).getCurrentPIDErrors(&z_error,
						       &dummy1, &dummy2);
    cartesian_orientation_pids_.at(0).getCurrentPIDErrors(&orientation_error,
							  &dummy1, &dummy2);
}

// Target pose stuffs
void
JointTrajectoryServo::resetTargetPose()
{
    std::lock_guard<std::mutex>	lock(_target_pose_mtx);

    _target_pose	      = geometry_msgs::PoseStamped();
    _target_pose.header.stamp = ros::Time(0);
}

bool
JointTrajectoryServo::haveRecentTargetPose(const ros::Duration& timeout) const
{
    std::lock_guard<std::mutex> lock(_target_pose_mtx);

    return (ros::Time::now() - _target_pose.header.stamp < timeout);
}

// End-effector pose stuffs
bool
JointTrajectoryServo::haveRecentEndEffectorPose(
			const ros::Duration& timeout) const
{
    return (ros::Time::now() - _ee_frame_transform_stamp < timeout);
}
}	// namespace aist_controllers

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, LOGNAME);

    ros::NodeHandle	nh("~");
    aist_controllers::JointTrajectoryServo	servo(nh);
    servo.run();

    return 0;
}
