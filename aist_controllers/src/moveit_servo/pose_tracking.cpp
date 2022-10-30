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

#include "moveit_servo/pose_tracking.h"

namespace
{
constexpr char LOGNAME[] = "pose_tracking";
constexpr double DEFAULT_LOOP_RATE = 100;  // Hz
constexpr double ROS_STARTUP_WAIT = 10;    // sec
}  // namespace

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
}

/************************************************************************
*  class PoseTracking							*
************************************************************************/
PoseTracking::PoseTracking()
    :nh_("~"),
     planning_scene_monitor_(create_planning_scene_monitor(
				 "robot_description")),
     robot_model_(nullptr),
     joint_model_group_(nullptr),
     move_group_name_(),
     loop_rate_(DEFAULT_LOOP_RATE),
     twist_stamped_pub_(),

     cartesian_position_pids_(),
     cartesian_orientation_pids_(),
     x_pid_config_(),
     y_pid_config_(),
     z_pid_config_(),

     ee_frame_transform_(),
     ee_frame_transform_stamp_(),
     target_pose_(),
     target_pose_mtx_(),

     ee_pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("ee_pose_debug",
							    1)),

     transform_buffer_(),
     transform_listener_(transform_buffer_),

     planning_frame_(),

     stop_requested_(false),
     angular_error_(boost::none),
     tracker_srv_(nh_, "pose_tracking", false),
     current_goal_(nullptr),
     ddr_(ros::NodeHandle(nh_, "pose_tracking")),
     timer_()
{
    readROSParams();

    robot_model_       = planning_scene_monitor_->getRobotModel();
    joint_model_group_ = robot_model_->getJointModelGroup(move_group_name_);

  // Initialize PID controllers
    initializePID(x_pid_config_,       cartesian_position_pids_);
    initializePID(y_pid_config_,       cartesian_position_pids_);
    initializePID(z_pid_config_,       cartesian_position_pids_);
    initializePID(angular_pid_config_, cartesian_orientation_pids_);

  // Use the C++ interface that Servo provides
    servo_ = std::make_unique<moveit_servo::Servo>(nh_,
						   planning_scene_monitor_);
    servo_->start();

  // Connect to Servo ROS interfaces
    target_pose_sub_ =
	nh_.subscribe<geometry_msgs::PoseStamped>(
	    "target_pose", 1, &PoseTracking::targetPoseCallback, this,
	    ros::TransportHints().reliable().tcpNoDelay(true));

  // Publish outgoing twist commands to the Servo object
    twist_stamped_pub_ =
	nh_.advertise<geometry_msgs::TwistStamped>(
	    servo_->getParameters().cartesian_command_in_topic, 1);

  // Setup action server
    tracker_srv_.registerGoalCallback(boost::bind(&PoseTracking::goalCallback,
						  this));
    tracker_srv_.registerPreemptCallback(boost::bind(
					     &PoseTracking::preemptCallback,
					     this));
    tracker_srv_.start();

  // Setup dynamic reconfigure server
    ddr_.registerVariable<double>("linear_proportional_gain",
				  x_pid_config_.k_p,
				  boost::bind(&PoseTracking
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for translation",
				  0.1, 20.0);
    ddr_.registerVariable<double>("linear_integral_gain",
				  x_pid_config_.k_i,
				  boost::bind(&PoseTracking
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for translation",
				  0.0, 20.0);
    ddr_.registerVariable<double>("linear_derivative_gain",
				  x_pid_config_.k_d,
				  boost::bind(&PoseTracking
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_d, _1),
				  "Derivative gain for translation",
				  0.0, 20.0);

    ddr_.registerVariable<double>("angular_proportinal_gain",
				  angular_pid_config_.k_p,
				  boost::bind(&PoseTracking
					      ::updateOrientationPID,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for rotation",
				  0.1, 20.0);
    ddr_.registerVariable<double>("angular_integral_gain",
				  angular_pid_config_.k_i,
				  boost::bind(&PoseTracking
					      ::updateOrientationPID,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for rotation",
				  0.0, 20.0);
    ddr_.registerVariable<double>("angular_derivative_gain",
				  angular_pid_config_.k_d,
				  boost::bind(&PoseTracking
					      ::updateOrientationPID,
					      this, &PIDConfig::k_d, _1),
				  "Derivative gain for rotation",
				  0.0, 20.0);
    ddr_.publishServicesTopics();

  // Start timer
    timer_ = nh_.createTimer(loop_rate_.expectedCycleTime(),
			     &PoseTracking::moveToPoseCallback, this);
}

planning_scene_monitor::PlanningSceneMonitorPtr
PoseTracking::create_planning_scene_monitor(
		const std::string& robot_description)
{
    using	namespace planning_scene_monitor;

    const auto	monitor = std::make_shared<planning_scene_monitor_t>(
				robot_description);
    if (!monitor->getPlanningScene())
    {
	ROS_ERROR_STREAM_NAMED(
	    LOGNAME, "(PoseTrackingServo) failed to get PlanningSceneMonitor");
	exit(EXIT_FAILURE);
    }

    monitor->startSceneMonitor();
    monitor->startWorldGeometryMonitor(
	PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
	PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
	false /* skip octomap monitor */);
    monitor->startStateMonitor();

    ROS_INFO_STREAM_NAMED(LOGNAME,
			  "(PoseTrackingServo) PlanningSceneMonitor started");

    return monitor;
}

void
PoseTracking::readROSParams()
{
  // Optional parameter sub-namespace specified in the launch file.
  // All other parameters will be read from this namespace.
    std::string parameter_ns;
    ros::param::get("~parameter_ns", parameter_ns);

  // If parameters have been loaded into sub-namespace
  // within the node namespace, append the parameter namespace
  // to load the parameters correctly.
    ros::NodeHandle nh = (parameter_ns.empty() ?
			  nh_ : ros::NodeHandle(nh_, parameter_ns));

  // Wait for ROS parameters to load
    ros::Time begin = ros::Time::now();
    while (ros::ok() && !nh.hasParam("planning_frame") &&
	   ((ros::Time::now() - begin).toSec() < ROS_STARTUP_WAIT))
    {
	ROS_WARN_STREAM_NAMED(LOGNAME,
			      "Waiting for parameter: planning_frame");
	ros::Duration(0.1).sleep();
    }

    std::size_t error = 0;

    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "planning_frame", planning_frame_);
    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "move_group_name", move_group_name_);
    if (!planning_scene_monitor_->getRobotModel()
				->hasJointModelGroup(move_group_name_))
    {
	++error;
	ROS_ERROR_STREAM_NAMED(
	    LOGNAME, "Unable to find the specified joint model group: "
	    << move_group_name_);
    }

    double publish_period;
    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "publish_period", publish_period);
    loop_rate_ = ros::Rate(1 / publish_period);

    x_pid_config_.dt	   = publish_period;
    y_pid_config_.dt	   = publish_period;
    z_pid_config_.dt	   = publish_period;
    angular_pid_config_.dt = publish_period;

    double windup_limit;
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
PoseTracking::goalCallback()
{
    current_goal_ = tracker_srv_.acceptNewGoal();

    ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTracking) goal ACCEPTED["
			  << current_goal_->target_offset << ']');

  // Reset stop requested flag before starting motions
    stop_requested_ = false;

  // Wait a bit for a target pose message to arrive.
  // The target pose may get updated by new messages as the robot moves
  // (in a callback function).
    const double	target_pose_timeout = 0.1;
    const auto		start_time = ros::Time::now();
    while ((!haveRecentTargetPose(target_pose_timeout) ||
	    !haveRecentEndEffectorPose(target_pose_timeout)) &&
	   ((ros::Time::now() - start_time).toSec() < target_pose_timeout))
    {
	if (servo_->getEEFrameTransform(ee_frame_transform_))
	    ee_frame_transform_stamp_ = ros::Time::now();
	ros::Duration(0.001).sleep();
    }

    if (!haveRecentTargetPose(target_pose_timeout))
    {
	tracker_srv_.setAborted();
	ROS_ERROR_STREAM_NAMED(
	    LOGNAME, "The target pose was not updated recently. Aborting.");
    }
}

void
PoseTracking::preemptCallback()
{
    doPostMotionReset();
    tracker_srv_.setPreempted();
    ROS_INFO_STREAM_NAMED(LOGNAME,
			  "Halting servo motion, a stop was requested.");
}

void
PoseTracking::moveToPoseCallback(const ros::TimerEvent&)
{
    if (!tracker_srv_.isActive())
	return;

  // Continue sending PID controller output to Servo
  // until one of the following conditions is met:
  // - Goal tolerance is satisfied
  // - Target pose becomes outdated
  // - Command frame transform becomes outdated
  // - Another thread requested a stop
    Eigen::Vector3d&	positional_error;
    Eigen::AngleAxisd&	angular_error;
    calculatePoseError(_current_goal->offset, positional_error, angular_error);

    if (std::abs(positional_error(0))   < _current_goal->positional_tolerance[0] &&
	std::abs(positional_error(1))   < _current_goal->positional_tolerance[1] &&
	std::abs(positional_error(2))   < _current_goal->positional_tolerance[2] &&
	std::abs(angular_error.angle()) < _current_goal->angular_tolerance)
    {
	doPostMotionReset();
	tracker_srv_.setSucceeded();
	ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTracking) goal SUCCEEDED");

	return;
    }

  // Attempt to update robot pose
    if (servo_->getEEFrameTransform(ee_frame_transform_))
	ee_frame_transform_stamp_ = ros::Time::now();

  // Check that end-effector pose (command frame transform) is recent enough.
    if (!haveRecentEndEffectorPose(target_pose_timeout))
    {
	doPostMotionReset();
	tracker_srv_.setAborted();
	ROS_ERROR_STREAM_NAMED(LOGNAME, "(PoseTracking) goal ABORTED["
			       << "The end effector pose was not updated in time.]");

	return;
    }

  // Compute servo command from PID controller output and send it
  // to the Servo object, for execution
    twist_stamped_pub_.publish(calculateTwistCommand(
				   current_goal_->target_offset));

  // For debugging
    ee_pose_pub_.publish(tf2::toMsg(tf2::Stamped<Eigen::Isometry3d>(
					ee_frame_transform_,
					ee_frame_transform_stamp_,
					target_pose_.header.frame_id)));
}

void
PoseTracking::targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(target_pose_mtx_);
    target_pose_ = *msg;

  // If the target pose is not defined in planning frame,
  // transform the target pose.
    if (target_pose_.header.frame_id != planning_frame_)
    {
	try
	{
	    geometry_msgs::TransformStamped
		target_to_planning_frame = transform_buffer_.lookupTransform(
						planning_frame_,
						target_pose_.header.frame_id,
						ros::Time(0),
						ros::Duration(0.1));
	    tf2::doTransform(target_pose_, target_pose_,
			     target_to_planning_frame);

	    if (target_pose_.header.stamp == ros::Time(0))
	    {
	      // Prevent doTransform from copying a stamp of 0,
	      // which will cause the haveRecentTargetPose check
	      // to fail servo motions
		target_pose_.header.stamp = ros::Time::now();
	    }
	}
	catch (const tf2::TransformException& ex)
	{
	    ROS_WARN_STREAM_NAMED(LOGNAME, "(PoseTracking) " << ex.what());
	    return;
	}
    }
}

bool
PoseTracking::haveRecentTargetPose(double timespan) const
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  return ((ros::Time::now() - target_pose_.header.stamp).toSec() < timespan);
}

bool
PoseTracking::haveRecentEndEffectorPose(double timespan) const
{
  return ((ros::Time::now() - ee_frame_transform_stamp_).toSec() < timespan);
}

void
PoseTracking::calculatePoseError(const geometry_msgs::Pose& offset,
				 Eigen::Vector3d& positional_error,
				 Eigen::AngleAxisd& angular_error) const
{
    Eigen::Quaterniond	q_desired;

    {
	std::lock_guard<std::mutex> lock(target_pose_mtx_);

      // Correct _target_pose by offset
	tf2::Transform	target_transform;
	tf2::fromMsg(target_pose_.pose, target_transform);
	tf2::Transform	offset_transform;
	tf2::fromMsg(offset, offset_transform);
	target_transform *= offset_transform;

	positional_error(0) = target_transform.getOrigin().x()
			    - ee_frame_transform_.translation()(0);
	positional_error(1) = target_transform.getOrigin().x()
			    - ee_frame_transform_.translation()(1);
	positional_error(2) = target_transform.getOrigin().x()
			    - ee_frame_transform_.translation()(2);

	tf2::convert(target_transform.getRotation(), q_desired);
    }

    angular_error = q_desired
		  * Eigen::Quaterniond(ee_frame_transform_.rotation())
			.inverse();
}

bool
PoseTracking::satisfiesPoseTolerance(
    const boost::array<double, 3>& positional_tolerance,
    double angular_tolerance,
    const Eigen::Vector3d& positional_error,
    const Eigen::AngleAxisd& angular_error) const
{
    return ((std::abs(positional_error(0))   < positional_tolerance[0]) &&
	    (std::abs(positional_error(1))   < positional_tolerance[1]) &&
	    (std::abs(positional_error(2))   < positional_tolerance[2]) &&
	    (std::abs(angular_error.angle()) < angular_tolerance));
}

geometry_msgs::TwistStampedConstPtr
PoseTracking::calculateTwistCommand(const Eigen::Vector3d& positional_error,
				    const Eigen::AngleAxisd& angular_error)
{
  // use the shared pool to create a message more efficiently
    auto msg = moveit::util::make_shared_from_pool<
		geometry_msgs::TwistStamped>();
    {
	std::lock_guard<std::mutex> lock(target_pose_mtx_);

	msg->header.frame_id = target_pose_.header.frame_id;
    }

  // Get twist components from PID controllers
    auto&	twist = msg->twist;
    twist.linear.x = cartesian_position_pids_[0]
		    .computeCommand(positional_error(0));
    twist.linear.y = cartesian_position_pids_[1]
		    .computeCommand(positional_error(1));
    twist.linear.z = cartesian_position_pids_[2]
		    .computeCommand(positional_error(2));

  // Anglular components
    const auto	ang_vel_magnitude = cartesian_orientation_pids_[0]
				   .computeCommand(angular_error.angle(),
						   loop_rate_
						   .expectedCycleTime());
    twist.angular.x = ang_vel_magnitude * angular_error.axis()[0];
    twist.angular.y = ang_vel_magnitude * angular_error.axis()[1];
    twist.angular.z = ang_vel_magnitude * angular_error.axis()[2];

    msg->header.stamp = ros::Time::now();

    return msg;
}

void
PoseTracking::doPostMotionReset()
{
    stopMotion();
    stop_requested_ = false;
    angular_error_  = boost::none;

  // Reset error integrals and previous errors of PID controllers
    cartesian_position_pids_[0].reset();
    cartesian_position_pids_[1].reset();
    cartesian_position_pids_[2].reset();
    cartesian_orientation_pids_[0].reset();
}

void
PoseTracking::stopMotion()
{
    stop_requested_ = true;

  // Send a 0 command to Servo to halt arm motion
    auto msg = moveit::util::make_shared_from_pool<
		geometry_msgs::TwistStamped>();
    {
	std::lock_guard<std::mutex> lock(target_pose_mtx_);
	msg->header.frame_id = target_pose_.header.frame_id;
    }
    msg->header.stamp = ros::Time::now();
    twist_stamped_pub_.publish(msg);
}

// PID controller stuffs
void
PoseTracking::updatePIDConfig(double x_proportional_gain,
			      double x_integral_gain,
			      double x_derivative_gain,
			      double y_proportional_gain,
			      double y_integral_gain,
			      double y_derivative_gain,
			      double z_proportional_gain,
			      double z_integral_gain,
			      double z_derivative_gain,
			      double angular_proportional_gain,
			      double angular_integral_gain,
			      double angular_derivative_gain)
{
    stopMotion();

    x_pid_config_.k_p = x_proportional_gain;
    x_pid_config_.k_i = x_integral_gain;
    x_pid_config_.k_d = x_derivative_gain;
    y_pid_config_.k_p = y_proportional_gain;
    y_pid_config_.k_i = y_integral_gain;
    y_pid_config_.k_d = y_derivative_gain;
    z_pid_config_.k_p = z_proportional_gain;
    z_pid_config_.k_i = z_integral_gain;
    z_pid_config_.k_d = z_derivative_gain;

    angular_pid_config_.k_p = angular_proportional_gain;
    angular_pid_config_.k_i = angular_integral_gain;
    angular_pid_config_.k_d = angular_derivative_gain;

    cartesian_position_pids_.clear();
    cartesian_orientation_pids_.clear();
    initializePID(x_pid_config_,       cartesian_position_pids_);
    initializePID(y_pid_config_,       cartesian_position_pids_);
    initializePID(z_pid_config_,       cartesian_position_pids_);
    initializePID(angular_pid_config_, cartesian_orientation_pids_);

    doPostMotionReset();
}

void
PoseTracking::updatePositionPIDs(double PIDConfig::* field, double value)
{
    std::lock_guard<std::mutex> lock(target_pose_mtx_);

    x_pid_config_.*field = value;
    y_pid_config_.*field = value;
    z_pid_config_.*field = value;

    cartesian_position_pids_.clear();
    initializePID(x_pid_config_, cartesian_position_pids_);
    initializePID(y_pid_config_, cartesian_position_pids_);
    initializePID(z_pid_config_, cartesian_position_pids_);
}

void
PoseTracking::updateOrientationPID(double PIDConfig::* field, double value)
{
    std::lock_guard<std::mutex> lock(target_pose_mtx_);

    angular_pid_config_.*field = value;

    cartesian_orientation_pids_.clear();
    initializePID(angular_pid_config_, cartesian_orientation_pids_);
}

void
PoseTracking::initializePID(const PIDConfig& pid_config,
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
PoseTracking::getPIDErrors(double& x_error, double& y_error,
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

void
PoseTracking::resetTargetPose()
{
    std::lock_guard<std::mutex>	lock(target_pose_mtx_);
    target_pose_	      = geometry_msgs::PoseStamped();
    target_pose_.header.stamp = ros::Time(0);
}

bool
PoseTracking::getEEFrameTransform(geometry_msgs::TransformStamped& transform)
{
    return servo_->getEEFrameTransform(transform);
}
}  // namespace moveit_servo
