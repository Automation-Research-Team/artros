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

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.

namespace
{
constexpr char LOGNAME[] = "pose_tracking_servo";
constexpr double DEFAULT_LOOP_RATE = 100;  // Hz
constexpr double ROS_STARTUP_WAIT = 10;    // sec
}  // namespace

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
}

/************************************************************************
*  class PoseTrackingServo						*
************************************************************************/
class PoseTrackingServo
{
  private:
    using planning_scene_monitor_t
			 = planning_scene_monitor::PlanningSceneMonitor;
    using planning_scene_monitor_p
			 = planning_scene_monitor::PlanningSceneMonitorPtr;
    using server_t	 = actionlib::SimpleActionServer<PoseTrackingAction>;
    using servo_status_t = moveit_servo::StatusCode;

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
		PoseTrackingServo()					;
		~PoseTrackingServo()					;

    void	run()							;

  private:
    static planning_scene_monitor_p
		createPlanningSceneMonitor(
		    const std::string& robot_description)		;
    void	readROSParams()						;
    void	servoStatusCB(const std_msgs::Int8ConstPtr& msg)	;
    void	targetPoseCB(const geometry_msgs::PoseStampedConstPtr& msg);
    void	goalCB()						;
    void	preemptCB()						;
    void	mainLoop()						;
    void	calculatePoseError(const geometry_msgs::Pose& offset,
				   Eigen::Vector3d& positional_error,
				   Eigen::AngleAxisd& angular_error) const;

    geometry_msgs::TwistStampedConstPtr
		calculateTwistCommand(const Eigen::Vector3d& positional_error,
				      const Eigen::AngleAxisd& angular_error);
    void	stopMotion()						;
    void	doPostMotionReset()					;

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
    bool	haveRecentTargetPose(double timeout)		const	;

  // End-effector pose stuffs
    bool	getEEFrameTransform(
		    geometry_msgs::TransformStamped& transform)	const	;
    bool	haveRecentEndEffectorPose(double timeout)	const	;

  private:
    ros::NodeHandle				nh_;

    planning_scene_monitor_p			planning_scene_monitor_;
    std::unique_ptr<moveit_servo::Servo>	servo_;
    servo_status_t				servo_status_;

    std::string					move_group_name_;
    const moveit::core::JointModelGroup*	joint_model_group_;

    ros::ServiceClient				reset_servo_status_;
    ros::Subscriber				servo_status_sub_;
    ros::Subscriber				target_pose_sub_;
    ros::Publisher				twist_stamped_pub_;
    ros::Publisher				target_pose_pub_;
    ros::Publisher				ee_pose_pub_;
    ros::Rate					loop_rate_;

  // Action server stuffs
    server_t					tracker_srv_;
    boost::shared_ptr<const server_t::Goal>	current_goal_;

  // Dynamic reconfigure server
    ddynamic_reconfigure::DDynamicReconfigure	ddr_;

    tf2_ros::Buffer				transform_buffer_;
    tf2_ros::TransformListener			transform_listener_;

  // PIDs
    std::vector<control_toolbox::Pid>		cartesian_position_pids_;
    std::vector<control_toolbox::Pid>		cartesian_orientation_pids_;
    PIDConfig					x_pid_config_,
						y_pid_config_,
						z_pid_config_,
						angular_pid_config_;

  // Transforms w.r.t. planning_frame_
    std::string					planning_frame_;
    Eigen::Isometry3d				ee_frame_transform_;
    ros::Time					ee_frame_transform_stamp_;
    geometry_msgs::PoseStamped			target_pose_;
    mutable std::mutex				target_pose_mtx_;
    constexpr static double			input_timeout_ = 0.1;
};

PoseTrackingServo::PoseTrackingServo()
    :nh_("~"),
     planning_scene_monitor_(createPlanningSceneMonitor("robot_description")),
     servo_(new moveit_servo::Servo(nh_, planning_scene_monitor_)),
     servo_status_(servo_status_t::INVALID),

     move_group_name_(),
     joint_model_group_(nullptr),

     reset_servo_status_(nh_.serviceClient<std_srvs::Empty>(
			     "reset_servo_status")),
     servo_status_sub_(nh_.subscribe(servo_->getParameters().status_topic, 1,
				     &PoseTrackingServo::servoStatusCB, this)),
     target_pose_sub_(),
     twist_stamped_pub_(),
     target_pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>(
			  "target_pose_debug", 1)),
     ee_pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>(
		      "ee_pose_debug", 1)),
     loop_rate_(DEFAULT_LOOP_RATE),

     tracker_srv_(nh_, "pose_tracking", false),
     current_goal_(nullptr),
     ddr_(ros::NodeHandle(nh_, "pose_tracking")),

     transform_buffer_(),
     transform_listener_(transform_buffer_),

     cartesian_position_pids_(),
     cartesian_orientation_pids_(),
     x_pid_config_(),
     y_pid_config_(),
     z_pid_config_(),

     planning_frame_(),
     ee_frame_transform_(),
     ee_frame_transform_stamp_(),
     target_pose_(),
     target_pose_mtx_()
{
    readROSParams();

  // Initialize PID controllers
    initializePID(x_pid_config_,       cartesian_position_pids_);
    initializePID(y_pid_config_,       cartesian_position_pids_);
    initializePID(z_pid_config_,       cartesian_position_pids_);
    initializePID(angular_pid_config_, cartesian_orientation_pids_);

  // Connect to Servo ROS interfaces
    target_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
    			"/target_pose", 1,
    			&PoseTrackingServo::targetPoseCB, this,
    			ros::TransportHints().reliable().tcpNoDelay(true));

  // Publish outgoing twist commands to the Servo object
    twist_stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
    			   servo_->getParameters().cartesian_command_in_topic,
    			   1);

  // Setup action server
    tracker_srv_.registerGoalCallback(boost::bind(&PoseTrackingServo::goalCB,
						  this));
    tracker_srv_.registerPreemptCallback(boost::bind(
					     &PoseTrackingServo::preemptCB,
					     this));
    tracker_srv_.start();

  // Setup dynamic reconfigure server
    ddr_.registerVariable<double>("linear_proportional_gain",
				  x_pid_config_.k_p,
				  boost::bind(&PoseTrackingServo
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for translation",
				  0.5, 100.0);
    ddr_.registerVariable<double>("linear_integral_gain",
				  x_pid_config_.k_i,
				  boost::bind(&PoseTrackingServo
					      ::updatePositionPIDs,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for translation",
				  0.0, 20.0);
    ddr_.registerVariable<double>("linear_derivative_gain",
				  x_pid_config_.k_d,
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
				  0.5, 100.0);
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

    ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) server started");
}

PoseTrackingServo::~PoseTrackingServo()
{
    stopMotion();
}

void
PoseTrackingServo::run()
{
    ros::AsyncSpinner	spinner(8);
    spinner.start();

    while (ros::ok())
    {
	mainLoop();
	loop_rate_.sleep();
    }

    spinner.stop();
    ros::waitForShutdown();
}

/*
 *  private member functions
 */
planning_scene_monitor::PlanningSceneMonitorPtr
PoseTrackingServo::createPlanningSceneMonitor(
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
PoseTrackingServo::readROSParams()
{
  // Optional parameter sub-namespace specified in the launch file.
  // All other parameters will be read from this namespace.
    std::string	parameter_ns;
    ros::param::get("~parameter_ns", parameter_ns);

  // If parameters have been loaded into sub-namespace
  // within the node namespace, append the parameter namespace
  // to load the parameters correctly.
    auto	nh = (parameter_ns.empty() ?
		      nh_ : ros::NodeHandle(nh_, parameter_ns));

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

  // Setup loop_rate_
    double	publish_period;
    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "publish_period", publish_period);
    loop_rate_ = ros::Rate(1 / publish_period);

    x_pid_config_.dt	   = publish_period;
    y_pid_config_.dt	   = publish_period;
    z_pid_config_.dt	   = publish_period;
    angular_pid_config_.dt = publish_period;

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
PoseTrackingServo::servoStatusCB(const std_msgs::Int8ConstPtr& msg)
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
	    LOGNAME, "(PoseTrackingServo) Servo status["
	    << moveit_servo::SERVO_STATUS_CODE_MAP.at(servo_status)
	    << ']');
	break;
      case servo_status_t::HALT_FOR_SINGULARITY:
      case servo_status_t::HALT_FOR_COLLISION:
      case servo_status_t::JOINT_BOUND:
	ROS_ERROR_STREAM_NAMED(
	    LOGNAME, "(PoseTrackingServo) Servo status["
	    << moveit_servo::SERVO_STATUS_CODE_MAP.at(servo_status)
	    << ']');
	break;
      default:
	ROS_INFO_STREAM_NAMED(
	    LOGNAME, "(PoseTrackingServo) Servo status["
	    << moveit_servo::SERVO_STATUS_CODE_MAP.at(servo_status)
	    << ']');
	break;
    }
}

void
PoseTrackingServo::targetPoseCB(const geometry_msgs::PoseStampedConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(target_pose_mtx_);

    target_pose_ = *msg;

  // If the target pose is not defined in planning frame,
  // transform the target pose.
    if (target_pose_.header.frame_id != planning_frame_)
    {
	try
	{
	    const auto
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
	    ROS_WARN_STREAM_NAMED(LOGNAME,
				  "(PoseTrackingServo) " << ex.what());
	    return;
	}
    }
}

void
PoseTrackingServo::goalCB()
{
    current_goal_ = tracker_srv_.acceptNewGoal();

    ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ACCEPTED["
			  << current_goal_->target_offset << ']');

    resetTargetPose();
    std_srvs::Empty	empty;
    reset_servo_status_.call(empty);

    servo_->start();

  // Wait a bit for a target pose message to arrive.
  // The target pose may get updated by new messages as the robot moves
  // (in a callback function).
    const auto	start_time = ros::Time::now();
    while ((!haveRecentTargetPose(input_timeout_) ||
	    !haveRecentEndEffectorPose(input_timeout_)) &&
	   ((ros::Time::now() - start_time).toSec() < input_timeout_))
    {
	if (servo_->getEEFrameTransform(ee_frame_transform_))
	    ee_frame_transform_stamp_ = ros::Time::now();
	ros::Duration(0.001).sleep();
    }
}

void
PoseTrackingServo::preemptCB()
{
    doPostMotionReset();
    tracker_srv_.setPreempted();
    ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal CANCELED");
}

void
PoseTrackingServo::mainLoop()
{
    if (!tracker_srv_.isActive())
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
	doPostMotionReset();
	tracker_srv_.setAborted();
	ROS_ERROR_STREAM_NAMED(
	    LOGNAME, "(PoseTrackingServo) goal ABORTED["
	    << moveit_servo::SERVO_STATUS_CODE_MAP.at(servo_status_)
	    << ']');
	return;

      default:
	break;
    }

  // Check that target pose is recent enough.
    if (!haveRecentTargetPose(input_timeout_))
    {
	doPostMotionReset();
	tracker_srv_.setAborted();
        ROS_ERROR_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ABORTED["
			       << "The target pose was not updated recently.]");

	return;
    }

  // Attempt to update robot pose.
    if (servo_->getEEFrameTransform(ee_frame_transform_))
	ee_frame_transform_stamp_ = ros::Time::now();

  // Check that end-effector pose (command frame transform) is recent enough.
    if (!haveRecentEndEffectorPose(input_timeout_))
    {
	doPostMotionReset();
	tracker_srv_.setAborted();
	ROS_ERROR_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ABORTED["
			       << "The end effector pose was not updated in time.]");

	return;
    }

  // Compute positional and angular errors.
    Eigen::Vector3d	positional_error;
    Eigen::AngleAxisd	angular_error;
    calculatePoseError(current_goal_->target_offset,
		       positional_error, angular_error);

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
	tracker_srv_.setSucceeded();
	ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal SUCCEEDED");

	return;
    }

  // Compute servo command from PID controller output and send it
  // to the Servo object, for execution
    twist_stamped_pub_.publish(calculateTwistCommand(positional_error,
						     angular_error));

  // For debugging
    ee_pose_pub_.publish(tf2::toMsg(tf2::Stamped<Eigen::Isometry3d>(
					ee_frame_transform_,
					ee_frame_transform_stamp_,
					planning_frame_)));
}

void
PoseTrackingServo::calculatePoseError(const geometry_msgs::Pose& offset,
				      Eigen::Vector3d& positional_error,
				      Eigen::AngleAxisd& angular_error) const
{
    geometry_msgs::PoseStamped	target_pose;
    {
	std::lock_guard<std::mutex> lock(target_pose_mtx_);

	target_pose = target_pose_;
    }

  // Correct target_pose by offset
    tf2::Transform	target_transform;
    tf2::fromMsg(target_pose.pose, target_transform);
    tf2::Transform	offset_transform;
    tf2::fromMsg(offset, offset_transform);
    tf2::toMsg(target_transform * offset_transform, target_pose.pose);

  // For debugging
    target_pose_pub_.publish(target_pose);

  // Compute errors
    positional_error(0) = target_pose.pose.position.x
			- ee_frame_transform_.translation()(0);
    positional_error(1) = target_pose.pose.position.y
			- ee_frame_transform_.translation()(1);
    positional_error(2) = target_pose.pose.position.z
			- ee_frame_transform_.translation()(2);

    Eigen::Quaterniond	q_desired;
    tf2::convert(target_pose.pose.orientation, q_desired);
    angular_error = q_desired
		  * Eigen::Quaterniond(ee_frame_transform_.rotation())
			.inverse();
}

geometry_msgs::TwistStampedConstPtr
PoseTrackingServo::calculateTwistCommand(
			const Eigen::Vector3d& positional_error,
			const Eigen::AngleAxisd& angular_error)
{
  // use the shared pool to create a message more efficiently
    const auto	msg = moveit::util::make_shared_from_pool<
			geometry_msgs::TwistStamped>();
    {
	std::lock_guard<std::mutex> lock(target_pose_mtx_);

	msg->header.frame_id = target_pose_.header.frame_id;
    }

  // Get twist components from PID controllers
    auto&	twist = msg->twist;
    twist.linear.x = cartesian_position_pids_[0]
		    .computeCommand(positional_error(0),
				    loop_rate_.expectedCycleTime());
    twist.linear.y = cartesian_position_pids_[1]
		    .computeCommand(positional_error(1),
				    loop_rate_.expectedCycleTime());
    twist.linear.z = cartesian_position_pids_[2]
		    .computeCommand(positional_error(2),
				    loop_rate_.expectedCycleTime());

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
PoseTrackingServo::doPostMotionReset()
{
    stopMotion();

  // Reset error integrals and previous errors of PID controllers
    cartesian_position_pids_[0].reset();
    cartesian_position_pids_[1].reset();
    cartesian_position_pids_[2].reset();
    cartesian_orientation_pids_[0].reset();
}

void
PoseTrackingServo::stopMotion()
{
  // Send a 0 command to Servo to halt arm motion
    const auto	msg = moveit::util::make_shared_from_pool<
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
PoseTrackingServo::updatePositionPIDs(double PIDConfig::* field, double value)
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
PoseTrackingServo::updateOrientationPID(double PIDConfig::* field,
					double value)
{
    std::lock_guard<std::mutex> lock(target_pose_mtx_);

    angular_pid_config_.*field = value;

    cartesian_orientation_pids_.clear();
    initializePID(angular_pid_config_, cartesian_orientation_pids_);
}

void
PoseTrackingServo::initializePID(const PIDConfig& pid_config,
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
PoseTrackingServo::getPIDErrors(double& x_error, double& y_error,
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
PoseTrackingServo::resetTargetPose()
{
    std::lock_guard<std::mutex>	lock(target_pose_mtx_);

    target_pose_	      = geometry_msgs::PoseStamped();
    target_pose_.header.stamp = ros::Time(0);
}

bool
PoseTrackingServo::haveRecentTargetPose(double timeout) const
{
    std::lock_guard<std::mutex> lock(target_pose_mtx_);

    return ((ros::Time::now() - target_pose_.header.stamp).toSec() < timeout);
}

// End-effector frame stuffs
bool
PoseTrackingServo::getEEFrameTransform(
			geometry_msgs::TransformStamped& transform) const
{
    return servo_->getEEFrameTransform(transform);
}

bool
PoseTrackingServo::haveRecentEndEffectorPose(double timeout) const
{
    return ((ros::Time::now() - ee_frame_transform_stamp_).toSec() < timeout);
}
}  // namespace aist_controllers

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, LOGNAME);

    aist_controllers::PoseTrackingServo	servo;
    servo.run();

    return 0;
}
