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
 *  \file	pose_tracking_servo.cpp
 *  \brief	ROS pose tracker of aist_moveit_servo::PoseTracking type
 */
#include <atomic>
#include <boost/optional.hpp>
#include <control_toolbox/pid.h>
#include <aist_moveit_servo/make_shared_from_pool.h>
#include <aist_moveit_servo/servo.h>
#include <aist_moveit_servo/status_codes.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <actionlib/server/simple_action_server.h>
#include <aist_moveit_servo/PoseTrackingAction.h>
#include <aist_utility/butterworth_lpf.h>
#include <aist_utility/spline_extrapolator.h>

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.

namespace
{
constexpr char		LOGNAME[]	  = "pose_tracking_servo";
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
operator -(const Pose& a, const Pose& b)
{
    Pose	ret;
    ret.position.x    = a.position.x	- b.position.x;
    ret.position.y    = a.position.y	- b.position.y;
    ret.position.z    = a.position.z	- b.position.z;
    ret.orientation.x = a.orientation.x	- b.orientation.x;
    ret.orientation.y = a.orientation.y	- b.orientation.y;
    ret.orientation.z = a.orientation.z	- b.orientation.z;
    ret.orientation.w = a.orientation.w	- b.orientation.w;

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

Pose
zero(Pose)
{
    Pose	ret;
    ret.position.x    = 0;
    ret.position.y    = 0;
    ret.position.z    = 0;
    ret.orientation.x = 0;
    ret.orientation.y = 0;
    ret.orientation.z = 0;
    ret.orientation.w = 0;

    return ret;
}

Point
operator +(const Point& a, const Point& b)
{
    Point	ret;
    ret.x = a.x	+ b.x;
    ret.y = a.y	+ b.y;
    ret.z = a.z	+ b.z;

    return ret;
}

Point
operator -(const Point& a, const Point& b)
{
    Point	ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;

    return ret;
}

Point
operator *(double c, const Point& a)
{
    Point	ret;
    ret.x = c * a.x;
    ret.y = c * a.y;
    ret.z = c * a.z;

    return ret;
}

Point
zero(Point)
{
    Point	ret;
    ret.x = 0;
    ret.y = 0;
    ret.z = 0;

    return ret;
}

}	// namespace geometry_msgs

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
*  class PoseTrackingServo						*
************************************************************************/
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
    using odom_t	 = nav_msgs::Odometry;
    using odom_cp	 = nav_msgs::OdometryConstPtr;
    using vector3_t	 = Eigen::Vector3d;
    using angle_axis_t	 = Eigen::AngleAxisd;
    using pid_t		 = control_toolbox::Pid;
    using lpf_t		 = aist_utility::ButterworthLPF<double, raw_pose_t>;
    using extrapolator_t = aist_utility::SplineExtrapolator<raw_pose_t, 3>;

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
		PoseTrackingServo(const ros::NodeHandle& nh)		;
		~PoseTrackingServo()					;

    void	run()							;
    void	tick()							;

  private:
    void	readROSParams()						;
    ros::Duration
		expectedCycleTime() const
		{
		    return ros::Duration(servo_.getParameters().publish_period);
		}

    void	servoStatusCB(const int8_cp& msg)	;
    void	targetPoseCB(const pose_cp& msg)			;
    void	odometryCB(const odom_cp& msg)				;
    void	goalCB()						;
    void	preemptCB()						;
    void	calculatePoseError(const raw_pose_t& offset,
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
    void	initializePositionPIDs(double PIDConfig::* field,
				       double value)			;
    void	initializeOrientationPID(double PIDConfig::* field,
					 double value)			;
    void	initializePID(const PIDConfig& pid_config, pid_t& pid)	;

  // Target pose stuffs
    void	resetTargetPose()					;
    bool	haveRecentTargetPose(const ros::Duration& timeout) const;

  // Odometry stuffs
    void	resetOdometry()						;
    bool	haveRecentOdometry(const ros::Duration& timeout) const	;

  private:
    ros::NodeHandle		nh_;

    Servo			servo_;
    servo_status_t		servo_status_;

    ros::ServiceClient		reset_servo_status_;
    const ros::Subscriber	servo_status_sub_;
    const ros::Subscriber	target_pose_sub_;
    const ros::Subscriber	odom_sub_;
    const ros::Publisher	twist_pub_;
    const ros::Publisher	target_pose_debug_pub_;
    const ros::Publisher	ee_pose_debug_pub_;
    DurationArray&		durations_;

  // Action server stuffs
    server_t			tracker_srv_;
    goal_cp			current_goal_;

  // Dynamic reconfigure server
    ddr_t			ddr_;

  // Filters for input target pose
    int				input_low_pass_filter_half_order_;
    double			input_low_pass_filter_cutoff_frequency_;
    lpf_t			input_low_pass_filter_;

  // Spline extrapolator
    extrapolator_t		input_extrapolator_;

  // PIDs
    std::array<PIDConfig, 4>	pid_configs_;
    std::array<pid_t, 4>	pids_;

  // Servo inputs
    pose_t			target_pose_;
    odom_t			odom_;
    mutable std::mutex		input_mtx_;
};

PoseTrackingServo::PoseTrackingServo(const ros::NodeHandle& nh)
    :nh_(nh),
     servo_(nh_, createPlanningSceneMonitor("robot_description")),
     servo_status_(servo_status_t::INVALID),

     reset_servo_status_(nh_.serviceClient<std_srvs::Empty>(
			     "reset_servo_status")),
     servo_status_sub_(nh_.subscribe(servo_.getParameters().status_topic, 1,
				     &PoseTrackingServo::servoStatusCB, this)),
     target_pose_sub_(nh_.subscribe(
			  "/target_pose", 1,
			  &PoseTrackingServo::targetPoseCB, this,
			  ros::TransportHints().reliable().tcpNoDelay(true))),
     odom_sub_(nh_.subscribe(
		   "/odom", 1, &PoseTrackingServo::odometryCB, this,
		   ros::TransportHints().reliable().tcpNoDelay(true))),
     twist_pub_(nh_.advertise<twist_t>(
		    servo_.getParameters().cartesian_command_in_topic, 1)),
     target_pose_debug_pub_(nh_.advertise<pose_t>(
				"desired_pose", 1)),
     ee_pose_debug_pub_(nh_.advertise<pose_t>(
			    "actual_pose", 1)),
     durations_(servo_.durations()),

     tracker_srv_(nh_, "pose_tracking", false),
     current_goal_(nullptr),
     ddr_(ros::NodeHandle(nh_, "pose_tracking")),

     input_low_pass_filter_half_order_(3),
     input_low_pass_filter_cutoff_frequency_(7.0),
     input_low_pass_filter_(input_low_pass_filter_half_order_,
			    input_low_pass_filter_cutoff_frequency_ *
			    expectedCycleTime().toSec()),

     input_extrapolator_(),

     pid_configs_(),
     pids_(),

     target_pose_(),
     odom_(),
     input_mtx_()
{
    readROSParams();

  // Initialize input lowpass-filter
    input_low_pass_filter_.initialize(input_low_pass_filter_half_order_,
				      input_low_pass_filter_cutoff_frequency_ *
				      expectedCycleTime().toSec());

  // Initialize PID controllers
    for (size_t i = 0; i < pids_.size(); ++i)
	initializePID(pid_configs_[i], pids_[i]);

  // Setup action server
    tracker_srv_.registerGoalCallback(boost::bind(&PoseTrackingServo::goalCB,
						  this));
    tracker_srv_.registerPreemptCallback(boost::bind(
					     &PoseTrackingServo::preemptCB,
					     this));
    tracker_srv_.start();

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
				  pid_configs_[0].k_p,
				  boost::bind(&PoseTrackingServo
					      ::initializePositionPIDs,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for translation",
				  0.5, 300.0);
    ddr_.registerVariable<double>("linear_integral_gain",
				  pid_configs_[0].k_i,
				  boost::bind(&PoseTrackingServo
					      ::initializePositionPIDs,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for translation",
				  0.0, 20.0);
    ddr_.registerVariable<double>("linear_derivative_gain",
				  pid_configs_[0].k_d,
				  boost::bind(&PoseTrackingServo
					      ::initializePositionPIDs,
					      this, &PIDConfig::k_d, _1),
				  "Derivative gain for translation",
				  0.0, 20.0);

    ddr_.registerVariable<double>("angular_proportinal_gain",
				  pid_configs_[3].k_p,
				  boost::bind(&PoseTrackingServo
					      ::initializeOrientationPID,
					      this, &PIDConfig::k_p, _1),
				  "Proportional gain for rotation",
				  0.5, 300.0);
    ddr_.registerVariable<double>("angular_integral_gain",
				  pid_configs_[3].k_i,
				  boost::bind(&PoseTrackingServo
					      ::initializeOrientationPID,
					      this, &PIDConfig::k_i, _1),
				  "Integral gain for rotation",
				  0.0, 20.0);
    ddr_.registerVariable<double>("angular_derivative_gain",
				  pid_configs_[3].k_d,
				  boost::bind(&PoseTrackingServo
					      ::initializeOrientationPID,
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

    for (ros::Rate loop_rate(1.0 / servo_.getParameters().publish_period);
	 ros::ok(); )
    {
	tick();
	loop_rate.sleep();
    }

    spinner.stop();
    ros::waitForShutdown();
}

void
PoseTrackingServo::tick()
{
    durations_.tick_begin = (ros::Time::now() -
			     durations_.header.stamp).toSec();

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
      {
	doPostMotionReset();
	PoseTrackingResult	result;
	result.status = static_cast<int8_t>(servo_status_);
	tracker_srv_.setAborted(result);
	ROS_ERROR_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ABORTED["
			       << SERVO_STATUS_CODE_MAP.at(servo_status_)
			       << ']');
	return;
      }

      default:
	break;
    }

  // Check that target pose is recent enough.
    if (!haveRecentTargetPose(current_goal_->timeout))
    {
    	doPostMotionReset();
	PoseTrackingResult	result;
	result.status = PoseTrackingResult::INPUT_TIMEOUT;
    	tracker_srv_.setAborted(result);
        ROS_ERROR_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ABORTED["
    			       << "The target pose was not updated recently."
			       << ']');

    	return;
    }

  // Compute positional and angular errors.
    vector3_t		positional_error;
    angle_axis_t	angular_error;
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
	PoseTrackingResult	result;
	result.status = PoseTrackingResult::NO_ERROR;
	tracker_srv_.setSucceeded(result);
	ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal SUCCEEDED");

	return;
    }

  // Publish tracking result as feedback.
    PoseTrackingFeedback	feedback;
    feedback.positional_error[0] = positional_error(0);
    feedback.positional_error[1] = positional_error(1);
    feedback.positional_error[2] = positional_error(2);
    feedback.angular_error	 = angular_error.angle();
    feedback.status		 = static_cast<int8_t>(servo_status_);
    tracker_srv_.publishFeedback(feedback);

  // Compute servo command from PID controller output and send it
  // to the Servo object, for execution
    twist_pub_.publish(calculateTwistCommand(positional_error, angular_error));

    durations_.twist_out = (ros::Time::now() -
			    durations_.header.stamp).toSec();
}

/*
 *  private member functions
 */
void
PoseTrackingServo::readROSParams()
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
    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "input_low_pass_filter_half_order",
				      input_low_pass_filter_half_order_);
    error += !rosparam_shortcuts::get(LOGNAME, nh,
				      "input_low_pass_filter_cutoff_frequency",
				      input_low_pass_filter_cutoff_frequency_);

  // Setup PID configurations
    double	windup_limit;
    error += !rosparam_shortcuts::get(LOGNAME, nh, "windup_limit",
				      windup_limit);
    for (size_t i = 0; i < pids_.size(); ++i)
    {
	pid_configs_[i].dt	     = expectedCycleTime().toSec();
	pid_configs_[i].windup_limit = windup_limit;
    }

    error += !rosparam_shortcuts::get(LOGNAME, nh, "x_proportional_gain",
				      pid_configs_[0].k_p);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "y_proportional_gain",
				      pid_configs_[1].k_p);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "z_proportional_gain",
				      pid_configs_[2].k_p);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "x_integral_gain",
				      pid_configs_[0].k_i);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "y_integral_gain",
				      pid_configs_[1].k_i);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "z_integral_gain",
				      pid_configs_[2].k_i);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "x_derivative_gain",
				      pid_configs_[0].k_d);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "y_derivative_gain",
				      pid_configs_[1].k_d);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "z_derivative_gain",
				      pid_configs_[2].k_d);

    error += !rosparam_shortcuts::get(LOGNAME, nh, "angular_proportional_gain",
				      pid_configs_[3].k_p);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "angular_integral_gain",
				      pid_configs_[3].k_i);
    error += !rosparam_shortcuts::get(LOGNAME, nh, "angular_derivative_gain",
				      pid_configs_[3].k_d);

    rosparam_shortcuts::shutdownIfError(ros::this_node::getName(), error);
}

void
PoseTrackingServo::servoStatusCB(const int8_cp& msg)
{
    servo_status_ = static_cast<servo_status_t>(msg->data);
}

void
PoseTrackingServo::targetPoseCB(const pose_cp& msg)
{
    std::lock_guard<std::mutex> lock(input_mtx_);

    target_pose_ = *msg;

  // Prevent doTransform from copying a stamp of 0,
  // which will cause the haveRecentTargetPose check to fail servo motions
    if (target_pose_.header.stamp == ros::Time(0))
	target_pose_.header.stamp = ros::Time::now();

    durations_.header	      = target_pose_.header;
    durations_.target_pose_in = (ros::Time::now() -
				 durations_.header.stamp).toSec();

  // If the target pose is not defined in planning frame, transform it.
    if (target_pose_.header.frame_id != servo_.getParameters().planning_frame)
    {
	auto Tpt = tf2::eigenToTransform(servo_.getFrameTransform(
					     target_pose_.header.frame_id));
	Tpt.header.stamp    = target_pose_.header.stamp;
	Tpt.header.frame_id = servo_.getParameters().planning_frame;
	Tpt.child_frame_id  = target_pose_.header.frame_id;
	tf2::doTransform(target_pose_, target_pose_, Tpt);
    }

    input_extrapolator_.update(ros::Time::now(), target_pose_.pose);
}

void
PoseTrackingServo::odometryCB(const odom_cp& msg)
{
    std::lock_guard<std::mutex>	lock(input_mtx_);

    odom_ = *msg;

    if (odom_.header.stamp == ros::Time(0))
	odom_.header.stamp = ros::Time::now();

    auto	Tpb = tf2::eigenToTransform(servo_.getFrameTransform(
						odom_.child_frame_id));
    Tpb.header.stamp	= odom_.header.stamp;
    Tpb.header.frame_id	= servo_.getParameters().planning_frame;
    Tpb.child_frame_id	= odom_.child_frame_id;
}

void
PoseTrackingServo::goalCB()
{
    resetTargetPose();
    resetOdometry();

  // Wait a bit for a target pose message to arrive.
  // The target pose may get updated by new messages as the robot moves
  // (in a callback function).
    for (const auto start_time = ros::Time::now();
	 ros::Time::now() - start_time < DEFAULT_INPUT_TIMEOUT;
	 ros::Duration(0.001).sleep())
    {
	if (haveRecentTargetPose(DEFAULT_INPUT_TIMEOUT))
	{
	    input_low_pass_filter_.reset(target_pose_.pose);

	    std_srvs::Empty	empty;
	    reset_servo_status_.call(empty);
	    servo_.start();

	    current_goal_ = tracker_srv_.acceptNewGoal();
	    ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ACCEPTED["
				  << current_goal_->target_offset << ']');

	    if (tracker_srv_.isPreemptRequested())
		preemptCB();

	    return;
	}
    }

  // No target pose available recently.
  // Once accept the pending goal and then abort it immediately.
    current_goal_ = tracker_srv_.acceptNewGoal();
    PoseTrackingResult	result;
    result.status = static_cast<int8_t>(servo_status_);
    tracker_srv_.setAborted(result);

    ROS_ERROR_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) Cannot accept goal because no target pose available recently.");
}

void
PoseTrackingServo::preemptCB()
{
    doPostMotionReset();
    PoseTrackingResult	result;
    result.status = static_cast<int8_t>(servo_status_);
    tracker_srv_.setPreempted(result);
    ROS_WARN_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal CANCELED");
}

void
PoseTrackingServo::calculatePoseError(const raw_pose_t& offset,
				      vector3_t& positional_error,
				      angle_axis_t& angular_error) const
{
    pose_t	target_pose;
    {
	std::lock_guard<std::mutex> lock(input_mtx_);

	target_pose = target_pose_;
    }

  // Apply input extrapolator
    // target_pose.pose = input_extrapolator_.pos(ros::Time::now());
    // normalize(target_pose.pose.orientation);
  //target_pose.pose.position = input_extrapolator_.pos(ros::Time::now());

  // Apply input low-pass filter
    target_pose.pose = input_low_pass_filter_.filter(target_pose.pose);
    normalize(target_pose.pose.orientation);

  // Correct target_pose by offset
    tf2::Transform	target_transform;
    tf2::fromMsg(target_pose.pose, target_transform);
    tf2::Transform	offset_transform;
    tf2::fromMsg(offset, offset_transform);
    tf2::toMsg(target_transform * offset_transform, target_pose.pose);

  // Publish corrected pose for debugging
    target_pose_debug_pub_.publish(target_pose);

  // Compute errors
    const auto	Tpe = servo_.getEEFrameTransform();
    positional_error(0) = target_pose.pose.position.x - Tpe.translation()(0);
    positional_error(1) = target_pose.pose.position.y - Tpe.translation()(1);
    positional_error(2) = target_pose.pose.position.z - Tpe.translation()(2);

    Eigen::Quaterniond	q_desired;
    tf2::convert(target_pose.pose.orientation, q_desired);
    angular_error = q_desired * Eigen::Quaterniond(Tpe.rotation()).inverse();

  // For debugging
    ee_pose_debug_pub_.publish(tf2::toMsg(
				   tf2::Stamped<Eigen::Isometry3d>(
				       Tpe, ros::Time::now(),
				       servo_.getParameters().planning_frame)));
}

PoseTrackingServo::twist_cp
PoseTrackingServo::calculateTwistCommand(const vector3_t& positional_error,
					 const angle_axis_t& angular_error)
{
  // use the shared pool to create a message more efficiently
    const auto	msg = moveit::util::make_shared_from_pool<twist_t>();
    {
	std::lock_guard<std::mutex> lock(input_mtx_);

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

void
PoseTrackingServo::doPostMotionReset()
{
    stopMotion();

  // Reset error integrals and previous errors of PID controllers
    for (auto&& pid : pids_)
	pid.reset();
}

void
PoseTrackingServo::stopMotion()
{
  // Send a 0 command to Servo to halt arm motion
    const auto	msg = moveit::util::make_shared_from_pool<twist_t>();
    {
	std::lock_guard<std::mutex> lock(input_mtx_);

	msg->header.frame_id = target_pose_.header.frame_id;
    }
    msg->header.stamp = ros::Time::now();
    twist_pub_.publish(msg);
}

// Low-pass filter stuffs
void
PoseTrackingServo::updateInputLowPassFilter(int half_order,
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
void
PoseTrackingServo::initializePositionPIDs(double PIDConfig::* field,
					  double value)
{
    std::lock_guard<std::mutex> lock(input_mtx_);

    for (size_t i = 0; i < 3; ++i)
    {
	pid_configs_[i].*field = value;
	initializePID(pid_configs_[i], pids_[i]);
    }
}

void
PoseTrackingServo::initializeOrientationPID(double PIDConfig::* field,
					    double value)
{
    std::lock_guard<std::mutex> lock(input_mtx_);

    pid_configs_[3].*field = value;
    initializePID(pid_configs_[3], pids_[3]);
}

void
PoseTrackingServo::initializePID(const PIDConfig& pid_config, pid_t& pid)
{
    pid.initPid(pid_config.k_p, pid_config.k_i, pid_config.k_d,
		pid_config.windup_limit, -pid_config.windup_limit, true);
}

// Target pose stuffs
void
PoseTrackingServo::resetTargetPose()
{
    std::lock_guard<std::mutex>	lock(input_mtx_);

    target_pose_	      = pose_t();
    target_pose_.header.stamp = ros::Time(0);
}

bool
PoseTrackingServo::haveRecentTargetPose(const ros::Duration& timeout) const
{
    std::lock_guard<std::mutex> lock(input_mtx_);

    return (ros::Time::now() - target_pose_.header.stamp < timeout);
}

// Odometry stuffs
void
PoseTrackingServo::resetOdometry()
{
    std::lock_guard<std::mutex>	lock(input_mtx_);

    odom_	       = odom_t();
    odom_.header.stamp = ros::Time(0);
}

bool
PoseTrackingServo::haveRecentOdometry(const ros::Duration& timeout) const
{
    std::lock_guard<std::mutex> lock(input_mtx_);

    return (ros::Time::now() - odom_.header.stamp < timeout);
}

}	// namespace aist_moveit_servo

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, LOGNAME);

    ros::NodeHandle	nh("~");
    aist_moveit_servo::PoseTrackingServo	servo(nh);
    servo.run();

    return 0;
}
