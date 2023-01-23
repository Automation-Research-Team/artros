/*
 *  \file	joint_trajectory_servo.cpp
 *  \brief	ROS pose tracker of aist_controllers::PoseTracking type
 */
#include <atomic>
#include <boost/optional.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <actionlib/server/simple_action_server.h>
#include <aist_controllers/PoseTrackingAction.h>
#include <aist_utility/butterworth_lpf.h>
#include <aist_utility/spline_extrapolator.h>

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.

namespace
{
constexpr char		LOGNAME[] = "joint_trajectory_servo";
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
    using server_t	= actionlib::SimpleActionServer<PoseTrackingAction>;
    using pose_t	= geoemtry_msgs::PoseStamped;
    using pose_cp	= geoemtry_msgs::PoseStampedConstPtr;
    using pose_raw	= geometry_msgs::Pose;
    using trajectory_t	= trajectory_msgs::JointTrajectory;
    
  public:
		JointTrajectoryServo(const ros::NodeHandle& nh)		;
		~JointTrajectoryServo()					;

    const std::string&
		planning_frame()				const	;
    void	run()							;

  private:
    static planning_scene_monitor_p
		createPlanningSceneMonitor(
		    const std::string& robot_description)		;
    void	tick()							;
    void	targetPoseCB(const pose_cp& msg)			;
    void	goalCB()						;
    void	preemptCB()						;

    void	stopMotion()						;
    void	doPostMotionReset()					;

  // Input low-pass filter stuffs
    void	updateInputLowPassFilter(int half_order,
					 double cutoff_frequency)	;

  // Target pose stuffs
    void	resetTargetPose()					;
    bool	haveRecentTargetPose(const ros::Duration& timeout) const;

  private:
    ros::NodeHandle				_nh;

  // MoveIt stuffs
    const planning_scene_monitor_p		_planning_scene_monitor;
    moveit::core::RobotStatePtr			_current_state;
    const moveit::core::JointModelGroup* const	_joint_model_group;

    ros::Subscriber				_target_pose_sub;
    const ros::Publisher			_command_pub;
    const ros::Publisher			_target_pose_pub;  // for debug
    const ros::Rate				_loop_rate;

  // Action server stuffs
    server_t					_tracker_srv;
    boost::shared_ptr<const server_t::Goal>	_current_goal;

  // Dynamic reconfigure server
    ddynamic_reconfigure::DDynamicReconfigure	_ddr;

  // Filter and extrapolator for input target pose
    int				_input_low_pass_filter_half_order;
    double			_input_low_pass_filter_cutoff_frequency;
    aist_utility::ButterworthLPF<double, pose_raw>
				_input_low_pass_filter;
    aist_utility::SplineExtrapolator<pose_raw>
				_input_extrapolator;
    mutable std::mutex		_input_mtx;

  // Output joint trajectory
    trajectory_t		_joint_trajectory;
};

JointTrajectoryServo::JointTrajectoryServo(const ros::NodeHandle& nh)
    :_nh(nh),
     _planning_scene_monitor(createPlanningSceneMonitor("robot_description")),
     _current_state(_planning_scene_monitor->getStateMonitor()
					   ->getCurrentState()),
     _joint_model_group(_current_state->getJointModelGroup(
			    _nh.param<std::string>("move_group_name", "arm"))),
     _target_pose_sub(_nh.subscribe<post_t>(
			  "/target_pose", 1,
			  &JointTrajectoryServo::targetPoseCB, this,
			  ros::TransportHints().reliable().tcpNodDelay(true))),
     _command_pub(_nh.advertise<trajectory_t>(_controller + "/command", 1)),
     _target_pose_pub(nh_.advertise<pose_t>("target_pose_debug", 1)),
     _ee_pose_pub(_nh.advertise<pose_t>("ee_pose_debug", 1)),
     _loop_rate(_nh.param<double>("loop_rate", DEFAULT_LOOP_RATE)),

     _tracker_srv(_nh, "pose_tracking", false),
     _current_goal(nullptr),
     _ddr(ros::NodeHandle(_nh)),

     _input_low_pass_filter_half_order(3),
     _input_low_pass_filter_cutoff_frequency(7.0),
     _input_low_pass_filter(_input_low_pass_filter_half_order,
			    _input_low_pass_filter_cutoff_frequency *
			    _loop_rate.expectedCycleTime().toSec()),
     _input_extrapolator(),
     _input_mtx(),

     _joint_trajectory()
{
  // Initialize input lowpass-filter
    _input_low_pass_filter.initialize(_input_low_pass_filter_half_order,
				      _input_low_pass_filter_cutoff_frequency *
				      _loop_rate.expectedCycleTime().toSec());

  // Setup output joint trajectory
    _joint_trajectory.header.stamp    = ros::Time(0);
    _joint_trajectory.header.frame_id = _nh.param<std::string>(
					    "planning_frame", "arm_base_link");
    _joint_trajectory.joint_names     = _joint_model_group
					    ->getActiveJointModelNames();
    
  // Put current joint positions to joint trajectory
    trajectory_msgs::JointTrajectoryPoint	point;
    _current_state->copyJointGroupPositions(_joint_model_group,
					    point.positions);
    point.time_from_start = _loop_rate.expectedCycleTime();
    _joint_trajectory.points.push_back(point);

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
    _ddr.publishServicesTopics();


    ROS_INFO_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) server started");
}

JointTrajectoryServo::~JointTrajectoryServo()
{
}

const std::string&
JointTrajectoryServo::planning_frame() const
{
    return _joint_trajectory.header.frame_id;
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
JointTrajectoryServo::tick()
{
    _current_state = _planning_scene_monitor->getStateMonitor()
					    ->getCurrentState();

    if (!_tracker_srv.isActive())
	return;

    if (!haveRecentTargetPose(_current_goal->timeout))
    {
	PoseTrackingResult	result;
	result.status = PoseTrackingResult::INPUT_TIMEOUT;
    	_tracker_srv.setAborted(result);

	ROS_ERROR_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal ABORTED["
    			       << "Target pose not recently updated]");
	return;
    }
    
    const auto	now = ros::Time::now();
    auto	target_pose = _input_extrapolator.pos(now);

  // Correct target pose by offset given in current goal.
    tf2::Transform      target_transform;
    tf2::fromMsg(target_pose.pose, target_transform);
    tf2::Transform      offset_transform;
    tf2::fromMsg(_current_goal->offset, offset_transform);
    tf2::toMsg(target_transform * offset_transform, target_pose.pose);

  // Publish corrected target pose for debugging.
    target_pose_pub_.publish(target_pose);

  // Compute joint positions for target pose with IK solver.
    joint_trajectory->header.stamp = ros::Time(0);
    

  // Continue sending PID controller output to Servo
  // until one of the following conditions is met:
  // - Servo status is not in emergency
  // - Target pose becomes outdated
  // - Command frame transform becomes outdated
  // - Goal tolerance is satisfied

  // Check servo status
    switch (_servo_status_)
    {
      case _servo_status_t::HALT_FOR_SINGULARITY:
      case _servo_status_t::HALT_FOR_COLLISION:
      case _servo_status_t::JOINT_BOUND:
      {
	doPostMotionReset();
	PoseTrackingResult	result;
	result.status = static_cast<int8_t>(_servo_status_);
	_tracker_srv.setAborted(result);
	ROS_ERROR_STREAM_NAMED(
	    LOGNAME, "(JointTrajectoryServo) goal ABORTED["
	    << moveit_servo::SERVO_STATUS_CODE_MAP.at(_servo_status_)
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
    if (_servo->getEEFrameTransform(_ee_frame_transform))
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
    feedback.status		 = static_cast<int8_t>(_servo_status_);
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
					planning_frame())));
}

void
JointTrajectoryServo::targetPoseCB(const pose_cp& target_pose)
{
    std::lock_guard<std::mutex> lock(_input_mtx);

    auto	pose = *target_pose;

    if (pose.header.frame_id != planning_frame())
    {
	auto	Tpt = tf2::eigenToTransform(
			_current_state->getGlobalLinkTransform(
			    pose.header.frame_id));
	Tpt.header.stamp    = pose.header.stamp;
	Tpt.header.frame_id = planning_frame();
	Tpt.child_frame_id  = pose.header.frame_id;
	tf2::doTransform(pose, pose, Tpt);
    }
    
    _input_extrapolator.update(ros::Time::now(),
			       _input_low_pass_filter.filter(pose.pose));
}

void
JointTrajectoryServo::goalCB()
{
    resetTargetPose();

    _current_goal = _tracker_srv.acceptNewGoal();
    ROS_INFO_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal ACCEPTED["
			  << _current_goal->target_offset << ']');
}

void
JointTrajectoryServo::preemptCB()
{
    doPostMotionReset();

    PoseTrackingResult	result;
    result.status = PoseTrackingResult::NO_ERROR;
    _tracker_srv.setPreempted(result);
    ROS_WARN_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal CANCELED");
}

void
JointTrajectoryServo::stopMotion()
{
  // Send a 0 command to Servo to halt arm motion
    const auto	msg = moveit::util::make_shared_from_pool<
			geometry_msgs::TwistStamped>();
    {
	std::lock_guard<std::mutex> lock(_input_mtx);

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

bool
JointTrajectoryServo::haveRecentTargetPose(const ros::Duration& timeout) const
{
    std::lock_guard<std::mutex> lock(_input_mtx);

    return (ros::Time::now() - _input_extrapolator.tp() < timeout);
}

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
