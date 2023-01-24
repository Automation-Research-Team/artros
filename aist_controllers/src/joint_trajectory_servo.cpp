/*
 *  \file	joint_trajectory_servo.cpp
 *  \brief	ROS pose tracker of aist_controllers::PoseTracking type
 */
#include <mutex>
#include <boost/optional.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <aist_controllers/PoseTrackingAction.h>
#include <aist_utility/butterworth_lpf.h>
#include <aist_utility/spline_extrapolator.h>

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.

namespace
{
constexpr char		LOGNAME[]	  = "joint_trajectory_servo";
constexpr double	DEFAULT_LOOP_RATE = 100;
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
operator -(const Pose& a)
{
    Pose	ret;
    ret.position.x    = -a.position.x;
    ret.position.y    = -a.position.y;
    ret.position.z    = -a.position.z;
    ret.orientation.x = -a.orientation.x;
    ret.orientation.y = -a.orientation.y;
    ret.orientation.z = -a.orientation.z;
    ret.orientation.w = -a.orientation.w;

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
    using pose_t	= geometry_msgs::PoseStamped;
    using pose_cp	= geometry_msgs::PoseStampedConstPtr;
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
    void	updateInputLowPassFilter(int half_order,
					 double cutoff_frequency)	;
    bool	haveRecentTargetPose(const ros::Duration& timeout) const;

  private:
    ros::NodeHandle				_nh;
    ros::Subscriber				_target_pose_sub;
    const ros::Publisher			_command_pub;
    const ros::Publisher			_target_pose_pub;  // for debug
    const ros::Publisher			_ee_pose_pub;	   // for debug
    ros::Rate					_loop_rate;

  // Action server stuffs
    server_t					_tracker_srv;
    boost::shared_ptr<const server_t::Goal>	_current_goal;

  // Dynamic reconfigure server
    ddynamic_reconfigure::DDynamicReconfigure	_ddr;

  // MoveIt stuffs
    const planning_scene_monitor_p		_planning_scene_monitor;
    moveit::core::RobotStatePtr			_current_state;
    const moveit::core::JointModelGroup* const	_joint_model_group;
    const std::string				_ee_frame;
    
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
     _target_pose_sub(_nh.subscribe<pose_t>(
			  "/target_pose", 1,
			  &JointTrajectoryServo::targetPoseCB, this,
			  ros::TransportHints().reliable().tcpNoDelay(true))),
     _command_pub(_nh.advertise<trajectory_t>(
		      _nh.param<std::string>("command_out_topic",
					     "/arm_controller/command"), 1)),
     _target_pose_pub(_nh.advertise<pose_t>("target_pose_debug", 1)),
     _ee_pose_pub(_nh.advertise<pose_t>("ee_pose_debug", 1)),
     _loop_rate(_nh.param<double>("loop_rate", DEFAULT_LOOP_RATE)),

     _tracker_srv(_nh, "pose_tracking", false),
     _current_goal(nullptr),
     
     _ddr(ros::NodeHandle(_nh)),

     _planning_scene_monitor(createPlanningSceneMonitor("robot_description")),
     _current_state(_planning_scene_monitor->getStateMonitor()
					   ->getCurrentState()),
     _joint_model_group(_current_state->getJointModelGroup(
			    _nh.param<std::string>("move_group_name", "arm"))),
     _ee_frame(_nh.param<std::string>("ee_frame_name", "magnet_tip_link")),

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
	stopMotion();
	
	PoseTrackingResult	result;
	result.status = PoseTrackingResult::INPUT_TIMEOUT;
    	_tracker_srv.setAborted(result);

	ROS_ERROR_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal ABORTED["
    			       << "Target pose not recently updated]");
	return;
    }
    
    const auto	now = ros::Time::now();
    pose_t	target_pose;
    target_pose.header.frame_id = planning_frame();
    target_pose.header.stamp	= now;
    target_pose.pose		= _input_extrapolator.pos(now);
    normalize(target_pose.pose.orientation);
    
  // Correct target pose by offset given in current goal.
    tf2::Transform      target_transform;
    tf2::fromMsg(target_pose.pose, target_transform);
    tf2::Transform      offset_transform;
    tf2::fromMsg(_current_goal->target_offset, offset_transform);
    tf2::toMsg(target_transform * offset_transform, target_pose.pose);

  // Publish corrected target pose for debugging.
    _target_pose_pub.publish(target_pose);

  // Compute joint positions for target pose with IK solver.
    auto	state = *_current_state;
    if (!state.setFromIK(_joint_model_group, target_pose.pose))
    {
	stopMotion();
	
	PoseTrackingResult	result;
	result.status = PoseTrackingResult::HALT_FOR_SINGULARITY;
    	_tracker_srv.setAborted(result);
	ROS_ERROR_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal ABORTED["
    			       << "Failed to solve IK]");
	return;
    }
    
  // Set joint positions to trajectory and publish.
    auto	positions = state.getVariablePositions();
    for (auto&& position : _joint_trajectory.points[0].positions)
	position = *positions++;
    _command_pub.publish(_joint_trajectory);

  // Get current transform from end-effector frame to planning frame.
    const auto	ee_frame_transform = _current_state->getGlobalLinkTransform(
					planning_frame()).inverse()
				   * _current_state->getGlobalLinkTransform(
					_ee_frame);
	
  // Publish tracking result as feedback.
    // PoseTrackingFeedback	feedback;
    // feedback.positional_error[0] = positional_error(0);
    // feedback.positional_error[1] = positional_error(1);
    // feedback.positional_error[2] = positional_error(2);
    // feedback.angular_error	 = angular_error.angle();
    // feedback.status		 = static_cast<int8_t>(_servo_status_);
    // _tracker_srv.publishFeedback(feedback);

  // For debugging
    _ee_pose_pub.publish(tf2::toMsg(tf2::Stamped<Eigen::Isometry3d>(
					ee_frame_transform, now,
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
    _current_goal = _tracker_srv.acceptNewGoal();
    ROS_INFO_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal ACCEPTED["
			  << _current_goal->target_offset << ']');
}

void
JointTrajectoryServo::preemptCB()
{
    stopMotion();

    PoseTrackingResult	result;
    result.status = PoseTrackingResult::NO_ERROR;
    _tracker_srv.setPreempted(result);
    ROS_WARN_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) goal CANCELED");
}

void
JointTrajectoryServo::stopMotion()
{
    trajectory_t	empty_trajectory;
    empty_trajectory.joint_names = _joint_trajectory.joint_names;

    _command_pub.publish(empty_trajectory);
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

    _input_low_pass_filter.reset(_input_extrapolator.xp());
}

bool
JointTrajectoryServo::haveRecentTargetPose(const ros::Duration& timeout) const
{
    std::lock_guard<std::mutex> lock(_input_mtx);

    return (ros::Time::now() - _input_extrapolator.tp() < timeout);
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
