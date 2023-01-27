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
#include <aist_utility/geometry_msgs.h>

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.

namespace
{
constexpr char		LOGNAME[]	  = "joint_trajectory_servo";
constexpr double	DEFAULT_LOOP_RATE = 100;
const	  ros::Duration	DEFAULT_INPUT_TIMEOUT{0.5};	// sec
constexpr double	ROBOT_STATE_WAIT_TIME = 10.0;  // seconds
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
    const std::string&
		root_frame()					const	;
    void	run()							;

  private:
    static planning_scene_monitor_p
		createPlanningSceneMonitor(
		    const std::string& robot_description,
		    const std::string& move_group_name)			;
    void	tick()							;
    void	targetPoseCB(const pose_cp& msg)			;
    void	goalCB()						;
    void	preemptCB()						;
    void	stopMotion()						;
    void	updateInputLowPassFilter(int half_order,
					 double cutoff_frequency)	;
    bool	haveRecentTargetPose(const ros::Duration& timeout) const;
    geometry_msgs::TransformStamped
		getFrameTransform(const std::string& frame)	const	;

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
    const std::string				_move_group_name;
    const planning_scene_monitor_p		_planning_scene_monitor;
    moveit::core::RobotStatePtr			_state;
    ros::Time					_state_stamp;
    mutable std::mutex				_state_mutex;
    const moveit::core::JointModelGroup* const	_joint_model_group;
    const std::string				_ee_frame;

  // Filter and extrapolator for input target pose
    pose_t			_target_pose;
    int				_input_low_pass_filter_half_order;
    double			_input_low_pass_filter_cutoff_frequency;
    aist_utility::ButterworthLPF<double, pose_raw>
				_input_low_pass_filter;
    aist_utility::SplineExtrapolator<pose_raw>
				_input_extrapolator;
    mutable std::mutex		_input_mutex;

  // Output joint trajectory
    trajectory_t		_joint_trajectory;
    std::map<std::string, int>	_active_joint_indices;
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

     _move_group_name(_nh.param<std::string>("move_group_name", "arm")),
     _planning_scene_monitor(createPlanningSceneMonitor("robot_description",
							_move_group_name)),
     _state(_planning_scene_monitor->getStateMonitor()->getCurrentState()),
     _state_stamp(0),
     _state_mutex(),
     _joint_model_group(_state->getJointModelGroup(_move_group_name)),
     _ee_frame(_nh.param<std::string>("ee_frame_name", "magnet_tip_link")),

     _target_pose(),
     _input_low_pass_filter_half_order(3),
     _input_low_pass_filter_cutoff_frequency(7.0),
     _input_low_pass_filter(_input_low_pass_filter_half_order,
			    _input_low_pass_filter_cutoff_frequency *
			    _loop_rate.expectedCycleTime().toSec()),
     _input_extrapolator(),
     _input_mutex(),

     _joint_trajectory()
{
  // Initialize input lowpass-filter
    _input_low_pass_filter.initialize(_input_low_pass_filter_half_order,
				      _input_low_pass_filter_cutoff_frequency *
				      _loop_rate.expectedCycleTime().toSec());

  // Setup output joint trajectory
    _joint_trajectory.header.stamp    = ros::Time(0);
    // _joint_trajectory.header.frame_id = _nh.param<std::string>(
    // 					    "planning_frame", "arm_base_link");
    _joint_trajectory.header.frame_id = _state->getRobotModel()
					      ->getRootLinkName();
    _joint_trajectory.joint_names     = _joint_model_group
					    ->getActiveJointModelNames();

  // Put current joint positions to joint trajectory
    trajectory_msgs::JointTrajectoryPoint	point;
    _state->copyJointGroupPositions(_joint_model_group, point.positions);
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


    ROS_INFO_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) server started["
			  << "planning_frame=" << planning_frame()
			  << ", root_frame=" << root_frame() << ']');
}

JointTrajectoryServo::~JointTrajectoryServo()
{
}

const std::string&
JointTrajectoryServo::planning_frame() const
{
    return _joint_trajectory.header.frame_id;
}

const std::string&
JointTrajectoryServo::root_frame() const
{
    return _joint_model_group->getParentModel().getRootLinkName();
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
    const std::string& robot_description, const std::string& move_group_name)
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

  // Confirm the planning scene monitor is ready to be used
    monitor->getStateMonitor()->enableCopyDynamics(true);

    if (!monitor->getStateMonitor()
		->waitForCompleteState(move_group_name, ROBOT_STATE_WAIT_TIME))
    {
	ROS_FATAL_STREAM_NAMED(LOGNAME, "Timeout waiting for current state");
	exit(EXIT_FAILURE);
    }

    ROS_INFO_STREAM_NAMED(LOGNAME, "PlanningSceneMonitor started");

    return monitor;
}

void
JointTrajectoryServo::tick()
{
    {
	std::lock_guard<std::mutex> lock(_state_mutex);
	_state = _planning_scene_monitor->getStateMonitor()->getCurrentState();
	_state_stamp = ros::Time::now();
    }

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

  // Compute target pose at current time by extrapolation.
    pose_t	target_pose;
    target_pose.header.frame_id	= planning_frame();
    target_pose.header.stamp	= _state_stamp;
    {
    	std::lock_guard<std::mutex>	lock(_input_mutex);
    	target_pose.pose = _input_extrapolator.pos(target_pose.header.stamp);
    }
    normalize(target_pose.pose.orientation);
  //target_pose = _target_pose;

  // Correct target pose by offset given in current goal.
    tf2::Transform      target_transform;
    tf2::fromMsg(target_pose.pose, target_transform);
    tf2::Transform      offset_transform;
    tf2::fromMsg(_current_goal->target_offset, offset_transform);
    tf2::toMsg(target_transform * offset_transform, target_pose.pose);

  // Publish corrected target pose for debugging.
    _target_pose_pub.publish(target_pose);

  // Compute joint positions for target pose with IK solver.
    std::cerr << "*** OK3" << std::endl;
    auto	state = *_state;
    if (!state.setFromIK(_joint_model_group, target_pose.pose, _ee_frame))
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
    std::cerr << "*** OK4" << std::endl;
    _state->copyJointGroupPositions(_joint_model_group,
				    _joint_trajectory.points[0].positions);
    std::cerr << "current:";
    for (const auto& pos : _joint_trajectory.points[0].positions)
	std::cerr << ' ' << pos;
    state.copyJointGroupPositions(_joint_model_group,
				  _joint_trajectory.points[0].positions);
    std::cerr << "\ntarget: ";
    for (const auto& pos : _joint_trajectory.points[0].positions)
	std::cerr << ' ' << pos;
    std::cerr << std::endl;
  //_command_pub.publish(_joint_trajectory);
    
  // Get current transform from end-effector frame to planning frame.
    std::cerr << "*** OK5" << std::endl;
    const auto	ee_frame_transform = getFrameTransform(_ee_frame);

  // Publish current end-effecgtor pose for debugging.
    _ee_pose_pub.publish(aist_utility::toPose(ee_frame_transform));
}

void
JointTrajectoryServo::targetPoseCB(const pose_cp& target_pose)
{
    std::lock_guard<std::mutex>	lock(_input_mutex);

    _target_pose = *target_pose;
    if (_target_pose.header.stamp == ros::Time(0))
	_target_pose.header.stamp = ros::Time::now();

  // Transform given pose to planning frame if necessary.
    if (_target_pose.header.frame_id != root_frame())
	tf2::doTransform(_target_pose, _target_pose,
			 getFrameTransform(_target_pose.header.frame_id));

    _input_extrapolator.update(ros::Time::now(),
			       _input_low_pass_filter.filter(_target_pose.pose));
}

void
JointTrajectoryServo::goalCB()
{
    for (const auto start_time = ros::Time::now();
	 ros::Time::now() - start_time < DEFAULT_INPUT_TIMEOUT;
	 ros::Duration(0.001).sleep())
    {
	if (haveRecentTargetPose(DEFAULT_INPUT_TIMEOUT))
	{
	    _input_low_pass_filter.reset(_target_pose.pose);

	    _current_goal = _tracker_srv.acceptNewGoal();
	    ROS_INFO_STREAM_NAMED(LOGNAME,
				  "(JointTrajectoryServo) goal ACCEPTED["
				  << _current_goal->target_offset << ']');
	    return;
	}
    }

  // No target pose available recently.
  // Once accept the pending goal and then abort it immediately.
    _tracker_srv.acceptNewGoal();
    PoseTrackingResult	result;
    result.status = PoseTrackingResult::INPUT_TIMEOUT;
    _tracker_srv.setAborted(result);

    ROS_ERROR_STREAM_NAMED(LOGNAME, "(JointTrajectoryServo) Cannot accept goal because no target pose available recently.");
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
  // Stop JointTrajectoryController by publishing empty trajectory.
    trajectory_t	empty_trajectory;
    empty_trajectory.joint_names = _joint_trajectory.joint_names;
    _command_pub.publish(empty_trajectory);

  // Invalidate buffered taget pose.
    _target_pose.header.stamp = ros::Time(0);
}

// Low-pass filter stuffs
void
JointTrajectoryServo::updateInputLowPassFilter(int half_order,
					       double cutoff_frequency)
{
    std::lock_guard<std::mutex>	lock(_input_mutex);

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
    std::lock_guard<std::mutex>	lock(_input_mutex);

    return (ros::Time::now() - _target_pose.header.stamp < timeout);
}

// Transform from given frame to planning frame
geometry_msgs::TransformStamped
JointTrajectoryServo::getFrameTransform(const std::string& frame) const
{
    std::lock_guard<std::mutex>	lock(_state_mutex);

    // auto transform = tf2::eigenToTransform(
    // 			_state->getGlobalLinkTransform(planning_frame())
    // 			.inverse() *
    // 			_state->getGlobalLinkTransform(frame));
  //transform.header.frame_id = planning_frame();
    auto transform = tf2::eigenToTransform(
			_state->getGlobalLinkTransform(frame));
    transform.header.frame_id = root_frame();
    transform.header.stamp    = _state_stamp;
    transform.child_frame_id  = frame;

    return transform;
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
