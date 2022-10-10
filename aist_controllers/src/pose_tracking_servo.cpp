/*
 *  \file	pose_head_servo.cpp
 *  \brief	ROS tracker of aist_controllers::PoseTrackingAction type
 */
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>

#include <aist_controllers/PoseTrackingAction.h>

namespace aist_controllers
{
static const std::string LOGNAME = "pose_tracking_servo";

/************************************************************************
*  static functions							*
************************************************************************/
std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Pose& pose)
{
    return out << pose.position.x << ','
	       << pose.position.y << ','
	       << pose.position.z << ';'
	       << pose.orientation.x << ','
	       << pose.orientation.y << ','
	       << pose.orientation.z << ','
	       << pose.orientation.w ;
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
    using server_t = actionlib::SimpleActionServer<PoseTrackingAction>;
    using servo_status_t    = moveit_servo::StatusCode;
    using tracking_status_t = moveit_servo::PoseTrackingStatusCode;

  public:
		PoseTrackingServo()					;
		~PoseTrackingServo()					;

    void	run()							;

  private:
    static planning_scene_monitor_p
		create_planning_scene_monitor(
		    const std::string& robot_description)		;
    void	execute_cb(const PoseTrackingGoalConstPtr& goal)	;
    void	servo_status_cb(const std_msgs::Int8ConstPtr& msg)	;

  private:
    ros::NodeHandle		_nh;
    planning_scene_monitor_p	_planning_scene_monitor;
    moveit_servo::PoseTracking	_tracker;
    servo_status_t		_servo_status;

    server_t			_tracker_srv;
    ros::Subscriber		_servo_status_sub;
    ros::ServiceClient		_reset_servo_status;
    const ros::Publisher	_target_pose_pub;

    std::thread			_move_to_pose_thread;
};

PoseTrackingServo::PoseTrackingServo()
    :_nh("~"),
     _planning_scene_monitor(create_planning_scene_monitor(
				 "robot_description")),
     _tracker(_nh, _planning_scene_monitor),
     _servo_status(servo_status_t::INVALID),
     _tracker_srv(_nh, "pose_tracking",
		  boost::bind(&PoseTrackingServo::execute_cb, this, _1),
		  false),
     _servo_status_sub(_nh.subscribe(
			   _tracker.servo_->getParameters().status_topic, 1,
			   &PoseTrackingServo::servo_status_cb, this)),
     _reset_servo_status(_nh.serviceClient<std_srvs::Empty>(
			     "reset_servo_status")),
     _target_pose_pub(_nh.advertise<geometry_msgs::PoseStamped>("target_pose",
								1, true)),
     _move_to_pose_thread()
{
    _tracker_srv.start();

    ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) initialized");
}

PoseTrackingServo::~PoseTrackingServo()
{
    _tracker.stopMotion();
    if (_move_to_pose_thread.joinable())
	_move_to_pose_thread.join();
}

void
PoseTrackingServo::run()
{
    ros::AsyncSpinner	spinner(8);
    spinner.start();

    ros::waitForShutdown();
}

planning_scene_monitor::PlanningSceneMonitorPtr
PoseTrackingServo::create_planning_scene_monitor(
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
PoseTrackingServo::execute_cb(const PoseTrackingGoalConstPtr& goal)
{
    ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ACCEPTED[("
			  << goal->target.pose << ")@"
			  << goal->target.header.frame_id << ']');

    _tracker.resetTargetPose();
    std_srvs::Empty	empty;
    _reset_servo_status.call(empty);
    tracking_status_t	tracking_status = tracking_status_t::INVALID;
    _tracker.servo_->start();

    _move_to_pose_thread = std::thread(
				[this,
				 lin_tol=goal->lin_tol, rot_tol=goal->rot_tol,
				 &tracking_status]
				{
				    tracking_status
					= this->_tracker.moveToPose(
					    {lin_tol[0],
					     lin_tol[1], lin_tol[2]},
					    rot_tol, 0.1);
				});

    for (ros::Rate rate(1 / _tracker.servo_->getParameters().publish_period);
	 ros::ok() && tracking_status == tracking_status_t::INVALID;
	 rate.sleep())
    {
	geometry_msgs::PoseStamped	target_pose = goal->target;
	target_pose.header.stamp = ros::Time::now();
	_target_pose_pub.publish(target_pose);
	
	if (_tracker_srv.isPreemptRequested())
	{
	    _tracker.stopMotion();
	    if (_move_to_pose_thread.joinable())
		_move_to_pose_thread.join();

	    _tracker_srv.setPreempted();
	    ROS_WARN_STREAM_NAMED(LOGNAME,
				  "(PoseTrackingServo) goal CANCELED");
	    return;
	}
	
	switch (_servo_status)
	{
	  case servo_status_t::HALT_FOR_SINGULARITY:
	  case servo_status_t::HALT_FOR_COLLISION:
	  case servo_status_t::JOINT_BOUND:
	    _tracker.stopMotion();
	    if (_move_to_pose_thread.joinable())
		_move_to_pose_thread.join();

	    _tracker_srv.setAborted();
	    ROS_ERROR_STREAM_NAMED(
		LOGNAME, "(PoseTrackingServo) goal ABORTED["
		<< moveit_servo::SERVO_STATUS_CODE_MAP.at(_servo_status)
		<< ']');
	    return;

	  default:
	    break;
	}
    }

    if (_move_to_pose_thread.joinable())
	_move_to_pose_thread.join();

    if (tracking_status == tracking_status_t::SUCCESS)
    {
	_tracker_srv.setSucceeded();
	ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal SUCCEEDED");
    }
    else
    {
	_tracker_srv.setAborted();
	ROS_ERROR_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ABORTED["
			       << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP
			              .at(tracking_status)
			       << ']');
    }
}

void
PoseTrackingServo::servo_status_cb(const std_msgs::Int8ConstPtr& msg)
{
    const auto	latest_servo_status = static_cast<servo_status_t>(msg->data);
    if (latest_servo_status != _servo_status)
    {
	_servo_status = latest_servo_status;

	switch (_servo_status)
	{
	  case servo_status_t::DECELERATE_FOR_SINGULARITY:
	  case servo_status_t::DECELERATE_FOR_COLLISION:
	    ROS_WARN_STREAM_NAMED(
		LOGNAME, "(PoseTrackingServo) Servo status["
		<< moveit_servo::SERVO_STATUS_CODE_MAP.at(_servo_status)
		<< ']');
	    break;
	  case servo_status_t::HALT_FOR_SINGULARITY:
	  case servo_status_t::HALT_FOR_COLLISION:
	  case servo_status_t::JOINT_BOUND:
	    ROS_ERROR_STREAM_NAMED(
		LOGNAME, "(PoseTrackingServo) Servo status["
		<< moveit_servo::SERVO_STATUS_CODE_MAP.at(_servo_status)
		<< ']');
	    break;
	  default:
	    ROS_INFO_STREAM_NAMED(
		LOGNAME, "(PoseTrackingServo) Servo status["
		<< moveit_servo::SERVO_STATUS_CODE_MAP.at(_servo_status)
		<< ']');
	    break;
	}
    }
}

}	// namespace aist_controllers

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, aist_controllers::LOGNAME);

    aist_controllers::PoseTrackingServo	servo;
    servo.run();

    return 0;
}
