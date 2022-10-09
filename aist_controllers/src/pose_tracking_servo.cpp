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
    return out << pose.position.x << ' '
	       << pose.position.y << ' '
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

    class StatusMonitor
    {
      private:
	using StatusCode = moveit_servo::StatusCode;

      public:
	StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
	    :_status_sub(nh.subscribe(topic, 1, &StatusMonitor::status_cb,
				      this)),
	     _status(StatusCode::INVALID)
	{
	}

      private:
	void
	status_cb(const std_msgs::Int8ConstPtr& msg)
	{
	    const auto	latest_status = static_cast<StatusCode>(msg->data);
	    if (latest_status != _status)
	    {
		_status = latest_status;

		ROS_INFO_STREAM_NAMED(
		    LOGNAME, "(PoseTrackingServo) Servo status: "
		    << moveit_servo::SERVO_STATUS_CODE_MAP.at(_status));
	    }
	}

      private:
	ros::Subscriber			_status_sub;
	moveit_servo::StatusCode	_status;
    };

  public:
		PoseTrackingServo(const std::string& action_ns)		;
		~PoseTrackingServo()					;

    void	run()							;

  private:
    static planning_scene_monitor_p
		create_planning_scene_monitor(
		    const std::string& robot_description)		;
    void	execute_cb(const PoseTrackingGoalConstPtr& goal)		;

  private:
    ros::NodeHandle		_nh;
    planning_scene_monitor_p	_planning_scene_monitor;
    moveit_servo::PoseTracking	_tracker;
    StatusMonitor		_status_monitor;

    server_t			_tracker_srv;
    const ros::Publisher	_target_pose_pub;

    std::thread			_move_to_pose_thread;
};

PoseTrackingServo::PoseTrackingServo(const std::string& action_ns)
    :_nh("~"),
     _planning_scene_monitor(create_planning_scene_monitor(
				 "robot_description")),
     _tracker(_nh, _planning_scene_monitor),
     _status_monitor(_nh, "status"),
     _tracker_srv(_nh, action_ns,
		  boost::bind(&PoseTrackingServo::execute_cb, this, _1),
		  false),
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
    using	status_t = moveit_servo::PoseTrackingStatusCode;
    
    ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ACCEPTED["
			  << goal->target.header.frame_id << '@'
			  << goal->target.pose << ']');

    _tracker.resetTargetPose();
    
    status_t	tracking_status = status_t::INVALID;
    
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
	 ros::ok() && tracking_status == status_t::INVALID; rate.sleep())
    {
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
	
	geometry_msgs::PoseStamped	target_pose = goal->target;
	target_pose.header.stamp = ros::Time::now();
	_target_pose_pub.publish(target_pose);
    }

    if (_move_to_pose_thread.joinable())
	_move_to_pose_thread.join();
    
    switch (tracking_status)
    {
      case status_t::SUCCESS:
	_tracker_srv.setSucceeded();
	ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal SUCCEEDED");
	break;
      case status_t::NO_RECENT_TARGET_POSE:
	_tracker_srv.setAborted();
	ROS_ERROR_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ABORTED[no recent target pose]");
	break;
      case status_t::NO_RECENT_END_EFFECTOR_POSE:
	_tracker_srv.setAborted();
	ROS_ERROR_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ABORTED[no recent end effector pose]");
	break;
      default:
	_tracker_srv.setAborted();
	ROS_ERROR_STREAM_NAMED(LOGNAME, "(PoseTrackingServo) goal ABORTED[unknown error]");
	break;
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

    aist_controllers::PoseTrackingServo	servo("pose_tracking");
    servo.run();

    return 0;
}
