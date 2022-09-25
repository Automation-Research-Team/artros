/*
 *  \file	pose_tracking_servo.cpp
 *  \brief	ROS tracker of aist_controllers::PoseHeadAction type
 */
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/TransformStamped.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>

#include <aist_controllers/PoseHeadAction.h>

namespace aist_controllers
{
static const std::string LOGNAME = "pose_tracking_servo";

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

    class StatusMonitor
    {
      private:
	using StatusCode	= moveit_servo::StatusCode;

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
		    LOGNAME, "Servo status: "
		    << moveit_servo::SERVO_STATUS_CODE_MAP.at(_status));
	    }
	}

      private:
	ros::Subscriber			_status_sub;
	moveit_servo::StatusCode	_status;
    };

  public:
		PoseTrackingServo()					;
		~PoseTrackingServo()					;

    void	run()							;

  private:
    static planning_scene_monitor_p
		create_planning_scene_monitor(
		    const std::string& robot_description)		;
    void	set_active_cb(std_srvs::SetBool::Request&  req,
			      std_srvs::SetBool::Response& res)		;
    void	preempt_cb()						;

  private:
    ros::NodeHandle		_nh;
    planning_scene_monitor_p	_planning_scene_monitor;
    moveit_servo::PoseTracking	_tracker;
    StatusMonitor		_status_monitor;

    Eigen::Vector3d		_lin_tol{0.01, 0.01, 0.01};
    double			_rot_tol = 0.1;

    std::thread			_move_to_pose_thread;
};

PoseTrackingServo::PoseTrackingServo()
    :_nh("~"),
     _planning_scene_monitor(create_planning_scene_monitor(
				 "robot_description")),
     _tracker(_nh, _planning_scene_monitor),
     _status_monitor(_nh, "status"),
     _lin_tol{0.01, 0.01, 0.01},
     _rot_tol(0.1),
     _move_to_pose_thread()
{
    ROS_INFO_STREAM_NAMED(LOGNAME, "PoseTrackingServo initialized.");
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
	    LOGNAME, "Error in setting up the PlanningSceneMonitor.");
	exit(EXIT_FAILURE);
    }

    monitor->startSceneMonitor();
    monitor->startWorldGeometryMonitor(
	PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
	PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
	false /* skip octomap monitor */);
    monitor->startStateMonitor();

    ROS_INFO_STREAM_NAMED(LOGNAME, "PlanningSceneMonitor started.");

    return monitor;
}

void
PoseTrackingServo::set_active_cb(std_srvs::SetBool::Request&  req,
				 std_srvs::SetBool::Response& res)
{
    if (req.data)
    {
	_move_to_pose_thread = std::thread([this]
					   {
					       this->_tracker.moveToPose(
						   this->_lin_tol,
						   this->_rot_tol, 0.1);
					   });

	ROS_INFO_STREAM_NAMED(LOGNAME,
			      "(PoseTrackingServo) tracker activated");
    }
    else
    {
	if (_move_to_pose_thread.joinable())
	    _move_to_pose_thread.join();
	ROS_INFO_STREAM_NAMED(LOGNAME,
			      "(PoseTrackingServo) tracker deactivated");
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
