/*
 *  \file	pose_head_servo.cpp
 *  \brief	ROS tracker of aist_controllers::PoseHeadAction type
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

#include <aist_controllers/PoseHeadAction.h>

namespace aist_controllers
{
static const std::string LOGNAME = "pose_head_servo";

/************************************************************************
*  class PoseHeadServo							*
************************************************************************/
class PoseHeadServo
{
  private:
    using planning_scene_monitor_t
		= planning_scene_monitor::PlanningSceneMonitor;
    using planning_scene_monitor_p
		= planning_scene_monitor::PlanningSceneMonitorPtr;
    using server_t = actionlib::SimpleActionServer<PoseHeadAction>;

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
		    LOGNAME, "(PoseHeadServo) Servo status: "
		    << moveit_servo::SERVO_STATUS_CODE_MAP.at(_status));
	    }
	}

      private:
	ros::Subscriber			_status_sub;
	moveit_servo::StatusCode	_status;
    };

  public:
		PoseHeadServo(const std::string& action_ns)		;
		~PoseHeadServo()					;

    void	run()							;

  private:
    static planning_scene_monitor_p
		create_planning_scene_monitor(
		    const std::string& robot_description)		;
    void	execute_cb(const PoseHeadGoalConstPtr& goal)		;

  private:
    ros::NodeHandle		_nh;
    planning_scene_monitor_p	_planning_scene_monitor;
    moveit_servo::PoseTracking	_tracker;
    StatusMonitor		_status_monitor;

    server_t			_tracker_srv;
    const ros::Publisher	_target_pose_pub;

    Eigen::Vector3d		_lin_tol{0.01, 0.01, 0.01};
    double			_rot_tol = 0.1;

    std::thread			_move_to_pose_thread;
};

PoseHeadServo::PoseHeadServo(const std::string& action_ns)
    :_nh("~"),
     _planning_scene_monitor(create_planning_scene_monitor(
				 "robot_description")),
     _tracker(_nh, _planning_scene_monitor),
     _status_monitor(_nh, "status"),
     _tracker_srv(_nh, action_ns,
		  boost::bind(&PoseHeadServo::execute_cb, this, _1),
		  false),
     _target_pose_pub(_nh.advertise<geometry_msgs::PoseStamped>("target_pose",
								1, true)),
     _lin_tol{0.01, 0.01, 0.01},
     _rot_tol(0.1),
     _move_to_pose_thread()
{
    _tracker_srv.start();

    ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseHeadServo) initialized");
}

PoseHeadServo::~PoseHeadServo()
{
    _tracker.stopMotion();
    if (_move_to_pose_thread.joinable())
	_move_to_pose_thread.join();
}

void
PoseHeadServo::run()
{
    ros::AsyncSpinner	spinner(8);
    spinner.start();

    ros::waitForShutdown();
}

planning_scene_monitor::PlanningSceneMonitorPtr
PoseHeadServo::create_planning_scene_monitor(
			const std::string& robot_description)
{
    using	namespace planning_scene_monitor;

    const auto	monitor = std::make_shared<planning_scene_monitor_t>(
				robot_description);
    if (!monitor->getPlanningScene())
    {
	ROS_ERROR_STREAM_NAMED(
	    LOGNAME, "(PoseHeadServo) failed to get PlanningSceneMonitor");
	exit(EXIT_FAILURE);
    }

    monitor->startSceneMonitor();
    monitor->startWorldGeometryMonitor(
	PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
	PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
	false /* skip octomap monitor */);
    monitor->startStateMonitor();

    ROS_INFO_STREAM_NAMED(LOGNAME,
			  "(PoseHeadServo) PlanningSceneMonitor started");

    return monitor;
}

void
PoseHeadServo::execute_cb(const PoseHeadGoalConstPtr& goal)
{
    ROS_INFO_STREAM_NAMED(LOGNAME, "(PoseHeadServo) goal ACCEPTED");

    _tracker.resetTargetPose();
    _move_to_pose_thread = std::thread([this]
				       {
					   this->_tracker.moveToPose(
					       this->_lin_tol,
					       this->_rot_tol, 0.1);
				       });

    for (ros::Rate rate(_nh.param<double>("rate", 50.0));
	 !_tracker_srv.isPreemptRequested() && ros::ok(); rate.sleep())
    {
	geometry_msgs::PoseStamped	target_pose = goal->target;
	target_pose.header.stamp = ros::Time::now();
	_target_pose_pub.publish(target_pose);
    }

    _tracker.stopMotion();
    if (_move_to_pose_thread.joinable())
	_move_to_pose_thread.join();
    _tracker_srv.setPreempted();

    ROS_WARN_STREAM_NAMED(LOGNAME, "(PoseHeadServo) goal CANCELED");
}


}	// namespace aist_controllers

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, aist_controllers::LOGNAME);

    aist_controllers::PoseHeadServo	servo("pose_head");
    servo.run();

    return 0;
}
