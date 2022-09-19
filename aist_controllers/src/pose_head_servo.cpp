/*
 *  \file	pose_head_servo.cpp
 *  \brief	ROS tracker of aist_controllers::PoseHeadAction type
 */
#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h>

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
*  class StatusMonitor							*
************************************************************************/
//! Class for monitoring status of moveit_servo
class StatusMonitor
{
  private:
    using StatusCode	= moveit_servo::StatusCode;

  public:
    StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
	:_status_sub(nh.subscribe(topic, 1, &StatusMonitor::status_cb, this)),
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

	    ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: "
				  << moveit_servo::SERVO_STATUS_CODE_MAP.at(
				      _status));
	}
    }

  private:
    ros::Subscriber		_status_sub;
    moveit_servo::StatusCode	_status;
};

/************************************************************************
*  class PoseHeadServo							*
************************************************************************/
class PoseHeadServo
{
  public:
    using planning_scene_monitor_t
		= planning_scene_monitor::PlanningSceneMonitor;
    using planning_scene_monitor_p
		= planning_scene_monitor::PlanningSceneMonitorPtr;

  public:
    PoseHeadServo()							;

    void	run()							;

  private:
    static planning_scene_monitor_p
		create_planning_scene_monitor(
		    const std::string& robot_description)		;

  private:
    ros::NodeHandle		_nh;
    planning_scene_monitor_p	_planning_scene_monitor;
    moveit_servo::PoseTracking	_tracker;
    ros::Publisher		_target_pose_pub;
    StatusMonitor		_status_monitor;
};

PoseHeadServo::PoseHeadServo()
    :_nh("~"),
     _planning_scene_monitor(create_planning_scene_monitor(
				 "robot_description")),
     _tracker(_nh, _planning_scene_monitor),
     _target_pose_pub(_nh.advertise<geometry_msgs::PoseStamped>("target_pose",
								1, true)),
     _status_monitor(_nh, "status")
{
}

void
PoseHeadServo::run()
{
    ros::AsyncSpinner	spinner(8);
    spinner.start();
}

PoseHeadServo::planning_scene_monitor_p
PoseHeadServo::create_planning_scene_monitor(
		  const std::string& robot_description)
{
    using	namespace planning_scene_monitor;

    const auto	monitor = std::make_shared<planning_scene_monitor_t>(
				robot_description);
    if (!monitor->getPlanningScene())
    {
	ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
	exit(EXIT_FAILURE);
    }

    monitor->startSceneMonitor();
    monitor->startWorldGeometryMonitor(
	PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
	PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
	false /* skip octomap monitor */);
    monitor->startStateMonitor();

    return monitor;
}


}	// namespace aist_controllers

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int
main(int argc, char** argv)
{
    ros::init(argc, argv, aist_controllers::LOGNAME);
    ros::NodeHandle	nh("~");
    ros::AsyncSpinner	spinner(8);
    spinner.start();

  // Load the planning scene monitor
    using	namespace planning_scene_monitor;

    PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene_monitor
	= std::make_shared<PlanningSceneMonitor>("robot_description");
    if (!planning_scene_monitor->getPlanningScene())
    {
	ROS_ERROR_STREAM_NAMED(aist_controllers::LOGNAME,
			       "Error in setting up the PlanningSceneMonitor.");
	exit(EXIT_FAILURE);
    }

    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(
	PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
	PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
	false /* skip octomap monitor */);
    planning_scene_monitor->startStateMonitor();

  // Create the pose tracker
    moveit_servo::PoseTracking	tracker(nh, planning_scene_monitor);

  // Make a publisher for sending pose commands
    ros::Publisher target_pose_pub =
	nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1 /* queue */,
						 true /* latch */);

  // Subscribe to servo status (and log it when it changes)
    aist_controllers::StatusMonitor	status_monitor(nh, "status");

    Eigen::Vector3d			lin_tol{ 0.01, 0.01, 0.01 };
    double				rot_tol = 0.1;

  // Get the current EE transform
    geometry_msgs::TransformStamped	current_ee_tf;
    tracker.getCommandFrameTransform(current_ee_tf);

  // Convert it to a Pose
    geometry_msgs::PoseStamped	target_pose;
    target_pose.header.frame_id  = current_ee_tf.header.frame_id;
    target_pose.pose.position.x  = current_ee_tf.transform.translation.x;
    target_pose.pose.position.y  = current_ee_tf.transform.translation.y;
    target_pose.pose.position.z  = current_ee_tf.transform.translation.z;
    target_pose.pose.orientation = current_ee_tf.transform.rotation;

  // Modify it a little bit
    target_pose.pose.position.x += 0.1;

  // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
  // waypoints
    tracker.resetTargetPose();

  // Publish target pose
    target_pose.header.stamp = ros::Time::now();
    target_pose_pub.publish(target_pose);

  // Run the pose tracking in a new thread
    std::thread	move_to_pose_thread([&tracker, &lin_tol, &rot_tol]
				    { tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */); });

    ros::Rate	loop_rate(50);
    for (size_t i = 0; i < 500; ++i)
    {
      // Modify the pose target a little bit each cycle
      // This is a dynamic pose target
	target_pose.pose.position.z += 0.0004;
	target_pose.header.stamp = ros::Time::now();
	target_pose_pub.publish(target_pose);

	loop_rate.sleep();
    }

  // Make sure the tracker is stopped and clean up
    tracker.stopMotion();
    move_to_pose_thread.join();

    return EXIT_SUCCESS;
}
