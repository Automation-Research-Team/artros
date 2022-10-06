/*******************************************************************************
 *      Title     : pose_tracking_example.cpp
 *      Project   : moveit_servo
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <tf/transform_datatypes.h>

static const std::string LOGNAME = "cpp_interface_example";

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

std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Transform& transform)
{
    return out << transform.translation.x << ' '
	       << transform.translation.y << ' '
	       << transform.translation.z << ';'
	       << transform.rotation.x << ','
	       << transform.rotation.y << ','
	       << transform.rotation.z << ','
	       << transform.rotation.w ;
}

// Class for monitoring status of moveit_servo
class StatusMonitor
{
  public:
    StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
    {
	sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
    }

  private:
    void
    statusCB(const std_msgs::Int8ConstPtr& msg)
    {
	moveit_servo::StatusCode
	    latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
	if (latest_status != status_)
	{
	    status_ = latest_status;
	    const auto& status_str
			    = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
	    ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
	}
    }

  private:
    moveit_servo::StatusCode	status_ = moveit_servo::StatusCode::INVALID;
    ros::Subscriber		sub_;
};

geometry_msgs::Vector3
getRPY(const geometry_msgs::Quaternion& q)
{
    geometry_msgs::Vector3	rpy;
    tf::Matrix3x3(tf::Quaternion(q.x, q.y, q.z, q.w)).getRPY(rpy.x,
							     rpy.y, rpy.z);
    return rpy;
}

geometry_msgs::Quaternion
getQuaternion(const geometry_msgs::Vector3& rpy)
{
    tf::Quaternion	q;
    q.setRPY(rpy.x, rpy.y, rpy.z);
    geometry_msgs::Quaternion	qq;
    qq.x = q.x();
    qq.y = q.y();
    qq.z = q.z();
    qq.w = q.w();

    return qq;
}

double
rad(double deg)
{
    return M_PI/180.0 * deg;
}

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int
main(int argc, char** argv)
{
    ros::init(argc, argv, LOGNAME);
    ros::NodeHandle	nh("~");
    ros::AsyncSpinner	spinner(8);
    spinner.start();

  // Load the planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene_monitor
	= std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    if (!planning_scene_monitor->getPlanningScene())
    {
	ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
	exit(EXIT_FAILURE);
    }

    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(
	planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
	planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
	false /* skip octomap monitor */);
    planning_scene_monitor->startStateMonitor();

  // Create the pose tracker
    moveit_servo::PoseTracking	tracker(nh, planning_scene_monitor);

  // Make a publisher for sending pose commands
    ros::Publisher target_pose_pub =
	nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1 /* queue */,
						 true /* latch */);

  // Subscribe to servo status (and log it when it changes)
    StatusMonitor	status_monitor(nh, "status");

    Eigen::Vector3d	lin_tol{ 0.001, 0.001, 0.001 };
    double		rot_tol = 0.01;

  // Get the current EE transform
    geometry_msgs::TransformStamped	current_ee_tf;
    tracker.getEEFrameTransform(current_ee_tf);
    std::cerr << "current_ee_tf: " << current_ee_tf.header.frame_id
	      << " <== " << current_ee_tf.child_frame_id
	      << '[' << current_ee_tf.transform << ']'
	      << std::endl;

  // Convert it to a Pose
    geometry_msgs::PoseStamped	target_pose;
    target_pose.header.frame_id  = current_ee_tf.header.frame_id;
    target_pose.pose.position.x  = current_ee_tf.transform.translation.x;
    target_pose.pose.position.y  = current_ee_tf.transform.translation.y;
    target_pose.pose.position.z  = current_ee_tf.transform.translation.z;
    target_pose.pose.orientation = current_ee_tf.transform.rotation;

  // Modify it a little bit
    target_pose.pose.position.x += 0.05;
  //target_pose.pose.position.y += 0.05;
  //target_pose.pose.position.z += 0.03;

    auto	rpy = getRPY(target_pose.pose.orientation);
  //rpy.z += rad(30);
  //target_pose.pose.orientation = getQuaternion(rpy);

  // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
  // waypoints
    tracker.resetTargetPose();

  // Publish target pose
    target_pose.header.stamp = ros::Time::now();
    target_pose_pub.publish(target_pose);
    std::cerr << "target_pose: " << target_pose.header.frame_id
	      << "@[" << target_pose.pose << ']'
	      << std::endl
	      << "--------" << std::endl;

  // Run the pose tracking in a new thread
    std::thread	move_to_pose_thread([&tracker, &lin_tol, &rot_tol]
				    { tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */); });

    ros::Rate	loop_rate(50);
    for (int n = 0; n < 100; ++n)
    {
	double	x_error, y_error, z_error, orientation_error;
	
	for (size_t i = 0; i < 10; ++i)
	{
	  // Modify the pose target a little bit each cycle
	  // This is a dynamic pose target
	    // target_pose.pose.position.z += 0.0004;
	    target_pose.header.stamp = ros::Time::now();
	    target_pose_pub.publish(target_pose);
	    // std::cerr << "target_pose: " << target_pose.header.frame_id
	    // 	      << "@[" << target_pose.pose << ']'
	    // 	      << std::endl;
	    tracker.getPIDErrors(x_error, y_error, z_error, orientation_error);
	    std::cerr << "PID errors=(" << x_error << ' ' << y_error << ' '
		      << z_error << ';' << orientation_error << std::endl;
	    
	    loop_rate.sleep();
	}

	for (size_t i = 0; i < 10; ++i)
	{
	  // Modify the pose target a little bit each cycle
	  // This is a dynamic pose target
	    // target_pose.pose.position.z -= 0.0004;
	    target_pose.header.stamp = ros::Time::now();
	    target_pose_pub.publish(target_pose);
	    // std::cerr << "target_pose: " << target_pose.header.frame_id
	    // 	      << "@[" << target_pose.pose << ']'
	    // 	      << std::endl;
	    tracker.getPIDErrors(x_error, y_error, z_error, orientation_error);

	    std::cerr << "PID errors=(" << x_error << ' ' << y_error << ' '
		      << z_error << ';' << orientation_error << std::endl;
	    
	    loop_rate.sleep();
	}
    }

  // Make sure the tracker is stopped and clean up
    tracker.stopMotion();
    move_to_pose_thread.join();

    return EXIT_SUCCESS;
}
