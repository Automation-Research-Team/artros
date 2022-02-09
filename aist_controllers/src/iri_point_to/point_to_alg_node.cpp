#include "point_to_alg_node.h"

PointToAlgNode::PointToAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<PointToAlgorithm>(),
    track_aserver_(private_node_handle_, "track"),
    point_to_aserver_(private_node_handle_, "point_to"),
    joint_motion_client_(private_node_handle_,"joint_motion", true)
{
  //init class attributes if necessary
    this->setRate(100);//in [Hz]
    this->pos.resize(3);
    this->pointing_axis.resize(3);
    this->new_point = false;
    this->tracker_new_point_ok = false;
    this->update_feedback = false;

  // [init publishers]
    this->joint_trajectory_publisher_ = this->private_node_handle_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1);
  
  // [init subscribers]
    this->tracker_point_to_subscriber_ = this->private_node_handle_.subscribe("tracker_point_to", 1, &PointToAlgNode::tracker_point_to_callback, this);
    pthread_mutex_init(&this->tracker_point_to_mutex_,NULL);

    this->joint_states_subscriber_ = this->private_node_handle_.subscribe("joint_states", 1, &PointToAlgNode::joint_states_callback, this);
    pthread_mutex_init(&this->joint_states_mutex_,NULL);

  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
    track_aserver_.registerStartCallback(boost::bind(&PointToAlgNode::trackStartCallback, this, _1));
    track_aserver_.registerStopCallback(boost::bind(&PointToAlgNode::trackStopCallback, this));
    track_aserver_.registerIsFinishedCallback(boost::bind(&PointToAlgNode::trackIsFinishedCallback, this));
    track_aserver_.registerHasSucceedCallback(boost::bind(&PointToAlgNode::trackHasSucceededCallback, this));
    track_aserver_.registerGetResultCallback(boost::bind(&PointToAlgNode::trackGetResultCallback, this, _1));
    track_aserver_.registerGetFeedbackCallback(boost::bind(&PointToAlgNode::trackGetFeedbackCallback, this, _1));
    track_aserver_.start();
    this->track_active=false;
    this->track_succeeded=false;
    this->track_finished=false;

    point_to_aserver_.registerStartCallback(boost::bind(&PointToAlgNode::point_toStartCallback, this, _1));
    point_to_aserver_.registerStopCallback(boost::bind(&PointToAlgNode::point_toStopCallback, this));
    point_to_aserver_.registerIsFinishedCallback(boost::bind(&PointToAlgNode::point_toIsFinishedCallback, this));
    point_to_aserver_.registerHasSucceedCallback(boost::bind(&PointToAlgNode::point_toHasSucceededCallback, this));
    point_to_aserver_.registerGetResultCallback(boost::bind(&PointToAlgNode::point_toGetResultCallback, this, _1));
    point_to_aserver_.registerGetFeedbackCallback(boost::bind(&PointToAlgNode::point_toGetFeedbackCallback, this, _1));
    point_to_aserver_.start();
    this->point_to_active=false;
    this->point_to_succeeded=false;
    this->point_to_finished=false;
    this->point_to_preempted = false;
    this->point_to_vel_error = false;
    this->state = waiting;

  
  // [init action clients]
}

PointToAlgNode::~PointToAlgNode(void)
{
  // [free dynamic memory]
    pthread_mutex_destroy(&this->tracker_point_to_mutex_);
    pthread_mutex_destroy(&this->joint_states_mutex_);
    std::vector<double>().swap(this->pos);
    std::vector<double>().swap(this->goal_angles);
    std::vector<double>().swap(this->pointing_axis);
    std::vector<double>().swap(this->prev_error);
    std::vector<double>().swap(this->acum_error);
}

void
PointToAlgNode::mainNodeThread(void)
{
    this->alg_.lock();
    point_to_states state = this->state;
    bool point_to_active = this->point_to_active;
    bool point_to_finished = this->point_to_finished;
    bool track_active = this->track_active;
    bool new_point = this->new_point;
    bool point_to_vel_error = this->point_to_vel_error;
    this->alg_.unlock();
    switch (state)
    {
      case waiting:
	if (point_to_vel_error)
	{
	    this->alg_.lock();
	    this->point_to_vel_error = false;
	    this->point_to_succeeded = false;
	    this->point_to_finished = true;
	    this->state = waiting;
	    this->alg_.unlock();
	    ROS_ERROR_STREAM("PointToAlgNode::mainNodeThread-> Point to action cancelled. Goal max velocity must be greater than 0.");
	}
	else if (track_active && !track_finished)
	{
	    this->alg_.lock();
	    this->state = start_tracking;
	    this->alg_.unlock();
	    ROS_INFO_STREAM("PointToAlgNode::mainNodeThread-> Start tracking");
	    break;
	}
	else if (point_to_active && !point_to_finished)
	{
	    this->alg_.lock();
	    this->state = solving;
	    this->alg_.unlock();
	}
	break;
      case solving:
	if (point_to_finished)
	{
	    this->alg_.lock();
	    this->state = waiting;
	    this->alg_.unlock();
	    ROS_ERROR_STREAM("PointToAlgNode::mainNodeThread-> Point to action cancelled");
	    break;
	}
	if (this->alg_.load_chain(this->pointing_frame) && get_joints() && set_joint_motion_goal() && joint_motionMakeActionRequest())
	{
	    this->alg_.lock();
	    this->state = motion;
	    this->alg_.unlock();
	}
	else
	{
	    this->alg_.lock();
	    this->point_to_succeeded = false;
	    this->point_to_finished = true;
	    this->state = waiting;
	    this->alg_.unlock();
	}
	break;
      case motion:
      {
	  if (point_to_finished || this->point_to_preempted)
	  {
	      joint_motion_client_.cancelGoal();
	      this->alg_.lock();
	      this->state = waiting;
	      this->point_to_preempted = false;
	      this->alg_.unlock();
	      ROS_WARN_STREAM("PointToAlgNode::mainNodeThread-> Point to action cancelled or preempted");
	      break;
	  }
	  actionlib::SimpleClientGoalState joint_motion_state(actionlib::SimpleClientGoalState::PENDING);
	  joint_motion_state = joint_motion_client_.getState();
	  if (joint_motion_state == actionlib::SimpleClientGoalState::SUCCEEDED)
	  {
	      this->alg_.lock();
	      this->point_to_succeeded = true;
	      this->point_to_finished = true;
	      this->state = waiting;
	      this->alg_.unlock();
	      ROS_INFO_STREAM("PointToAlgNode::mainNodeThread-> SUCCEEDED");
	  }
	  else if (!(joint_motion_state == actionlib::SimpleClientGoalState::PENDING || joint_motion_state == actionlib::SimpleClientGoalState::ACTIVE))
	  {
	      this->alg_.lock();
	      this->point_to_succeeded = false;
	      this->point_to_finished = true;
	      this->state = waiting;
	      this->alg_.unlock();
	      ROS_ERROR_STREAM("PointToAlgNode::mainNodeThread-> Joint_motion action failed.");
	  }
      }
      break;
      case start_tracking:
	if (!track_active)
	{
	    this->alg_.lock();
	    this->state = waiting;
	    ROS_INFO_STREAM("PointToAlgNode::mainNodeThread-> Tracking cancelled");
	    this->alg_.unlock();
	}
	else if (new_point)
	{
	    if (this->alg_.load_chain(this->pointing_frame))
	    {
		this->alg_.lock();
		std::vector<double>().swap(this->prev_error);
		std::vector<double>().swap(this->acum_error);
		this->state = tracking_target;
		this->alg_.unlock();
	    }
	    else
	    {
		this->alg_.lock();
		this->new_point = false;
		this->alg_.unlock();
		ROS_ERROR_STREAM("PointToAlgNode::mainNodeThread-> Start tracking failed.");
	    }
	}
	break;
      case tracking_target:
	if (!track_active)
	{
	    this->alg_.lock();
	    this->state = waiting;
	    this->alg_.unlock();
	    ROS_INFO_STREAM("PointToAlgNode::mainNodeThread-> Tracking cancelled");
	    stop_servos();
	    break;
	}
	if (new_point)
	{
	    if (!get_joints())
	    {
		this->alg_.lock();
		this->update_feedback = false;
		this->new_point = false;
		this->alg_.unlock();
		ROS_ERROR_STREAM("PointToAlgNode::mainNodeThread-> No solution found. Stop movement.");
		stop_servos();
		break;
	    }
	    else
		this->tracker_new_point_ok = true;
	    ROS_DEBUG_STREAM("PointToAlgNode::mainNodeThread-> New point");
	    this->alg_.lock();
	    this->update_feedback = true;
	    this->new_point = false;
	    this->alg_.unlock();
	}
	if (this->tracker_new_point_ok)
	    send_pid_output();
	break;
    }

  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]

}

bool
PointToAlgNode::get_joints(void)
{
    double roll, pitch, yaw, x, y, z;
  // double roll, pitch, yaw, x_head, y_head, z_head, x_target, y_target, z_target, diff_x, diff_y, diff_z;
  //First get the point in the pan_tilt_base_frame tf_base_p
    tf::StampedTransform tf_base_pointf;
    tf::Transform tf_base_p, tf_pointf_p;
  // ROS_INFO_STREAM("PointToAlgNode::get_joints -> point in " << this->pos[0] << ", " << this->pos[1] << ", " << this->pos[2]);
    tf_pointf_p.setOrigin(tf::Vector3(this->pos[0],
				      this->pos[1], this->pos[2]));
    tf_pointf_p.setRotation(tf::Quaternion(0, 0, 0, 1));

    std::string base_frame_w_prefix;
    if (this->config_.tf_prefix == "")
	base_frame_w_prefix = this->config_.base_frame;
    else
	base_frame_w_prefix = this->config_.tf_prefix + "/" + this->config_.base_frame;
  
  // ROS_INFO_STREAM("PointToAlgNode::get_joints -> point frame " << this->point_frame << "; base_frame " << base_frame_w_prefix << "; pointing frame" << "; time " << this->point_to_time);
    if (!this->tf_listener_.waitForTransform(base_frame_w_prefix,
					     this->point_frame,
					     this->point_to_time,
					     ros::Duration(1.0)))
    {
	ROS_ERROR_STREAM("PointToAlgNode::get_joints-> No transform from "
			 << base_frame_w_prefix << " to "
			 << this->point_frame << " at time "
			 << this->point_to_time);
	return false;
    }
    this->tf_listener_.lookupTransform(base_frame_w_prefix, this->point_frame,
				       this->point_to_time, tf_base_pointf);
  // ROS_INFO_STREAM("PointToAlgNode::get_joints -> tf base point_frame translation " << tf_base_pointf.getOrigin().x() << ", " << tf_base_pointf.getOrigin().y() << ", " << tf_base_pointf.getOrigin().z());
    tf::Matrix3x3(tf_base_pointf.getRotation()).getRPY(roll, pitch, yaw);
  // ROS_INFO_STREAM("PointToAlgNode::get_joints -> tf base point_frame rotation " << roll << ", " << pitch << ", " << yaw);
    tf_base_p = tf_base_pointf*tf_pointf_p;
    x = tf_base_p.getOrigin().x();
    y = tf_base_p.getOrigin().y();
    z = tf_base_p.getOrigin().z();
  
    double norm = std::sqrt(std::pow(this->pointing_axis[0], 2) + std::pow(this->pointing_axis[1], 2) + std::pow(this->pointing_axis[2], 2));
    std::vector<double> p_axis;
    p_axis.push_back(this->pointing_axis[0]/norm);
    p_axis.push_back(this->pointing_axis[1]/norm);
    p_axis.push_back(this->pointing_axis[2]/norm);

  //Get Axis rotations
    tf::Transform t;
    if (fabs(p_axis[0] - 1.0) < std::numeric_limits<double>::epsilon())
    {
	roll = 0;
	pitch = -atan2(z, sqrt(x*x+y*y));
	yaw = atan2(y, x);
	t.setOrigin(tf::Vector3(x, y, z));
	tf::Quaternion q;
	q.setRPY(0, -atan2(z, sqrt(x*x+y*y)), atan2(y, x));
	t.setRotation(q);
    }
    else if (fabs(p_axis[1] - 1.0) < std::numeric_limits<double>::epsilon())
    {
	t.setOrigin(tf::Vector3(x, y, z));
	tf::Quaternion q,q_axis, q_finish;
	q_axis.setRPY(0,0,-M_PI/2);
	q.setRPY(0, -atan2(z, sqrt(x*x+y*y)), atan2(y, x));
	q_finish = q*q_axis;
	tf::Matrix3x3(q_finish).getRPY(roll, pitch, yaw);
	t.setRotation(q_finish);
    }
    else if (fabs(p_axis[2] - 1.0) < std::numeric_limits<double>::epsilon())
    {
	t.setOrigin(tf::Vector3(x, y, z));
	tf::Quaternion q,q_axis, q_finish;
	q.setRPY(0, -atan2(z, sqrt(x*x+y*y)), atan2(y, x));
	q_axis.setRPY(0,M_PI/2,0);
	q_finish = q*q_axis;
	tf::Matrix3x3(q_finish).getRPY(roll, pitch, yaw);
	t.setRotation(q_finish);
    }
    else
    {
	ROS_ERROR("PointToAlgNode::get_joints-> PointTo only works with the principal axis X Y Z");
	return false;
    }

    std::vector<double> angles, pos, rot;
    rot.resize(3);
    rot[0] = roll;
    rot[1] = pitch;
    rot[2] = yaw;
    pos.resize(3);
    pos[0] = x;
    pos[1] = y;
    pos[2] = z;

  // ROS_INFO_STREAM("PointToAlgNode::get_joints -> point out " << pos[0] << ", " << pos[1] << ", " << pos[2] << ", " << rot[0] << ", " << rot[1] << ", " << rot[2]);
  //set_current position
    std::vector<double> current_angles;
    std::vector<std::string> joint_names;
    this->alg_.get_joint_names(joint_names);
    for (int i = 0; i < joint_names.size(); i++)
    {
	for (int j = 0; j < this->joint_states.name.size(); j++)
	{
	    if (this->joint_states.name[j] == joint_names[i])
		current_angles.push_back(this->joint_states.position[j]);
	}
    }

    bool success = this->alg_.get_inverse_kinematics(pos, rot, current_angles, angles);
    if (success)
    {
      // ROS_INFO_STREAM("Success!! joints angles:");
	std::vector<double>().swap(this->goal_angles);
	this->goal_angles.swap(angles);
      // ROS_INFO_STREAM("PointToAlgNode::get_joints -> goal_angles");
      // for (unsigned int i = 0; i < this->goal_angles.size(); i++)
      //   std::cout << "angle " << (int)i << " " << this->goal_angles[i] << std::endl;
    }
    return success;
}

void
PointToAlgNode::stop_servos(void)
{
    this->joint_trajectory_JointTrajectory_msg_.header.frame_id
	= this->config_.frame_id;

    this->alg_.get_joint_names(
	this->joint_trajectory_JointTrajectory_msg_.joint_names);
    trajectory_msgs::JointTrajectoryPoint point;

    for (uint8_t i = 0; i < this->joint_trajectory_JointTrajectory_msg_.joint_names.size(); i++)
    {
	for (uint8_t j = 0; j < this->joint_states.name.size(); j++)
	{
	    if (this->joint_states.name[j].compare(this->joint_trajectory_JointTrajectory_msg_.joint_names[i]) == 0)
	    {
		point.positions.push_back(this->joint_states.position[j]);
		point.velocities.push_back(0.25);
	    }
	}
    }
    point.time_from_start = ros::Duration(2);

    std::vector<trajectory_msgs::JointTrajectoryPoint>().swap(this->joint_trajectory_JointTrajectory_msg_.points);
    this->joint_trajectory_JointTrajectory_msg_.points.push_back(point);

    this->joint_trajectory_JointTrajectory_msg_.header.stamp = ros::Time::now();
    this->joint_trajectory_publisher_.publish(this->joint_trajectory_JointTrajectory_msg_);
}

void
PointToAlgNode::calculate_error(std::vector<double> &error)
{
    std::vector<double>().swap(error);
    std::vector<std::string> joint_names;
    this->alg_.get_joint_names(joint_names);
    joint_states_mutex_enter();
    for (uint8_t i = 0; i < joint_names.size(); i++)
    {
	for (uint8_t j = 0; j < this->joint_states.name.size(); j++)
	{
	    if (this->joint_states.name[j].compare(joint_names[i]) == 0)
		error.push_back(this->goal_angles[i] - this->joint_states.position[j]);
	}
    }
    joint_states_mutex_exit();
}

bool
PointToAlgNode::set_joint_motion_goal(void)
{
    std::vector<double> error;
    this->joint_motion_goal_.trajectory.header.frame_id = this->config_.frame_id;
    this->alg_.get_joint_names(this->joint_motion_goal_.trajectory.joint_names);
    control_msgs::JointTolerance tol;
    tol.position = this->config_.pos_tol;
    tol.velocity = this->config_.vel_tol;
    tol.acceleration = this->config_.acc_tol;
    std::vector<control_msgs::JointTolerance>().swap(this->joint_motion_goal_.path_tolerance);
    std::vector<control_msgs::JointTolerance>().swap(this->joint_motion_goal_.goal_tolerance);
    for (uint8_t i; i < this->joint_motion_goal_.trajectory.joint_names.size(); i++)
    {
	tol.name = this->joint_motion_goal_.trajectory.joint_names[i];
	this->joint_motion_goal_.path_tolerance.push_back(tol);
	this->joint_motion_goal_.goal_tolerance.push_back(tol);
    }

    calculate_error(error);

    double max = 0.0;
    for (uint8_t i = 0; i < error.size(); i++)
    {
	if (fabs(error[i]) > fabs(max))
	    max = error[i];
    }

    double duration = fabs(max/this->max_velocity);
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(duration*this->config_.time_escalar);
    this->joint_motion_goal_.goal_time_tolerance = point.time_from_start*this->config_.time_tol;
  // ROS_INFO_STREAM("Max time " << point.time_from_start);
    for (uint8_t i = 0; i < this->goal_angles.size(); i++)
    {
	point.positions.push_back(this->goal_angles[i]);
	if (duration > 0.0)
	    point.velocities.push_back(fabs(error[i]/duration));
	else
	    point.velocities.push_back(0.0);
      //Just for tracking
      // ROS_INFO_STREAM("  joint[" << (int)i << "] error -> " << error[i] << "; vel -> " << error[i]/duration << "; time -> " << );
    }
    std::vector<trajectory_msgs::JointTrajectoryPoint>().swap(this->joint_motion_goal_.trajectory.points);
    this->joint_motion_goal_.trajectory.points.push_back(point);
    this->joint_motion_goal_.trajectory.header.stamp = ros::Time::now();
    return true;
}

void
PointToAlgNode::send_pid_output(void)
{
    this->joint_trajectory_JointTrajectory_msg_.header.frame_id = this->config_.frame_id;
    this->alg_.get_joint_names(this->joint_trajectory_JointTrajectory_msg_.joint_names);

  //PID
    trajectory_msgs::JointTrajectoryPoint point;
    std::vector<double> error;
    calculate_error(error);
    bool first = this->prev_error.size() == 0;
    for (uint8_t i=0; i<error.size(); i++)
    {
	if (first)
	{
	    this->acum_error.push_back(0.0);
	    point.effort.push_back(this->config_.kp*error[i]);
	    this->prev_error.push_back(error[i]);
	}
	else
	{

	    double dt = (ros::Time::now()-this->last_time).toSec();
	    if (dt < std::numeric_limits<double>::epsilon())
		dt = std::numeric_limits<double>::epsilon();
	    double derivative = (error[i] - this->prev_error[i])/dt;
	    this->acum_error[i] += (error[i] + (error[i] - this->prev_error[i])/2)*dt;
	    point.effort.push_back(this->config_.kp*error[i]+this->config_.ki*this->acum_error[i]+this->config_.kd*derivative);
	    this->prev_error[i] = error[i];
	}
    }
    this->last_time = ros::Time::now();

    std::vector<trajectory_msgs::JointTrajectoryPoint>().swap(this->joint_trajectory_JointTrajectory_msg_.points);
    this->joint_trajectory_JointTrajectory_msg_.points.push_back(point);

    this->joint_trajectory_JointTrajectory_msg_.header.stamp = ros::Time::now();
    this->joint_trajectory_publisher_.publish(this->joint_trajectory_JointTrajectory_msg_);
}

/*  [subscriber callbacks] */
void
PointToAlgNode::tracker_point_to_callback(
    const control_msgs::PointHeadActionGoal::ConstPtr& msg)
{
    ROS_DEBUG("PointToAlgNode::tracker_point_to_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
    this->alg_.lock();
  //this->tracker_point_to_mutex_enter();
    if (this->track_active)
    {
	this->point_frame = msg->goal.target.header.frame_id;
	this->point_to_time = msg->goal.target.header.stamp;
    
	this->pos[0] = msg->goal.target.point.x;
	this->pos[1] = msg->goal.target.point.y;
	this->pos[2] = msg->goal.target.point.z;
	this->pointing_frame = msg->goal.pointing_frame;
	this->pointing_axis[0] = msg->goal.pointing_axis.x;
	this->pointing_axis[1] = msg->goal.pointing_axis.y;
	this->pointing_axis[2] = msg->goal.pointing_axis.z;
	this->new_point = true;
    }
    else
	ROS_ERROR_STREAM("PointToAlgNode::tracker_point_to_callback-> Tracking not active. Dropping this message.");

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
    this->alg_.unlock();
  //this->tracker_point_to_mutex_exit();
}

void
PointToAlgNode::tracker_point_to_mutex_enter(void)
{
    pthread_mutex_lock(&this->tracker_point_to_mutex_);
}

void
PointToAlgNode::tracker_point_to_mutex_exit(void)
{
    pthread_mutex_unlock(&this->tracker_point_to_mutex_);
}

void
PointToAlgNode::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
    this->joint_states_mutex_enter();
    this->joint_states = *msg;

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
    this->joint_states_mutex_exit();
}

void
PointToAlgNode::joint_states_mutex_enter(void)
{
    pthread_mutex_lock(&this->joint_states_mutex_);
}

void
PointToAlgNode::joint_states_mutex_exit(void)
{
    pthread_mutex_unlock(&this->joint_states_mutex_);
}


/*  [service callbacks] */


/*  [action callbacks] */
void
PointToAlgNode::trackStartCallback(const iri_joints_msgs::pointHeadTrackerGoalConstPtr& goal)
{
    this->alg_.lock();
    if (this->point_to_active)
    {
	ROS_WARN("PointToAlgNode::trackStartCallback-> PointTo Tracker goal rejected: PointTo action is active.");
	this->track_active = false;
	this->track_finished = true;
    }
    else
    {
	ROS_DEBUG("PointToAlgNode::trackStartCallback-> Track active");
	this->track_active = true;
	this->track_finished = false;
    }
    this->track_succeeded = false;
    this->alg_.unlock();
}

void
PointToAlgNode::trackStopCallback(void)
{
    ROS_DEBUG("PointToAlgNode::-> Track inactive");
    this->alg_.lock();
  //stop action
    this->track_active=false;
    this->track_succeeded = true;
    this->track_finished = true;
    this->update_feedback = false;
    this->alg_.unlock();
}

bool
PointToAlgNode::trackIsFinishedCallback(void)
{
    bool ret = false;

    this->alg_.lock();
  //if action has finish for any reason
    ret = this->track_finished;
    this->alg_.unlock();

    return ret;
}

bool
PointToAlgNode::trackHasSucceededCallback(void)
{
    bool ret = false;

    this->alg_.lock();
  //if goal was accomplished
    ret = this->track_succeeded;
    this->track_active=false;
    this->alg_.unlock();

    return ret;
}

void
PointToAlgNode::trackGetResultCallback(iri_joints_msgs::pointHeadTrackerResultPtr& result)
{
    this->alg_.lock();
  //update result data to be sent to client
  //result->data = data;
    this->alg_.unlock();
}

void
PointToAlgNode::trackGetFeedbackCallback(iri_joints_msgs::pointHeadTrackerFeedbackPtr& feedback)
{
    this->alg_.lock();
  //update feedback data to be sent to client
    if (this->update_feedback)
    {
	this->alg_.get_joint_names(feedback->joint_names);
	feedback->target_angle = this->goal_angles;
	feedback->target_frame_id = this->point_frame;
	feedback->target.x = this->pos[0];
	feedback->target.y = this->pos[1];
	feedback->target.z = this->pos[2];
    }
  //ROS_INFO("feedback: %s", feedback->data.c_str());
    this->alg_.unlock();
}

void
PointToAlgNode::point_toStartCallback(const control_msgs::PointHeadGoalConstPtr& goal)
{
    this->alg_.lock();
    if (this->track_active)
    {
	this->point_to_active = false;
	this->point_to_finished = true;
	this->point_to_succeeded = false;
	ROS_WARN_STREAM("PointToAlgNode::point_toStartCallback-> PointTo goal rejected: PointTo tracking is active.");
	this->alg_.unlock();
	return;
    }
  //check goal
    if (this->point_to_active)
	this->point_to_preempted = true;
    this->point_to_active=true;
    this->point_frame = goal->target.header.frame_id;
    this->point_to_time = goal->target.header.stamp;
  
    this->pos[0] = goal->target.point.x;
    this->pos[1] = goal->target.point.y;
    this->pos[2] = goal->target.point.z;
    this->pointing_frame = goal->pointing_frame;
    this->pointing_axis[0] = goal->pointing_axis.x;
    this->pointing_axis[1] = goal->pointing_axis.y;
    this->pointing_axis[2] = goal->pointing_axis.z;
    this->max_velocity = fabs(goal->max_velocity);
    if (fabs(this->max_velocity) < std::numeric_limits<double>::epsilon())
	this->point_to_vel_error = true;


    this->point_to_succeeded = false;
    this->point_to_finished = false;
  //execute goal
    this->alg_.unlock();
}

void
PointToAlgNode::point_toStopCallback(void)
{
    this->alg_.lock();
  //stop action
    this->point_to_active=false;
    this->point_to_succeeded = false;
    this->point_to_finished = true;
    this->alg_.unlock();
}

bool
PointToAlgNode::point_toIsFinishedCallback(void)
{
    bool ret = false;

    this->alg_.lock();
  //if action has finish for any reason
    ret = this->point_to_finished;
    this->alg_.unlock();

    return ret;
}

bool
PointToAlgNode::point_toHasSucceededCallback(void)
{
    bool ret = false;

    this->alg_.lock();
  //if goal was accomplished
    ret = this->point_to_succeeded;
    this->point_to_active=false;
    this->alg_.unlock();

    return ret;
}

void
PointToAlgNode::point_toGetResultCallback(control_msgs::PointHeadResultPtr& result)
{
    this->alg_.lock();
  //update result data to be sent to client
  //result->data = data;
    this->alg_.unlock();
}

void PointToAlgNode::point_toGetFeedbackCallback(control_msgs::PointHeadFeedbackPtr& feedback)
{
    this->alg_.lock();
  //update feedback data to be sent to client
  //ROS_INFO("feedback: %s", feedback->data.c_str());
    this->alg_.unlock();
}

void
PointToAlgNode::joint_motionDone(const actionlib::SimpleClientGoalState& state,  const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
    this->alg_.lock();
    if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
	ROS_INFO("PointToAlgNode::joint_motionDone-> Goal Achieved!");
    else
	ROS_WARN("PointToAlgNode::joint_motionDone-> %s", state.toString().c_str());

  //copy & work with requested result
    this->alg_.unlock();
}

void
PointToAlgNode::joint_motionActive()
{
    this->alg_.lock();
  //ROS_INFO("PointToAlgNode::joint_motionActive: Goal just went active!");
    this->alg_.unlock();
}

void
PointToAlgNode::joint_motionFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
    this->alg_.lock();
  //ROS_INFO("PointToAlgNode::joint_motionFeedback: Got Feedback!");

    bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
    if( !feedback_is_ok )
    {
	joint_motion_client_.cancelGoal();
      //ROS_INFO("PointToAlgNode::joint_motionFeedback: Cancelling Action!");
    }
    this->alg_.unlock();
}


/*  [action requests] */
bool
PointToAlgNode::joint_motionMakeActionRequest()
{
  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).
    bool ok;
  // this->alg_.unlock();
    if(this->joint_motion_client_.isServerConnected())
    {
      //ROS_INFO("PointToAlgNode::joint_motionMakeActionRequest: Server is Available!");
      //send a goal to the action server
      //joint_motion_goal_.data = my_desired_goal;
	this->joint_motion_client_.sendGoal(this->joint_motion_goal_,
					    boost::bind(&PointToAlgNode::joint_motionDone,     this, _1, _2),
					    boost::bind(&PointToAlgNode::joint_motionActive,   this),
					    boost::bind(&PointToAlgNode::joint_motionFeedback, this, _1));
	ROS_INFO("PointToAlgNode::joint_motionMakeActionRequest-> Goal Sent.");
      // ok=true;
	return true;
    }
    else
    {
	ROS_ERROR("PointToAlgNode::joint_motionMakeActionRequest-> action server is not connected. Check remap or server presence.");
      // ok=false;
	return false;
    }
  // this->alg_.lock();
    return ok;
}


void
PointToAlgNode::node_config_update(Config &config, uint32_t level)
{
    this->alg_.lock();
    this->config_=config;
    this->alg_.unlock();
}

void
PointToAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int
main(int argc,char *argv[])
{
    return algorithm_base::main<PointToAlgNode>(argc, argv, "point_to_alg_node");
}
