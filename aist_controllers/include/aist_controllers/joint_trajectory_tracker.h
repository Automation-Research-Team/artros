/*
 *  \file	joint_trajectory_tracker.cpp
 *  \brief	ROS tracker using JointTrajectoryController
 */
#ifndef AIST_CONTROLLERS_JOINT_TRAJECTORY_TRACKER_H
#define AIST_CONTROLLERS_JOINT_TRAJECTORY_TRACKER_H

#include <ros/ros.h>
#include <cmath>

#include <actionlib/server/simple_action_server.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <urdf/model.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>

namespace aist_controllers
{
/************************************************************************
*  static functions							*
************************************************************************/
static std::ostream&
operator <<(std::ostream& out, const std::vector<double>& v)
{
    for (const auto& val : v)
	out << ' ' << val * 180.0/M_PI;
    return out;
}
    
/************************************************************************
*  class JointTrajectoryTracker<ACTION>					*
************************************************************************/
template <class ACTION>
class JointTrajectoryTracker
{
  private:
    using state_cp	= control_msgs::JointTrajectoryControllerStateConstPtr;
    using trajectory_t	= trajectory_msgs::JointTrajectory;
    using error_t	= trajectory_msgs::JointTrajectoryPoint;
    using server_t	= actionlib::SimpleActionServer<ACTION>;
    using goal_cp	= boost::shared_ptr<const typename server_t::Goal>;
    using feedback_t	= typename server_t::Feedback;

    class Tracker
    {
      public:
			Tracker(const std::string& robot_desc_string,
				const std::string& base_link,
				double publish_rate)			;

	size_t		njoints()				const	;
	const trajectory_t&
			command()				const	;
	const feedback_t&
			feedback()				const	;

	void		init(const state_cp& state,
			     const std::string& pointing_frame)		;
	void		read(const state_cp& state)			;
	bool		update(const goal_cp& goal)			;

      private:
	void		clamp(KDL::JntArray& jnt_pos)		const	;
	static std::string
			solver_error_message(int result)		;
	static void	jointsFromKDL(const KDL::JntArray& kdl_joints,
				      std::vector<double>& joints)	;
	static void	jointsToKDL(const std::vector<double>& joints,
				    KDL::JntArray& kdl_joints)		;

      private:
	std::string					_base_link;
	std::string					_pointing_frame;
	const ros::Duration				_duration;

	urdf::Model					_urdf;
	tf::TransformListener				_listener;
	trajectory_t					_command;
	error_t						_error;
	feedback_t					_feedback;

	KDL::Tree					_tree;
	KDL::Chain					_chain;
	KDL::JntArray					_jnt_pos_min;
	KDL::JntArray					_jnt_pos_max;

	std::unique_ptr<KDL::ChainJntToJacSolver>	_jac_solver;
	std::unique_ptr<KDL::ChainFkSolverPos>		_pos_fksolver;
	std::unique_ptr<KDL::ChainIkSolverVel>		_vel_iksolver;
	std::unique_ptr<TRAC_IK::TRAC_IK>		_pos_iksolver;
    };

  public:
		JointTrajectoryTracker(const std::string& action_ns)	;

  private:
    void	goal_cb()						;
    void	preempt_cb()						;
    void	state_cb(const state_cp& msg)				;

  private:
    ros::NodeHandle		_nh;
    const std::string		_controller;
    ros::Subscriber		_state_sub;
    const ros::Publisher	_command_pub;

    Tracker			_tracker;

    state_cp			_last_state;
    server_t			_tracker_srv;
    goal_cp			_current_goal;
};

template <class ACTION>
JointTrajectoryTracker<ACTION>
    ::JointTrajectoryTracker(const std::string& action_ns)
    :_nh("~"),
     _controller(_nh.param<std::string>("controller",
					"/pos_joint_traj_controller")),
     _state_sub(_nh.subscribe(_controller + "/state", 100,
			      &JointTrajectoryTracker::state_cb, this)),
     _command_pub(_nh.advertise<trajectory_t>(_controller + "/command", 10)),
     _tracker(_nh.param<std::string>(
		  _nh.param<std::string>("robot_description",
					 "/robot_description"),
		  std::string()),
	      _nh.param<std::string>("base_link", "base_link"),
	      _nh.param<double>(_controller + "/state_publish_rate", 50)),
     _tracker_srv(_nh, action_ns, false),
     _current_goal(nullptr)
{
  // Initialize action server
    _tracker_srv.registerGoalCallback(
    	std::bind(&JointTrajectoryTracker::goal_cb, this));
    _tracker_srv.registerPreemptCallback(
    	std::bind(&JointTrajectoryTracker::preempt_cb, this));
    _tracker_srv.start();
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::goal_cb()
{
    _current_goal = _tracker_srv.acceptNewGoal();

    ROS_INFO_STREAM("(JointTrajectoryTracker) Goal accepted");

    try
    {
	_tracker.init(_last_state, _current_goal->pointing_frame);
    }
    catch (const std::exception& err)
    {
	_tracker_srv.setAborted();
	ROS_ERROR_STREAM("(JointTrajectoryTracker) Goal aborted["
			 << err.what() << ']');
    }
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::preempt_cb()
{
  // Stops the controller.
    trajectory_t	empty;
    empty.joint_names = _tracker.command().joint_names;
    _command_pub.publish(empty);

    _tracker_srv.setPreempted();
    ROS_INFO_STREAM("(JointTrajectoryTracker) Goal cancelled");
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::state_cb(const state_cp& state)
{
    _last_state = state;

    if (!_tracker_srv.isActive())
	return;

    try
    {
	_tracker.read(state);

	const auto	success = _tracker.update(_current_goal);

	_tracker_srv.publishFeedback(_tracker.feedback());
	_command_pub.publish(_tracker.command());

	ROS_DEBUG_STREAM("(JointTrajectoryTracker) published command["
			 << _tracker.command().points[0].positions << '[');
	
	if (success)
	{
	    _tracker_srv.setSucceeded();
	    ROS_INFO_STREAM("(JointTrajectoryTracker) Goal succeeded");
	}
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(JointTrajectoryTracker) " << err.what());
      //ROS_ERROR_STREAM("(JointTrajectoryTracker) Goal aborted");
      //_tracker_srv.setAborted();
    }
}

/************************************************************************
*  class JointTrajectoryTracker<ACTION>::Tracker			*
************************************************************************/
template <class ACTION>
JointTrajectoryTracker<ACTION>::Tracker
			      ::Tracker(const std::string& robot_desc_string,
					const std::string& base_link,
					double publish_rate)
    :_base_link(base_link), _pointing_frame(), _duration(1.0/publish_rate),
     _urdf(), _listener(), _command(), _feedback(),
     _tree(), _chain(), _jnt_pos_min(), _jnt_pos_max(),
     _jac_solver(), _pos_fksolver(), _vel_iksolver(), _pos_iksolver()
{
  // Load URDF model.
    if (!_urdf.initString(robot_desc_string))
	throw std::runtime_error("Failed to parse urdf string.");

  // Construct KDL tree from robot_description parameter.
    if (!kdl_parser::treeFromString(robot_desc_string, _tree))
	throw std::runtime_error("Failed to construct kdl tree");

    ROS_INFO_STREAM("(JointTrajectoryTracker) tracker initialized");
}

template <class ACTION> size_t
JointTrajectoryTracker<ACTION>::Tracker::njoints() const
{
    return _chain.getNrOfJoints();
}

template <class ACTION>
const typename JointTrajectoryTracker<ACTION>::trajectory_t&
JointTrajectoryTracker<ACTION>::Tracker::command() const
{
    return _command;
}

template <class ACTION>
const typename JointTrajectoryTracker<ACTION>::feedback_t&
JointTrajectoryTracker<ACTION>::Tracker::feedback() const
{
    return _feedback;
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::Tracker::init(const state_cp& state,
					      const std::string& pointing_frame)
{
    if (!state)
	throw std::runtime_error("No controller state available");

    if (pointing_frame == _pointing_frame)
	return;

  // Get chain from _base_link to pointing_frame.
    if (!_tree.getChain(_base_link, pointing_frame, _chain))
	throw std::runtime_error("Couldn't create chain from "
				 + _base_link + " to " + pointing_frame);

  // Check number of joints between controller state and URDF chain.
    if (state->joint_names.size() != njoints())
	throw std::runtime_error("Number of joints mismatch: controller["
				 + std::to_string(state->joint_names.size())
				 + "] != urdf chain["
				 + std::to_string(njoints())
				 + ']');

  // Update pointing frame.
    _pointing_frame = pointing_frame;

  // Prepare trajectory command.
    _command.header.stamp	= ros::Time(0);
    _command.header.frame_id	= state->header.frame_id;
    _command.joint_names	= state->joint_names;
    _command.points.resize(1);

  // Set joint limits.
    _jnt_pos_min.resize(njoints());
    _jnt_pos_max.resize(njoints());
    for (size_t i = 0; i < njoints(); ++i)
    {
	const auto&	joint_name = _command.joint_names[i];

	_jnt_pos_min(i)	= _urdf.joints_[joint_name]->limits->lower;
	_jnt_pos_max(i)	= _urdf.joints_[joint_name]->limits->upper;
	ROS_INFO_STREAM(joint_name << ": limits=["
			<< _jnt_pos_min(i) << ", " << _jnt_pos_max(i) << ']');
    }

  // Create solvers.
    _jac_solver.reset(new KDL::ChainJntToJacSolver(_chain));
    _pos_fksolver.reset(new KDL::ChainFkSolverPos_recursive(_chain));
    _vel_iksolver.reset(new KDL::ChainIkSolverVel_wdls(_chain));
    _pos_iksolver.reset(new TRAC_IK::TRAC_IK(_chain,
					     _jnt_pos_min, _jnt_pos_max));

  // Get current joint positions and velocities.
    read(state);
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::Tracker::read(const state_cp& state)
{
    _command.points[0] = state->actual;
    _error	       = state->error;
}

template <class ACTION> std::string
JointTrajectoryTracker<ACTION>::Tracker::solver_error_message(int error)
{
    switch (error)
    {
      case -1:
	return "Not properly initialized with valid chain or limits";
      case -3:
	return "No solutions";
    }

    return "Select the best from " + std::to_string(error) + " solution(s)";
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::Tracker::clamp(KDL::JntArray& jnt_pos) const
{
    for (size_t i = 0; i < jnt_pos.rows(); ++i)
	jnt_pos(i) = std::clamp(jnt_pos(i), _jnt_pos_min(i), _jnt_pos_max(i));
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::Tracker::jointsFromKDL(
    const KDL::JntArray& kdl_joints, std::vector<double>& joints)
{
    for (size_t i = 0; i < joints.size(); ++i)
	joints[i] = kdl_joints(i);
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::Tracker::jointsToKDL(
    const std::vector<double>& joints, KDL::JntArray& kdl_joints)
{
    for (size_t i = 0; i < joints.size(); ++i)
	kdl_joints(i) = joints[i];
}

}	// namespace aist_controllers
#endif	// !AIST_CONTROLLERS_JOINT_TRAJECTORY_TRACKER_H
