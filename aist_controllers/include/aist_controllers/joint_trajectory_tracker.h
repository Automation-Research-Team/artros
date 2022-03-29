/*
 *  \file	joint_trajectory_tracker.cpp
 *  \brief	ROS tracker using JointTrajectoryController
 */
#ifndef AIST_CONTROLLERS_JOINT_TRAJECTORY_TRACKER_H
#define AIST_CONTROLLERS_JOINT_TRAJECTORY_TRACKER_H

#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <cmath>

#include <actionlib/server/simple_action_server.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>

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
template <class T> inline static const T&
clamp(const T& val, const T& low, const T& high)
{
    return (val < low  ? low :
	    val > high ? high : val);
}

static std::ostream&
operator <<(std::ostream& out, const KDL::JntArray& jnt_array)
{
    for (size_t i = 0; i < jnt_array.rows(); ++i)
	out << ' ' << jnt_array(i);
    return out;
}

static std::ostream&
operator <<(std::ostream& out, const KDL::Twist& twist)
{
    for (size_t i = 0; i < 6; ++i)
	out << ' ' << twist(i);
    return out;
}

static std::ostream&
operator <<(std::ostream& out, const KDL::Wrench& wrench)
{
    for (size_t i = 0; i < 6; ++i)
	out << ' ' << wrench(i);
    return out;
}

static std::ostream&
operator <<(std::ostream& out, const tf::Vector3& v)
{
    return out << ' ' << v.x() << ' ' << v.y() << ' ' << v.z();
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
    using server_t	= actionlib::SimpleActionServer<ACTION>;
    using goal_cp	= boost::shared_ptr<const typename server_t::Goal>;
    using feedback_t	= typename server_t::Feedback;

    class Tracker
    {
      public:
			Tracker(const std::string& robot_desc_string,
				const std::string& base_link,
				const std::string& effector_link)	;

	const trajectory_t&
			trajectory()				const	;
	const feedback_t&
			feedback()				const	;

	void		init(const state_cp& state)			;
	void		read(const state_cp& state)			;
	bool		update(const goal_cp& goal)			;

      private:
	KDL::Frame	get_chain_transform()			const	;
	KDL::Wrench	compute_wrench(const goal_cp& goal)		;

      private:
	std::string					_base_link;
	std::string					_effector_link;

	urdf::Model					_urdf;
	trajectory_t					_trajectory;
	feedback_t					_feedback;

	KDL::Tree					_tree;
	KDL::Chain					_chain;
	KDL::JntArray					_jnt_pos;
	KDL::JntArray					_jnt_vel;
	KDL::JntArray					_jnt_pos_min;
	KDL::JntArray					_jnt_pos_max;
	KDL::Jacobian					_jacobian;
	std::vector<urdf::JointLimits>			_limits;
	double						_goal_error;
	tf::TransformListener				_listener;

	boost::scoped_ptr<KDL::ChainJntToJacSolver>	_jac_solver;
	boost::scoped_ptr<KDL::ChainFkSolverPos>	_pos_solver;
	boost::scoped_ptr<KDL::ChainIkSolverVel>	_vel_iksolver;
	boost::scoped_ptr<KDL::ChainIkSolverPos>	_pos_iksolver;
    };

  public:
		JointTrajectoryTracker(const std::string& action_ns)	;

  private:
    void	goal_cb()						;
    void	preempt_cb()						;
    void	state_cb(const state_cp& msg)				;

  private:
    ros::NodeHandle		_nh;
    ros::Subscriber		_state_sub;
    const ros::Publisher	_command_pub;

    Tracker			_tracker;

    state_cp			_last_state;
    server_t			_tracker_srv;
    goal_cp			_current_goal;
};

template <class ACTION>
JointTrajectoryTracker<ACTION>::JointTrajectoryTracker(
					const std::string& action_ns)
    :_nh("~"),
     _state_sub(_nh.subscribe("/state", 1,
			      &JointTrajectoryTracker::state_cb, this)),
     _command_pub(_nh.advertise<trajectory_t>(
		      '/' + _nh.param<std::string>("controller",
						   "pos_joint_traj_controller")
		      + "/command", 2)),
     _tracker(_nh.param<std::string>(
		  _nh.param<std::string>("robot_description",
					 "/robot_description"),
		  std::string()),
	      _nh.param<std::string>("base_link", "base_link"),
	      _nh.param<std::string>("effector_link", "effector_link")),
     _last_state(nullptr),
     _tracker_srv(_nh, action_ns, false),
     _current_goal(nullptr)
{
  // Initialize action server
    _tracker_srv.registerGoalCallback(
    	boost::bind(&JointTrajectoryTracker::goal_cb, this));
    _tracker_srv.registerPreemptCallback(
    	boost::bind(&JointTrajectoryTracker::preempt_cb, this));
    _tracker_srv.start();
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::goal_cb()
{
    _current_goal = _tracker_srv.acceptNewGoal();

    ROS_INFO_STREAM("(JointTrajectoryTracker) Goal accepted");

    try
    {
	_tracker.init(_last_state);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(JointTrajectoryTracker) " << err.what());
	_tracker_srv.setAborted();
    }
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::preempt_cb()
{
  // Stops the controller.
    trajectory_msgs::JointTrajectory	empty;
    empty.joint_names = _tracker.trajectory().joint_names;
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
	_command_pub.publish(_tracker.trajectory());

	if (success)
	{
	    _tracker_srv.setSucceeded();
	    ROS_INFO_STREAM("(JointTrajectoryTracker) Goal succeeded");
	}
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(JointTrajectoryTracker) " << err.what());
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
					const std::string& effector_link)
    :_base_link(base_link), _effector_link(effector_link),
     _urdf(), _trajectory(),
     _tree(), _chain(),
     _jnt_pos(), _jnt_vel(), _jnt_pos_min(), _jnt_pos_max(), _jacobian(),
     _goal_error(0.001),
     _listener(),
     _jac_solver(), _pos_solver(), _vel_iksolver(), _pos_iksolver()
{
  // Load URDF model.
    if (!_urdf.initString(robot_desc_string))
	throw std::runtime_error("Failed to parse urdf string.");

  // Construct KDL tree from robot_description parameter.
    if (!kdl_parser::treeFromString(robot_desc_string, _tree))
	throw std::runtime_error("Failed to construct kdl tree");

  // Get chain from root to leaf.
    if (!_tree.getChain(_base_link, _effector_link, _chain))
	throw std::runtime_error("Couldn't create chain from "
				 + _base_link + " to " + _effector_link);

  // Resize state variable arrays.
    _trajectory.joint_names.resize(_chain.getNrOfJoints());
    _jnt_pos.resize(_trajectory.joint_names.size());
    _jnt_vel.resize(_trajectory.joint_names.size());
    _jnt_pos_min.resize(_trajectory.joint_names.size());
    _jnt_pos_max.resize(_trajectory.joint_names.size());
    _jacobian.resize(_trajectory.joint_names.size());

  // Create solvers.
    _jac_solver.reset(new KDL::ChainJntToJacSolver(_chain));
    _pos_solver.reset(new KDL::ChainFkSolverPos_recursive(_chain));
    _vel_iksolver.reset(new KDL::ChainIkSolverVel_pinv(_chain));
    _pos_iksolver.reset(new KDL::ChainIkSolverPos_NR_JL(
			    _chain, _jnt_pos_min, _jnt_pos_max,
			    *_pos_solver, *_vel_iksolver));

    ROS_INFO_STREAM("(JointTrajectoryTracker) tracker initialized: base_link="
		    << _base_link << ", effector_link=" << _effector_link);
}

template <class ACTION>
const typename JointTrajectoryTracker<ACTION>::trajectory_t&
JointTrajectoryTracker<ACTION>::Tracker::trajectory() const
{
    return _trajectory;
}

template <class ACTION>
const typename JointTrajectoryTracker<ACTION>::feedback_t&
JointTrajectoryTracker<ACTION>::Tracker::feedback() const
{
    return _feedback;
}

template <class ACTION> KDL::Frame
JointTrajectoryTracker<ACTION>::Tracker::get_chain_transform() const
{
  // Get the current pose of effector_link w.r.t. base_link.
    KDL::Frame	pose_kdl;
    _pos_solver->JntToCart(_jnt_pos, pose_kdl);

    return pose_kdl;

  // Convert to tf::Transform
    // tf::Transform	transform;
    // tf::poseKDLToTF(pose_kdl, transform);

    // return transform;
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::Tracker::init(const state_cp& state)
{
    if (!state)
	throw std::runtime_error("No controller state available");

    auto&	joint_names = _trajectory.joint_names;
    if (state->joint_names.size() != joint_names.size())
	throw std::runtime_error("Number of joints mismatch: controller["
				 + std::to_string(state->joint_names.size())
				 + "] != urdf chain["
				 + std::to_string(joint_names.size())
				 + ']');

  // Get initial joint positions and joint limits.
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
	joint_names[i]	= state->joint_names[i];
	_jnt_pos(i)	= state->actual.positions[i];
	_jnt_vel(i)	= state->actual.velocities[i];
	_jnt_pos_min(i)	= _urdf.joints_[joint_names[i]]->limits->lower;
	_jnt_pos_max(i)	= _urdf.joints_[joint_names[i]]->limits->upper;
	ROS_DEBUG_STREAM(joint_names[i] << '[' << i << "]: " << _jnt_pos(i)
			 << ", limits: ("
			 << _jnt_pos_min(i) << ", " << _jnt_pos_max(i) << ')');
    }
}

template <class ACTION> void
JointTrajectoryTracker<ACTION>::Tracker::read(const state_cp& state)
{
    _trajectory.header.stamp = ros::Time(0);
    _trajectory.points.resize(1);
    _trajectory.points[0] = state->actual;

    for (size_t i = 0; i < _jnt_pos.rows(); ++i)
    {
	_jnt_pos(i) = state->actual.positions[i];
	_jnt_vel(i) = state->actual.velocities[i];
    }
}

}	// namespace aist_controllers
#endif	// !AIST_CONTROLLERS_JOINT_TRAJECTORY_TRACKER_H
