/*
 *  \file	joint_group_tracker.cpp
 *  \brief	ROS tracker using JointGroupController
 */
#ifndef AIST_CONTROLLERS_JOINT_GROUP_TRACKER_H
#define AIST_CONTROLLERS_JOINT_GROUP_TRACKER_H

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

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

namespace aist_controllers
{
/************************************************************************
*  class JointGroupTracker<ACTION>					*
************************************************************************/
template <class ACTION>
class JointGroupTracker
{
  private:
    using state_cp	= sensor_msgs::JointStateConstPtr;
    using positions_t	= std_msgs::Float64MultiArray;
    using server_t	= actionlib::SimpleActionServer<ACTION>;
    using goal_cp	= boost::shared_ptr<const typename server_t::Goal>;
    using feedback_t	= typename server_t::Feedback;

    class Tracker
    {
      public:
			Tracker(const std::string& robot_desc_string,
				const std::string& base_link)		;

	size_t		njoints()				const	;
	const std::string&
			joint_name(size_t n)			const	;
	const positions_t&
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

	urdf::Model					_urdf;
	tf::TransformListener				_listener;
	positions_t					_command;
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
		JointGroupTracker(const std::string& action_ns)	;

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
JointGroupTracker<ACTION>
    ::JointGroupTracker(const std::string& action_ns)
    :_nh("~"),
     _state_sub(_nh.subscribe("/joint_states", 10,
			      &JointGroupTracker::state_cb, this)),
     _command_pub(_nh.advertise<positions_t>(
		      _nh.param<std::string>("controller",
					     "/pos_joint_group_controller")
		      + "/command", 2)),
     _tracker(_nh.param<std::string>(
		  _nh.param<std::string>("robot_description",
					 "/robot_description"),
		  std::string()),
	      _nh.param<std::string>("base_link", "base_link")),
     _tracker_srv(_nh, action_ns, false),
     _current_goal(nullptr)
{
  // Initialize action server
    _tracker_srv.registerGoalCallback(
    	std::bind(&JointGroupTracker::goal_cb, this));
    _tracker_srv.registerPreemptCallback(
    	std::bind(&JointGroupTracker::preempt_cb, this));
    _tracker_srv.start();
}

template <class ACTION> void
JointGroupTracker<ACTION>::goal_cb()
{
    _current_goal = _tracker_srv.acceptNewGoal();

    ROS_INFO_STREAM("(JointGroupTracker) Goal accepted");

    try
    {
	_tracker.init(_last_state, _current_goal->pointing_frame);
    }
    catch (const std::exception& err)
    {
	_tracker_srv.setAborted();
	ROS_ERROR_STREAM("(JointGroupTracker) Goal aborted["
			 << err.what() << ']');
    }
}

template <class ACTION> void
JointGroupTracker<ACTION>::preempt_cb()
{
    _tracker_srv.setPreempted();
    ROS_INFO_STREAM("(JointGroupTracker) Goal cancelled");
}

template <class ACTION> void
JointGroupTracker<ACTION>::state_cb(const state_cp& state)
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

	if (success)
	{
	    _tracker_srv.setSucceeded();
	    ROS_INFO_STREAM("(JointGroupTracker) Goal succeeded");
	}
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(JointGroupTracker) " << err.what());
      //ROS_ERROR_STREAM("(JointGroupTracker) Goal aborted");
      //_tracker_srv.setAborted();
    }
}

/************************************************************************
*  class JointGroupTracker<ACTION>::Tracker			*
************************************************************************/
template <class ACTION>
JointGroupTracker<ACTION>::Tracker
			      ::Tracker(const std::string& robot_desc_string,
					const std::string& base_link)
    :_base_link(base_link), _pointing_frame(),
     _urdf(), _listener(), _command(), _feedback(), _tree(), _chain(),
     _jnt_pos_min(), _jnt_pos_max(),
     _jac_solver(), _pos_fksolver(), _vel_iksolver(), _pos_iksolver()
{
  // Load URDF model.
    if (!_urdf.initString(robot_desc_string))
	throw std::runtime_error("Failed to parse urdf string.");

  // Construct KDL tree from robot_description parameter.
    if (!kdl_parser::treeFromString(robot_desc_string, _tree))
	throw std::runtime_error("Failed to construct kdl tree");

    ROS_INFO_STREAM("(JointGroupTracker) tracker initialized");
}

template <class ACTION> size_t
JointGroupTracker<ACTION>::Tracker::njoints() const
{
    return _chain.getNrOfJoints();
}

template <class ACTION> const std::string&
JointGroupTracker<ACTION>::Tracker::joint_name(size_t n) const
{
    return _chain.getSegment(n).getJoint().getName();
}

template <class ACTION>
const typename JointGroupTracker<ACTION>::positions_t&
JointGroupTracker<ACTION>::Tracker::command() const
{
    return _command;
}

template <class ACTION>
const typename JointGroupTracker<ACTION>::feedback_t&
JointGroupTracker<ACTION>::Tracker::feedback() const
{
    return _feedback;
}

template <class ACTION> void
JointGroupTracker<ACTION>::Tracker::init(const state_cp& state,
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

  // Update pointing frame.
    _pointing_frame = pointing_frame;

  // Prepare positions command.
    _command.layout.dim.resize(1);
    _command.layout.dim[0].label  = "joint_pos";
    _command.layout.dim[0].size   = njoints();
    _command.layout.dim[0].stride = njoints();
    _command.layout.data_offset   = 0;
    _command.data.resize(njoints());

  // Set joint limits.
    _jnt_pos_min.resize(njoints());
    _jnt_pos_max.resize(njoints());
    for (size_t i = 0; i < njoints(); ++i)
    {
	const auto&	joint_name = this->joint_name(i);

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
JointGroupTracker<ACTION>::Tracker::read(const state_cp& state)
{
    for (size_t i = 0; i < njoints(); ++i)
    {
	const auto&	joint_name = this->joint_name(i);
	size_t		j = 0;

	for (; j < state->name.size(); ++j)
	    if (state->name[j] == joint_name)
	    {
		_command.data[i] = state->position[j];
		break;
	    }
	if (j == state->name.size())
	    throw std::runtime_error("Joint[" + joint_name
				     + "] not found in input joint state");
    }
}

template <class ACTION> std::string
JointGroupTracker<ACTION>::Tracker::solver_error_message(int error)
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
JointGroupTracker<ACTION>::Tracker::clamp(KDL::JntArray& jnt_pos) const
{
    for (size_t i = 0; i < jnt_pos.rows(); ++i)
	jnt_pos(i) = std::clamp(jnt_pos(i), _jnt_pos_min(i), _jnt_pos_max(i));
}

template <class ACTION> void
JointGroupTracker<ACTION>::Tracker::jointsFromKDL(
    const KDL::JntArray& kdl_joints, std::vector<double>& joints)
{
    for (size_t i = 0; i < joints.size(); ++i)
	joints[i] = kdl_joints(i);
}

template <class ACTION> void
JointGroupTracker<ACTION>::Tracker::jointsToKDL(
    const std::vector<double>& joints, KDL::JntArray& kdl_joints)
{
    for (size_t i = 0; i < joints.size(); ++i)
	kdl_joints(i) = joints[i];
}

}	// namespace aist_controllers
#endif	// !AIST_CONTROLLERS_JOINT_GROUP_TRACKER_H
