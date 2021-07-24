/*
 *  $Id
 */
#pragma once

#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <cmath>

#include <actionlib/server/simple_action_server.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
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
    
/************************************************************************
*  class joint_position_tracker<ACTION>					*
************************************************************************/
template <class ACTION>
class joint_position_tracker
{
  private:
    using state_cp	= control_msgs::JointTrajectoryControllerStateConstPtr;
    using server_t	= actionlib::SimpleActionServer<ACTION>;
    using goal_cp	= boost::shared_ptr<typename server_t::Goal>;
    
    using vector3_t	= tf::Vector3;
    using point_t	= tf::Point;

    class commander
    {
      public:
			commander(const std::string& robot_desc_string,
				  const std::string& base_link,
				  const std::string& effector_link)	;

	const std::vector<std::string>&
			joint_names()				const	;

	void		init(const state_cp& state)			;
	void		read(const state_cp& state)			;
	trajectory_msgs::JointTrajectory
			compute_trajectory(const goal_cp&  goal)	;

      private:
	tf::Transform	get_transform()				const	;
	KDL::Wrench	compute_wrench(const goal_cp& goal)		;
	
      private:
	std::string			_base_link;
	std::string			_effector_link;
	urdf::Model			_urdf;
	std::vector<std::string>	_joint_names;
	KDL::Tree			_tree;
	KDL::Chain			_chain;
	KDL::JntArray			_jnt_pos;
	KDL::Jacobian			_jacobian;
	std::vector<urdf::JointLimits>	_limits;
	double				_goal_error;
	tf::TransformListener		_listener;
    
	boost::scoped_ptr<KDL::ChainFkSolverPos>	_pose_solver;
	boost::scoped_ptr<KDL::ChainJntToJacSolver>	_jac_solver;
    };
    
  public:
		joint_position_tracker(const std::string& action_ns)	;

  private:
    void	goal_cb()						;
    void	preempt_cb()						;
    void	state_cb(const state_cp& msg)				;

  private:
    ros::NodeHandle	_nh;
    ros::Subscriber	_state_sub;
    ros::Publisher	_command_pub;

    commander		_commander;

    state_cp		_last_state;
    server_t		_tracker_srv;
    goal_cp		_current_goal;
};

template <class ACTION>
joint_position_tracker<ACTION>::joint_position_tracker(
					const std::string& action_ns)
    :_nh("~"),
     _state_sub(_nh.subscribe("/state", 1,
			      &joint_position_tracker::state_cb, this)),
     _command_pub(_nh.advertise<trajectory_msgs::JointTrajectory>("command",
								  2)),
     _commander(_nh.param<std::string>(_nh.param<std::string>(
						   "robot_description",
						   "/robot_description"),
				       std::string()),
		_nh.param<std::string>("base_link", "base_link"),
		_nh.param<std::string>("effector_link", "effector_link")),
     _last_state(nullptr),
     _tracker_srv(_nh, action_ns, false),
     _current_goal(nullptr)
{
  // Initialize action server
    _tracker_srv.registerGoalCallback(boost::bind(&goal_cb, this));
    _tracker_srv.registerPreemptCallback(boost::bind(&preempt_cb, this));
    _tracker_srv.start();
}

template <class ACTION> void
joint_position_tracker<ACTION>::goal_cb()
{
    _current_goal = _tracker_srv.acceptNewGoal();

    ROS_INFO_STREAM("(joint_position_tracker) Goal accepted");

    try
    {
	_commander.init(_last_state);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(joint_position_tracker) " << err.what());
	_tracker_srv.setRejected();
    }
}

template <class ACTION> void
joint_position_tracker<ACTION>::preempt_cb()
{
  // Stops the controller.
    trajectory_msgs::JointTrajectory	empty;
    empty.joint_names = _commander.joint_names();
    _command_pub.publish(empty);

    _tracker_srv.setPreempted();
    ROS_INFO_STREAM("(joint_position_tracker) Goal cancelled");
}

template <class ACTION> void
joint_position_tracker<ACTION>::state_cb(const state_cp& state)
{
    _last_state = state;

    if (!_tracker_srv.isActive())
	return;

    try
    {
	_commander.read(state);
	_command_pub.publish(_commander.compute_trajectory(_current_goal));
	_tracker_srv.publishFeedback(_commander.compute_feedback());

      //the computed solution by the iterative solver can provide larger errors
      //due to MAX_ITERATIONS and correction_delta stop conditions
	if (feedback.pointing_angle_error <= _goal_error)
	{
	    ROS_DEBUG_STREAM("goal succeeded with error: "
			     << feedback.pointing_angle_error << " radians");
	    _tracker_srv.setSucceeded();
	}
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(joint_position_tracker) " << err.what());
	_tracker_srv.setAborted();
    }
}

/************************************************************************
*  class joint_position_tracker<ACTION>::commander			*
************************************************************************/
template <class ACTION>
joint_position_tracker<ACTION>
	::commander::commander(const std::string& robot_desc_string,
			       const std::string& base_link,
			       const std::string& effector_link)
	    :_base_link(base_link),
	     _effector_link(effector_link)
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
    _joint_names.resize(_chain.getNrOfJoints());
    _jnt_pos.resize(_joint_names.size());
    _jacobian.resize(_joint_names.size());
    _limits.resize(_joint_names.size());

  // Reset solvers.
    _pose_solver.reset(new KDL::ChainFkSolverPos_recursive(_chain));
    _jac_solver.reset(new KDL::ChainJntToJacSolver(_chain));
}

template <class ACTION> const std::vector<std::string>&
joint_position_tracker<ACTION>::commander::joint_names() const
{
    return _joint_names;
}

template <class ACTION> tf::Transform
joint_position_tracker<ACTION>::commander::get_transform() const
{
  // Get the current pose of effector_link w.r.t. base_link.
    KDL::Frame	pose_kdl;
    _pose_solver->JntToCart(_jnt_pos, pose_kdl);

  // Convert to tf::Transform
    tf::Transform	transform;
    tf::poseKDLToTF(pose_kdl, transform);

    return transform;
}

template <class ACTION> void
joint_position_tracker<ACTION>::commander::init(const state_cp& state)
{
    if (!state)
	throw std::runtime_error("No controller state available");

    if (state->joint_names.size() != _joint_names.size())
	throw std::runtime_error("Number of joints mismatch: controller["
				 + std::to_string(state->joint_names.size())
				 + "] <==> urdf chain["
				 + std::to_string(_joint_names.size())
				 + ']');

  // Get initial joint positions and joint limits.
    for (size_t i = 0; i < state->joint_names.size(); ++i)
    {
	_jnt_pos(i)	= state->actual.positions[i];
	_joint_names[i]	= state->joint_names[i];
	_limits[i]	= *(_urdf.joints_[_joint_names[i]]->limits);
	ROS_DEBUG_STREAM(_joint_names[i] << '[' << i << "]: " << _jnt_pos(i)
			 << ", limits: (" << _limits[i].lower
			 << ", " << _limits[i].upper << ')');
    }
}

template <class ACTION> void
joint_position_tracker<ACTION>::commander::read(const state_cp& state)
{
    for (size_t i = 0; i < _jnt_pos.rows(); ++i)
	_jnt_pos(i) = state->actual.positions[i];
}
    

}



