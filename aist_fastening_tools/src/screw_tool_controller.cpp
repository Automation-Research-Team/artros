// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
//
/*!
 *  \file	screw_tool_controller.cpp
 *  \brief	controller for screw tools
 */
#include <ros/ros.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <actionlib/server/simple_action_server.h>
#include <aist_fastening_tools/ScrewToolCommandAction.h>
#include <aist_fastening_tools/ScrewToolStatus.h>
#include <aist_utility/butterworth_lpf.h>

namespace aist_fastening_tools
{
/************************************************************************
*  static functions							*
************************************************************************/
static int32_t
target_speed(double speed)
{
    speed = std::clamp(speed, -1.0, 1.0);

    return (speed >= 0.0 ? int32_t(1023*speed) : 1024 - int32_t(1023*speed));
}

static double
get_normalized(int32_t value)
{
    value = std::clamp(value, 0, 2047);
    return (value < 1024 ? value : 1024 - value) / 1023.0;
}

/************************************************************************
*  class ScrewToolController						*
************************************************************************/
class ScrewToolController
{
  private:
    using dynamixel_states_t	= dynamixel_workbench_msgs::DynamixelStateList;
    using dynamixel_states_cp	= dynamixel_workbench_msgs::
					DynamixelStateListConstPtr;
    using dynamixel_command_t	= dynamixel_workbench_msgs::DynamixelCommand;
    using server_t		= actionlib::SimpleActionServer<
					ScrewToolCommandAction>;
    using goal_cp		= boost::shared_ptr<const server_t::Goal>;
    using ddynamic_reconfigure_t= ddynamic_reconfigure::DDynamicReconfigure;
    using filter_t		= aist_utility::ButterworthLPF<double, double>;

    enum Stage	{ ACTIVE, LOOSEN, RETIGHTEN, DONE };

  public:
		ScrewToolController(ros::NodeHandle& nh,
				    const std::string& driver_ns)	;

  private:
    void	goal_cb()						;
    void	preempt_cb()						;
    void	dynamixel_states_cb(const dynamixel_states_cp& states)	;

  private:
    bool	is_satisfied(double ratio, double max_ratio,
			     const ros::Duration& min_period)		;
    bool	send_dynamixel_command(const std::string& addr_name,
				       int32_t value)			;
    void	set_period(ros::Duration& period, double sec)		;
    void	set_filter_half_order(int half_order)			;
    void	set_filter_cutoff_frequency(double cutoff_frequency)	;

  private:
  // Basic stuffs
    const std::string		_node_ns;
    const uint8_t		_motor_id;
    Stage			_stage;
    ros::Time			_start_time;

  // Dynamixel driver stuffs
    const ros::Subscriber	_dynamixel_states_sub;
    ros::ServiceClient		_dynamixel_command;

  // Status publishment stuffs
    ros::Publisher		_status_pub;

  // Action stuffs
    server_t			_command_srv;
    goal_cp			_active_goal;

  // Parameters
    ddynamic_reconfigure_t	_ddr;
    ros::Duration		_loosen_period;      // period before retighten
    double			_max_stall_speed;
    ros::Duration		_min_stall_period;
    double			_max_noload_current;
    ros::Duration		_min_noload_period;

  // Current filtering stuffs
    const ros::Duration		_control_period;
    double			_current;
    filter_t			_filter;
};

ScrewToolController::ScrewToolController(ros::NodeHandle& nh,
					 const std::string& driver_ns)
    :_node_ns(nh.getNamespace()),
     _motor_id(nh.param<int>("motor_id", 1)),
     _stage(DONE),
     _start_time(),
     _dynamixel_states_sub(nh.subscribe<dynamixel_states_t>(
			       driver_ns + "/dynamixel_state", 1,
			       &dynamixel_states_cb, this)),
     _dynamixel_command(nh.serviceClient<dynamixel_command_t>(
			    driver_ns + "/dynamixel_command")),
     _status_pub(nh.advertise<ScrewToolStatus>("status", 1)),
     _command_srv(nh, "command", false),
     _active_goal(nullptr),
     _ddr(nh),
     _loosen_period(1.0),
     _max_stall_speed(0.01),
     _min_stall_period(0.5),
     _max_noload_current(0.3),
     _min_noload_period(0.5),
     _control_period(nh.param<double>("control_period", 0.01)),
     _current(0.0),
     _filter(2, 7.0*_control_period.toSec())
{
  // Setup ScrewToolCommand action server.
    _command_srv.registerGoalCallback(boost::bind(&goal_cb, this));
    _command_srv.registerPreemptCallback(boost::bind(&preempt_cb, this));
    _command_srv.start();

  // Setup ddynamic_reconfigure server.
    _ddr.registerVariable<double>("loosen_period",
				  _loosen_period.toSec(),
				  boost::bind(&ScrewToolController::set_period,
					      this, _loosen_period, _1),
				  "Period of loosening before retightening",
				  0.1, 5.0, "control parameters");
    _ddr.registerVariable<double>("max_stall_speed", &_max_stall_speed,
				  "Maximum ratio of speed to be judged as stalled",
				  0.0, 0.05, "control parameters");
    _ddr.registerVariable<double>("min_stall_period",
				  _min_stall_period.toSec(),
				  boost::bind(&ScrewToolController::set_period,
					      this, _min_stall_period, _1),
				  "Minimum period required to be judged as stalled",
				  0.1, 1.0, "control parameters");
    _ddr.registerVariable<double>("max_noload_current", &_max_noload_current,
				  "Maximum ratio of current to be judged as unloaded",
				  0.0, 1.0, "control parameters");
    _ddr.registerVariable<double>("min_noload_period",
				  _min_noload_period.toSec(),
				  boost::bind(&ScrewToolController::set_period,
					      this, _min_noload_period, _1),
				  "Minimum period required to be judged as unloaded",
				  0.1, 1.0, "control parameters");
    _ddr.registerVariable<int>("filter_half_order", _filter.half_order(),
			       boost::bind(&ScrewToolController::
					   set_filter_half_order, this, _1),
			       "Half order of current low pass filter",
			       1, 5, "filtering parameters");
    _ddr.registerVariable<double>("filter_cutoff_frequency",
				  _filter.cutoff()/_control_period.toSec(),
				  boost::bind(&ScrewToolController::
					      set_filter_cutoff_frequency,
					      this, _1),
				  "Cutoff frequency of current low pass filter",
				  1, 30, "filtering parameters");

    _ddr.publishServicesTopicsAndUpdateConfigData();

    ROS_INFO_STREAM('(' << _node_ns << ") controller started with motor ID["
		    << int(_motor_id) << ']');
}

void
ScrewToolController::goal_cb()
{
    _active_goal = _command_srv.acceptNewGoal();
    _stage	 = ACTIVE;
    _start_time  = ros::Time::now();
    send_dynamixel_command("Torque_Enable", 1);
    send_dynamixel_command("Moving_Speed", target_speed(_active_goal->speed));

    ROS_INFO_STREAM('(' << _node_ns << ") goal ACCEPTED: "
		    << (_active_goal->speed > 0 ? "tighten" : "loosen")
		    << " with speed=" << _active_goal->speed);
}

void
ScrewToolController::preempt_cb()
{
    _command_srv.setPreempted();
    send_dynamixel_command("Moving_Speed",  0);
    send_dynamixel_command("Torque_Enable", 0);

    ROS_INFO_STREAM('(' << _node_ns << ") goal PREEMPTED");
}

void
ScrewToolController::dynamixel_states_cb(const dynamixel_states_cp& states)
{
  // Find a dynamixel state with my motor ID.
    const auto	state = std::find_if(states->dynamixel_state.begin(),
				     states->dynamixel_state.end(),
				     [motor_id=_motor_id](const auto& state)
				     { return state.id == motor_id; });
    if (state == states->dynamixel_state.end())
    {
	ROS_ERROR_STREAM('(' << _node_ns << ") no motors with ID["
			 << int(_motor_id)
			 << "] found in incoming dynamixel state list!");
	return;
    }

  // Read current value.
    _current = get_normalized(state->present_current);

  // Publish tool status.
    ScrewToolStatus	status;
    status.header.stamp	= ros::Time::now();
    status.speed	= get_normalized(state->present_velocity);
    status.current	= _filter.filter(_current);
    _status_pub.publish(status);

  // Read current value and apply low-pass filter.
  // Check if an active goal is available.
    if (!_command_srv.isActive())
	return;

  // Publish speed and filtered current as a feedback.
    ScrewToolCommandFeedback	feedback;
    feedback.speed   = status.speed;
    feedback.current = status.current;
    _command_srv.publishFeedback(feedback);

    if (_active_goal->speed > 0.0)
	switch (_stage)
	{
	  case ACTIVE:
	    if (is_satisfied(feedback.speed,
			     _max_stall_speed, _min_stall_period))
	    {
		if (_active_goal->retighten)
		{
		    send_dynamixel_command("Moving_Speed", target_speed(0.0));
		    ros::Duration(0.1).sleep();

		    ROS_INFO_STREAM('(' << _node_ns
				    << ") slightly loosen screw");

		    send_dynamixel_command("Moving_Speed",
					   target_speed(-_active_goal->speed));
		    _stage	= LOOSEN;
		    _start_time = ros::Time::now();
		}
		else
		    _stage = DONE;
	    }
	    break;
	  case LOOSEN:
	    if (ros::Time::now() - _start_time > _loosen_period)
	    {
		send_dynamixel_command("Moving_Speed", target_speed(0.0));
		ros::Duration(0.1).sleep();

		ROS_INFO_STREAM('(' << _node_ns << ") retighten screw");

		send_dynamixel_command("Moving_Speed",
				       target_speed(_active_goal->speed));
		_stage	    = RETIGHTEN;
		_start_time = ros::Time::now();
	    }
	    break;
	  case RETIGHTEN:
	    if (is_satisfied(feedback.speed,
			     _max_stall_speed, _min_stall_period))
		_stage = DONE;
	    break;
	  default:
	    break;
	}
    else if (is_satisfied(status.current,
			  _max_noload_current, _min_noload_period))
	_stage = DONE;

    if (_stage == DONE)
    {
	send_dynamixel_command("Moving_Speed", target_speed(0.0));
	send_dynamixel_command("Enable_Torque", 0);
	_command_srv.setSucceeded();

	ROS_INFO_STREAM('(' << _node_ns << ") goal SUCCEEDED");
    }
}

bool
ScrewToolController::is_satisfied(double ratio, double max_ratio,
				  const ros::Duration& min_period)
{
    if (std::abs(ratio) > max_ratio)
	_start_time = ros::Time::now();

    return (ros::Time::now() - _start_time > min_period);
}

bool
ScrewToolController::send_dynamixel_command(const std::string& addr_name,
					    int32_t value)
{
    dynamixel_command_t	command;
    command.request.id	      = _motor_id;
    command.request.addr_name = addr_name;
    command.request.value     = value;
    _dynamixel_command.call(command);

    return command.response.comm_result;
}

void
ScrewToolController::set_period(ros::Duration& period, double sec)
{
    period = ros::Duration(sec);
}

void
ScrewToolController::set_filter_half_order(int half_order)
{
    _filter.initialize(half_order, _filter.cutoff());
    _filter.reset(_current);
}

void
ScrewToolController::set_filter_cutoff_frequency(double cutoff_frequency)
{
    _filter.initialize(_filter.half_order(),
		       cutoff_frequency*_control_period.toSec());
    _filter.reset(_current);
}

}	// namespace aist_fastening_tools

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "screw_tool_controller");

    try
    {
	ros::NodeHandle	nh("~");
	const auto	driver_ns = nh.param<std::string>("driver_ns",
							  "screw_tool_driver");

	aist_fastening_tools::ScrewToolController controller(nh, driver_ns);
	ros::spin();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
