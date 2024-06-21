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
// Author: Toshio Ueshiba
//
/*!
  \file	 Calibrator.cpp
  \brief Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/
#include <fstream>
#include <sstream>
#include <ctime>
#include <cstdlib>	// for std::getenv()
#include <sys/stat.h>	// for mkdir()
#include <errno.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <aist_handeye_calibration_msgs/srv/get_sample_list.hpp>
#include <aist_handeye_calibration_msgs/srv/compute_calibration.hpp>
#include <aist_handeye_calibration_msgs/action/take_sample.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <aist_utility/geometry_msgs.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include "HandeyeCalibration.h"

//#define DEBUG

namespace aist_handeye_calibration
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
class Calibrator : public rclcpp::Node
{
  public:
    using transform_t	= geometry_msgs::msg::TransformStamped;

  private:
    using pose_t		= geometry_msgs::msg::PoseStamped;
    using pose_p		= pose_t::UniquePtr;
    using GetSampleList		= aist_handeye_calibration_msgs::srv
					::GetSampleList;
    using GetSampleListSrvPtr	= rclcpp::Service<GetSampleList>::SharedPtr;
    using GetSampleListReqPtr	= GetSampleList::Request::SharedPtr;
    using GetSampleListResPtr	= GetSampleList::Response::SharedPtr;
    using ComputeCalibration	= aist_handeye_calibration_msgs::srv
					::ComputeCalibration;
    using ComputeCalibrationSrvPtr
				= rclcpp::Service<ComputeCalibration>::SharedPtr;
    using ComputeCalibrationReqPtr
				= ComputeCalibration::Request::SharedPtr;
    using ComputeCalibrationResPtr
				= ComputeCalibration::Response::SharedPtr;
    using Trigger		= std_srvs::srv::Trigger;
    using TriggerSrvPtr		= rclcpp::Service<Trigger>::SharedPtr;
    using TriggerReqPtr		= Trigger::Request::SharedPtr;
    using TriggerResPtr		= Trigger::Response::SharedPtr;
    using Empty			= std_srvs::srv::Empty;
    using EmptySrvPtr		= rclcpp::Service<Empty>::SharedPtr;
    using EmptyReqPtr		= Empty::Request::SharedPtr;
    using EmptyResPtr		= Empty::Response::SharedPtr;
    using TakeSample		= aist_handeye_calibration_msgs::action
					::TakeSample;
    using TakeSampleSrvPtr	= rclcpp_action::Server<TakeSample>::SharedPtr;
    using TakeSampleGoalPtr	= std::shared_ptr<const TakeSample::Goal>;
    using TakeSampleGoalHandlePtr
	      = std::shared_ptr<rclcpp_action::ServerGoalHandle<TakeSample> >;
    
  public:
		Calibrator(const rclcpp::NodeOptions& options)		;

  private:
     std::string	node_name()	const	{ return get_name(); }
    template <class T>
    T			declare_read_only_parameter(const std::string& name,
						    const T& default_value);
    const std::string&	camera_frame()				const	;
    const std::string&	effector_frame()			const	;
    const std::string&	marker_frame()				const	;
    const std::string&	world_frame()				const	;

    void	pose_cb(pose_p pose)					;
    void	get_sample_list(const GetSampleListReqPtr,
				GetSampleListResPtr res)		;
    void	compute_calibration(const ComputeCalibrationReqPtr,
				    ComputeCalibrationResPtr res)	;
    void	save_calibration(const TriggerReqPtr, TriggerResPtr res);
    void	reset(const EmptyReqPtr, EmptyResPtr)			;
    rclcpp_action::GoalResponse
		take_sample_goal_cb(const rclcpp_action::GoalUUID&,
				    TakeSampleGoalPtr)			;
    rclcpp_action::CancelResponse
		take_sample_cancel_cb(TakeSampleGoalHandlePtr)		;
    void	take_sample_accept_cb(TakeSampleGoalHandlePtr gh)	;

  private:
    const rclcpp::Subscription<pose_t>::SharedPtr _pose_sub;
    const GetSampleListSrvPtr			  _get_sample_list_srv;
    const ComputeCalibrationSrvPtr		  _compute_calibration_srv;
    const TriggerSrvPtr				  _save_calibration_srv;
    const EmptySrvPtr				  _reset_srv;
    const TakeSampleSrvPtr			  _take_sample_srv;
    TakeSampleGoalHandlePtr			  _current_goal_handle;
    
    tf2_ros::Buffer				  _tf2_buffer;
    const tf2_ros::TransformListener		  _tf2_listener;

    std::vector<transform_t>	_Tcm;	//!< in:  camera <- marker   transform
    std::vector<transform_t>	_Twe;	//!< in:  world  <- effector transform
    transform_t			_Tec;	//!< out: effector <- camera transform
    transform_t			_Twm;	//!< out: world    <- marker transform

    const bool			_use_dual_quaternion;
    const bool			_eye_on_hand;
    const std::string		_camera_name;
};

Calibrator::Calibrator(const rclcpp::NodeOptions& options)
    :rclcpp::Node("calibrator", options),
     _pose_sub(create_subscription<pose_t>("/pose", 1,
					   std::bind(&Calibrator::pose_cb,
						     this,
						     std::placeholders::_1))),
     _get_sample_list_srv(create_service<GetSampleList>(
			      node_name() + "/get_sample_list",
			      std::bind(&Calibrator::get_sample_list, this,
					std::placeholders::_1,
					std::placeholders::_2))),
     _compute_calibration_srv(create_service<ComputeCalibration>(
				  node_name() + "/compute_calibration",
				  std::bind(&Calibrator::compute_calibration,
					    this,
					    std::placeholders::_1,
					    std::placeholders::_2))),
     _save_calibration_srv(create_service<Trigger>(
			       node_name() + "/save_calibration",
			       std::bind(&Calibrator::save_calibration, this,
					 std::placeholders::_1,
					 std::placeholders::_2))),
     _reset_srv(create_service<Empty>(node_name() + "/reset",
				      std::bind(&Calibrator::reset, this,
						std::placeholders::_1,
						std::placeholders::_2))),
     _take_sample_srv(rclcpp_action::create_server<TakeSample>(
			  this, node_name() + "/take_sample",
			  std::bind(&Calibrator::take_sample_goal_cb, this,
				    std::placeholders::_1,
				    std::placeholders::_2),
			  std::bind(&Calibrator::take_sample_cancel_cb, this,
				    std::placeholders::_1),
			  std::bind(&Calibrator::take_sample_accept_cb, this,
				    std::placeholders::_1))),
     _current_goal_handle(nullptr),
     _tf2_buffer(get_clock()),
     _tf2_listener(_tf2_buffer),
     _use_dual_quaternion(declare_read_only_parameter<bool>(
			      "use_dual_quaternion", true)),
     _eye_on_hand(declare_read_only_parameter<bool>("eye_on_hand", true)),
     _camera_name(declare_read_only_parameter<std::string>("camera_name",
							   "camera"))
{
    RCLCPP_INFO_STREAM(get_logger(), "initializing calibrator...");

    if (_eye_on_hand)
    {
	_Tec.header.frame_id = declare_read_only_parameter<std::string>(
				   "robot_effector_frame", "tool0");
	_Twm.header.frame_id = declare_read_only_parameter<std::string>(
				   "robot_base_frame", "base_link");
    }
    else
    {
	_Twm.header.frame_id = declare_read_only_parameter<std::string>(
				   "robot_effector_frame", "tool0");
	_Tec.header.frame_id = declare_read_only_parameter<std::string>(
				   "robot_base_frame", "base_link");
    }

    _Tec.child_frame_id = "";
    _Twm.child_frame_id = declare_read_only_parameter<std::string>(
			      "marker_frame", "marker_frame");
}

template <class T> T
Calibrator::declare_read_only_parameter(const std::string& name,
					const T& default_value)
{
    return declare_parameter(
		name, default_value,
		ddynamic_reconfigure2::read_only_param_desc<T>(name));
}

const std::string&
Calibrator::camera_frame() const
{
    return _Tec.child_frame_id;
}

const std::string&
Calibrator::effector_frame() const
{
    return _Tec.header.frame_id;
}

const std::string&
Calibrator::marker_frame() const
{
    return _Twm.child_frame_id;
}

const std::string&
Calibrator::world_frame() const
{
    return _Twm.header.frame_id;
}

void
Calibrator::pose_cb(pose_p pose)
{
    if (!_current_goal_handle)
	return;

    try
    {
	using namespace	aist_utility;
	
      // Set camera frame.
	_Tec.child_frame_id = pose->header.frame_id;

      // Convert marker pose to camera <= object transform.
	auto	result = std::make_shared<TakeSample::Result>();
	result->transform_cm = aist_utility::toTransform(*pose,
							 marker_frame());

      // Lookup world <= effector transform at the moment marker detected.
	result->transform_we = _tf2_buffer.lookupTransform(
				   world_frame(), effector_frame(),
				   pose->header.stamp,
				   tf2::durationFromSec(1.0));
	RCLCPP_DEBUG_STREAM(get_logger(),
			    "Tcm: " << result->transform_cm.transform);
	RCLCPP_DEBUG_STREAM(get_logger(),
			    "Twe: " << result->transform_we.transform);

	_Tcm.emplace_back(result->transform_cm);
	_Twe.emplace_back(result->transform_we);

	_current_goal_handle->succeed(result);

	RCLCPP_INFO_STREAM(get_logger(), "take_sample(): succeeded");
    }
    catch (const std::exception& err)
    {
	_current_goal_handle->abort(nullptr);

	RCLCPP_ERROR_STREAM(get_logger(),
			    "take_sample(): aborted[" << err.what() << ']');
    }
}

void
Calibrator::get_sample_list(const GetSampleListReqPtr, GetSampleListResPtr res)
{
    res->success	= true;
    res->message	= std::to_string(_Tcm.size()) + " samples in hand.";
    res->transform_cm	= _Tcm;
    res->transform_we	= _Twe;

    RCLCPP_INFO_STREAM(get_logger(), "get_sample_list(): " << res->message);
}

void
Calibrator::compute_calibration(const ComputeCalibrationReqPtr,
				ComputeCalibrationResPtr res)
{
    try
    {
	RCLCPP_INFO_STREAM(get_logger(),
			   "compute_calibration(): computing with "
			   << (_use_dual_quaternion ? "DUAL quaternion"
						    : "SINGLE quaternion")
			   << " algorithm...");

	std::vector<TU::Transform<double> >	Tcm, Twe;
	for (size_t i = 0; i < _Tcm.size(); ++i)
	{
	    Tcm.emplace_back(_Tcm[i].transform);
	    Twe.emplace_back(_Twe[i].transform);
	}

	const auto	Tec = (_use_dual_quaternion ?
			       TU::cameraToEffectorDual(Tcm, Twe) :
			       TU::cameraToEffectorSingle(Tcm, Twe));
	const auto	Twm = TU::objectToWorld(Tcm, Twe, Tec);

	const auto	now = get_clock()->now();
	_Tec.header.stamp = now;
	_Tec.transform	  = Tec;
	_Twm.header.stamp = now;
	_Twm.transform	  = Twm;

	std::ostringstream	sout;
	TU::evaluateAccuracy(sout, Tcm, Twe, Tec, Twm);
	res->success	  = true;
	res->message	  = sout.str();
	res->transform_ec = _Tec;
	res->transform_wm = _Twm;

	RCLCPP_INFO_STREAM(get_logger(),
			   "compute_calibration(): " << res->message);
    }
    catch (const std::exception& err)
    {
	res->success = false;
	res->message = err.what();

	RCLCPP_ERROR_STREAM(get_logger(),
			    "compute_calibration(): " << res->message);
    }
}

void
Calibrator::save_calibration(const TriggerReqPtr, TriggerResPtr res)
{
    try
    {
	using	aist_utility::operator <<;

	const auto	tval = time(nullptr);
	const auto	tstr = ctime(&tval);
	tstr[strlen(tstr)-1] = '\0';

	YAML::Emitter	emitter;
	emitter << YAML::BeginMap
		<< YAML::Key   << "calibration_date"
		<< YAML::Value << tstr
		<< YAML::Key   << "eye_on_hand"
		<< YAML::Value << _eye_on_hand
		<< YAML::Key   << "use_dual_quaternion"
		<< YAML::Value << _use_dual_quaternion
		<< YAML::Key   << "camera_transform"
		<< YAML::Value << _Tec
		<< YAML::Key   << "marker_transform"
		<< YAML::Value << _Twm;
	aist_utility::operator <<(emitter << YAML::Key << "Tcm"<< YAML::Value,
				  _Tcm);
	aist_utility::operator <<(emitter << YAML::Key << "Twe"<< YAML::Value,
				  _Twe);
	emitter << YAML::EndMap;

      // Read calibration file name from parameter server.
	const auto	calib_file = std::string(getenv("HOME"))
				   + "/.ros/aist_handeye_calibration/"
				   + _camera_name + ".yaml";

      // Open/create parent directory of the calibration file.
	const auto	dir = calib_file.substr(0,
						calib_file.find_last_of('/'));
	struct stat	buf;
	if (stat(dir.c_str(), &buf) && mkdir(dir.c_str(), S_IRWXU))
	    throw std::runtime_error("cannot create " + dir + ": "
						      + strerror(errno));

      // Open calibration file.
	std::ofstream	out(calib_file.c_str());
	if (!out)
	    throw std::runtime_error("cannot open " + calib_file + ": "
						    + strerror(errno));

      // Save calitration results.
	out << emitter.c_str() << std::endl;

	res->success = true;
	res->message = "saved in " + calib_file;

	RCLCPP_INFO_STREAM(get_logger(),
			   "save_calibration(): " << res->message);
    }
    catch (const std::exception& err)
    {
	res->success = false;
	res->message = err.what();

	RCLCPP_ERROR_STREAM(get_logger(),
			    "compute_calibration(): " << res->message);
    }
}

void
Calibrator::reset(const EmptyReqPtr, EmptyResPtr)
{
    _Tcm.clear();
    _Twe.clear();

    RCLCPP_INFO_STREAM(get_logger(), "reset(): all samples cleared.");
}

rclcpp_action::GoalResponse
Calibrator::take_sample_goal_cb(const rclcpp_action::GoalUUID&,
				TakeSampleGoalPtr)
{
    RCLCPP_INFO_STREAM(get_logger(), "TakeSampleAction: new goal received");

    return (!_current_goal_handle ?
	    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
	    rclcpp_action::GoalResponse::REJECT);
}

rclcpp_action::CancelResponse
Calibrator::take_sample_cancel_cb(TakeSampleGoalHandlePtr)
{
    _current_goal_handle.reset();

    RCLCPP_WARN_STREAM(get_logger(),
		       "TakeSampleAction: current goal canceled");

    return rclcpp_action::CancelResponse::ACCEPT;
}

void
Calibrator::take_sample_accept_cb(TakeSampleGoalHandlePtr gh)
{
    _current_goal_handle = gh;

    RCLCPP_INFO_STREAM(get_logger(), "take_sample(): new goal accepted");
}

}	// namespace aist_handeye_calibration

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_handeye_calibration::Calibrator)
