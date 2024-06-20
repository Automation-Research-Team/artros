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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aist_utility/geometry_msgs.h>
#include "Calibrator.h"
#include "HandeyeCalibration.h"

//#define DEBUG

namespace aist_handeye_calibration
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
Calibrator::Calibrator(const ros::NodeHandle& nh)
    :_nh(nh),
     _pose_sub(_nh.subscribe("/pose", 5, &Calibrator::pose_cb, this)),
     _get_sample_list_srv(
	 _nh.advertiseService("get_sample_list",
			      &Calibrator::get_sample_list, this)),
     _compute_calibration_srv(
	 _nh.advertiseService("compute_calibration",
			      &Calibrator::compute_calibration, this)),
     _save_calibration_srv(
	 _nh.advertiseService("save_calibration",
			      &Calibrator::save_calibration, this)),
     _reset_srv(_nh.advertiseService("reset", &Calibrator::reset, this)),
     _take_sample_srv(_nh, "take_sample", false),
     _tf2_buffer(),
     _tf2_listener(_tf2_buffer),
     _use_dual_quaternion(_nh.param<bool>("use_dual_quaternion", true)),
     _eye_on_hand(_nh.param<bool>("eye_on_hand", true)),
     _timeout(_nh.param<double>("timeout", 5.0))
{
    ROS_INFO_STREAM("initializing calibrator...");

    if (_eye_on_hand)
    {
	_Tec.header.frame_id = _nh.param<std::string>("robot_effector_frame",
						      "tool0");
	_Twm.header.frame_id = _nh.param<std::string>("robot_base_frame",
						      "base_link");
    }
    else
    {
	_Twm.header.frame_id = _nh.param<std::string>("robot_effector_frame",
						      "tool0");
	_Tec.header.frame_id = _nh.param<std::string>("robot_base_frame",
						      "base_link");
    }

    _Tec.child_frame_id = "";
    _Twm.child_frame_id = _nh.param<std::string>("marker_frame",
						 "marker_frame");

    _take_sample_srv.registerGoalCallback(boost::bind(&Calibrator::take_sample,
						      this));
    _take_sample_srv.registerPreemptCallback(boost::bind(&Calibrator::cancel,
							 this));
    _take_sample_srv.start();
}

Calibrator::~Calibrator()
{
}

void
Calibrator::run()
{
    ros::spin();
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
Calibrator::pose_cb(const poseMsg_cp& poseMsg)
{
    if (!_take_sample_srv.isActive())
	return;

    try
    {
      // Set camera frame.
	_Tec.child_frame_id = poseMsg->header.frame_id;

      // Convert marker pose to camera <= object transform.
	TakeSampleResult	result;
	result.Tcm = aist_utility::toTransform(*poseMsg, marker_frame());

      // Lookup world <= effector transform at the moment marker detected.
	result.Twe = _tf2_buffer.lookupTransform(world_frame(),
						 effector_frame(),
						 poseMsg->header.stamp,
						 _timeout);
	ROS_DEBUG_STREAM(result.Tcm);
	ROS_DEBUG_STREAM(result.Twe);

	_Tcm.emplace_back(result.Tcm);
	_Twe.emplace_back(result.Twe);

	_take_sample_srv.setSucceeded(result);

	ROS_INFO_STREAM("take_sample(): succeeded");
    }
    catch (const std::exception& err)
    {
	_take_sample_srv.setAborted();

	ROS_ERROR_STREAM("take_sample(): aborted[" << err.what() << ']');
    }
}

bool
Calibrator::get_sample_list(GetSampleList::Request&,
			    GetSampleList::Response& res)
{
    res.success = true;
    res.message = std::to_string(_Tcm.size()) + " samples in hand.";
    res.Tcm	= _Tcm;
    res.Twe	= _Twe;

    ROS_INFO_STREAM("get_sample_list(): " << res.message);

    return true;
}

bool
Calibrator::compute_calibration(ComputeCalibration::Request&,
				ComputeCalibration::Response& res)
{
    try
    {
	using transform_t	= TU::Transform<double>;

	ROS_INFO_STREAM("compute_calibration(): computing with "
			<< (_use_dual_quaternion ? "DUAL quaternion"
						 : "SINGLE quaternion")
			<< " algorithm...");

	std::vector<transform_t>	Tcm, Twe;
	for (size_t i = 0; i < _Tcm.size(); ++i)
	{
	    Tcm.emplace_back(_Tcm[i].transform);
	    Twe.emplace_back(_Twe[i].transform);
	}

	const auto	Tec = (_use_dual_quaternion ?
			       TU::cameraToEffectorDual(Tcm, Twe) :
			       TU::cameraToEffectorSingle(Tcm, Twe));
	const auto	Twm = TU::objectToWorld(Tcm, Twe, Tec);

	const auto	now = ros::Time::now();
	_Tec.header.stamp = now;
	_Tec.transform	  = Tec;
	_Twm.header.stamp = now;
	_Twm.transform	  = Twm;

	std::ostringstream	sout;
	TU::evaluateAccuracy(sout, Tcm, Twe, Tec, Twm);
	res.success = true;
	res.message = sout.str();
	res.Tec	    = _Tec;
	res.Twm	    = _Twm;

	ROS_INFO_STREAM("compute_calibration(): " << res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	ROS_ERROR_STREAM("compute_calibration(): " << res.message);
    }

    return res.success;
}

bool
Calibrator::save_calibration(std_srvs::Trigger::Request&,
			     std_srvs::Trigger::Response& res)
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
				   + _nh.param<std::string>("camera_name",
							    "camera")
				   + ".yaml";

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

	res.success = true;
	res.message = "saved in " + calib_file;

	ROS_INFO_STREAM("save_calibration(): " << res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	ROS_ERROR_STREAM("compute_calibration(): " << res.message);
    }

    return res.success;
}

bool
Calibrator::reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    _Tcm.clear();
    _Twe.clear();

    ROS_INFO_STREAM("reset(): all samples cleared.");

    return true;
}

void
Calibrator::take_sample()
{
    _take_sample_srv.acceptNewGoal();

    ROS_INFO_STREAM("take_sample(): accepted new goal");
}

void
Calibrator::cancel()
{
    ROS_WARN_STREAM("cancel(): taking sample canceled.");
    _take_sample_srv.setPreempted();
}

}	// namespace aist_handeye_calibration
