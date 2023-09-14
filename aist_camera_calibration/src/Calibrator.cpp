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
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <actionlib/server/simple_action_server.h>
#include <aist_camera_calibration/GetSampleList.h>
#include <aist_camera_calibration/ComputeCalibration.h>
#include <aist_camera_calibration/TakeSampleAction.h>
#include <yaml-cpp/yaml.h>
#include <aist_utility/geometry_msgs.h>

namespace aist_camera_calibration
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
class Calibrator
{
  private:
    using element_t	  = double;
    using corres_msg_t	  = aist_aruco_ros::PointCorrespondenceArrayArray;
    using corres_msg_cp	  = aist_aruco_ros
				::PointCorrespondenceArrayArrayConstPtr;
    using action_server_t = actionlib::SimpleActionServer<TakeSampleAction>;

  public:
		Calibrator(const ros::NodeHandle& nh)			;
		~Calibrator()						;

    void	run()							;

  private:
    void	corres_cb(const corres_msg_cp& corres)			;
    bool	get_sample_list(GetSampleList::Request&,
				GetSampleList::Response& res)		;
    bool	compute_calibration(ComputeCalibration::Request&,
				    ComputeCalibration::Response& res)	;
    bool	save_calibration(std_srvs::Trigger::Request&,
				 std_srvs::Trigger::Response& res)	;
    bool	reset(std_srvs::Empty::Request&,
		      std_srvs::Empty::Response&)			;
    void	goal_cb()						;
    void	preempt_cb()						;

  private:
    ros::NodeHandle		_nh;
    ros::Subscriber		_corres_sub;

    action_server_t		_take_sample_srv;
    const ros::ServiceServer	_get_sample_list_srv;
    const ros::ServiceServer	_compute_calibration_srv;
    const ros::ServiceServer	_save_calibration_srv;
    const ros::ServiceServer	_reset_srv;
};

Calibrator::Calibrator(const ros::NodeHandle& nh)
    :_nh(nh),
     _corres_sub(_nh.subscribe("/point_correspondences_list", 5,
			       &Calibrator::corres_cb, this)),
     _take_sample_srv(_nh, "take_sample", false),
     _get_sample_list_srv(
	 _nh.advertiseService("get_sample_list",
			      &Calibrator::get_sample_list, this)),
     _compute_calibration_srv(
	 _nh.advertiseService("compute_calibration",
			      &Calibrator::compute_calibration, this)),
     _save_calibration_srv(
	 _nh.advertiseService("save_calibration",
			      &Calibrator::save_calibration, this)),
     _reset_srv(_nh.advertiseService("reset", &Calibrator::reset, this))
{
    _take_sample_srv.registerGoalCallback(boost::bind(&Calibrator::goal_cb,
						      this));
    _take_sample_srv.registerPreemptCallback(boost::bind(
						 &Calibrator::preempt_cb,
						 this));
    _take_sample_srv.start();

    ROS_INFO_STREAM("Calibrator initialized");
}

Calibrator::~Calibrator()
{
}

void
Calibrator::run()
{
    ros::spin();
}

void
Calibrator::corres_cb(const corres_msg_cp& corres)
{
    if (!_take_sample_srv.isActive())
	return;

    try
    {
      // Convert marker pose to camera <= object transform.
	TakeSampleResult	result;
	result.correspondences_list = *corres;

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
    ROS_INFO_STREAM("get_sample_list(): " << res.message);

    return true;
}

bool
Calibrator::compute_calibration(ComputeCalibration::Request&,
				ComputeCalibration::Response& res)
{
    try
    {
	ROS_INFO_STREAM("compute_calibration()");

	res.success = true;

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
	YAML::Emitter	emitter;
	emitter << YAML::BeginMap;

	const auto	tval = time(nullptr);
	const auto	tstr = ctime(&tval);
	tstr[strlen(tstr)-1] = '\0';
	emitter << YAML::Key   << "calibration_date"
		<< YAML::Value << tstr;

	emitter << YAML::EndMap;

      // Read calibration file name from parameter server.
	std::string	calib_file;
	_nh.param<std::string>("calib_file", calib_file,
			       getenv("HOME") + std::string("/.ros/aist_camera_calibration/calib.yaml"));

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
    ROS_INFO_STREAM("All samples cleared");

    return true;
}

void
Calibrator::goal_cb()
{
    _take_sample_srv.acceptNewGoal();
    ROS_INFO_STREAM("ACCEPTED new goal to take samples");
}

void
Calibrator::preempt_cb()
{
    _take_sample_srv.setPreempted();
    ROS_WARN_STREAM("CANCELED taking samples");
}

}	// namespace aist_camera_calibration

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "calibrator");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	ros::NodeHandle				nh("~");
	aist_camera_calibration::Calibrator	calibrator(nh);
	calibrator.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
