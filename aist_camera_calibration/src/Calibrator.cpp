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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <yaml-cpp/yaml.h>

#include <aist_camera_calibration/GetSampleList.h>
#include <aist_camera_calibration/ComputeCalibration.h>
#include <aist_camera_calibration/TakeSampleAction.h>
#include <aist_utility/geometry_msgs.h>
#include "CameraCalibrator.h"

namespace aist_camera_calibration
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
class Calibrator
{
  private:
    using element_t	  = double;
    using corres_msg_cp	  = aist_aruco_ros
				::PointCorrespondenceArrayArrayConstPtr;
    using action_server_t = actionlib::SimpleActionServer<TakeSampleAction>;
    using point2_t	  = TU::Point2<element_t>;
    using point3_t	  = TU::Point3<element_t>;
    using corres22_t	  = std::pair<point2_t, point2_t>;
    using corres32_t	  = std::pair<point3_t, point2_t>;

    template <class CORRES>
    using correses_t	  = std::vector<CORRES>;
    template <class CORRES>
    using correses_data_t = std::vector<correses_t<CORRES> >;
    template <class CORRES>
    using correses_list_t = std::vector<correses_data_t<CORRES> >;

    template <class SRC_PNT, class DST_PNT=point2_t>
    struct to_corres
    {
	std::pair<SRC_PNT, DST_PNT>
	operator ()(const aist_aruco_ros::PointCorrespondence& corres) const
	{
	    SRC_PNT	p;
	    p[0] = corres.point.x;
	    p[1] = corres.point.y;
	    DST_PNT	q;
	    q[0] = corres.image_point.u;
	    q[1] = corres.image_point.v;
	    
	    return {p, q};
	}
    };
    template <class DST_PNT>
    struct to_corres<point3_t, DST_PNT>
    {
	std::pair<point3_t, DST_PNT>
	operator ()(const aist_aruco_ros::PointCorrespondence& corres) const
	{
	    point3_t	p;
	    p[0] = corres.point.x;
	    p[1] = corres.point.y;
	    p[2] = corres.point.z;
	    DST_PNT	q;
	    q[0] = corres.image_point.u;
	    q[1] = corres.image_point.v;
	    
	    return {p, q};
	}
    };
    
  public:
		Calibrator(const ros::NodeHandle& nh,
			   const std::string& nodelet_name)		;
		~Calibrator()						;

    void	run()							;

  private:
    void	goal_cb()						;
    void	preempt_cb()						;
    void	corres_cb(const corres_msg_cp& corres_msg)		;
    template <class CORRES>
    static bool	add_correspondences_data(const corres_msg_cp& corres_msg,
					 correses_list_t<CORRES>& clist);
    bool	get_sample_list(GetSampleList::Request&,
				GetSampleList::Response& res)		;
    bool	compute_calibration(ComputeCalibration::Request&,
				    ComputeCalibration::Response& res)	;
    bool	save_calibration(std_srvs::Trigger::Request&,
				 std_srvs::Trigger::Response& res)	;
    bool	reset(std_srvs::Empty::Request&,
		      std_srvs::Empty::Response&)			;
    const std::string&
		getName()					const	;

  private:
    const std::string		_nodelet_name;
    
    ros::NodeHandle		_nh;
    ros::Subscriber		_corres_sub;

    action_server_t		_take_sample_srv;
    const ros::ServiceServer	_get_sample_list_srv;
    const ros::ServiceServer	_compute_calibration_srv;
    const ros::ServiceServer	_save_calibration_srv;
    const ros::ServiceServer	_reset_srv;

    bool			_planar_reference;
    correses_list_t<corres22_t>	_correses_list22;
    correses_list_t<corres22_t>	_correses_list32;
};

Calibrator::Calibrator(const ros::NodeHandle& nh,
		       const std::string& nodelet_name)
    :_nodelet_name(nodelet_name),
     _nh(nh),
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
     _reset_srv(_nh.advertiseService("reset", &Calibrator::reset, this)),
     _planar_reference(_nh.param<bool>("planar_reference", true)),
     _correses_list22(),
     _correses_list32()
{
    _take_sample_srv.registerGoalCallback(boost::bind(&Calibrator::goal_cb,
						      this));
    _take_sample_srv.registerPreemptCallback(boost::bind(
						 &Calibrator::preempt_cb,
						 this));
    _take_sample_srv.start();

    NODELET_INFO_STREAM('(' << getName() << ") Calibrator initialized");
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
Calibrator::goal_cb()
{
    _take_sample_srv.acceptNewGoal();
    NODELET_INFO_STREAM('(' << getName()
			<< ") ACCEPTED new goal to take samples");
}

void
Calibrator::preempt_cb()
{
    _take_sample_srv.setPreempted();
    NODELET_WARN_STREAM('(' << getName()
			<< ") CANCELED taking samples");
}

void
Calibrator::corres_cb(const corres_msg_cp& corres_msg)
{
    if (!_take_sample_srv.isActive())
	return;

    if (_planar_reference)
    {
	if (!add_correspondences_data(corres_msg, _correses_list22))
	{
	    _take_sample_srv.setAborted();
	    
	    NODELET_ERROR_STREAM('(' << getName()
				 << ") ABORTED taking samples because #cameras["
				 << corres_msg->correspondences_list.size()
				 << "] in the new data is different from that["
				 << _correses_list22.front().size()
				 << "] in the previous data.");
	}
    }
    else
    {
	if (!add_correspondences_data(corres_msg, _correses_list32))
	{
	    _take_sample_srv.setAborted();
	    
	    NODELET_ERROR_STREAM('(' << getName()
				 << ") ABORTED taking samples because #cameras["
				 << corres_msg->correspondences_list.size()
				 << "] in the new data is different from that["
				 << _correses_list32.front().size()
				 << "] in the previous data.");
	}
    }
    
    TakeSampleResult	result;
    result.correspondences_list = *corres_msg;
    _take_sample_srv.setSucceeded(result);

    NODELET_INFO_STREAM('(' << getName() << ") SUCCEEDED in taking samples");
}

template <class CORRES> bool
Calibrator::add_correspondences_data(const corres_msg_cp& corres_msg,
				     correses_list_t<CORRES>& clist)
{
    const auto	ncameras = corres_msg->correspondences_list.size();
    
    if (clist.size() > 0 && clist.front().size() != ncameras)
	return false;

    correses_data_t<CORRES>	correses_data(ncameras);
    auto			correses = correses_data.begin();
    for (const auto& correses_msg : corres_msg->correspondences_list)
    {
	correses->resize(correses_msg.correspondences.size());
	std::transform(correses_msg.correspondences.cbegin(),
		       correses_msg.correspondences.cend(),
		       correses->begin(),
		       to_corres<typename CORRES::first_type>());
	++correses;
    }
    clist.push_back(correses_data);
    
    return true;
}


bool
Calibrator::get_sample_list(GetSampleList::Request&,
			    GetSampleList::Response& res)
{
    res.success = true;
    NODELET_INFO_STREAM('(' << getName() << ") GetSampleList: "
			<< res.message);

    return true;
}

bool
Calibrator::compute_calibration(ComputeCalibration::Request&,
				ComputeCalibration::Response& res)
{
    try
    {
	res.success = true;
	NODELET_INFO_STREAM('(' << getName() << ") ComputeCalibration: "
			    << res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	NODELET_ERROR_STREAM('(' << getName() << ") ComputeCalibration: "
			     << res.message);
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

	NODELET_INFO_STREAM('(' << getName() << ") SaveCalibration: "
			    << res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	NODELET_ERROR_STREAM('(' << getName() << ") SaveCalibration: "
			    << res.message);
    }

    return res.success;
}

bool
Calibrator::reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    _correses_list22.clear();
    _correses_list32.clear();
    
    NODELET_INFO_STREAM('(' << getName() << ") All samples cleared");

    return true;
}

const std::string&
Calibrator::getName() const
{
    return _nodelet_name;
}
    
/************************************************************************
*  class CalibratorNodelet						*
************************************************************************/
class CalibratorNodelet : public nodelet::Nodelet
{
  public:
			CalibratorNodelet()				{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<Calibrator>	_node;
};

void
CalibratorNodelet::onInit()
{
    NODELET_INFO("aist_camera_calibrations::CalibratorNodelet::onInit()");
    _node.reset(new Calibrator(getPrivateNodeHandle(), getName()));
}

}	// namespace aist_camera_calibration

PLUGINLIB_EXPORT_CLASS(aist_camera_calibration::CalibratorNodelet,
		       nodelet::Nodelet);
