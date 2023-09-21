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
#include <filesystem>
#include <errno.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <actionlib/server/simple_action_server.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <yaml-cpp/yaml.h>

#include <aist_aruco_ros/PointCorrespondenceArrayArray.h>
#include <aist_camera_calibration/TakeSampleAction.h>
#include <aist_camera_calibration/GetSampleList.h>
#include <aist_camera_calibration/ComputeCalibration.h>
#include <aist_utility/geometry_msgs.h>
#include "CameraCalibrator.h"
#include "TU/Camera++.h"
#include "TU/Quaternion.h"

namespace aist_camera_calibration
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> std::ostream&
operator <<(std::ostream& out, const TU::Point2<T>& p)
{
    return out << p[0] << ' ' << p[1];
}

template <class T> std::ostream&
operator <<(std::ostream& out, const TU::Point3<T>& p)
{
    return out << p[0] << ' ' << p[1] << ' ' << p[2];
}

template <class S, class T> std::ostream&
operator <<(std::ostream& out, const std::pair<S, T>& p)
{
    return out << '(' << p.first << " : " << p.second << ')';
}

template <class T> std::ostream&
operator <<(std::ostream& out, const std::vector<T>& v)
{
    out << '[';
    for (const auto& item : v)
	out << ' ' << item;
    out << ']';
    if (!std::is_arithmetic_v<T>)
    	out << std::endl;
    return out;
}

/************************************************************************
*  class Calibrator							*
************************************************************************/
class Calibrator
{
  public:
    using camera_info_t		= sensor_msgs::CameraInfo;
    using pose_t		= geometry_msgs::PoseStamped;

  private:
    using element_t		= double;
    using correses_msg_t	= aist_aruco_ros::PointCorrespondenceArray;
    using correses_set_msg_t	= aist_aruco_ros
				      ::PointCorrespondenceArrayArray;
    using correses_set_msg_cp	= aist_aruco_ros
				      ::PointCorrespondenceArrayArrayConstPtr;
    using action_server_t	= actionlib::SimpleActionServer<
				      TakeSampleAction>;

    using point2_t		= TU::Point2<element_t>;
    using point3_t		= TU::Point3<element_t>;
    using corres22_t		= std::pair<point2_t, point2_t>;
    using corres32_t		= std::pair<point3_t, point2_t>;
    template <class CORRES>
    using correses_t		= std::vector<CORRES>;
    template <class CORRES>
    using correses_set_t	= std::vector<correses_t<CORRES> >;
    template <class CORRES>
    using correses_sets_t	= std::vector<correses_set_t<CORRES> >;
    using camera_t		= TU::Camera<TU::IntrinsicWithDistortion<
						 TU::Intrinsic<element_t> > >;

    template <class SRC_PNT, class DST_PNT=point2_t>
    struct to_corres
    {
	std::pair<SRC_PNT, DST_PNT>
	operator ()(const aist_aruco_ros::PointCorrespondence& corres) const
	{
	    SRC_PNT	p;
	    p[0] = corres.source_point.x;
	    p[1] = corres.source_point.y;
	    DST_PNT	q;
	    q[0] = corres.image_point.x;
	    q[1] = corres.image_point.y;

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
	    p[0] = corres.source_point.x;
	    p[1] = corres.source_point.y;
	    p[2] = corres.source_point.z;
	    DST_PNT	q;
	    q[0] = corres.image_point.x;
	    q[1] = corres.image_point.y;

	    return {p, q};
	}
    };

    struct to_camera_name
    {
	const std::string&
	operator ()(const correses_msg_t& correses)		const	;
    };

    struct to_camera_info
    {
	to_camera_info(const ros::Time& stamp)	:_stamp(stamp)		{}

	camera_info_t
	operator ()(const camera_t& camera,
		    const correses_msg_t& correses)		const	;

      private:
	const ros::Time	_stamp;
    };

    struct to_pose
    {
	pose_t
	operator ()(const camera_t& camera,
		    const correses_msg_t& correses)		const	;
    };

  public:
		Calibrator(const ros::NodeHandle& nh,
			   const std::string& nodelet_name)		;
		~Calibrator()						;

  private:
    const std::string&
		getName()					const	;
    void	goal_cb()						;
    void	preempt_cb()						;
    void	corres_cb(const correses_set_msg_cp& correses_set_msg)	;
    bool	get_sample_list(GetSampleList::Request&,
				GetSampleList::Response& res)		;
    bool	compute_calibration(ComputeCalibration::Request&,
				    ComputeCalibration::Response& res)	;
    bool	save_calibration(std_srvs::Trigger::Request&,
				 std_srvs::Trigger::Response& res)	;
    bool	reset(std_srvs::Empty::Request&,
		      std_srvs::Empty::Response&)			;
    correses_sets_t<corres22_t>
		convert_correspondences_sets()			const	;
    correses_set_t<corres32_t>
		rearrange_correspondences_sets()		const	;
    bool	save_calibration(const correses_msg_t& correses,
				 const camera_info_t& intrinsic,
				 const pose_t& pose)		const	;

  private:
    const std::string			_nodelet_name;

    ros::NodeHandle			_nh;
    ros::Subscriber			_corres_sub;

    action_server_t			_take_sample_srv;
    const ros::ServiceServer		_get_sample_list_srv;
    const ros::ServiceServer		_compute_calibration_srv;
    const ros::ServiceServer		_save_calibration_srv;
    const ros::ServiceServer		_reset_srv;

    bool				_planar_reference;
    std::vector<correses_set_msg_t>	_correspondences_sets;
    std::vector<camera_info_t>		_intrinsics;
    std::vector<pose_t>			_poses;
};

Calibrator::Calibrator(const ros::NodeHandle& nh,
		       const std::string& nodelet_name)
    :_nodelet_name(nodelet_name),
     _nh(nh),
     _corres_sub(_nh.subscribe("/point_correspondences_set", 5,
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
     _correspondences_sets(),
     _intrinsics(),
     _poses()
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

const std::string&
Calibrator::getName() const
{
    return _nodelet_name;
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
Calibrator::corres_cb(const correses_set_msg_cp& correses_set_msg)
{
    if (!_take_sample_srv.isActive())
	return;

    const auto	ncameras = correses_set_msg->correspondences_set.size();

    if (_correspondences_sets.size() > 0 &&
	_correspondences_sets.front().correspondences_set.size() != ncameras)
    {
	_take_sample_srv.setAborted();

	NODELET_ERROR_STREAM('(' << getName()
			     << ") ABORTED taking samples because #cameras["
			     << ncameras
			     << "] in the new data is different from that["
			     << _correspondences_sets.front()
			        .correspondences_set.size()
			     << "] in the first data.");
	return;
    }

    _correspondences_sets.push_back(*correses_set_msg);

  // If using planar calibration object, set the reference frame of each
  // camera to the first camera frame.
    if (_planar_reference)
    {
	auto&		correspondences_set = _correspondences_sets.back();
	const auto	reference_frame = correspondences_set
					 .correspondences_set.front()
					 .header.frame_id;
	for (auto&& correspondences : correspondences_set.correspondences_set)
	    correspondences.reference_frame = reference_frame;
    }

    TakeSampleResult	result;
    result.correspondences_set = correses_set_msg->correspondences_set;
    _take_sample_srv.setSucceeded(result);

    NODELET_INFO_STREAM('(' << getName() << ") SUCCEEDED in taking samples");
}

bool
Calibrator::get_sample_list(GetSampleList::Request&,
			    GetSampleList::Response& res)
{
    res.correspondences_sets = _correspondences_sets;

    NODELET_INFO_STREAM('(' << getName() << ") Get "
			<< res.correspondences_sets.size()
			<< " samples obtained");
    return true;
}

bool
Calibrator::compute_calibration(ComputeCalibration::Request&,
				ComputeCalibration::Response& res)
{
    try
    {
	TU::CameraCalibrator<element_t>	calibrator;
	TU::Array<camera_t>		cameras;

	if (_planar_reference)
	{
	    const auto	correses_sets = convert_correspondences_sets();
	    std::cerr << correses_sets;

	    const auto	planes = calibrator.planeCalib(correses_sets.cbegin(),
	    					       correses_sets.cend(),
	    					       cameras, false, true);
	}
	else
	{
	    const auto	correses_set = rearrange_correspondences_sets();
	    std::cerr << correses_set;

	    cameras.resize(correses_set.size());

	    auto	camera = cameras.begin();
	    for (const auto& correses : correses_set)
	    	calibrator.volumeCalib(correses.cbegin(), correses.cend(),
	    			       *camera++, true);
	}

	res.camera_names.clear();
	res.intrinsics.clear();
	res.poses.clear();

	if (!_correspondences_sets.empty())
	{
	    const auto&	correspondences_set = _correspondences_sets.front()
					      .correspondences_set;

	    std::transform(correspondences_set.cbegin(),
			   correspondences_set.cend(),
			   std::back_inserter(res.camera_names),
			   to_camera_name());
	    std::transform(cameras.cbegin(), cameras.cend(),
			   correspondences_set.cbegin(),
			   std::back_inserter(res.intrinsics),
			   to_camera_info(ros::Time::now()));
	    std::transform(cameras.cbegin(), cameras.cend(),
			   correspondences_set.cbegin(),
			   std::back_inserter(res.poses), to_pose());
	}

	res.success = true;
	res.error   = calibrator.reprojectionError();

	_intrinsics = res.intrinsics;
	_poses	    = res.poses;

	NODELET_INFO_STREAM('(' << getName()
			    << ") Succesfully computed calibration with reprojection error: "
			    << res.error << "(pix)");
    }
    catch (const std::exception& err)
    {
	res.success = false;

	NODELET_ERROR_STREAM('(' << getName()
			     << ") Failed to compute calibration: "
			     << err.what());
    }

    return true;
}

bool
Calibrator::save_calibration(std_srvs::Trigger::Request&,
			     std_srvs::Trigger::Response& res)
{

    for (size_t i = 0; i < _intrinsics.size(); ++i)
	if (!save_calibration(_correspondences_sets.front()
			      .correspondences_set[i],
			      _intrinsics[i], _poses[i]))
	{
	    res.success = false;
	    res.message = "failed";

	    NODELET_ERROR_STREAM('(' << getName()
				 << ") Failed to save calibration");

	    return true;
	}

    res.success = true;
    res.message = "succeeded";

    NODELET_INFO_STREAM('(' << getName() << ") Succesfully saved calibration");

    return true;
}

bool
Calibrator::reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    _correspondences_sets.clear();
    _intrinsics.clear();
    _poses.clear();

    NODELET_INFO_STREAM('(' << getName()
			<< ") All samples and cameras cleared");

    return true;
}

Calibrator::correses_sets_t<Calibrator::corres22_t>
Calibrator::convert_correspondences_sets() const
{
    correses_sets_t<corres22_t>	correses_sets(_correspondences_sets.size());

  // For each marker position...
    auto	correses_set = correses_sets.begin();
    for (const auto& correspondences_set : _correspondences_sets)
    {
      // Allocate #cameras arrays for storing correspondences.
	correses_set->resize(correspondences_set.correspondences_set.size());

      // For each camera...
	auto	correses = correses_set->begin();
	for (const auto& correspondences :
		 correspondences_set.correspondences_set)
	{
	    std::transform(correspondences.correspondences.cbegin(),
			   correspondences.correspondences.cend(),
			   std::back_inserter(*correses++),
			   to_corres<corres22_t::first_type>());
	}

	++correses_set;
    }

    return correses_sets;
}

Calibrator::correses_set_t<Calibrator::corres32_t>
Calibrator::rearrange_correspondences_sets() const
{
    correses_set_t<corres32_t>	correses_set(_correspondences_sets.empty() ?
					     0 :
					     _correspondences_sets.front()
					     .correspondences_set.size());

  // For each marker position...
    for (const auto& correspondences_set : _correspondences_sets)
    {
      // For each camera...
	auto	correses = correses_set.begin();
	for (const auto& correspondences :
		 correspondences_set.correspondences_set)
	{
	    std::transform(correspondences.correspondences.cbegin(),
			   correspondences.correspondences.cend(),
			   std::back_inserter(*correses++),
			   to_corres<corres32_t::first_type>());
	}
    }

    return correses_set;
}

bool
Calibrator::save_calibration(const correses_msg_t& correses,
			     const camera_info_t& intrinsic,
			     const pose_t& pose) const
{
    YAML::Emitter	emitter;
    emitter << YAML::BeginMap;

    emitter << YAML::Key   << "image_width"
	    << YAML::Value << correses.width
	    << YAML::Key   << "image_height"
	    << YAML::Value << correses.height
	    << YAML::Key   << "camera_name"
	    << YAML::Value << correses.camera_name;

    emitter << YAML::Key   << "camera_matrix"
	    << YAML::Value
	    << YAML::BeginMap
	    << YAML::Key   << "rows"
	    << YAML::Value << 3
	    << YAML::Key   << "cols"
	    << YAML::Value << 3
	    << YAML::Key   << "data"
	    << YAML::Value
	    << YAML::Flow
	    << YAML::BeginSeq;
    for (const auto& val : intrinsic.K)
	emitter << val;
    emitter << YAML::EndSeq
	    << YAML::EndMap;

    emitter << YAML::Key   << "distortion_model"
	    << YAML::Value << intrinsic.distortion_model;
    emitter << YAML::Key   << "distortion_coefficients"
	    << YAML::Value
	    << YAML::BeginMap
	    << YAML::Key   << "rows"
	    << YAML::Value << 1
	    << YAML::Key   << "cols"
	    << YAML::Value << intrinsic.D.size()
	    << YAML::Key   << "data"
	    << YAML::Value
	    << YAML::Flow
	    << YAML::BeginSeq;
    for (const auto& val : intrinsic.D)
	emitter << val;
    emitter << YAML::EndSeq
	    << YAML::EndMap;

    emitter << YAML::Key   << "rectification_matrix"
	    << YAML::Value
	    << YAML::BeginMap
	    << YAML::Key   << "rows"
	    << YAML::Value << 3
	    << YAML::Key   << "cols"
	    << YAML::Value << 3
	    << YAML::Key   << "data"
	    << YAML::Value
	    << YAML::Flow
	    << YAML::BeginSeq;
    for (const auto& val : intrinsic.R)
	emitter << val;
    emitter << YAML::EndSeq
	    << YAML::EndMap;

    emitter << YAML::Key   << "projection_matrix"
	    << YAML::Value
	    << YAML::BeginMap
	    << YAML::Key   << "rows"
	    << YAML::Value << 3
	    << YAML::Key   << "cols"
	    << YAML::Value << 4
	    << YAML::Key   << "data"
	    << YAML::Value
	    << YAML::Flow
	    << YAML::BeginSeq;
    for (const auto& val : intrinsic.P)
	emitter << val;
    emitter << YAML::EndSeq
	    << YAML::EndMap;

    emitter << YAML::Key   << "camera_pose"
	    << YAML::Value
	    << YAML::BeginMap
	    << YAML::Key   << "parent"
	    << YAML::Value << pose.header.frame_id
	    << YAML::Key   << "child"
	    << YAML::Value << correses.header.frame_id
	    << YAML::Key   << "transform"
	    << YAML::Value
	    << YAML::Flow
	    << YAML::BeginMap
	    << YAML::Key   << "x"
	    << YAML::Value << pose.pose.position.x
	    << YAML::Key   << "y"
	    << YAML::Value << pose.pose.position.y
	    << YAML::Key   << "z"
	    << YAML::Value << pose.pose.position.z
	    << YAML::Key   << "qx"
	    << YAML::Value << pose.pose.orientation.x
	    << YAML::Key   << "qy"
	    << YAML::Value << pose.pose.orientation.y
	    << YAML::Key   << "qz"
	    << YAML::Value << pose.pose.orientation.z
	    << YAML::Key   << "qw"
	    << YAML::Value << pose.pose.orientation.w
	    << YAML::EndMap;

    const auto	tval = time(nullptr);
    const auto	tstr = ctime(&tval);
    tstr[strlen(tstr)-1] = '\0';
    emitter << YAML::Key   << "calibration_date"
	    << YAML::Value << tstr;

    emitter << YAML::EndMap;

    try
    {
	namespace fs = std::filesystem;

      // Check existence of calibration directory and create if not present.
	const fs::path	calib_dir(std::string(getenv("HOME"))
				  + "/.ros/camera_info");
	if (!fs::exists(calib_dir))
	    fs::create_directories(calib_dir);
	else if (!fs::is_directory(calib_dir))
	    throw std::runtime_error('\"' + calib_dir.string()
				     + "\" exists but not a directory");

	const auto	calib_file = calib_dir
				   / fs::path(correses.camera_name + ".yaml");
	std::ofstream	out(calib_file);
	if (!out)
	    throw std::runtime_error("cannot open " + calib_file.string()
				     + ": " + strerror(errno));

      // Save calitration results.
	out << emitter.c_str() << std::endl;

	NODELET_INFO_STREAM('(' << getName() << ") SaveCalibration: saved in "
			    << calib_file.string());
    }
    catch (const std::exception& err)
    {
	NODELET_ERROR_STREAM('(' << getName() << ") SaveCalibration: "
			     << err.what());
	return false;
    }

    return true;
}

/************************************************************************
*  struct Calibrator::to_camera_name					*
************************************************************************/
const std::string&
Calibrator::to_camera_name::operator()(const correses_msg_t& correses) const
{
    return correses.camera_name;
}

/************************************************************************
*  struct Calibrator::to_camera_info					*
************************************************************************/
Calibrator::camera_info_t
Calibrator::to_camera_info::operator()(const camera_t& camera,
				       const correses_msg_t& correses) const
{
    camera_info_t	camera_info;

    camera_info.header = correses.header;
    camera_info.header.stamp = _stamp;

  // Set distortion parameters.
    camera_info.distortion_model = "plumb_bob";
    camera_info.D.resize(5);
    camera_info.D[0] = camera.d1();
    camera_info.D[1] = camera.d2();
    camera_info.D[2] = 0.0;
    camera_info.D[3] = 0.0;
    camera_info.D[4] = 0.0;

  // Set intrinsic parameters.
    const auto	K = camera.K();
    camera_info.K[0] = K[0][0];
    camera_info.K[1] = 0.0;
    camera_info.K[2] = K[0][2];
    camera_info.K[3] = 0.0;
    camera_info.K[4] = K[1][1];
    camera_info.K[5] = K[1][2];
    camera_info.K[6] = 0.0;
    camera_info.K[7] = 0.0;
    camera_info.K[8] = 1.0;

  // Set rotation matrix.
    camera_info.R[0] = 1.0;
    camera_info.R[1] = 0.0;
    camera_info.R[2] = 0.0;
    camera_info.R[3] = 0.0;
    camera_info.R[4] = 1.0;
    camera_info.R[5] = 0.0;
    camera_info.R[6] = 0.0;
    camera_info.R[7] = 0.0;
    camera_info.R[8] = 1.0;

  // Set projection matrix.
    camera_info.P[ 0] = camera_info.K[0];
    camera_info.P[ 1] = camera_info.K[1];
    camera_info.P[ 2] = camera_info.K[2];
    camera_info.P[ 3] = 0.0;
    camera_info.P[ 4] = camera_info.K[3];
    camera_info.P[ 5] = camera_info.K[4];
    camera_info.P[ 6] = camera_info.K[5];
    camera_info.P[ 7] = 0.0;
    camera_info.P[ 8] = camera_info.K[6];
    camera_info.P[ 9] = camera_info.K[7];
    camera_info.P[10] = camera_info.K[8];
    camera_info.P[11] = 0.0;

  // Set binning.
    camera_info.binning_x = 0;
    camera_info.binning_y = 0;

    return camera_info;
}

/************************************************************************
*  struct Calibrator::to_pose						*
************************************************************************/
Calibrator::pose_t
Calibrator::to_pose::operator()(const camera_t& camera,
				const correses_msg_t& correses) const
{
    pose_t	pose;
    pose.header.frame_id = correses.reference_frame;
    pose.pose.position.x = camera.t()[0];
    pose.pose.position.y = camera.t()[1];
    pose.pose.position.z = camera.t()[2];

    const TU::Matrix<element_t, 3, 3>	R = transpose(camera.Rt());
    const TU::Quaternion<element_t>	q(R);
    pose.pose.orientation.x = q.vector()[0];
    pose.pose.orientation.y = q.vector()[1];
    pose.pose.orientation.z = q.vector()[2];
    pose.pose.orientation.w = q.scalar();

    return pose;
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
