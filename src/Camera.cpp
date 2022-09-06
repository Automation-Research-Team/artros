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
 *  \file	Camera.cpp
 */
#if PHO_SOFTWARE_VERSION_MAJOR >= 1
#  if PHO_SOFTWARE_VERSION_MINOR >= 4
#    define HAVE_MOTIONCAM
#    if PHO_SOFTWARE_VERSION_MINOR >= 5
#      define HAVE_INTERREFLECTIONS_FILTERING
#      define HAVE_HARDWARE_TRIGGER
#      define HAVE_MOTIONCAM_EXPOSURE
#      if PHO_SOFTWARE_VERSION_MINOR >= 7
#        define HAVE_HARDWARE_TRIGGER_SIGNAL
#        define HAVE_INTERREFLECTION_FILTER_STRENGTH
#	 if PHO_SOFTWARE_VERSION_MINOR >= 8
#          define HAVE_LED_POWER
#          define HAVE_LED_SHUTTER_MULTIPLIER
#	   if PHO_SOFTWARE_VERSION_MINOR >= 9
#            define HAVE_COLOR
#          endif
#        endif
#      endif
#    endif
#  endif
#endif

#include "Camera.h"
#include <ros/package.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <nodelet/nodelet.h>
#include <x86intrin.h>

namespace aist_phoxi_camera
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> static std::ostream&
operator <<(std::ostream& out, const pho::api::PhoXiFeature<T>& feature)
{
    return out << feature.GetValue();
}

static size_t
npoints_valid(const pho::api::PointCloud32f& cloud)
{
    size_t	n = 0;
    for (int v = 0; v < cloud.Size.Height; ++v)
    {
	auto	p = cloud[v];

	for (const auto q = p +  cloud.Size.Width; p != q; ++p)
	    if (float(p->z) != 0.0f)
		++n;
    }

    return n;
}

static void
scale_copy(const float* in, const float* ie, uint8_t* out, float scale)
{
#if defined(AVX2)
    const auto	s = _mm256_set1_ps(scale);

    for (; in != ie; out += 32)
    {
	auto i_lo = _mm256_cvtps_epi32(_mm256_mul_ps(_mm256_loadu_ps(in), s));
	in += 8;
	auto i_hi = _mm256_cvtps_epi32(_mm256_mul_ps(_mm256_loadu_ps(in), s));
	in += 8;
	const auto s_lo = _mm256_packs_epi32(
				_mm256_permute2f128_si256(i_lo, i_hi, 0x20),
				_mm256_permute2f128_si256(i_lo, i_hi, 0x31));

	i_lo = _mm256_cvtps_epi32(_mm256_mul_ps(_mm256_loadu_ps(in), s));
	in += 8;
	i_hi = _mm256_cvtps_epi32(_mm256_mul_ps(_mm256_loadu_ps(in), s));
	in += 8;
	const auto s_hi = _mm256_packs_epi32(
				_mm256_permute2f128_si256(i_lo, i_hi, 0x20),
				_mm256_permute2f128_si256(i_lo, i_hi, 0x31));

	_mm256_storeu_si256(reinterpret_cast<__m256i*>(out),
			    _mm256_packus_epi16(
				_mm256_permute2f128_si256(s_lo, s_hi, 0x20),
				_mm256_permute2f128_si256(s_lo, s_hi, 0x31)));
    }
#else
    std::transform(in, ie, out,
		   [scale](const auto& x)->uint8_t
		   { auto y = scale * x; return y > 255 ? 255 : y; });
#endif
}

static void
scale_copy(const float* in, const float* ie, float* out, float scale)
{
#if defined(AVX2)
    const auto	s = _mm256_set1_ps(scale);

    for (; in != ie; in += 8, out += 8)
	_mm256_storeu_ps(out, _mm256_mul_ps(_mm256_loadu_ps(in), s));
#else
    std::transform(in, ie, out,
		   [scale](const auto& x)->float
		   { return scale * x; });
#endif
}

/************************************************************************
*  class Camera								*
************************************************************************/
Camera::Camera(const ros::NodeHandle& nh, const std::string& nodelet_name)
    :
#if defined(PROFILE)
     profiler_t(6),
#endif
     _nh(nh),
     _nodelet_name(nodelet_name),
     _factory(),
     _device(nullptr),
     _frame(nullptr),
     _frame_id(_nh.param<std::string>("frame", "sensor")),
     _rate(_nh.param<double>("rate", 10.0)),
     _denseCloud(false),
     _intensityScale(0.5),
     _cloud(new cloud_t),
     _normal_map(new image_t),
     _depth_map(new image_t),
     _confidence_map(new image_t),
     _texture(new image_t),
     _cinfo(new cinfo_t),
     _ddr(_nh),
     _trigger_frame_server(_nh.advertiseService("trigger_frame",
						&Camera::trigger_frame,	this)),
     _save_frame_server(_nh.advertiseService("save_frame",
					     &Camera::save_frame, this)),
     _save_settings_server(_nh.advertiseService("save_settings",
						&Camera::save_settings, this)),
     _restore_settings_server(_nh.advertiseService("restore_settings",
						   &Camera::restore_settings,
						   this)),
     _it(_nh),
     _cloud_publisher(	       _nh.advertise<cloud_t>("pointcloud",	1)),
     _normal_map_publisher(    _it.advertise(	      "normal_map",	1)),
     _depth_map_publisher(     _it.advertise(	      "depth_map",	1)),
     _confidence_map_publisher(_it.advertise(	      "confidence_map", 1)),
     _texture_publisher(       _it.advertise(	      "texture",	1)),
     _camera_info_publisher(   _nh.advertise<cinfo_t>("camera_info",	1))
{
    using namespace	pho::api;

  // Search for a device with specified ID.
    auto	id = _nh.param<std::string>("id",
					    "InstalledExamples-basic-example");
    for (size_t pos; (pos = id.find('\"')) != std::string::npos; )
	id.erase(pos, 1);

    if (!_factory.isPhoXiControlRunning())
    {
	NODELET_ERROR_STREAM("PhoXiControll is not running.");
	throw std::runtime_error("");
    }

    for (const auto& devinfo : _factory.GetDeviceList())
	if (devinfo.HWIdentification == id)
	{
	    _device = _factory.Create(devinfo.GetTypeHWIdentification());
	    break;
	}
    if (!_device)
    {
	NODELET_ERROR_STREAM("Failed to find camera[" << id << "].");
	throw std::runtime_error("");
    }

  // Connect to the device.
    if (!_device->Connect())
    {
	NODELET_ERROR_STREAM("Failed to open camera[" << id << "].");
	throw std::runtime_error("");
    }

  // Stop acquisition.
    _device->StopAcquisition();

    NODELET_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") Initializing configuration.");

  // Setup ddynamic_reconfigure services.
    switch (PhoXiDeviceType::Value(_device->GetType()))
    {
      case PhoXiDeviceType::PhoXiScanner:
	setup_ddr_phoxi();
	break;
#if defined(HAVE_MOTIONCAM)
      case PhoXiDeviceType::MotionCam3D:
	setup_ddr_motioncam();
	break;
#endif
      default:
	NODELET_ERROR_STREAM('('
			     << _device->HardwareIdentification.GetValue()
			     << ") Unknown device type["
			     << std::string(_device->GetType()) << ']');
	throw std::runtime_error("");
    }

    setup_ddr_common();

    _ddr.publishServicesTopics();

  // Start acquisition.
    set_camera_info();
    _device->ClearBuffer();
    _device->StartAcquisition();

    NODELET_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") aist_phoxi_camera is active.");
}

Camera::~Camera()
{
    if (_device && _device->isConnected())
    {
	_device->StopAcquisition();
	_device->Disconnect();
    }
}

void
Camera::tick()
{
    using namespace	pho::api;

    if (_device->TriggerMode == PhoXiTriggerMode::Freerun &&
	_device->isAcquiring())
    {
	if ((_frame = _device->GetFrame(PhoXiTimeout::ZeroTimeout)) &&
	    _frame->Successful)
	    publish_frame();
    }
}

double
Camera::rate() const
{
    return _rate;
}

void
Camera::setup_ddr_phoxi()
{
    using namespace	pho::api;

  // 1. CapturingMode
    const auto	modes = _device->SupportedCapturingModes.GetValue();
    if (modes.size() > 1)
    {
	std::map<std::string, int>	enum_resolution;
	int				idx = 0;
	for (int i = 0; i < modes.size(); ++i)
	{
	    const auto&	resolution = modes[i].Resolution;
	    enum_resolution.emplace(std::to_string(resolution.Width) + 'x' +
				    std::to_string(resolution.Height), i);
	    if (modes[i] == _device->CapturingMode)
		idx = i;
	}
	_ddr.registerEnumVariable<int>("resolution", idx,
				       boost::bind(&Camera::set_resolution,
						   this, _1),
				       "Image resolution", enum_resolution);
    }

  // 2. CapturingSettings
  // 2.1 ShutterMultiplier
    _ddr.registerVariable<int>(
	    "shutter_multiplier",
	    _device->CapturingSettings->ShutterMultiplier,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::ShutterMultiplier, _1, false,
			"ShutterMultiplier"),
	    "The number of repeats of indivisual pattern",
	    1, 20, "capturing_settings");

  // 2.2 ScanMultiplier
    _ddr.registerVariable<int>(
	    "scan_multiplier",
	    _device->CapturingSettings->ScanMultiplier,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::ScanMultiplier, _1, false,
			"ScanMultiplier"),
	    "The number of scans taken and merged to sigle output",
	    1, 20, "capturing_settings");

  // 2.3 CameraOnlyMode
    _ddr.registerVariable<bool>(
	    "camera_only_mode",
	    _device->CapturingSettings->AmbientLightSuppression,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, bool>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::CameraOnlyMode, _1,
			false, "CameraOnlyMode"),
	    "Use the scanner internal camera to capture only 2D images.",
	    false, true, "capturing_settings");

  // 2.4 AmbientLightSuppression
    _ddr.registerVariable<bool>(
	    "ambient_light_suppression",
	    _device->CapturingSettings->AmbientLightSuppression,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, bool>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::AmbientLightSuppression, _1,
			false, "AmbientLightSuppression"),
	    "Enables the mode that suppress ambient illumination.",
	    false, true, "capturing_settings");

  // 2.5 MaximumFPS
    _ddr.registerVariable<double>(
	    "maximum_fps",
	    _device->CapturingSettings->MaximumFPS,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, double>,
			this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::MaximumFPS, _1, false,
			"MaximumFPS"),
	    "Maximum fps in freerun mode",
	    0.0, 30.0, "capturing_settings");

  // 2.6 SinglePatternExposure
    const auto	exposures = _device->SupportedSinglePatternExposures.GetValue();
    std::map<std::string, double>	enum_single_pattern_exposure;
    for (const auto& exposure: exposures)
	enum_single_pattern_exposure.emplace(std::to_string(exposure),
					     exposure);
    _ddr.registerEnumVariable<double>(
	    "single_pattern_exposure",
	    _device->CapturingSettings->SinglePatternExposure,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, double>,
			this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::SinglePatternExposure, _1,
			false, "SinglePatternExposure"),
	    "Exposure time for a single patter in miliseconds",
	    enum_single_pattern_exposure, "", "capturing_settings");

  // 2.7 CodingStrategy
    if (_device->CapturingSettings->CodingStrategy !=
	PhoXiCodingStrategy::NoValue)
    {
	const std::map<std::string, int>
	    enum_coding_strategy = {
		{"Normal",	     PhoXiCodingStrategy::Normal},
		{"Interreflections", PhoXiCodingStrategy::Interreflections}};
	_ddr.registerEnumVariable<int>(
    	    "coding_strategy",
    	    _device->CapturingSettings->CodingStrategy,
    	    boost::bind(&Camera::set_field<PhoXiCapturingSettings,
					   PhoXiCodingStrategy>,
    			this,
    			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::CodingStrategy, _1, false,
			"CodingStrategy"),
    	    "Coding strategy", enum_coding_strategy, "", "capturing_settings");
    }

  // 2.8 CodingQuality
    if (_device->CapturingSettings->CodingQuality !=
	PhoXiCodingQuality::NoValue)
    {
	const std::map<std::string, int>
	    enum_coding_quality = {{"Fast",  PhoXiCodingQuality::Fast},
				   {"High",  PhoXiCodingQuality::High},
				   {"Ultra", PhoXiCodingQuality::Ultra}};
	_ddr.registerEnumVariable<int>(
    	    "coding_quality",
    	    _device->CapturingSettings->CodingQuality,
    	    boost::bind(&Camera::set_field<PhoXiCapturingSettings,
					   PhoXiCodingQuality>,
    			this,
    			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::CodingQuality, _1, false,
			"CodingQuality"),
    	    "Coding quality", enum_coding_quality, "", "capturing_settings");
    }

  // 2.9 TextureSource
    if (_device->CapturingSettings->TextureSource !=
    	PhoXiTextureSource::NoValue)
    {
	const std::map<std::string, int>
	    enum_texture_source = {{"Computed", PhoXiTextureSource::Computed},
				   {"LED",	PhoXiTextureSource::LED},
				   {"Laser",    PhoXiTextureSource::Laser},
				   {"Focus",    PhoXiTextureSource::Focus},
				   {"Color",    PhoXiTextureSource::Color}};
	_ddr.registerEnumVariable<int>(
    	    "texture_source",
    	    _device->CapturingSettings->TextureSource,
    	    boost::bind(&Camera::set_field<PhoXiCapturingSettings,
					   PhoXiTextureSource>,
    			this,
    			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::TextureSource, _1, false,
			"TextureSource"),
    	    "Source used for texture image",
	    enum_texture_source, "", "capturing_settings");
    }

  // 2.10 LaserPower
    _ddr.registerVariable<int>(
	    "laser_power",
	    _device->CapturingSettings->LaserPower,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::LaserPower, _1, false,
			"LaserPower"),
	    "Laser power", 1, 4095, "capturing_settings");

  // 2.11 LEDPower
    _ddr.registerVariable<int>(
	    "led_power",
	    _device->CapturingSettings->LEDPower,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::LEDPower, _1, false,
			"LEDPower"),
	    "LED power", 0, 4095, "capturing_settings");

#if defined(HAVE_LED_SHUTTER_MULTIPLIER)
  // 2.12 LEDShutterMultiplier
    _ddr.registerVariable<int>(
	    "led_shutter_multiplier",
	    _device->CapturingSettings->LEDShutterMultiplier,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::LEDShutterMultiplier, _1,
			false, "LEDShutterMultiplier"),
	    "LED shutter multiplier", 1, 20, "capturing_settings");
#endif
}

#if defined(HAVE_MOTIONCAM)
void
Camera::setup_ddr_motioncam()
{
    using namespace	pho::api;

  // 1. General settings
  // 1.1 operation mode
    if (_device->MotionCam->OperationMode != PhoXiOperationMode::NoValue)
    {
	const std::map<std::string, int>
	    enum_operation_mode = {{"Camera",  PhoXiOperationMode::Camera},
				   {"Scanner", PhoXiOperationMode::Scanner},
				   {"Mode2D",  PhoXiOperationMode::Mode2D}};
	_ddr.registerEnumVariable<int>(
    	    "operation_mode",
    	    _device->MotionCam->OperationMode,
    	    boost::bind(&Camera::set_field<PhoXiMotionCam, PhoXiOperationMode>,
			this,
    			&PhoXi::MotionCam, &PhoXiMotionCam::OperationMode, _1,
			true, "OperationMode"),
    	    "Operation mode", enum_operation_mode, "", "motioncam");
    }

  // 1.2 laser power
    _ddr.registerVariable<int>(
	    "laser_power",
	    _device->MotionCam->LaserPower,
	    boost::bind(&Camera::set_field<PhoXiMotionCam, int>, this,
			&PhoXi::MotionCam, &PhoXiMotionCam::LaserPower, _1,
			false, "LaserPower"),
	    "Laser power",
	    1, 4095, "motioncam");

#if defined(HAVE_LED_POWER)
  // 1.3 led power
    _ddr.registerVariable<int>(
	    "led_power",
	    _device->MotionCam->LEDPower,
	    boost::bind(&Camera::set_field<PhoXiMotionCam, int>, this,
			&PhoXi::MotionCam, &PhoXiMotionCam::LEDPower, _1,
			false, "LEDPower"),
	    "LED power",
	    0, 4095, "motioncam");

#endif

  // 1.4 maximum fps
    _ddr.registerVariable<double>(
	    "maximum_fps",
	    _device->MotionCam->MaximumFPS,
	    boost::bind(&Camera::set_field<PhoXiMotionCam, double>, this,
			&PhoXi::MotionCam, &PhoXiMotionCam::MaximumFPS, _1,
			false, "MaximumFPS"),
	    "Maximum fps",
	    0.0, 20.0, "motioncam");

#  if defined(HAVE_HARDWARE_TRIGGER)
  // 1.5 hardware trigger
    _ddr.registerVariable<bool>(
	    "hardware_trigger",
	    _device->MotionCam->HardwareTrigger,
	    boost::bind(&Camera::set_field<PhoXiMotionCam, bool>, this,
			&PhoXi::MotionCam,
			&PhoXiMotionCam::HardwareTrigger, _1, false,
			"HardwareTrigger"),
	    "Hardware trigger",
	    false, true, "motioncam");

#    if defined(HAVE_HARDWARE_TRIGGER_SIGNAL)
  // 1.6 hardware trigger signal
    const std::map<std::string, int>
	enum_hardware_trigger_signal =
	{{"Falling", PhoXiHardwareTriggerSignal::Falling},
	 {"Rising",  PhoXiHardwareTriggerSignal::Rising},
	 {"Both",    PhoXiHardwareTriggerSignal::Both}};
	_ddr.registerEnumVariable<int>(
    	    "hardware_trigger_signal",
    	    _device->MotionCam->HardwareTriggerSignal,
    	    boost::bind(&Camera::set_field<PhoXiMotionCam,
					   PhoXiHardwareTriggerSignal>,
			this,
    			&PhoXi::MotionCam,
			&PhoXiMotionCam::HardwareTriggerSignal, _1, false,
			"HardwareTriggerSignal"),
    	    "Hardware trigger siganl",
	    enum_hardware_trigger_signal, "", "motioncam");
#    endif
#  endif

  // 2, MotionCam camera mode
  // 2,1 exposure
    std::map<std::string, double>	enum_exposures;
    for (auto exposure : _device->SupportedSinglePatternExposures.GetValue())
	enum_exposures.emplace(std::to_string(exposure), exposure);
    _ddr.registerEnumVariable<double>(
	    "camera_exposure",
	    _device->MotionCamCameraMode->Exposure,
	    boost::bind(&Camera::set_field<PhoXiMotionCamCameraMode, double>,
			this,
			&PhoXi::MotionCamCameraMode,
			&PhoXiMotionCamCameraMode::Exposure, _1, false,
			"CameraExposure"),
	    "Exposure time in miliseconds",
	    enum_exposures, "", "motioncam_camera_mode");

  // 2,2 sampling topology
    if (_device->MotionCamCameraMode->SamplingTopology !=
	PhoXiSamplingTopology::NoValue)
    {
	const std::map<std::string, int>
	    enum_sampling_topology = {{"Standard",
				       PhoXiSamplingTopology::Standard}};
	_ddr.registerEnumVariable<int>(
    	    "sampling_topology",
    	    _device->MotionCamCameraMode->SamplingTopology,
    	    boost::bind(&Camera::set_field<PhoXiMotionCamCameraMode,
					   PhoXiSamplingTopology>,
    			this,
    			&PhoXi::MotionCamCameraMode,
			&PhoXiMotionCamCameraMode::SamplingTopology, _1, false,
			"SamplingTopology"),
    	    "Sampling topology",
	    enum_sampling_topology, "", "motioncam_camera_mode");
    }

  // 2,3 output topology
    if (_device->MotionCamCameraMode->OutputTopology !=
	PhoXiOutputTopology::NoValue)
    {
	const std::map<std::string, int>
	    enum_output_topology = {{"IrregularGrid",
				     PhoXiOutputTopology::IrregularGrid},
				    {"Raw",
				     PhoXiOutputTopology::Raw},
				    {"RegularGrid",
				     PhoXiOutputTopology::RegularGrid}};
	_ddr.registerEnumVariable<int>(
    	    "output_topology",
    	    _device->MotionCamCameraMode->OutputTopology,
    	    boost::bind(&Camera::set_field<PhoXiMotionCamCameraMode,
					   PhoXiOutputTopology>,
			this,
    			&PhoXi::MotionCamCameraMode,
			&PhoXiMotionCamCameraMode::OutputTopology, _1, false,
			"OutputTopology"),
    	    "Output topology",
	    enum_output_topology, "", "motioncam_camera_mode");
    }

  // 2,4 coding strategy
    if (_device->MotionCamCameraMode->CodingStrategy !=
	PhoXiCodingStrategy::NoValue)
    {
	const std::map<std::string, int>
	    enum_coding_strategy = {{"Normal",
				     PhoXiCodingStrategy::Normal},
				    {"Interreflections",
				     PhoXiCodingStrategy::Interreflections}};
	_ddr.registerEnumVariable<int>(
    	    "camera_coding_strategy",
    	    _device->MotionCamCameraMode->CodingStrategy,
    	    boost::bind(&Camera::set_field<PhoXiMotionCamCameraMode,
					   PhoXiCodingStrategy>,
			this,
    			&PhoXi::MotionCamCameraMode,
			&PhoXiMotionCamCameraMode::CodingStrategy, _1, false,
			"CodingStrategy"),
    	    "Coding strategy",
	    enum_coding_strategy, "", "motioncam_camera_mode");
    }

  // 3. MotionCam scanner mode
  // 3.1 shutter multiplier
    _ddr.registerVariable<int>(
	"shutter_multiplier",
	_device->MotionCamScannerMode->ShutterMultiplier,
	boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode, int>, this,
		    &PhoXi::MotionCamScannerMode,
		    &PhoXiMotionCamScannerMode::ShutterMultiplier, _1, false,
		    "ShutterMultiplier"),
	"Shutter multiplier", 1, 20, "motioncam_scanner_mode");

  // 3.2 scan multiplier
    _ddr.registerVariable<int>(
	"scan_multiplier",
	_device->MotionCamScannerMode->ScanMultiplier,
	boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode, int>, this,
		    &PhoXi::MotionCamScannerMode,
		    &PhoXiMotionCamScannerMode::ScanMultiplier, _1, false,
		    "ScanMultiplier"),
	"Scan multiplier", 1, 20, "motioncam_scanner_mode");

  // 3.3 coding strategy
    if (_device->MotionCamScannerMode->CodingStrategy !=
	PhoXiCodingStrategy::NoValue)
    {
	const std::map<std::string, int>
	    enum_coding_strategy = {{"Normal",
				     PhoXiCodingStrategy::Normal},
				    {"Interreflections",
				     PhoXiCodingStrategy::Interreflections}};
	_ddr.registerEnumVariable<int>(
	    "scanner_coding_strategy",
	    _device->MotionCamScannerMode->CodingStrategy,
	    boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode,
					   PhoXiCodingStrategy>,
			this,
			&PhoXi::MotionCamScannerMode,
			&PhoXiMotionCamScannerMode::CodingStrategy, _1, false,
			"CodingStrategy"),
	    "Coding  strategy", enum_coding_strategy, "",
	    "motioncam_scanner_mode");
    }

  // 3.4 coding quality
    if (_device->MotionCamScannerMode->CodingQuality !=
	PhoXiCodingQuality::NoValue)
    {
	const std::map<std::string, int>
	    enum_coding_quality = {{"Fast",  PhoXiCodingQuality::Fast},
				   {"High",  PhoXiCodingQuality::High},
				   {"Ultra", PhoXiCodingQuality::Ultra}};
	_ddr.registerEnumVariable<int>(
	    "coding_quality",
	    _device->MotionCamScannerMode->CodingQuality,
	    boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode,
					   PhoXiCodingQuality>,
			this,
			&PhoXi::MotionCamScannerMode,
			&PhoXiMotionCamScannerMode::CodingQuality, _1, false,
			"CodingQuality"),
	    "Coding quality", enum_coding_quality, "",
	    "motioncam_scanner_mode");
    }

  // 3.5 texture source
    if (_device->MotionCamScannerMode->TextureSource !=
	PhoXiTextureSource::NoValue)
    {
	const std::map<std::string, int>
	    enum_texture_source = {{"Computed", PhoXiTextureSource::Computed},
				   {"LED",	PhoXiTextureSource::LED},
				   {"Laser",	PhoXiTextureSource::Laser},
				   {"Focus",	PhoXiTextureSource::Focus},
				   {"Color",	PhoXiTextureSource::Color}};
	_ddr.registerEnumVariable<int>(
	    "texture_source",
	    _device->MotionCamScannerMode->TextureSource,
	    boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode,
					   PhoXiTextureSource>,
			this,
			&PhoXi::MotionCamScannerMode,
			&PhoXiMotionCamScannerMode::TextureSource, _1, false,
			"TextureSource"),
	    "Texture source", enum_texture_source, "",
	    "motioncam_scanner_mode");
    }

  // 3.6 exposure
#  if defined(HAVE_MOTIONCAM_EXPOSURE)
    _ddr.registerEnumVariable<double>(
	"scanner_exposure",
	_device->MotionCamScannerMode->Exposure,
	boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode, double>,
		    this,
		    &PhoXi::MotionCamScannerMode,
		    &PhoXiMotionCamScannerMode::Exposure, _1, false,
		    "Exposure"),
	"Exposure", enum_exposures, "", "motioncam_scanner_mode");
#  endif
}
#endif

void
Camera::setup_ddr_common()
{
    using namespace	pho::api;

  // 1. TriggerMode
    if (_device->TriggerMode.GetValue() != PhoXiTriggerMode::NoValue)
    {
	const std::map<std::string, int>
	    enum_trigger = {{"Freerun",  PhoXiTriggerMode::Freerun},
			    {"Software", PhoXiTriggerMode::Software},
			    {"Hardware", PhoXiTriggerMode::Hardware}};
	_ddr.registerEnumVariable<int>(
	    "trigger_mode", _device->TriggerMode.GetValue(),
	    boost::bind(&Camera::set_feature<PhoXiTriggerMode, int>, this,
			&PhoXi::TriggerMode, _1, true),
	    "Trigger mode", enum_trigger);
    }

  // 2. Timeout
    const std::map<std::string, int>
	enum_timeout = {{"ZeroTimeout", PhoXiTimeout::ZeroTimeout},
			{"Infinity",    PhoXiTimeout::Infinity},
			{"LastStored",  PhoXiTimeout::LastStored},
			{"Default",     PhoXiTimeout::Default}};
    _ddr.registerEnumVariable<int>(
	    "timeout",
	    _device->Timeout.GetValue(),
	    boost::bind(&Camera::set_feature<PhoXiTimeout, int>, this,
			&PhoXi::Timeout, _1, false),
	    "Timeout settings", enum_timeout);

  // 3, ProcessingSettings
  // 3.1 ConfidenceValue
    _ddr.registerVariable<double>(
	    "confidence",
	    _device->ProcessingSettings->Confidence,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, double>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::Confidence, _1, false,
			"Confidence"),
	    "Confidence value", 0.0, 100.0, "processing_settings");

  // 3.2 SurfaceSmoothness
    if (_device->ProcessingSettings->SurfaceSmoothness !=
    	PhoXiSurfaceSmoothness::NoValue)
    {
	const std::map<std::string, int>
	    enum_surface_smoothness = {{"Sharp",
					PhoXiSurfaceSmoothness::Sharp},
				       {"Normal",
					PhoXiSurfaceSmoothness::Normal},
				       {"Smooth",
					PhoXiSurfaceSmoothness::Smooth}};
	_ddr.registerEnumVariable<int>(
    	    "surface_smoothness",
    	    _device->ProcessingSettings->SurfaceSmoothness,
    	    boost::bind(&Camera::set_field<PhoXiProcessingSettings,
					   PhoXiSurfaceSmoothness>,
    			this,
    			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::SurfaceSmoothness, _1, false,
			"SurfaceSmoothness"),
    	    "Surface smoothness",
	    enum_surface_smoothness, "", "processing_settings");
    }

  // 3.3 CalibrationVolumeOnly
    _ddr.registerVariable<bool>(
	    "calibration_volume_only",
	    _device->ProcessingSettings->CalibrationVolumeOnly,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, bool>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::CalibrationVolumeOnly, _1,
			false, "CalibrationVolumeOnly"),
	    "Calibration volume only", false, true, "processing_settings");

  // 3.4 NormalsEstimationRadius
    _ddr.registerVariable<int>(
	    "normals_estimation_radius",
	    _device->ProcessingSettings->NormalsEstimationRadius,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, int>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::NormalsEstimationRadius, _1,
			false, "NormalsEstimationRadius"),
	    "Normals estimation radius", 0, 4, "processing_settings");

#if defined(HAVE_INTERREFLECTIONS_FILTERING)
  // 3.5 InterreflectionsFiltering
    _ddr.registerVariable<bool>(
	    "interreflections_filtering",
	    _device->ProcessingSettings->InterreflectionsFiltering,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, bool>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::InterreflectionsFiltering,
			_1, false, "InterreflectionsFiltering"),
	    "Interreflections filtering", false, true, "processing_settings");

#  if defined(HAVE_INTERREFLECTION_FILTER_STRENGTH)
  // 3.6 InterreflectionFilterStrength
    _ddr.registerVariable<double>(
	    "interreflection_filter_strength",
	    _device->ProcessingSettings->InterreflectionFilterStrength,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, double>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::InterreflectionFilterStrength,
			_1, false, "InterreflectionFilterStrength"),
	    "Interreflection filter strength", 0, 4, "processing_settings");
#  endif
#endif

#if defined(HAVE_COLOR)    
  // 4. ColorSettings
  // 4.1 Iso
    const auto	color_iso = _device->SupportedColorIso.GetValue();
    if (color_iso.size() > 1)
    {
	std::map<std::string, int>	enum_color_iso;
	int				idx = 0;
	for (int i = 0; i < color_iso.size(); ++i)
	{
	    enum_color_iso.emplace(std::to_string(color_iso[i]), color_iso[i]);
	    if (color_iso[i] == _device->ColorSettings.GetValue().Iso)
		idx = i;
	}
	_ddr.registerEnumVariable<int>(
	    "color_iso", color_iso[idx],
	    boost::bind(&Camera::set_field<PhoXiColorSettings, int>, this,
			&PhoXi::ColorSettings,
			&PhoXiColorSettings::Iso, _1, false, "Iso"),
	    "Color ISO", enum_color_iso, "", "color_settings");
    }
    
  // 4.2 Exposure
    const auto	color_exposure = _device->SupportedColorExposure.GetValue();
    if (color_exposure.size() > 1)
    {
	std::map<std::string, double>	enum_color_exposure;
	int				idx = 0;
	for (int i = 0; i < color_exposure.size(); ++i)
	{
	    enum_color_exposure.emplace(std::to_string(color_exposure[i]),
					color_exposure[i]);
	    if (color_exposure[i] == _device->ColorSettings.GetValue().Exposure)
		idx = i;
	}
	_ddr.registerEnumVariable<double>(
	    "color_exposure", color_exposure[idx],
	    boost::bind(&Camera::set_field<PhoXiColorSettings, double>, this,
			&PhoXi::ColorSettings,
			&PhoXiColorSettings::Exposure, _1, false, "Exposure"),
	    "Color exposure", enum_color_exposure, "", "color_settings");
    }
    
  // 4.3 CapturingMode
    const auto	color_modes = _device->SupportedColorCapturingModes.GetValue();
    if (color_modes.size() > 1)
    {
	std::map<std::string, int>	enum_resolution;
	int				idx = 0;
	for (int i = 0; i < color_modes.size(); ++i)
	{
	    const auto&	resolution = color_modes[i].Resolution;
	    enum_resolution.emplace(std::to_string(resolution.Width) + 'x' +
				    std::to_string(resolution.Height), i);
	    if (color_modes[i] == _device->ColorSettings.GetValue().CapturingMode)
		idx = i;
	}
	_ddr.registerEnumVariable<int>("color_capturing_mode", idx,
				       boost::bind(
					   &Camera::set_color_resolution,
					   this, _1),
				       "Color capturing mode", enum_resolution,
				       "", "color_settings");
    }

  // 4.4 Gamma
    _ddr.registerVariable<double>(
	    "gamma", _intensityScale,
	    boost::bind(&Camera::set_field<PhoXiColorSettings, double>, this,
			&PhoXi::ColorSettings,
			&PhoXiColorSettings::Gamma, _1, false, "Gamma"),
	    "Color gamma correction", 0.0, 5.0, "", "color_settings");
    
  // 4.5 WhiteBalance
    const auto	white_balance_presets
		    = _device->SupportedColorWhiteBalancePresets.GetValue();
    if (white_balance_presets.size() > 1)
    {
	std::map<std::string, std::string>	enum_white_balance;
	int					idx = 0;
	for (int i = 0; i < white_balance_presets.size(); ++i)
	{
	    const auto&	white_balance = white_balance_presets[i];
	    enum_white_balance.emplace(white_balance, i);
	    if (white_balance == _device->ColorSettings.GetValue().WhiteBalance.Preset)
		idx = i;
	}
	_ddr.registerEnumVariable<int>("white_balance", idx,
				       boost::bind(
					   &Camera::set_white_balance,
					   this, _1),
				       "White balance", enum_white_balance,
				       "", "color_settings");
    }
#endif
    
  // 5. OutputSettings
    _ddr.registerVariable<bool>(
	    "send_point_cloud",
	    _device->OutputSettings->SendPointCloud,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendPointCloud, _1, true,
			"SendPointCloud"),
	    "Publish point cloud if set.", false, true, "output_settings");
    _ddr.registerVariable<bool>(
	    "send_normal_map",
	    _device->OutputSettings->SendNormalMap,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendNormalMap, _1, true,
			"SendNormalMap"),
	    "Publish normal map if set.", false, true, "output_settings");
    _ddr.registerVariable<bool>(
	    "send_depth_map",
	    _device->OutputSettings->SendDepthMap,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendDepthMap, _1, true,
			"SendDepthMap"),
	    "Publish depth map if set.", false, true, "output_settings");
    _ddr.registerVariable<bool>(
	    "send_confidence_map",
	    _device->OutputSettings->SendConfidenceMap,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendConfidenceMap, _1, true,
			"SendConfidenceMap"),
	    "Publish confidence map if set.", false, true, "output_settings");
    _ddr.registerVariable<bool>(
	    "send_texture",
	    _device->OutputSettings->SendTexture,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendTexture, _1, true,
			"SendTexture"),
	    "Publish texture if set.", false, true, "output_settings");

  // 6. Density of the cloud
    _ddr.registerVariable<bool>(
	    "dense_cloud", _denseCloud,
	    boost::bind(&Camera::set_member<bool>, this,
			boost::ref(_denseCloud), _1, "dense_cloud"),
	    "Dense cloud if set.", false, true);

  // 7. Intensity scale
    _ddr.registerVariable<double>(
	    "intensity_scale", _intensityScale,
	    boost::bind(&Camera::set_member<double>, this,
			boost::ref(_intensityScale), _1, "intensity_scale"),
	    "Multiplier for intensity values of published texture",
	    0.05, 5.0);
}

void
Camera::set_resolution(int idx)
{
    const auto acq = _device->isAcquiring();
    if (acq)
	_device->StopAcquisition();

    const auto modes = _device->SupportedCapturingModes.GetValue();
    if (idx < modes.size())
	_device->CapturingMode = modes[idx];
    NODELET_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set resolution to "
			<< _device->CapturingMode.GetValue().Resolution.Width
			<< 'x'
			<< _device->CapturingMode.GetValue().Resolution.Height);

    set_camera_info();
    _device->ClearBuffer();
    if (acq)
	_device->StartAcquisition();
}

template <class F, class T> void
Camera::set_feature(pho::api::PhoXiFeature<F> pho::api::PhoXi::* feature,
		    T value, bool suspend)
{
    const auto acq = _device->isAcquiring();
    if (suspend && acq)
	_device->StopAcquisition();

    auto&	f = _device.operator ->()->*feature;
    f.SetValue(value);
    NODELET_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set " << f.GetName() << " to " << f.GetValue());

    if (suspend)
    {
	set_camera_info();
	_device->ClearBuffer();
	if (acq)
	    _device->StartAcquisition();
    }
}

template <class F, class T> void
Camera::set_field(pho::api::PhoXiFeature<F> pho::api::PhoXi::* feature,
		  T F::* field, T value, bool suspend,
		  const std::string& field_name)
{
    const auto acq = _device->isAcquiring();
    if (suspend && acq)
	_device->StopAcquisition();

    auto&	f   = _device.operator ->()->*feature;
    auto	val = f.GetValue();
    val.*field = value;
    f.SetValue(val);
    NODELET_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set " << f.GetName() << "::" << field_name
			<< " to "   << f.GetValue().*field);

    if (suspend)
    {
	set_camera_info();
	_device->ClearBuffer();
	if (acq)
	    _device->StartAcquisition();
    }
}

template <class T> void
Camera::set_member(T& member, T value, const std::string& name)
{
    member = value;
    NODELET_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set " << name << " to " << member);
}

void
Camera::set_color_resolution(int idx)
{
    const auto acq = _device->isAcquiring();
    if (acq)
	_device->StopAcquisition();

    const auto modes = _device->SupportedColorCapturingModes.GetValue();
    if (idx < modes.size())
	_device->CapturingMode = modes[idx];
    NODELET_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set color resolution to "
			<< _device->ColorSettings.GetValue()
				.CapturingMode.Resolution.Width
			<< 'x'
			<< _device->ColorSettings.GetValue()
				.CapturingMode.Resolution.Height);

    set_camera_info();
    _device->ClearBuffer();
    if (acq)
	_device->StartAcquisition();
}

bool
Camera::trigger_frame(std_srvs::Trigger::Request&  req,
		      std_srvs::Trigger::Response& res)
{
    using namespace	pho::api;

    NODELET_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") trigger_frame: service requested");

    const auto	frameId = _device->TriggerFrame(true, true);

    NODELET_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") trigger_frame: triggered");

    switch (frameId)
    {
      case -1:
	res.success = false;
	res.message = "failed. [TriggerFrame not accepted]";
	break;
      case -2:
	res.success = false;
	res.message = "failed. [device is not running]";
	break;
      case -3:
	res.success = false;
	res.message = "failed. [communication error]";
	break;
      case -4:
	res.success = false;
	res.message = "failed. [WaitForGrabbingEnd is not supported]";
	break;
      default:
	if (!(_frame = _device->GetSpecificFrame(frameId,
						 PhoXiTimeout::Infinity)))
	{
	    res.success = false;
	    res.message = "failed. [not found frame #"
			+ std::to_string(frameId) + ']';
	    break;
	}

	NODELET_INFO_STREAM('('
			    << _device->HardwareIdentification.GetValue()
			    << ") trigger_frame: frame got");

	if (_frame->Info.FrameIndex != frameId)
	{
	    res.success = false;
	    res.message = "failed. [triggered frame(#"
			+ std::to_string(frameId)
			+ ") is not captured frame(#"
			+ std::to_string(_frame->Info.FrameIndex)
			+ ")]";
	    break;
	}

	publish_frame();	// publish
	res.success = true;
	res.message = "succeeded. [frame #" + std::to_string(frameId) + ']';
	break;
    }

    if (res.success)
	NODELET_INFO_STREAM('('
			    << _device->HardwareIdentification.GetValue()
			    << ") trigger_frame: "
			    << res.message);
    else
	NODELET_ERROR_STREAM('('
			     << _device->HardwareIdentification.GetValue()
			     << ") trigger_frame: "
			     << res.message);

    return true;
}

bool
Camera::save_frame(SetString::Request& req, SetString::Response& res)
{
    if (_frame == nullptr || !_frame->Successful)
    {
	res.success = false;
	NODELET_ERROR_STREAM('('
			     << _device->HardwareIdentification.GetValue()
			     << ") save_frame: failed. [no frame data]");
    }
    else if (!_frame->SaveAsPly(req.in + ".ply"))
    {
	res.success = false;
	NODELET_ERROR_STREAM('('
			     << _device->HardwareIdentification.GetValue()
			     << ") save_frame: failed to save PLY to "
			     << req.in + ".ply");
    }
    else
    {
	res.success = true;
	NODELET_INFO_STREAM('('
			    << _device->HardwareIdentification.GetValue()
			    << ") save_frame: succeeded to save PLY to "
			    << req.in + ".ply");
    }

    return true;
}

bool
Camera::save_settings(std_srvs::Trigger::Request&  req,
		      std_srvs::Trigger::Response& res)
{
    res.success = _device->SaveSettings();

    if (res.success)
    {
	res.message = "succesfully saved settings";
	NODELET_INFO_STREAM('('
			    << _device->HardwareIdentification.GetValue()
			    << ") save_settings: "
			    << res.message);
    }
    else
    {
	res.message = "failed to save settings";
	NODELET_ERROR_STREAM('('
			     << _device->HardwareIdentification.GetValue()
			     << ") save_settings: "
			     << res.message);
    }

    return true;
}

bool
Camera::restore_settings(std_srvs::Trigger::Request&  req,
			 std_srvs::Trigger::Response& res)
{
    const auto acq = _device->isAcquiring();
    if (acq)
	_device->StopAcquisition();

    res.success = _device->ResetActivePreset();

    if (res.success)
    {
	res.message = "succesfully restored settings.";
	NODELET_INFO_STREAM('('
			    << _device->HardwareIdentification.GetValue()
			    << ") restore_settings: "
			    << res.message);
    }
    else
    {
	res.message = "failed to restore settings.";
	NODELET_ERROR_STREAM('('
			     << _device->HardwareIdentification.GetValue()
			     << ") restore_settings: "
			     << res.message);
    }

    set_camera_info();
    _device->ClearBuffer();
    if (acq)
	_device->StartAcquisition();

    return true;
}

void
Camera::set_camera_info()
{
    using namespace	pho::api;

    constexpr static size_t	PhoXiNominalWidth	= 2064;
    constexpr static size_t	PhoXiNominalHeight	= 1544;
    constexpr static size_t	MotionCamNominalWidth	= 1680;
    constexpr static size_t	MotionCamNominalHeight	= 1200;

    const auto& calib = _device->CalibrationSettings.GetValue();

  // Set header.
    _cinfo->header.frame_id = _frame_id;

  // Set height and width.
    const auto	mode = _device->CapturingMode.GetValue();
    _cinfo->height = mode.Resolution.Height;
    _cinfo->width  = mode.Resolution.Width;

  // Set distortion parameters.
    _cinfo->distortion_model = "plumb_bob";
    _cinfo->D.resize(8);
    std::copy_n(std::begin(calib.DistortionCoefficients),
		std::size(_cinfo->D), std::begin(_cinfo->D));

  // Set intrinsic parameters.
    bool	isMotionCam = (PhoXiDeviceType::Value(_device->GetType()) ==
			       PhoXiDeviceType::MotionCam3D);
    const auto	scale_u = double(mode.Resolution.Width)
			/ double(isMotionCam ? MotionCamNominalWidth
					     : PhoXiNominalWidth);
    const auto	scale_v = double(mode.Resolution.Height)
			/ double(isMotionCam ? MotionCamNominalHeight
					     : PhoXiNominalHeight);
    _cinfo->K[0] = scale_u * calib.CameraMatrix[0][0];
    _cinfo->K[1] = scale_u * calib.CameraMatrix[0][1];
    _cinfo->K[2] = scale_u * calib.CameraMatrix[0][2];
    _cinfo->K[3] = scale_v * calib.CameraMatrix[1][0];
    _cinfo->K[4] = scale_v * calib.CameraMatrix[1][1];
    _cinfo->K[5] = scale_v * calib.CameraMatrix[1][2];
    _cinfo->K[6] =	     calib.CameraMatrix[2][0];
    _cinfo->K[7] =	     calib.CameraMatrix[2][1];
    _cinfo->K[8] =	     calib.CameraMatrix[2][2];

  // Set _cinfo->R to be an identity matrix.
    std::fill(std::begin(_cinfo->R), std::end(_cinfo->R), 0.0);
    _cinfo->R[0] = _cinfo->R[4] = _cinfo->R[8] = 1.0;

  // Set 3x4 camera matrix.
    for (int i = 0; i < 3; ++i)
	for (int j = 0; j < 3; ++j)
	    _cinfo->P[4*i + j] = _cinfo->K[3*i + j];
    _cinfo->P[3] = _cinfo->P[7] = _cinfo->P[11] = 0.0;

  // No binning
    _cinfo->binning_x = _cinfo->binning_y = 0;

  // ROI is same as entire image.
    _cinfo->roi.width = _cinfo->roi.height = 0;
}

void
Camera::publish_frame()
{
  // Common setting.
    const auto	now = ros::Time::now();

  // Publish point cloud.
    profiler_start(0);
    constexpr float	distanceScale = 0.001;	// milimeters -> meters
    publish_cloud(now, distanceScale);

  // Publish normal_map, depth_map, confidence_map and texture.
    profiler_start(1);
    publish_image(_frame->NormalMap, _normal_map_publisher, _normal_map, now,
		  sensor_msgs::image_encodings::TYPE_32FC3, 1);
    profiler_start(2);
    publish_image(_frame->DepthMap, _depth_map_publisher, _depth_map, now,
		  sensor_msgs::image_encodings::TYPE_32FC1, distanceScale);
    profiler_start(3);
    publish_image(_frame->ConfidenceMap, _confidence_map_publisher,
		  _confidence_map, now,
		  sensor_msgs::image_encodings::TYPE_32FC1, 1);
    profiler_start(4);
    publish_image(_frame->Texture, _texture_publisher, _texture, now,
		  sensor_msgs::image_encodings::MONO8, _intensityScale);

  // publish camera_info
    profiler_start(5);
    publish_camera_info(now);

    profiler_print(std::cerr);

    NODELET_DEBUG_STREAM('('
			 << _device->HardwareIdentification.GetValue()
			 << ") frame published [#"
			 << _frame->Info.FrameIndex << ']');
}

void
Camera::publish_cloud(const ros::Time& stamp, float distanceScale)
{
    using namespace	sensor_msgs;

    const auto&	phoxi_cloud = _frame->PointCloud;
    if (_cloud_publisher.getNumSubscribers() == 0 ||
    	!_device->OutputSettings->SendPointCloud || phoxi_cloud.Empty())
	return;

  // Convert pho::api::PointCloud32f to sensor_msgs::PointCloud2
    _cloud->header.stamp    = stamp;
    _cloud->header.frame_id = _frame_id;
    _cloud->is_bigendian    = false;
    _cloud->is_dense	    = _denseCloud;

    PointCloud2Modifier	modifier(*_cloud);
    if (_device->OutputSettings->SendTexture)
	if (_device->OutputSettings->SendNormalMap)
	    modifier.setPointCloud2Fields(7,
					  "x",	      1, PointField::FLOAT32,
					  "y",	      1, PointField::FLOAT32,
					  "z",	      1, PointField::FLOAT32,
					  "rgb",      1, PointField::UINT32,
					  "normal_x", 1, PointField::FLOAT32,
					  "normal_y", 1, PointField::FLOAT32,
					  "normal_z", 1, PointField::FLOAT32);
	else
	    modifier.setPointCloud2Fields(4,
					  "x",	      1, PointField::FLOAT32,
					  "y",	      1, PointField::FLOAT32,
					  "z",	      1, PointField::FLOAT32,
					  "rgb",      1, PointField::UINT32);
    else
	if (_device->OutputSettings->SendNormalMap)
	    modifier.setPointCloud2Fields(6,
					  "x",	      1, PointField::FLOAT32,
					  "y",	      1, PointField::FLOAT32,
					  "z",	      1, PointField::FLOAT32,
					  "normal_x", 1, PointField::FLOAT32,
					  "normal_y", 1, PointField::FLOAT32,
					  "normal_z", 1, PointField::FLOAT32);
	else
	    modifier.setPointCloud2Fields(3,
					  "x",	      1, PointField::FLOAT32,
					  "y",	      1, PointField::FLOAT32,
					  "z",	      1, PointField::FLOAT32);

    if (_cloud->is_dense)
    {
	const auto	npoints = npoints_valid(phoxi_cloud);
	modifier.resize(npoints);
	_cloud->height = 1;
	_cloud->width  = npoints;
    }
    else
    {
	modifier.resize(phoxi_cloud.Size.Height * phoxi_cloud.Size.Width);
	_cloud->height = phoxi_cloud.Size.Height;
	_cloud->width  = phoxi_cloud.Size.Width;
    }
    _cloud->row_step = _cloud->width * _cloud->point_step;

    PointCloud2Iterator<float>	xyz(*_cloud, "x");

    for (int v = 0; v < phoxi_cloud.Size.Height; ++v)
    {
	auto	p = phoxi_cloud[v];

	for (const auto q = p + phoxi_cloud.Size.Width; p != q; ++p)
	{
	    if (float(p->z) == 0.0f)
	    {
		if (!_cloud->is_dense)
		{
		    xyz[0] = xyz[1] = xyz[2]
			   = std::numeric_limits<float>::quiet_NaN();
		    ++xyz;
		}
	    }
	    else
	    {
		xyz[0] = float(p->x) * distanceScale;
		xyz[1] = float(p->y) * distanceScale;
		xyz[2] = float(p->z) * distanceScale;
		++xyz;
	    }
	}
    }


    if (_device->OutputSettings->SendTexture)
    {
	PointCloud2Iterator<uint8_t> rgb(*_cloud, "rgb");

	for (int v = 0; v < phoxi_cloud.Size.Height; ++v)
	{
	    auto	p = phoxi_cloud[v];
	    auto	r = _frame->Texture[v];

	    for (const auto q = p + phoxi_cloud.Size.Width; p != q; ++p, ++r)
	    {
		rgb[0] = rgb[1] = rgb[2]
		       = uint8_t(std::min(*r * _intensityScale, 255.0));
		++rgb;
	    }
	}
    }

    if (_device->OutputSettings->SendNormalMap)
    {
	if (_device->ProcessingSettings->NormalsEstimationRadius == 0)
	{
	    NODELET_ERROR_STREAM('('
				 << _device->HardwareIdentification.GetValue()
				 << ") normals_estimation_radius must be positive");
	    return;
	}

	PointCloud2Iterator<float>	normal(*_cloud, "normal_x");

	for (int v = 0; v < phoxi_cloud.Size.Height; ++v)
	{
	    auto	p = phoxi_cloud[v];
	    auto	r = _frame->NormalMap[v];

	    for (const auto q = p + phoxi_cloud.Size.Width; p != q; ++p, ++r)
	    {
		normal[0] = r->x;
		normal[1] = r->y;
		normal[2] = r->z;
		++normal;
	    }
	}
    }

    _cloud_publisher.publish(_cloud);
}

template <class T> void
Camera::publish_image(const pho::api::Mat2D<T>& phoxi_image,
		      const image_transport::Publisher& publisher,
		      const image_p& image,
		      const ros::Time& stamp,
		      const std::string& encoding,
		      typename T::ElementChannelType scale) const
{
    using namespace	sensor_msgs;
    using		element_ptr = const typename T::ElementChannelType*;

    if (publisher.getNumSubscribers() == 0 || phoxi_image.Empty())
	return;

    image->header.stamp    = stamp;
    image->header.frame_id = _frame_id;
    image->encoding	   = encoding;
    image->is_bigendian    = 0;
    image->height	   = phoxi_image.Size.Height;
    image->width	   = phoxi_image.Size.Width;
    image->step		   = image->width
			   *  image_encodings::numChannels(image->encoding)
			   * (image_encodings::bitDepth(image->encoding)/8);
    image->data.resize(image->step * image->height);

    const auto	p = reinterpret_cast<element_ptr>(phoxi_image[0]);
    const auto	q = reinterpret_cast<element_ptr>(
			phoxi_image[phoxi_image.Size.Height]);
    if (image->encoding == image_encodings::MONO8)
	scale_copy(p, q, image->data.data(), scale);
    else if (image->encoding.substr(0, 4) == "32FC")
	scale_copy(p, q, reinterpret_cast<float*>(image->data.data()), scale);
    else
    {
	NODELET_ERROR_STREAM("Unsupported image type!");
	throw std::logic_error("");
    }

    publisher.publish(image);
}

void
Camera::publish_camera_info(const ros::Time& stamp) const
{
    if (_camera_info_publisher.getNumSubscribers() == 0)
	return;

    _cinfo->header.stamp = stamp;
    _camera_info_publisher.publish(_cinfo);
}

}	// namespace aist_phoxi_camera
