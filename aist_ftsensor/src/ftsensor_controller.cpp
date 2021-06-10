/*
 *  \file	ftsensor_controller.cpp
 *  \brief	source file of a class for controlling force-torque sensors
 */
#include "ftsensor_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace aist_ftsensor
{
/************************************************************************
*  class ForceTorqueSensorController					*
************************************************************************/
bool
ForceTorqueSensorController::init(hw_interface_t* hw,
				  ros::NodeHandle& root_nh,
				  ros::NodeHandle& controller_nh)
{
  // get publishing period
    double	pub_rate;
    if (!controller_nh.getParam("publish_rate", pub_rate))
    {
	ROS_ERROR_STREAM("(aist_ftsensor_controller) Parameter 'publish_rate' not set");
	return false;
    }

  // setup sensors
    for (const auto& sensor_name : hw->getNames())
	_sensors.push_back(sensor_p(new Sensor(hw, root_nh,
					       sensor_name, pub_rate)));

    return true;
}

void
ForceTorqueSensorController::starting(const ros::Time& time)
{
    for (const auto& sensor : _sensors)
	sensor->starting(time);
}

void
ForceTorqueSensorController::update(const ros::Time& time,
				    const ros::Duration& period)
{
    for (const auto& sensor : _sensors)
	sensor->update(time, period);
}

void
ForceTorqueSensorController::stopping(const ros::Time& time)
{
    for (const auto& sensor : _sensors)
	sensor->stopping(time);
}

/************************************************************************
*  class ForceTorqueSensorController::Sensor				*
************************************************************************/
ForceTorqueSensorController::Sensor::Sensor(hw_interface_t* hw,
					    ros::NodeHandle& root_nh,
					    const std::string& sensor_name,
					    double pub_rate)
    :_sensor(hw->getHandle(sensor_name)),
     _pub(new publisher_t(root_nh, sensor_name, 4)),
     _pub_rate(pub_rate),
     _last_pub_time(0)
{
    ROS_INFO_STREAM("(aist_ftsensor_controller) Got " << sensor_name.c_str());
}

void
ForceTorqueSensorController::Sensor::starting(const ros::Time& time)
{
    _last_pub_time = time;
}

void
ForceTorqueSensorController::Sensor::update(const ros::Time& time,
					    const ros::Duration& period)
{
    if (_pub_rate > 0.0 && _last_pub_time + ros::Duration(1.0/_pub_rate) < time)
    {
      // try to publish
	if (_pub->trylock())
	{
	  // we're actually publishing, so increment time
	    _last_pub_time = _last_pub_time + ros::Duration(1.0/_pub_rate);

	  // populate message
	    _pub->msg_.header.stamp    = time;
	    _pub->msg_.header.frame_id = _sensor.getFrameId();

	    _pub->msg_.wrench.force.x  = _sensor.getForce()[0];
	    _pub->msg_.wrench.force.y  = _sensor.getForce()[1];
	    _pub->msg_.wrench.force.z  = _sensor.getForce()[2];
	    _pub->msg_.wrench.torque.x = _sensor.getTorque()[0];
	    _pub->msg_.wrench.torque.y = _sensor.getTorque()[1];
	    _pub->msg_.wrench.torque.z = _sensor.getTorque()[2];

	    _pub->unlockAndPublish();
	}
    }
}

}	// namespace aist_ftsensor

PLUGINLIB_EXPORT_CLASS(aist_ftsensor::ForceTorqueSensorController,
		       controller_interface::ControllerBase)
