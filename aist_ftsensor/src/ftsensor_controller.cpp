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
    const auto&	sensor_names = hw->getNames();

  // get all joint states from the hardware interface
    for (const auto& sensor_name : sensor_names)
	ROS_DEBUG_STREAM("Got sensor " << sensor_name.c_str());

  // get publishing period
    if (!controller_nh.getParam("publish_rate", _pub_rate))
    {
	ROS_ERROR_STREAM("Parameter 'publish_rate' not set");
	return false;
    }

    for (const auto& sensor_name : sensor_names)
    {
      // sensor handle
	_sensors.push_back(hw->getHandle(sensor_name));

      // realtime publisher
	publisher_p	pub(new publisher_t(root_nh, sensor_name, 4));
	_pubs.push_back(pub);
    }

  // Last published times
    _last_pub_times.resize(sensor_names.size());

    return true;
}

void
ForceTorqueSensorController::starting(const ros::Time& time)
{
  // initialize time
    for (auto&& last_pub_time : _last_pub_times)
	last_pub_time = time;
}

void
ForceTorqueSensorController::update(const ros::Time& time,
				    const ros::Duration& /*period*/)
{
  // limit rate of publishing
    unsigned	i = 0;
    for (const auto& sensor : _sensors)
    {
	sensor->update(time);

	auto&&		last_pub_time = _last_pub_times[i];

	if (_pub_rate > 0.0 &&
	    last_pub_time + ros::Duration(1.0/_pub_rate) < time)
	{
	  // try to publish
	    if (pub->trylock())
	    {
	      // we're actually publishing, so increment time
		last_pub_time = last_pub_time + ros::Duration(1.0/_pub_rate);

	      // populate message
		const auto&	sensor	  = _sensors[i];
		pub->msg_.header.stamp	  = time;
		pub->msg_.header.frame_id = sensor.getFrameId();

		pub->msg_.wrench.force.x  = sensor.getForce()[0];
		pub->msg_.wrench.force.y  = sensor.getForce()[1];
		pub->msg_.wrench.force.z  = sensor.getForce()[2];
		pub->msg_.wrench.torque.x = sensor.getTorque()[0];
		pub->msg_.wrench.torque.y = sensor.getTorque()[1];
		pub->msg_.wrench.torque.z = sensor.getTorque()[2];

		pub->unlockAndPublish();
	    }
	}

	++i;
    }
}

void
ForceTorqueSensorController::stopping(const ros::Time& /*time*/)
{
}

/************************************************************************
*  class ForceTorqueSensorController::Sensor				*
************************************************************************/
void
ForceTorqueSensorController::Sensor::update(const ros::Time& time)
{
    if (_pub_rate > 0.0 &&
	_last_pub_time + ros::Duration(1.0/_pub_rate) < time)
    {
      // try to publish
	if (_pub->trylock())
	{
	  // we're actually publishing, so increment time
	    last_pub_time = last_pub_time + ros::Duration(1.0/_pub_rate);

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

}

PLUGINLIB_EXPORT_CLASS(aist_ftsensor::ForceTorqueSensorController,
		       controller_interface::ControllerBase)
