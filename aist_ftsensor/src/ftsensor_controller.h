/*!
 *  \file	ftsensor_controller.h
 *  \brief	header of a ROS node class for controlling force sensors
 */
#pragma once

#include <controller_interface/controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>

namespace aist_ftsensor
{
/************************************************************************
*  class ForceTorqueSensorController					*
************************************************************************/
// this controller gets access to the ForceTorqueSensorInterface
class ForceTorqueSensorController
    : public controller_interface::Controller<
		 hardware_interface::ForceTorqueSensorInterface>
{
  private:
    using hw_interface_t = hardware_interface::ForceTorqueSensorInterface;
    using hw_handle_t	 = hardware_interface::ForceTorqueSensorHandle;
    using publisher_t	 = realtime_tools::RealtimePublisher<
				geometry_msgs::WrenchStamped>;
    using publisher_p	 = std::shared_ptr<publisher_t>;

    class Sensor
    {
      public:
		Sensor()						{}

	void	init();
	void	update(const ros::Time& time,
		       const ros::Duration& /*period*/)			;
      private:
	hw_handle_t	_sensor;
	publisher_p	_pub;
	ros::Time	_last_pub_time;
    };

  public:
			ForceTorqueSensorController()			{}

    virtual bool	init(hw_interface_t* hw,
			     ros::NodeHandle &root_nh,
			     ros::NodeHandle& controller_nh)		;
    virtual void	starting(const ros::Time& time)			;
    virtual void	update(const ros::Time& time,
			       const ros::Duration& /*period*/)		;
    virtual void	stopping(const ros::Time& /*time*/)		;

  private:
    std::vector<Sensor>	_sensors;
    double		_pub_rate;
};

}	// namespace aist_ftsensor
