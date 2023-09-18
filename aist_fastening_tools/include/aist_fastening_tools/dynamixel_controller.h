/*!
  \file	 dynamixel_controller.h
*/
#pragma once

#include <ros/ros.h>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_workbench_toolbox/dynamixel_driver.h>
#include <dynamixel_workbench_msgs/GetDynamixelInfo.h>

#include "aist_fastening_tools/message_header.h"
#include "aist_fastening_tools/DynamixelReadState.h"
#include "aist_fastening_tools/DynamixelWriteCommand.h"

namespace aist_fastening_tools
{
class DynamixelController
{
  public:
		DynamixelController(ros::NodeHandle& nh)		;

  private:
    bool	initMotor(DynamixelDriver& driver, uint8_t motor_id,
			  bool baudrate_flag)				;
    int32_t	searchMotor(uint8_t motor_id);

    bool	write_command_cb(DynamixelWriteCommand::Request &req,
				 DynamixelWriteCommand::Response &res)	;
    bool	read_state_cb(DynamixelReadState::Request &req,
			      DynamixelReadState::Response &res)	;

  private:
    ros::ServiceServer					_read_state_srv;
    ros::ServiceServer					_write_command_srv;

    std::vector<std::unique_ptr<DynamixelDriver> >	_drivers;
    std::vector<std::string>				_u2d2_connect_ids;
};
}	// namespace aist_fastening_tools
