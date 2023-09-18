/*!
  \file	 dynamixel_controller.cpp
*/
#include <sstream>
#include <fstream>
#include "aist_fastening_tools/dynamixel_controller.h"

namespace aist_fastening_tools
{
/************************************************************************
*  static functions							*
************************************************************************/
std::vector<std::string>
split_string(const std::string& str, char sep)
{
    std::vector<std::string>	v;
    std::stringstream		ss(str);
    std::string			buffer;
    while (std::getline(ss, buffer, sep))
	v.push_back(buffer);

    return v;
}

/************************************************************************
*  class DynamixelController						*
************************************************************************/
DynamixelController::DynamixelController(ros::NodeHandle& nh)
    :_read_state_srv(nh.advertiseService(
			 "aist_fastening_tools/dynamixel_read_state",
			 &DynamixelController::read_state_cb, this)),
     _write_command_srv(nh.advertiseService(
			    "aist_fastening_tools/dynamixel_write_command",
			    &DynamixelController::write_command_cb, this)),
     _drivers(),
     _u2d2_connect_ids()
{
  // Enumerate serial ports.
    std::vector<std::string>	access_points;
    const auto			num_controllers
				    = nh.param<int>("num_controllers", 1);

    for (int i = 0; i < num_controllers; ++i)
    {
	auto	access_point = "serial_port_" + std::to_string(i);
	ROS_INFO_STREAM("Trying to find controller connected on port "
			<< access_point);
	if (nh.getParam(access_point, access_point))
	    access_points.push_back(access_point);
	else
	{
	    ROS_ERROR("More controllers requested via num_controllers parameter than "
		      "exist in config file. (%s)",
		      access_point.c_str());
	    return;
	}
    }

  // Allocate pointers to drivers.
    _drivers.resize(num_controllers);

  // init and search for XL-320
    for (int idx = 0; idx < access_points.size(); ++idx)
    {
	_u2d2_connect_ids.push_back("");

	auto&	access_point	= access_points[idx];
	auto&	u2d2_connect_id	= _u2d2_connect_ids.back();

	if (std::ifstream ifs(access_point); ifs.is_open())
	{
	    ROS_INFO_STREAM("Trying to connect to controller: "
			    << access_point);

	    auto&	driver = _drivers[idx];
	    driver.reset(new DynamixelDriver);

	    bool	exist_flag = false;
	    for (auto baudrate : {9600, 57600, 115200, 1000000})
	    {

		driver->init(access_point.c_str(), baudrate);

		std::array<uint8_t, 10>	id_list;
		uint8_t			id_cnt = 0;
		if (driver->scan(id_list.data(), &id_cnt, id_list.size()))
		{
		    for (int i = 0; i < id_cnt; ++i)
		    {
			if (i == 0)
			    ROS_INFO("Connection to controller was found. (%s)",
				     access_point.c_str());
			ROS_INFO("[ID] %u, [Model Name] %s, [VERSION] %.1f",
				 id_list[i],
				 driver->getModelName(id_list[i]),
				 driver->getProtocolVersion());

		      // Initial Setting(XL320)
			if (initMotor(*driver,
				      id_list[i], baudrate != 1000000))
			{
			    if (!u2d2_connect_id.empty())
				u2d2_connect_id += ',';
			    u2d2_connect_id += std::to_string(id_list[i]);
			}
		    }

		    exist_flag = true;
		    break;
		}
	    }
	    if (!exist_flag)
	    {
		ROS_INFO("Connection to controller was found. (%s)",
			 access_point.c_str());
		ROS_ERROR("But no XL-320 motors seem to be connected.");
	    }
	}
	else
	{
	    ROS_WARN("No exist File (%s)", access_point.c_str());
	    access_point = "";
	}
    }
}

bool
DynamixelController::initMotor(DynamixelDriver& driver,
			       uint8_t motor_id, bool baudrate_flag)
{
    if (baudrate_flag)
    {
	if (!driver.writeRegister(motor_id, "Baud_Rate", 1000000))
	{
	    ROS_ERROR("The value of Baud_Rate could not be set to 1000000.");
	    return false;
	}
    }

    if (!driver.writeRegister(motor_id, "Torque_Enable", 0))
    {
	ROS_ERROR("The value of Torque_Enable could not be set to 0.");
	return false;
    }
    if (!driver.writeRegister(motor_id, "Control_Mode", 1))
    {
	ROS_ERROR("The value of Control_Mode could not be set to 1.");
	return false;
    }
    if (!driver.writeRegister(motor_id, "Torque_Enable", 1))
    {
	ROS_ERROR("The value of Torque_Enable could not be set to 1.");
	return false;
    }
  // if (driver.writeRegister(motor_id, "Torque_Limit", 1023))
  // {
  // 	ROS_ERROR("The value of Torque_Limit could not be set to 1023.");
  // 	return false;
  // }

    return true;
}

int
DynamixelController::searchMotor(uint8_t motor_id)
{
    for (int idx = 0; idx < _u2d2_connect_ids.size(); ++idx)
    {
	const auto&	u2d2_connect_id = _u2d2_connect_ids[idx];

	if (!u2d2_connect_id.empty())
	{
	    const auto	ids = split_string(u2d2_connect_id, ',');

	    for (const auto& id : ids)
		if (std::stoi(id) == motor_id)
		    return idx;
	}
    }

    return -1;
}

bool
DynamixelController::read_state_cb(DynamixelReadState::Request&  req,
				   DynamixelReadState::Response& res)
{
    const auto	motor_idx = searchMotor(req.motor_id);
    if (motor_idx == -1)
    {
	ROS_ERROR("The specified Motor_ID (%d) can not be found.",
		  req.motor_id);
	res.success = false;
    }
    else
    {
	int32_t	state = 0;

	if (_drivers[motor_idx]->readRegister(req.motor_id,
					      req.item_name.c_str(), &state))
	{
	    res.success = true;
	    res.value	= state;
	}
	else
	    res.success = false;
    }

    return true;
}

bool
DynamixelController::write_command_cb(DynamixelWriteCommand::Request&  req,
				      DynamixelWriteCommand::Response& res)
{
    res.success = true;

    const auto	motor_idx = searchMotor(req.motor_id);
    if (motor_idx == -1)
    {
	ROS_ERROR("The specified Motor_ID (%d) can not be found.",
		  req.motor_id);
	res.success = false;
    }
    else
    {
	if (_drivers[motor_idx]->writeRegister(req.motor_id,
					       req.item_name.c_str(),
					       req.value))
	    res.success = true;
	else
	    res.success = false;
    }

    return true;
}
}	// namespace aist_fastening_tools

int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "dynamixel_controller");

    ros::NodeHandle				nh("~");
    aist_fastening_tools::DynamixelController	controller(nh);

    for (ros::Rate loop_rate(1000); ros::ok(); )
    {
	ros::spinOnce();
	loop_rate.sleep();
    }

    return 0;
}
