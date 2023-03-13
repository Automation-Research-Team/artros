/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/
/*      Title     : servo_server.cpp
 *      Project   : aist_moveit_servo
 *      Created   : 12/31/2018
 *      Author    : Andy Zelenak, Toshio Ueshiba
 */
#include <aist_moveit_servo/servo.h>

namespace aist_moveit_servo
{
/************************************************************************
*  class ServoServer							*
************************************************************************/
class ServoServer : public Servo
{
  public:
		ServoServer(const ros::NodeHandle& nh,
			    const std::string& robot_description,
			    const std::string& logname)			;

    void	run()							;

  private:
    void	twistCmdCB(const twist_cp& twist_cmd)			;
    twist_t	twistCmd()					const	;

  private:
    ros::NodeHandle		nh_;
    const ros::Subscriber	twist_cmd_sub_;
    twist_cp			twist_cmd_;
    mutable std::mutex		twist_mtx_;
};

ServoServer::ServoServer(const ros::NodeHandle& nh,
			 const std::string& robot_description,
			 const std::string& logname)
    :Servo(nh, robot_description, logname),
     nh_(nh),
     twist_cmd_sub_(nh_.subscribe("delta_twist_cmds", 1,
				  &ServoServer::twistCmdCB, this)),
     twist_cmd_(new twist_t()),
     twist_mtx_()
{
    ROS_INFO_STREAM_NAMED(logname, "(ServoServer) server started");
}

void
ServoServer::run()
{
    ros::AsyncSpinner	spinner(8);
    spinner.start();

    for (ros::Rate rate(1.0/servoParameters().publish_period);
	 ros::ok(); rate.sleep())
    {
	update();
	publishTrajectory(twistCmd(), nullptr);
    }

    ros::waitForShutdown();	// Wait for ros to shutdown
}

void
ServoServer::twistCmdCB(const twist_cp& twist_cmd)
{
    const std::lock_guard<std::mutex>	lock(twist_mtx_);

    twist_cmd_ = twist_cmd;
}

ServoServer::twist_t
ServoServer::twistCmd() const
{
    const std::lock_guard<std::mutex>	lock(twist_mtx_);

    return *twist_cmd_;
}
}	// namespace aist_moveit_servo

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    using namespace	aist_moveit_servo;

    constexpr char	LOGNAME[] = "servo_server";

    ros::init(argc, argv, LOGNAME);

    ServoServer	servo_server(ros::NodeHandle("~"), "robot_description",
			     LOGNAME);
    servo_server.run();		// Start the servo server in the ros spinner

    return 0;
}
