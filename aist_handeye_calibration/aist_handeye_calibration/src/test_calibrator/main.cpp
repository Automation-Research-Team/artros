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
#include <cstdlib>	// for std::getenv()
#include <aist_utility/geometry_msgs.h>
#include "HandeyeCalibration.h"

namespace TU
{
template <class T> void
doJob(const std::string& camera_name, bool single)
{
    using	aist_utility::operator >>;

  // Load YAML containing sampled transformations.
    const auto		calib_file = std::string(getenv("HOME"))
				   + "/.ros/aist_handeye_calibration/"
				   + camera_name + ".yaml";
    const YAML::Node	node = YAML::LoadFile(calib_file);
    const auto		eye_on_hand = node["eye_on_hand"].as<bool>();

    std::vector<geometry_msgs::TransformStamped>	Tcm_msgs, Twe_msgs;
    node["Tcm"] >> Tcm_msgs;
    node["Twe"] >> Twe_msgs;

  // Convert geometry_msgs::TransformStamped to TU::Transform<T>.
    std::vector<Transform<T> >	Tcm, Twe;
    for (const auto& Tcm_msg : Tcm_msgs)
	Tcm.push_back(Tcm_msg.transform);
    for (const auto& Twe_msg : Twe_msgs)
	Twe.push_back(Twe_msg.transform);

  // Do calibration.
    const auto	Tec = (single ? cameraToEffectorSingle(Tcm, Twe)
			      : cameraToEffectorDual(Tcm, Twe));
    const auto	Twm = objectToWorld(Tcm, Twe, Tec);
    evaluateAccuracy(std::cout, Tcm, Twe, Tec, Twm);

  // Print calibration result.
    std::cout << "\n=== camera_transform ===\n" << Tec << std::endl
	      << "=== marker_transform ===\n" << Twm << std::endl << std::endl;
}
}	// namespace TU

int
main(int argc, char* argv[])
{
    bool	single = false;
    std::string	camera_name("camera");
    for (int c; (c = getopt(argc, argv, "sc:")) !=EOF; )
	switch (c)
	{
	  case 's':
	    single = true;
	    break;
	  case 'c':
	    camera_name = optarg;
	    break;
	}

    try
    {
	TU::doJob<double>(camera_name, single);
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what();
	return -1;
    }

    return 0;
}
