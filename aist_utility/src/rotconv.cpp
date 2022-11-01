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
#include <cctype>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <aist_utility/tf.h>

namespace aist_utility
{
constexpr tfScalar	RAD = M_PI/180.0;
constexpr tfScalar	DEG = 180.0/M_PI;

static void
fromQuaternion(const tf::Quaternion& q)
{
    const tf::Matrix3x3	rot(q);
    tfScalar		roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);
    tf::Vector3		rpy(roll, pitch, yaw);

    rpy *= DEG;

    std::cerr << "--- rot ---\n" << rot << std::endl;
    std::cerr << "--- rpy(deg.) ---\n" << rpy << std::endl;
}

static void
fromRPY(const tf::Vector3& rpy_in_degree)
{
    const auto		rpy = rpy_in_degree * RAD;
    tf::Quaternion	q;
    q.setRPY(rpy.x(), rpy.y(), rpy.z());
    const tf::Matrix3x3	rot(q);

    std::cerr << "--- quaternion ---\n" << q << std::endl;
    std::cerr << "--- rot ---\n" << rot << std::endl;
}

static void
fromRot(const tf::Matrix3x3& rot)
{
    tfScalar	roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);

    tf::Vector3		rpy(roll, pitch, yaw);
    tf::Quaternion	q;
    q.setRPY(rpy.x(), rpy.y(), rpy.z());

    rpy *= DEG;

    std::cerr << "--- rpy(deg.) ---\n" << rpy << std::endl;
    std::cerr << "--- quaternion ---\n" << q << std::endl;
}

}

int
main()
{
    std::cerr << std::setiosflags(std::ios_base::fixed);

    for (;;)
    {
	using namespace	aist_utility;

	std::vector<tfScalar>	values;
	std::cerr << "\n(specify three/four/nine parameters)>> ";
	for (char c; std::cin.get(c); )
	    if (c =='\n')
		break;
	    else
	    {
		std::cin.unget();
		tfScalar	value;
		std::cin >> value;
		values.push_back(value);
	    }

	switch (values.size())
	{
	  case 3:
	    fromRPY(tf::Vector3(values[0], values[1], values[2]));
	    break;
	  case 4:
	    fromQuaternion(tf::Quaternion(values[0], values[1],
					  values[2], values[3]));
	    break;
	  case 9:
	    fromRot(tf::Matrix3x3(values[0], values[1], values[2],
				  values[3], values[4], values[5],
				  values[6], values[7], values[8]));
	    break;
	  default:
	    std::cerr << "Specify three/four/nine values!" << std::endl;
	    break;
	}
    }
}
