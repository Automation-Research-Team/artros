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
 *  \file	tf2.h
 *  \author	Toshio Ueshiba
 *  \brief	Utilities
 */
#pragma once

#include <tf2/transform_datatypes.h>
#include <tf2/Transform.h>

namespace aist_utility
{
inline std::ostream&
operator <<(std::ostream& out, const tf2::Vector3& v)
{
    return out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const tf2::Quaternion& q)
{
    return out << '[' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w()
	       << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const tf2::Matrix3x3& m)
{
    return out << '[' << m[0] << "\n " << m[1] << "\n " << m[2] << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const tf2::Transform& transform)
{
    return out << '['  << transform.getOrigin()
	       << "; " << transform.getRotation() << ']';
}

template <class T> inline std::ostream&
operator <<(std::ostream& out, const tf2::Stamped<T>& stamped)
{
    return out << static_cast<const tf::T&>(stamped)
	       << '@' << stamped.frame_id_;
}

}	// namespace aist_utility
