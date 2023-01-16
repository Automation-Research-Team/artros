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
 *  \file	spline_filter.h
 *  \author	Toshio Ueshiba
 *  \brief	Butterworth low-pass filter of even order
 */
#pragma once

#include <ros/ros.h>

namespace aist_utility
{
template <class T> T	zero(T)				{ return 0; }

template <class T, class T=S>
class CubicSplineFilter
{
  public:
    using time_type	= ros::Time;
    using value_type	= T;		//!< type of signal to be filtered

  public:
    explicit	CubicSplineFilter()
		{
		    reset(ros::Time(0), zero(value_type()));
		}

    void	reset(const time_type& t, const value_type& x)
		{
		    _a0 = x;
		    _a3 = _a2 = _a1 = zero(value_type());
		    _tp = t;
		    _xp = x;
		}

    void	update(const time_type& t, const value_type& x)
		{
		    _a0 = pos(t);			// position at t
		    _a1 = vel(t);			// velocity at t

		    const auto	dt_inv = 1/(t - _tp).toSec();
		    const auto	v = (x - _xp)*dt_inv;	// velocity at _tp

		    _a2 =  dt_inv	 *(-(v+2*_a1) + (3*dt_inv)*(x-_a0));
		    _a3 = (dt_inv*dt_inv)*( (v+  _a1) - (2*dt_inv)*(x-_a0));

		    _tp = t;
		    _xp = x;
		}

    value_type	pos(const time_type& t) const
		{
		    const auto	dt = (t - _tp).toSec();

		    return _a0 + dt*(_a1 + dt*(_a2 + dt*_a3));
		}

    value_type	vel(const time_type& t) const
		{
		    const auto	dt = (t - _tp).toSec();

		    return _a1 + dt*(2*_a2 + (3*dt)*_a3);
		}

    value_type	acc(const time_type& t) const
		{
		    const auto	dt = (t - t0).toSec();

		    return 2*(_a2 + (3*dt)*_a3);
		}

  private:
    value_type	_a0, _a1, _a2, _a3;
    time_type	_tp;
    value_type	_xp;
};

}  // namespace moveit_servo
