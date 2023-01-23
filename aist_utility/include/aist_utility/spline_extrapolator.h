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
 *  \file	spline_extrapolator.h
 *  \author	Toshio Ueshiba
 *  \brief	Spline extrapolator
 */
#pragma once

#include <ros/ros.h>

namespace aist_utility
{
template <class T> T	zero(T)				{ return 0; }

template <class T, size_t N=3>
class SplineExtrapolator
{
  public:
    using value_type	= T;		//!< type of signal to be interpolated

  public:
    explicit	SplineExtrapolator(const value_type& x=zero(value_type()))
		{
		    reset(ros::Time::now(), x);
		}

    void	reset(const ros::Time& t, const value_type& x)
		{
		    _a[0] = x;
		    std::fill(_a.begin() + 1, _a.end(), zero(value_type()));
		    _tp = t;
		    _xp = x;
		}

    const ros::Time&
		tp() const
		{
		    return _tp;
		}
	    
    const value_type&
		xp() const
		{
		    return _xp;
		}
	    
    void	update(const ros::Time& t, const value_type& x)
		{
		    update(std::integral_constant<size_t, N>(),
			   (t - _tp).toSec(), x - _xp);
		    _tp = t;
		    _xp = x;
		}

    value_type	pos(const ros::Time& t) const
		{
		    return pos(std::integral_constant<size_t, 0>(),
			       (t - _tp).toSec());
		}

    value_type	vel(const ros::Time& t) const
		{
		    return vel(std::integral_constant<size_t, 1>(),
			       (t - _tp).toSec());
		}

    value_type	acc(const ros::Time& t) const
		{
		    return acc(std::integral_constant<size_t, 2>(),
			       (t - _tp).toSec());
		}

  private:
    void	update(std::integral_constant<size_t, 2>,
		       double dt, const value_type& dx)
		{
		    const auto	rdt = 1/dt;
		    const auto	vp  = rdt*dx;

		    _a[0] = pos(std::integral_constant<size_t, 0>(), dt);
		    _a[1] =      -vp + 2*rdt*(_xp - _a[0]);
		    _a[2] = rdt*( vp -	 rdt*(_xp - _a[0]));
		}
    void	update(std::integral_constant<size_t, 3>,
		       double dt, const value_type& dx)
		{
		    const auto	rdt = 1/dt;
		    const auto	vp  = rdt*dx;

		    _a[0] = pos(std::integral_constant<size_t, 0>(), dt);
		    _a[1] = vel(std::integral_constant<size_t, 1>(), dt);
		    _a[2] =	rdt*(-(vp + 2*_a[1]) + 3*rdt*(_xp - _a[0]));
		    _a[3] = rdt*rdt*( (vp +   _a[1]) - 2*rdt*(_xp - _a[0]));
		}
    void	update(std::integral_constant<size_t, 4>,
		       double dt, const value_type& dx)
		{
		    const auto	rdt = 1/dt;
		    const auto	vp  = rdt*dx;

		    _a[0] = pos(std::integral_constant<size_t, 0>(), dt);
		    _a[1] = vel(std::integral_constant<size_t, 1>(), dt);
		    _a[2] = vel(std::integral_constant<size_t, 2>(), dt);
		    _a[3] =	rdt*(-2*_a[2] + rdt*(-(vp + 3*_a[1])
						     + 4*rdt*(_xp - _a[0])));
		    _a[4] = rdt*rdt*(   _a[2] + rdt*( (vp + 2*_a[1])
						     - 3*rdt*(_xp - _a[0])));
		}

    template <size_t N_>
    value_type	pos(std::integral_constant<size_t, N_>, double dt) const
		{
		    return _a[N_]
			 + dt*pos(std::integral_constant<size_t, N_+1>(), dt);
		}
    value_type	pos(std::integral_constant<size_t, N>, double dt) const
		{
		    return _a[N];
		}

    template <size_t N_>
    value_type	vel(std::integral_constant<size_t, N_>, double dt) const
		{
		    return N_*_a[N_]
			 + dt*vel(std::integral_constant<size_t, N_+1>(), dt);
		}
    value_type	vel(std::integral_constant<size_t, N>, double dt) const
		{
		    return N*_a[N];
		}

    template <size_t N_>
    value_type	acc(std::integral_constant<size_t, N_>, double dt) const
		{
		    return (N_*(N_-1))*_a[N_]
			 + dt*vel(std::integral_constant<size_t, N_+1>(), dt);
		}
    value_type	acc(std::integral_constant<size_t, N>, double dt) const
		{
		    return (N*(N-1))*_a[N];
		}

  private:
    std::array<value_type, N+1>	_a;
    ros::Time			_tp;
    value_type			_xp;
};

}  // namespace moveit_servo
