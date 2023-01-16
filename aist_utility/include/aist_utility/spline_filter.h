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

#include <cmath>
#include <vector>

namespace aist_utility
{
/*!
  Butterworth low-pass filter of even order
  \param T	type of signal to be filtered
*/
template <class T>
class CubicSplineFilter
{
  public:
    using value_type	= T;	//!< type of signal to be filtered

  public:
  /*!
    バタワースフィルタを生成
    \param half_order	フィルタの次数の半分
    \param cutoff	カットオフ周波数fcをサンプリング周波数fsで正規化した値
			すなわちfc/fs
   */
    explicit	CubicSplineFilter()
		    :_a0(0), _a1(0), _a2(0), _a3(0), _tp(0), _xp(0)
		{
		    initialize();
		}

    void	initialize()
		{
		    _a0 = 0;
		    _a1 = 0;
		    _
		}

    void	update(value_type t, value_type x)
		{
		    _a0 = pos(t);
		    _a1 = vel(t);

		    const auto	dt_inv = 1/(t - _tp);
		    const auto	v = (x - _xp)*dt_inv;
		    
		    _a2 = (-(v + 2*_a1) + 3*(x - _a0)*dt_inv)*dt_inv;
		    _a3 = ( (v +   _a1) - 2*(x - _a0)*dt_inv)*dt_inv*dt_inv;
		    
		    _tp = t;
		    _xp = x;
		}
    
    value_type	pos(value_type t) const
		{
		    const auto	dt = t - _tp;

		    return _a0 + (_a1 + (_a2 + _a3*dt)*dt)*dt;
		}
    
    value_type	vel(value_type t) const
		{
		    const auto	dt = t - _tp;

		    return _a1 + (2*_a2 + 3*_a3*dt)*dt;
		}
    
    value_type	acc(value_type t) const
		{
		    const auto	dt = t - t0;

		    return 2*(_a2 + 3*_a3*dt);
		}
    
    void	reset(value_type t, value_type x)
		{
		    _a0 = x;
		    _a1 = 0;
		    _a2 = 0;
		    _a3 = 0;
		    _tp = t;
		    _xp = x;
		}

  private:
    value_type	_a0, _a1, _a2, _a3;
    value_type	_tp;
    value_type	_xp;
};

}  // namespace moveit_servo
