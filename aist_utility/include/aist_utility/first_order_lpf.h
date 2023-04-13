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
 *  \file	first_order_lpf.h
 *  \author	Toshio Ueshiba
 *  \brief	First order low-pass filter
 */
#pragma once

#include <cmath>
#include <vector>

namespace aist_utility
{
/*!
  Butterworth low-pass filter of even order
  \param S	type of coefficients
  \param T	type of signal to be filtered
*/
template <class S, class T=S>
class FirstOrderLPF
{
  public:
    using element_type	= S;	//!< type of coefficients
    using value_type	= T;	//!< type of signal to be filtered

  public:
  /*!
    1階ローパスフィルタを生成
    \param decay	カットオフ周波数fcをサンプリング周波数fsで正規化した値
			すなわちfc/fs
  */
    explicit	FirstOrderLPF(element_type decay)
		{
		    initialize(decay);
		}

    void	initialize(element_type decay)
		{
		    _decay = decay;
		}

    element_type
		decay() const
		{
		    return _decay;
		}

    value_type	filter(const value_type& x) const
		{
		    _y1 = (1 - _decay)*x + _decay*_y1;

		    return _y1;
		}

    void	reset(const value_type& x)
		{
		    _y1 = x;
		}

  private:
    element_type	_decay;
    mutable value_type	_y1;		// output value in previous step
};

}  // namespace moveit_servo
