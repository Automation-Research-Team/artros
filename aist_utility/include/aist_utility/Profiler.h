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
 *  \file	Profiler.h
 *  \author	Toshio Ueshiba
 *  \brief	Utilities
 */
#pragma once

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <iomanip>

namespace aist_utility
{
/************************************************************************
*  class Profiler							*
************************************************************************/
class Profiler
{
  public:
    using time_t	= ros::Time;
    using duration_t	= ros::Duration;

  public:
		Profiler()
		    :_accums(), _t0(0), _stamp_idx(0), _nframes(0)	{}

    void	reset(const time_t& t=ros::Time::now())
		{
		    _accums.clear();
		    _t0	       = t;
		    _stamp_idx = 0;
		    _nframes   = 0;
		}

    void	start(const time_t& t=ros::Time::now())
		{
		    _t0	       = t;
		    _stamp_idx = 0;
		}

    void	stamp()
		{
		    if (_accums.size() > 0)	// initialized?
		    {
			if (_stamp_idx >= _accums.size())
			    throw std::runtime_error("Profiler::stamp(): run out of stamp buffer");
			_accums[_stamp_idx] += (ros::Time::now() - _t0);
		    }

		    ++_stamp_idx;
		}

    void	stop()
		{
		    stamp();

		    if (_accums.size() == 0)
			_accums.resize(_stamp_idx);
		    else
			++_nframes;
		}

    void	print(std::ostream& out) const
		{
		    if (_nframes == 0)
			return;
		    
		    for (size_t i = 0; i < _accums.size(); ++i)
			print(out, lap_time(i));
		    out << '|';
		    print(out, total_time());
		}

  private:
    void	print(std::ostream& out, double sec) const
		{
		    out << std::setw(6) << 1000.0*sec << "ms("
			<< std::setw(4) << 1.0/sec << "fps)";
		}

    double	lap_time(size_t i) const
		{
		    return (i == 0 ? _accums[0].toSec()/_nframes :
			    (_accums[i] - _accums[i-1]).toSec()/_nframes);
		}

    double	total_time() const
		{
		    return _accums.back().toSec()/_nframes;
		}

  private:
    std::vector<duration_t>	_accums;
    time_t			_t0;
    size_t			_stamp_idx;
    size_t			_nframes;
};

inline std::ostream&
operator <<(std::ostream& out, const Profiler& profiler)
{
    profiler.print(out);

    return out;
}
}	// namespace aist_utility
