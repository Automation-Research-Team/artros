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
 *  \file	ply.cpp
 *  \author	Toshio Ueshiba
 *  \brief	Save depth and color images to Ordered PLY file
 */
#include <array>
#include <boost/iterator/iterator_adaptor.hpp>
#include <sensor_msgs/image_encodings.h>
#include <aist_utility/ply.h>
#include <aist_utility/sensor_msgs.h>
#include "oply/OrderedPly.h"

namespace aist_utility
{
/************************************************************************
*  class assignment_iterator<ITER>					*
************************************************************************/
namespace detail
{
  template <class ITER>
  class assignment_proxy
  {
    public:
      using value_type = typename std::iterator_traits<ITER>::value_type;

    public:
      assignment_proxy(ITER iter) :_iter(iter)	{}

      template <class T_>
      const auto&	operator =(const T_& val) const
			{
			    assign(*_iter, val);
			    return *this;
			}

    private:
      template <class T_>
      static void	assign(T_& dst, const T_& val)
			{
			    dst = val;
			}
      static void	assign(float& meters, uint16_t milimeters)
			{
			    meters = 0.001f * milimeters;
			}
      static void	assign(float& grey, const std::array<uint8_t, 3>& rgb)
			{
			    grey = 0.3f*rgb[0] + 0.59f*rgb[1] + 0.11f*rgb[2];
			}
      static void	assign(std::array<uint8_t, 3>& rgb, uint8_t grey)
      			{
      			    rgb[2] = rgb[1] = rgb[0] = grey;
      			}

    private:
      const ITER 	_iter;
  };
}	// namespace detail

template <class ITER>
class assignment_iterator
    : public boost::iterator_adaptor<assignment_iterator<ITER>,
				     ITER,
				     boost::use_default,
				     boost::use_default,
				     detail::assignment_proxy<ITER> >
{
  public:
    using super = boost::iterator_adaptor<assignment_iterator<ITER>,
					  ITER,
					  boost::use_default,
					  boost::use_default,
					  detail::assignment_proxy<ITER> >;
    using	typename super::reference;
    friend	class boost::iterator_core_access;

  public:
    assignment_iterator(ITER iter) :super(iter)	{}

  private:
    reference	dereference()		const	{ return {super::base()}; }
};

template <class ITER> assignment_iterator<ITER>
make_assignment_iterator(ITER iter)
{
    return assignment_iterator<ITER>(iter);
}

/************************************************************************
*  static functions							*
************************************************************************/
template <class T, class ITER> static ITER
copy_image(const sensor_msgs::Image& image, ITER out)
{
    for (int v = 0; v < image.height; ++v)
	out = std::copy_n(ptr<T>(image, v), image.width, out);

    return out;
}

/************************************************************************
*  Save depth and color images to Ordered PLY file			*
************************************************************************/
void
savePly(const sensor_msgs::CameraInfo& camera_info,
	const sensor_msgs::Image& image,  const sensor_msgs::Image& depth,
	const sensor_msgs::Image& normal, const std::string& file)
{
    using	namespace sensor_msgs;

    OrderedPly	oply;

  // Set version
    oply.version = PC_VER_1_2;

  // Set number of points.
    oply.size = depth.height * depth.width;
    oply.last = oply.size;

  // Set point coordinates and depth values.
    oply.point.resize(oply.size);
    oply.depth.resize(oply.size);
    if (depth.encoding == image_encodings::MONO16 ||
	depth.encoding == image_encodings::TYPE_16UC1)
    {
	depth_to_points<uint16_t>(camera_info, depth, oply.point.begin(),
				  milimeters<uint16_t>);
	copy_image<uint16_t>(depth,
			     make_assignment_iterator(oply.depth.begin()));
    }
    else if (depth.encoding == image_encodings::TYPE_32FC1)
    {
	depth_to_points<float>(camera_info, depth, oply.point.begin(),
			       milimeters<float>);
	copy_image<float>(depth, make_assignment_iterator(oply.depth.begin()));
    }
    else
	throw std::runtime_error("savePly(): unknown depth encoding["
				 + depth.encoding + ']');

  // Set normals (REQUIRED for Photoneo Localization).
    oply.normal.resize(oply.size);
    copy_image<std::array<float, 3> >(normal, oply.normal.begin());

  // Set color and texture values.
    oply.color.resize(oply.size);
    oply.texture.resize(oply.size);
    if (image.encoding == image_encodings::MONO8 ||
	image.encoding == image_encodings::TYPE_8UC1)
    {
	copy_image<uint8_t>(image,
			    make_assignment_iterator(oply.color.begin()));
	copy_image<uint8_t>(image,
			    make_assignment_iterator(oply.texture.begin()));
    }
    else if (image.encoding == image_encodings::RGB8 ||
	     image.encoding == image_encodings::TYPE_8UC3)
    {
	copy_image<std::array<uint8_t, 3> >(
	    image, make_assignment_iterator(oply.color.begin()));
	copy_image<std::array<uint8_t, 3> >(
	    image, make_assignment_iterator(oply.texture.begin()));
    }
    else
	throw std::runtime_error("savePly(): unknown image encoding["
				 + image.encoding + ']');

  // Empty confidence values (Not required for Photoneo Localization).
    oply.confidence.clear();

  // Set position and orientation of the camera frame.
    oply.view   = {0.0, 0.0, 0.0};
    oply.x_axis = {1.0, 0.0, 0.0};
    oply.y_axis = {0.0, 1.0, 0.0};
    oply.z_axis = {0.0, 0.0, 1.0};

  // Set frame size.
    oply.frame_width  = depth.width;
    oply.frame_height = depth.height;
    oply.frame_index  = 0;

  // Copy camera parameters.
    std::copy_n(std::begin(camera_info.K), 9, std::begin(oply.cm));
    std::copy_n(std::begin(camera_info.D), 5, std::begin(oply.dm));

  // Other parameters.
    oply.width	    = depth.width;
    oply.height	    = depth.height;
    oply.horizontal = 1;
    oply.vertical   = 1;

  // Write Ordered PLY to the spceified file.
    OPlyWriter	writer(file, oply);
    writer.write();
}

}	// namespace aist_utility
