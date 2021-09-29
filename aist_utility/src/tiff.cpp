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
/*!
 *  \file	tiff.cpp
 *  \author	Toshio UESHIBA
 *  \brief	Save/Restore sensor_msgs::Image to/from TIFF file
 */
#include <sensor_msgs/image_encodings.h>
#include <tiffio.h>
#include <aist_utility/tiff.h>

namespace aist_utility
{
/************************************************************************
*  global functions to save/load TIFF images				*
************************************************************************/
void
saveTiff(const sensor_msgs::Image& image, const std::string& file)
{
    using	namespace sensor_msgs;

    TIFF* const	tiff = TIFFOpen(file.c_str(), "w");
    if (!tiff)
	throw std::runtime_error("saveTiff(): cannot open file["
				 + file + ']');

    const auto	bitsPerSample	= image_encodings::bitDepth(image.encoding);
    const auto	samplesPerPixel = image_encodings::numChannels(image.encoding);
    const auto	photometric	= (image_encodings::isColor(image.encoding) ?
				   PHOTOMETRIC_RGB : PHOTOMETRIC_MINISBLACK);

    TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH,	image.width);
    TIFFSetField(tiff, TIFFTAG_IMAGELENGTH,	image.height);
    TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE,	bitsPerSample);
    TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL,	samplesPerPixel);
    TIFFSetField(tiff, TIFFTAG_ROWSPERSTRIP,	1);
    TIFFSetField(tiff, TIFFTAG_COMPRESSION,	COMPRESSION_NONE);
    TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC,	photometric);
    TIFFSetField(tiff, TIFFTAG_FILLORDER,	FILLORDER_MSB2LSB);
    TIFFSetField(tiff, TIFFTAG_PLANARCONFIG,	PLANARCONFIG_CONTIG);
    TIFFSetField(tiff, TIFFTAG_ORIENTATION,	ORIENTATION_TOPLEFT);
    TIFFSetField(tiff, TIFFTAG_XRESOLUTION,	72.0);
    TIFFSetField(tiff, TIFFTAG_YRESOLUTION,	72.0);
    TIFFSetField(tiff, TIFFTAG_RESOLUTIONUNIT,	RESUNIT_INCH);

    for (int n = 0, offset = 0; n < image.height; ++n)
    {
	TIFFWriteEncodedStrip(tiff, n, image.data.data() + offset, image.step);
	offset += image.step;
    }

    TIFFClose(tiff);
}

sensor_msgs::ImagePtr
loadTiff(const std::string& file)
{
    using	namespace sensor_msgs;

    TIFF* const	tiff = TIFFOpen(file.c_str(), "r");
    if (!tiff)
	throw std::runtime_error("loadTiff(): cannot open file[" + file + ']');

    uint32	width, height;
    uint16	bitsPerSample, samplesPerPixel, photometric;

    if (!TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH,	     &width)		||
	!TIFFGetField(tiff, TIFFTAG_IMAGELENGTH,     &height)		||
	!TIFFGetField(tiff, TIFFTAG_BITSPERSAMPLE,   &bitsPerSample)	||
	!TIFFGetField(tiff, TIFFTAG_SAMPLESPERPIXEL, &samplesPerPixel)	||
	!TIFFGetField(tiff, TIFFTAG_PHOTOMETRIC,     &photometric))
    {
	throw std::runtime_error("loadTiff(): cannot get necessary fields");
    }

    sensor_msgs::ImagePtr	image(new sensor_msgs::Image());
    image->width  = width;
    image->height = height;

    switch (photometric)
    {
      case PHOTOMETRIC_MINISBLACK:
	switch(bitsPerSample)
	{
	  case 8:
	    image->encoding = image_encodings::MONO8;
	    break;
	  case 16:
	    image->encoding = image_encodings::MONO16;
	    break;
	  case 32:
	    switch (samplesPerPixel)
	    {
	      case 1:
		image->encoding = image_encodings::TYPE_32FC1;
		break;
	      case 2:
		image->encoding = image_encodings::TYPE_32FC2;
		break;
	      case 3:
		image->encoding = image_encodings::TYPE_32FC3;
		break;
	      case 4:
		image->encoding = image_encodings::TYPE_32FC4;
		break;
	      default:
		throw std::runtime_error("loadTiff(): unsupported #samples per pixel["
					 + std::to_string(samplesPerPixel)
					 + "] under #bits per sample["
					 + std::to_string(bitsPerSample)
					 + ']');
	    }
	    break;
	  case 64:
	    switch (samplesPerPixel)
	    {
	      case 1:
		image->encoding = image_encodings::TYPE_64FC1;
		break;
	      case 2:
		image->encoding = image_encodings::TYPE_64FC2;
		break;
	      case 3:
		image->encoding = image_encodings::TYPE_64FC3;
		break;
	      case 4:
		image->encoding = image_encodings::TYPE_64FC4;
		break;
	      default:
		throw std::runtime_error("loadTiff(): unsupported #samples per pixel["
					 + std::to_string(samplesPerPixel)
					 + "] under #bits per sample["
					 + std::to_string(bitsPerSample)
					 + ']');
	    }
	    break;
	  default:
	    throw std::runtime_error("loadTiff(): unsupported #bits per sample["
				     + std::to_string(bitsPerSample) + ']');
	}
	break;

      case PHOTOMETRIC_RGB:
	switch(bitsPerSample)
	{
	  case 8:
	    image->encoding = image_encodings::RGB8;
	    break;
	  case 16:
	    image->encoding = image_encodings::RGB16;
	    break;
	  default:
	    throw std::runtime_error("loadTiff(): unsupported #bits per sample["
				     + std::to_string(bitsPerSample) + ']');
	}
	break;

      default:
	throw std::runtime_error("loadTiff(): unsupported photometic["
				 + std::to_string(photometric) + ']');
    }

    image->step		= image->width * bitsPerSample * samplesPerPixel / 8;
    image->is_bigendian = false;
    image->data.resize(image->height * image->step);

    const auto	nBytesPerStrip = TIFFStripSize(tiff);
    const auto	nStrips        = TIFFNumberOfStrips(tiff);

    for (int n = 0, offset = 0; n < nStrips; ++n)
    {
	const auto nBytes = TIFFReadEncodedStrip(tiff, n,
						 image->data.data() + offset,
						 nBytesPerStrip);
	if (nBytes < 0)
	    throw std::runtime_error("loadTiff(): failed to read strip");

	offset += nBytes;
    }

    TIFFClose(tiff);

    return image;
}

}	// namespace aist_utility
