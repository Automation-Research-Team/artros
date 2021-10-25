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
  \file		binarize.h
  \brief	$BBgDE$N(B2$BCM2=%"%k%4%j%:%`$N<BAu(B
*/
#include <algorithm>

namespace TU
{
//! $BBgDE$N(B2$BCM2=%"%k%4%j%:%`$K$h$jM?$($i$l$?%G!<%?Ns$r(B2$BCM2=$9$k!%(B
/*!
  $B%G!<%?Ns$O>:=g$K%=!<%H$5$l!$ogCM$H$J$k%G!<%?$9$J$o$A8eH>It$N@hF,%G!<%?$,JV$5$l$k!%(B
  \param begin	$B%G!<%?Ns$N@hF,$r<($9H?I|;R(B
  \param end	$B%G!<%?Ns$NKvHx$N<!$r<($9H?I|;R(B
  \return	$BogCM$H$J$k%G!<%?$r<($9H?I|;R(B
*/
template <class Iterator> Iterator
binarize(Iterator begin, Iterator end)
{
  // $BMWAG?t$HJ?6QCM$r7W;;!%(B
    long	n = 0;
    double	mean = 0;
    for (Iterator iter = begin; iter != end; ++iter)
    {
	++n;
	mean += *iter;
    }
    mean /= n;

  // $B>:=g$K%=!<%H!%(B
    std::sort(begin, end);

  // $BBgDE$NH=JL4p=`$K$h$j:GE,$J$7$-$$CM$r7hDj!%(B
    Iterator	thresh = begin;
    long	nLow = 0;		// $B$7$-$$CM0J2<$NMWAG?t(B
    double	cumulationLow = 0;	// $B$7$-$$CM0J2<$NN_@QCM(B
    double	interVarianceMax = 0;	// $B%/%i%94VJ,;6$N:GBgCM(B
    for (Iterator iter = begin, head = begin; iter != end; ++iter)
    {
	if (*iter != *head)
	{
	    double	interVariance = cumulationLow - nLow * mean;
	    ((interVariance *= interVariance) /= nLow) /= (n - nLow);
	    if (interVariance > interVarianceMax)
	    {
		interVarianceMax = interVariance;
		thresh = iter;
	    }
	    head = iter;
	}

	++nLow;
	cumulationLow += *iter;
    }

    return thresh;
}

}
