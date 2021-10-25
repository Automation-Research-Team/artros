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
  \brief	大津の2値化アルゴリズムの実装
*/
#include <algorithm>

namespace TU
{
//! 大津の2値化アルゴリズムにより与えられたデータ列を2値化する．
/*!
  データ列は昇順にソートされ，閾値となるデータすなわち後半部の先頭データが返される．
  \param begin	データ列の先頭を示す反復子
  \param end	データ列の末尾の次を示す反復子
  \return	閾値となるデータを示す反復子
*/
template <class Iterator> Iterator
binarize(Iterator begin, Iterator end)
{
  // 要素数と平均値を計算．
    long	n = 0;
    double	mean = 0;
    for (Iterator iter = begin; iter != end; ++iter)
    {
	++n;
	mean += *iter;
    }
    mean /= n;

  // 昇順にソート．
    std::sort(begin, end);

  // 大津の判別基準により最適なしきい値を決定．
    Iterator	thresh = begin;
    long	nLow = 0;		// しきい値以下の要素数
    double	cumulationLow = 0;	// しきい値以下の累積値
    double	interVarianceMax = 0;	// クラス間分散の最大値
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
