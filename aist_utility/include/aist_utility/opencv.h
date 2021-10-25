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
 *  \file	opencv.h
 *  \author	Toshio Ueshiba
 *  \brief	Utilities
 */
#ifndef AIST_UTILITY_OPENCV_H
#define AIST_UTILITY_OPENCV_H

#include <opencv2/core/core.hpp>
#include <tf/tf.h>

namespace aist_utility
{
namespace opencv
{
/************************************************************************
*  global functions							*
************************************************************************/
//! Exterior product of two vectors.
template <class T, int M, int N> ::cv::Matx<T, M, N>
operator %(const cv::Matx<T, M, 1>& x, const cv::Matx<T, N, 1>& y)
{
    cv::Matx<T, M, N>	mat;
    for (size_t i = 0; i < M; ++i)
	for (size_t j = 0; j < N; ++j)
	    mat(i, j) = x(i) * y(j);
    return mat;
}

template <class T, int N> T
angle(const cv::Vec<T, N>& x, const cv::Vec<T, N>& y)
{
    return std::acos(x.dot(y)/(cv::norm(x)*cv::norm(y))) * 180.0/M_PI;
}

template <class T> cv::Matx<T, 3, 3>
rodrigues(const cv::Matx<T, 3, 1>& r)
{
    using matrix_t = cv::Matx<T, 3, 3>;

    const auto	theta = r.norm();
    r /= theta;
    const auto	c = std::cos(theta), s = std::sin(theta);
    matrix_t	R = matrix_t::zeros();
    for (int i = 0; i < 3; ++i)
    {
	R(i, i) += c;

	for (int j = 0; j < 3; ++j)
	    R(i, j) += (1 - c)*r(i)*r(j);
    }
    R(0, 1) -= s * r(2);
    R(0, 2) += s * r(1);
    R(1, 0) += s * r(2);
    R(1, 2) -= s * r(0);
    R(2, 0) -= s * r(1);
    R(2, 1) += s * r(0);

    return R;
}

inline cv::Point3f
pointTFToCV(const tf::Point& p)
{
    return {p.x(), p.y(), p.z()};
}

/************************************************************************
*  struct Plane<T, N>							*
************************************************************************/
template <class T, size_t N>
class Plane
{
  public:
    using vector_type	= cv::Vec<T, N>;
    using value_type	= T;

  public:
    Plane()					    :_n(), _d(0)	{}
    Plane(const vector_type& norm, value_type dist) :_n(norm), _d(dist)	{}
    template <class ITER>
    Plane(ITER begin, ITER end)				{ fit(begin, end); }

    template <class ITER>
    void		fit(ITER begin, ITER end)	;

    constexpr
    static size_t	ndataMin()			{ return N;  }
    const vector_type&	normal()		const	{ return _n; }
    value_type		distance()		const	{ return _d; }
    value_type		distance(const vector_type& point) const
			{
			    return std::abs(signed_distance(point));
			}
    value_type		signed_distance(const vector_type& point) const
			{
			    return _n.dot(point) + _d;
			}
    vector_type		cross_point(const vector_type& view_vector) const
			{
			    return (-_d/_n.dot(view_vector)) * view_vector;
			}

    friend std::ostream&
			operator <<(std::ostream& out, const Plane& plane)
			{
			    out << plane._n << ": " << plane._d;
			}

  private:
    vector_type	_n;	// normal
    value_type	_d;	// distance from the origin
};

template <class T, size_t N> template <class ITER> void
Plane<T, N>::fit(ITER begin, ITER end)
{
    using matrix_type = cv::Matx<T, N, N>;

    const auto	ext   =	[](const auto& x)
			{
			    matrix_type	y;
			    for (size_t i = 0; i < N; ++i)
				for (size_t j = 0; j < N; ++j)
				    y(i, j) = x(i) * x(j);
			    return y;
			};

  // Check #points.
    const auto	npoints = std::distance(begin, end);
    if (npoints < 3)
	throw std::runtime_error("Failed to fit a plane: three or more points required!");

  // Compute centroid.
    auto	centroid = vector_type::zeros();
    for (auto iter = begin; iter != end; ++iter)
	centroid += *iter;
    centroid *= value_type(1)/value_type(npoints);

  // Compute moment matrix.
    auto	moments = matrix_type::zeros();
    for (auto iter = begin; iter != end; ++iter)
	moments += ext(*iter - centroid);

    matrix_type	evectors;
    vector_type	evalues;
    cv::eigen(moments, evalues, evectors);

    _n = vector_type::all(0);
    for (size_t j = 0; j < N; ++j)
	_n(j) = evectors(N - 1, j);
    _d = -_n.dot(centroid);
    if (_d < 0)
    {
	_n *= value_type(-1);
	_d *= -1;
    }

    ROS_DEBUG_STREAM("plane = " << *this << ", err = "
		     << std::sqrt(std::abs(evalues(N-1))/value_type(npoints))
		     << ", computed from " << npoints << " points.");
}

/************************************************************************
*  class Rigidity<T, D>							*
************************************************************************/
template <class T, size_t D>
class Rigidity
{
  public:
    using element_t	= T;
    using vector_t	= cv::Matx<T, D, 1>;
    using matrix_t	= cv::Matx<T, D, D>;

  public:
		Rigidity(const vector_t& t=vector_t::zeros(),
			 const matrix_t& R=matrix_t::eye())
		    :_t(t), _R(R)			{}
    template <class ITER>
		Rigidity(ITER begin, ITER end)		{ fit(begin, end); }

    const auto&	t()				const	{ return _t; }
    const auto&	R()				const	{ return _R; }
    vector_t	operator ()(const vector_t& x)	const	{ return _t + _R * x; }
    template <class ITER>
    element_t	fit(ITER begin, ITER end)		;

  private:
    vector_t	_t;
    matrix_t	_R;
};

template <class T, size_t D> template <class ITER>
typename Rigidity<T, D>::element_t
Rigidity<T, D>::fit(ITER begin, ITER end)
{
  // 充分な個数の点対があるか？
    const size_t	ndata = std::distance(begin, end);
    if (ndata < D)		// beginが有効か？
	throw std::invalid_argument("Rigidity::fit(): not enough data!!");

  // 重心の計算
    vector_t	xc(vector_t::zeros()), yc(vector_t::zeros());
    for (auto corres = begin; corres != end; ++corres)
    {
	xc += corres->first;
	yc += corres->second;
    }
    const auto	k = element_t(1)/element_t(ndata);
    xc *= k;
    yc *= k;

  // モーメント行列の計算
    element_t	sqr_d = 0;
    matrix_t	M(matrix_t::zeros());
    for (auto corres = begin; corres != end; ++corres)
    {
	const vector_t	dx = corres->first  - xc;
	const vector_t	dy = corres->second - yc;

	sqr_d += (dx.dot(dx) + dy.dot(dy));
	M     += dx % dy;
    }

  // モーメント行列を特異値分解し，U, Vの行列式が正になるように補正
    cv::SVD	svd(M, cv::SVD::FULL_UV);
    if (cv::determinant(svd.u) < 0)
    {
	svd.u.col(D-1) *= -1;
	svd.w.at<element_t>(D-1) *= -1;
    }
    if (cv::determinant(svd.vt) < 0)
    {
	svd.vt.row(D-1) *= -1;
	svd.w.at<element_t>(D-1) *= -1;
    }

  // 点群間の相似変換の計算
    cv::transpose(svd.u * svd.vt, _R);
    _t = yc - _R * xc;

  // 残差平均の計算
    for (size_t i = 0; i < D; ++i)
	sqr_d -= 2 * svd.w.at<element_t>(i);

    return std::sqrt(std::abs(sqr_d)/ndata);
}

template <class T, size_t D> std::ostream&
operator <<(std::ostream& out, const Rigidity<T, D>& rigidity)
{
    return out << rigidity.R() << std::endl
	       << rigidity.t() << std::endl;
}

/************************************************************************
*  class Similarity<T, D>						*
************************************************************************/
template <class T, size_t D>
class Similarity
{
  public:
    using element_t	= T;
    using vector_t	= cv::Matx<T, D, 1>;
    using matrix_t	= cv::Matx<T, D, D>;

  public:
		Similarity(const vector_t& t=vector_t::zeros(),
			   const matrix_t& sR=matrix_t::eye())
		    :_t(t), _sR(sR)			{}

    template <class ITER>
		Similarity(ITER begin, ITER end)	{ fit(begin, end); }


    const auto&	t()		const	{ return _t; }
    const auto&	sR()		const	{ return _sR; }
    element_t	s()		const	{ return cv::norm(_sR.col(0));}
    matrix_t	R()		const	{ return _sR * (1/s()); }

    vector_t	operator ()(const vector_t& x) const
		{
		    return _t + _sR * x;
		}

    template <class ITER>
    element_t	fit(ITER begin, ITER end)	;

  private:
    vector_t	_t;
    matrix_t	_sR;
};

template <class T, size_t D> template <class ITER>
typename Similarity<T, D>::element_t
Similarity<T, D>::fit(ITER begin, ITER end)
{
  // 充分な個数の点対があるか？
    const size_t	ndata = std::distance(begin, end);
    if (ndata < D)
	throw std::invalid_argument("Similarity::fit(): not enough data!!");

  // 重心の計算
    vector_t	xc(vector_t::zeros()), yc(vector_t::zeros());
    for (auto corres = begin; corres != end; ++corres)
    {
	xc += corres->first;
	yc += corres->second;
    }
    const auto	k = element_t(1)/element_t(ndata);
    xc *= k;
    yc *= k;

  // モーメント行列の計算
    element_t	sqr_dx = 0;
    element_t	sqr_dy = 0;
    matrix_t	M(matrix_t::zeros());
    for (auto corres = begin; corres != end; ++corres)
    {
	const vector_t	dx = corres->first  - xc;
	const vector_t	dy = corres->second - yc;

	sqr_dx += dx.dot(dx);
	sqr_dy += dy.dot(dy);
	M      += dx % dy;
    }

  // モーメント行列を特異値分解し，U, Vの行列式が正になるように補正
    cv::SVD	svd(M, cv::SVD::FULL_UV);
    if (cv::determinant(svd.u) < 0)
    {
	svd.u.col(D-1) *= -1;
	svd.w.at<element_t>(D-1) *= -1;
    }
    if (cv::determinant(svd.vt) < 0)
    {
	svd.vt.row(D-1) *= -1;
	svd.w.at<element_t>(D-1) *= -1;
    }

  // スケールの計算
    element_t	scale = 0;
    for (size_t i = 0; i < D; ++i)
	scale += svd.w.at<element_t>(i);
    scale /= sqr_dx;

  // 点群間の相似変換の計算
    cv::transpose(scale * svd.u * svd.vt, _sR);
    _t = yc - _sR * xc;

  // 残差平均の計算
    return std::sqrt(std::abs(sqr_dy - sqr_dx*scale*scale)/ndata);
}

template <class T, size_t D> std::ostream&
operator <<(std::ostream& out, const Similarity<T, D>& similarity)
{
    return out << similarity.sR() << std::endl
	       << similarity.t()  << std::endl;
}

}	// namespace opencv
}	// namespace aist_utility

#endif	// !AIST_UTILITY_OPENCV_H
