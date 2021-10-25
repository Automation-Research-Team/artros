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
 *  \file	eigen.h
 *  \author	Toshio Ueshiba
 *  \brief	Utilities
 */
#ifndef AIST_UTILITY_EIGEN_H
#define AIST_UTILITY_EIGEN_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace aist_utility
{
namespace eigen
{
/************************************************************************
*  global functions							*
************************************************************************/
//! Exterior product of two vectors.
template <class T, int M, int N> Eigen::Matrix<T, M, N>
operator %(const Eigen::Matrix<T, M, 1>& x,
	   const Eigen::Matrix<T, N, 1>& y)
{
    return x * y.transpose();
}

template <class T> Eigen::Matrix<T, 3, 3>
skew(const Eigen::Matrix<T, 3, 1>& vec)
{
    Eigen::Matrix<T, 3, 3>	mat;
    mat <<       0, -vec(2),  vec(1),
	    vec(2),	  0, -vec(0),
	   -vec(1),  vec(0),	   0;
    return mat;
}

template <class T> Eigen::Matrix<T, 3, 3>
rodrigues(const Eigen::Matrix<T, 3, 1>& r)
{
    using matrix_t = Eigen::Matrix<T, 3, 3>;

    const auto	theta = r.norm();
    r /= theta;
    const auto	c = std::cos(theta), s = std::sin(theta);
    matrix_t	R = matrix_t::Zero();
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

template <class ITER> typename std::iterator_traits<ITER>::value_type
mean(ITER begin, ITER end)
{
    using value_type = typename std::iterator_traits<ITER>::value_type;

    size_t	n = 0;
    value_type	m = value_type::Zero();
    for (; begin != end; ++begin, ++n)
	m += *begin;
    return m/n;
}

template <class ITER> typename std::iterator_traits<ITER>::value_type
variance(ITER begin, ITER end)
{
    using value_type = typename std::iterator_traits<ITER>::value_type;

    const auto	m = mean(begin, end);
    size_t	n = 0;
    value_type	v = value_type::Zero();
    for (; begin != end; ++begin, ++n)
	v += (*begin - m).squaredNorm();
    return v/n;
}

/************************************************************************
*  class Plane<T, N>							*
************************************************************************/
template <class T, size_t N>
class Plane
{
  public:
    using vector_t	= Eigen::Matrix<T, N, 1>;
    using element_t	= T;

  public:
    Plane()					:_n(), _d(0)		{}
    Plane(const vector_t& norm, element_t dist) :_n(norm), _d(dist)	{}
    template <class ITER>
    Plane(ITER begin, ITER end)				{ fit(begin, end); }

    template <class ITER>
    element_t		fit(ITER begin, ITER end)	;

    constexpr
    static size_t	ndataMin()			{ return N;  }
    const vector_t&	normal()		const	{ return _n; }
    element_t		distance()		const	{ return _d; }
    element_t		distance(const vector_t& point) const
			{
			    return std::abs(signed_distance(point));
			}
    element_t		signed_distance(const vector_t& point) const
			{
			    return _n.dot(point) + _d;
			}
    vector_t		cross_point(const vector_t& view_vector) const
			{
			    return (-_d/_n.dot(view_vector)) * view_vector;
			}

    friend std::ostream&
			operator <<(std::ostream& out, const Plane& plane)
			{
			    out << plane._n.transpose() << ": " << plane._d;
			}

  private:
    vector_t	_n;	// normal
    element_t	_d;	// distance from the origin
};

template <class T, size_t N> template <class ITER> T
Plane<T, N>::fit(ITER begin, ITER end)
{
    using matrix_t = Eigen::Matrix<T, N, N>;

    const auto	ext   =	[](const auto& x)
			{
			    matrix_t	y;
			    for (size_t i = 0; i < N; ++i)
				for (size_t j = 0; j < N; ++j)
				    y(i, j) = x(i) * x(j);
			    return y;
			};

  // Check #points.
    const auto	npoints = std::distance(begin, end);
    if (npoints < 3)
	throw std::runtime_error("Failed to fit a plane: three or more points required!");

  // Compute moment matrix.
    const vector_t	centroid = mean(begin, end);
    matrix_t		moments  = matrix_t::Zero();
    for (auto iter = begin; iter != end; ++iter)
	moments += ext(*iter - centroid);

  // Compute eigenvector corresponding to the smallest eigenvalue of
  // the moment matrix.
    Eigen::SelfAdjointEigenSolver<matrix_t>	eigensolver(moments);
    _n = eigensolver.eigenvectors().col(0);
    _d = -_n.dot(centroid);
    if (_d < 0)
    {
	_n *= element_t(-1);
	_d *= -1;
    }

  // Return RMS error.
    return std::sqrt(eigensolver.eigenvalues()(0) / element_t(npoints));
}

/************************************************************************
*  class Similarity<T, D>						*
************************************************************************/
template <class T, size_t D>
class Similarity
{
  public:
    using element_t	= T;
    using vector_t	= Eigen::Matrix<T, D, 1>;
    using matrix_t	= Eigen::Matrix<T, D, D>;

  public:
		Similarity(const vector_t& t=vector_t::Zero(),
			   const matrix_t& sR=matrix_t::Identity())
		    :_t(t), _sR(sR)			{}
    template <class ITER>
		Similarity(ITER begin, ITER end)	{ fit(begin, end); }


    const auto&	t()			const	{ return _t; }
    const auto&	sR()			const	{ return _sR; }
    element_t	s()			const	{ return _sR.col(0).norm(); }
    matrix_t	R()			const	{ return _sR / s(); }

    vector_t	operator ()(const vector_t& x) const
		{
		    return _sR * x + _t;
		}
    template <class ITER>
    void	fit(ITER begin, ITER end)	;

  private:
    vector_t	_t;
    matrix_t	_sR;
};

template <class T, size_t D> template <class ITER> void
Similarity<T, D>::fit(ITER begin, ITER end)
{
    using namespace Eigen;

  // 充分な個数の点対があるか？
    const size_t	ndata = std::distance(begin, end);
    if (ndata == 0)		// beginが有効か？
	throw std::invalid_argument("Similarity::fit(): 0-length input data!!");
  // 重心の計算
    vector_t	xc = vector_t::Zero(), yc = vector_t::Zero();
    for (auto corres = begin; corres != end; ++corres)
    {
	xc += corres->first;
	yc += corres->second;
    }
    xc /= ndata;
    yc /= ndata;

  // モーメント行列の計算
    matrix_t	M = matrix_t::Zero();
    for (auto corres = begin; corres != end; ++corres)
	M += vector_t(corres->first - xc) % vector_t(corres->second - yc);

  // モーメント行列を特異値分解し，U, Vの行列式が正になるように補正
    JacobiSVD<matrix_t>	svd(M, ComputeFullU | ComputeFullV);
    matrix_t	Ut = svd.matrixU().transpose();
    if (Ut.determinant() < 0)
	Ut.row(D-1) *= -1;
    matrix_t	V = svd.matrixV();
    if (V.determinant() < 0)
	V.col(D-1) *= -1;

  // スケールの計算
    element_t	sqr_x = 0;
    for (auto corres = begin; corres != end; ++corres)
	sqr_x += vector_t(corres->first - xc).squaredNorm();
    element_t	scale = 0;
    for (size_t i = 0; i < D; ++i)
	scale += svd.singularValues()(i);
    scale /= sqr_x;

  // 点群間の相似変換の計算
    _sR = scale * V * Ut;
    _t  = yc - _sR * xc;
}

template <class T, size_t D> std::ostream&
operator <<(std::ostream& out, const Similarity<T, D>& similarity)
{
    return out << similarity.sR() << std::endl
	       << similarity.t().transpose()  << std::endl;
}

}	// namespace eigen
}	// namespace aist_utility
#endif	// !AIST_UTILITY_EIGEN_H
