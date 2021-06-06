/*!
 *  \file	Eigen.h
 *  \author	Toshio UESHIBA
 *  \brief	Utilities
 */
#ifndef AIST_FTSENSOR_SIMILARITY_H
#define AIST_FTSENSOR_SIMILARITY_H

#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/LU>

namespace aist_ftsensor
{
/************************************************************************
*  global functions							*
************************************************************************/
//! Exterior product of two vectors.
template <class T, int M, int N> ::Eigen::Matrix<T, M, N>
operator %(const ::Eigen::Matrix<T, M, 1>& x,
	   const ::Eigen::Matrix<T, N, 1>& y)
{
    ::Eigen::Matrix<T, M, N>	mat;
    for (size_t i = 0; i < M; ++i)
	for (size_t j = 0; j < N; ++j)
	    mat(i, j) = x(i) * y(j);
    return mat;
}

/************************************************************************
*  class Similarity<T, D>						*
************************************************************************/
template <class T, size_t D>
class Similarity
{
  public:
    using element_t	= T;
    using vector_t	= ::Eigen::Matrix<T, D, 1>;
    using matrix_t	= ::Eigen::Matrix<T, D, D>;

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
    using namespace ::Eigen;

  // 充分な個数の点対があるか？
    const size_t	ndata = std::distance(begin, end);
    if (ndata == 0)		// beginが有効か？
	throw std::invalid_argument("Similarity::fit(): 0-length input data!!");

  // 重心の計算
    vector_t	xc(vector_t::Zero()), yc(vector_t::Zero());
    for (auto corres = begin; corres != end; ++corres)
    {
	xc += corres->first;
	yc += corres->second;
    }
    xc /= ndata;
    yc /= ndata;

  // モーメント行列の計算
    matrix_t	M(matrix_t::Zero());
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
	       << similarity.t()  << std::endl;
}

}	// namespace aist_ftsensor
#endif	// !AIST_FTSENSOR_SIMILRITY_H
