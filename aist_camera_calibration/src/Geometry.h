/*!
  \file		Geometry++.h
  \author	Toshio UESHIBA
  \brief	点，直線，超平面および各種の幾何変換に関するクラスの定義と実装
*/
#pragma once

#include "TU/Minimize.h"
#include <limits>

namespace TU
{
/************************************************************************
*  class Normalize<S, D>						*
************************************************************************/
//! 点の非同次座標の正規化変換を行うクラス
/*!
  \f$\TUud{x}{}=[\TUtvec{x}{}, 1]^\top~
  (\TUvec{x}{} \in \TUspace{R}{d})\f$に対して，以下のような平行移動と
  スケーリングを行う:
  \f[
	\TUud{y}{} =
	\TUbeginarray{c} s^{-1}(\TUvec{x}{} - \TUvec{c}{}) \\ 1	\TUendarray =
	\TUbeginarray{ccc}
	  s^{-1} \TUvec{I}{d} & -s^{-1}\TUvec{c}{} \\ \TUtvec{0}{d} & 1
	\TUendarray
	\TUbeginarray{c} \TUvec{x}{} \\ 1 \TUendarray =
	\TUvec{T}{}\TUud{x}{}
  \f]
  \f$s\f$と\f$\TUvec{c}{}\f$は，振幅の2乗平均値が空間の次元\f$d\f$に,
  重心が原点になるよう決定される．
*/
template <class S, size_t D>
class Normalize
{
  private:
    constexpr static size_t	D1 = D + 1;

  public:
    using element_type	= S;
    using point_type	= Eigen::Matrix<element_type, D, 1>;
    using matrix_type	= Eigen::Matrix<element_type, D1, D1>;

  public:
  //! 空間の次元を指定して正規化変換オブジェクトを生成する．
  /*!
    恒等変換として初期化される．
    \param d		空間の次元
  */
    Normalize()	:_npoints(0), _scale(1), _centroid(d)	{}

  //! 与えられた点群の非同次座標から正規化変換オブジェクトを生成する．
  /*!
    振幅の2乗平均値が spaceDim(), 重心が原点になるような正規化変換が計算される．
    \param begin	点群の先頭を示す反復子
    \param end		点群の末尾を示す反復子
  */
    template <class ITER_>
    Normalize(ITER_ begin, ITER_ end)
	:_npoints(0), _scale(1.0), _centroid()	{ insert(begin, end); }

    template <class ITER_>
    void		insert(ITER_ begin, ITER_ end)	;

  //! この正規化変換が適用される空間の次元を返す．
  /*!
    \return	空間の次元(同次座標のベクトルとしての次元は spaceDim()+1)
  */
    size_t		spaceDim()	const	{ return _centroid.size(); }


  //! 与えられた点に正規化変換を適用してその非同次座標を返す．
  /*!
    \param x	点の非同次座標(spaceDim() 次元)
    \return	正規化された点の非同次座標(spaceDim() 次元)
  */
    template <class S_, size_t D_>
    point_type		operator ()(const Array<S_, D_>& x) const
			{
			    return (x - _centroid)/_scale;
			}

  //! 与えられた点に正規化変換を適用してその同次座標を返す．
  /*!
    \param x	点の非同次座標(spaceDim() 次元)
    \return	正規化された点の同次座標(spaceDim()+1次元)
  */
    template <class S_, size_t D_>
    auto		normalizeP(const Array<S_, D_>& x) const
			{
			    return homogeneous((*this)(x));
			}

    matrix_type		T()		const	;
    matrix_type		Tt()		const	;
    matrix_type		Tinv()		const	;
    matrix_type		Ttinv()		const	;

  //! 正規化変換のスケーリング定数を返す．
  /*!
    \return	スケーリング定数(与えられた点列の振幅の2乗平均値)
  */
    element_type	scale()		const	{return _scale;}

  //! 正規化変換の平行移動成分を返す．
  /*!
    \return	平行移動成分(与えられた点列の重心)
  */
    const point_type&	centroid()	const	{return _centroid;}

  private:
    size_t		_npoints;	//!< これまでに与えた点の総数
    element_type	_scale;		//!< これまでに与えた点の振幅のRMS値
    point_type		_centroid;	//!< これまでに与えた点群の重心
};

//! 新たに点群を追加してその非同次座標から現在の正規化変換を更新する．
/*!
  振幅の2乗平均値が spaceDim(), 重心が原点になるような正規化変換が計算される．
  \param begin			点群の先頭を示す反復子
  \param end			点群の末尾を示す反復子
  \throw std::invalid_argument	これまでに与えられた点の総数が0の場合に送出
*/
template <class S, size_t D> template <class ITER_> void
Normalize<S, D>::insert(ITER_ begin, ITER_ end)
{
    if (_npoints == 0)
    {
	if (begin == end)
	    throw std::invalid_argument("Normalize::insert(): 0-length input data!!");
    }

    _scale = _npoints*(spaceDim()*_scale*_scale + _centroid*_centroid);
    _centroid *= _npoints;
    for (; begin != end; ++begin)
    {
	_scale	  += square(*begin);
	_centroid += *begin;
	++_npoints;
    }
    if (_npoints == 0)
	throw std::invalid_argument("Normalize::insert(): no input data accumulated!!");
    _centroid /= _npoints;
    _scale = _scale/_npoints - _centroid*_centroid;

    if (_scale < 0)
	throw std::invalid_argument("Normalize::insert(): negative square scale!!");

    _scale = sqrt(_scale/spaceDim());
}

//! 正規化変換行列を返す．
/*!
  \return	変換行列:
		\f$
		\TUvec{T}{} =
		\TUbeginarray{ccc}
		 s^{-1} \TUvec{I}{d} & -s^{-1}\TUvec{c}{} \\ \TUtvec{0}{d} & 1
		\TUendarray
		\f$
*/
template <class S, size_t D> auto
Normalize<S, D>::T() const -> matrix_type
{
    matrix_type	TT(spaceDim()+1, spaceDim()+1);
    for (size_t i = 0; i < spaceDim(); ++i)
    {
	TT[i][i] = 1.0 / _scale;
	TT[i][spaceDim()] = -_centroid[i] / _scale;
    }
    TT[spaceDim()][spaceDim()] = 1.0;

    return TT;
}

//! 正規化変換の転置行列を返す．
/*!
  \return	変換の転置行列:
		\f$
		\TUtvec{T}{} =
		\TUbeginarray{ccc}
		 s^{-1} \TUvec{I}{d} & \TUvec{0}{d} \\ -s^{-1}\TUtvec{c}{} & 1
		\TUendarray
		\f$
*/
template <class S, size_t D> auto
Normalize<S, D>::Tt() const -> matrix_type
{
    matrix_type	TTt(spaceDim()+1, spaceDim()+1);
    for (size_t i = 0; i < spaceDim(); ++i)
    {
	TTt[i][i] = 1.0 / _scale;
	TTt[spaceDim()][i] = -_centroid[i] / _scale;
    }
    TTt[spaceDim()][spaceDim()] = 1.0;

    return TTt;
}

//! 正規化変換の逆行列を返す．
/*!
  \return	変換の逆行列:
		\f$
		\TUinv{T}{} =
		\TUbeginarray{ccc}
		 s \TUvec{I}{d} & \TUvec{c}{} \\ \TUtvec{0}{d} & 1
		\TUendarray
		\f$
*/
template <class S, size_t D> auto
Normalize<S, D>::Tinv() const -> matrix_type
{
    matrix_type	TTinv(spaceDim()+1, spaceDim()+1);
    for (size_t i = 0; i < spaceDim(); ++i)
    {
	TTinv[i][i] = _scale;
	TTinv[i][spaceDim()] = _centroid[i];
    }
    TTinv[spaceDim()][spaceDim()] = 1.0;

    return TTinv;
}

//! 正規化変換の逆行列の転置を返す．
/*!
  \return	変換の逆行列の転置:
		\f$
		\TUtinv{T}{} =
		\TUbeginarray{ccc}
		 s \TUvec{I}{d} & \TUvec{0}{d} \\ \TUtvec{c}{} & 1
		\TUendarray
		\f$
*/
template <class S, size_t D> auto
Normalize<S, D>::Ttinv() const -> matrix_type
{
    matrix_type	TTtinv(spaceDim()+1, spaceDim()+1);
    for (size_t i = 0; i < spaceDim(); ++i)
    {
	TTtinv[i][i] = _scale;
	TTtinv[spaceDim()][i] = _centroid[i];
    }
    TTtinv[spaceDim()][spaceDim()] = 1.0;

    return TTtinv;
}

/************************************************************************
*  class HyperPlane<T, D>						*
************************************************************************/
//! d次元射影空間中の超平面を表現するクラス
/*!
  d次元射影空間の点\f$\TUud{x}{} \in \TUspace{R}{d+1}\f$に対して
  \f$\TUtud{p}{}\TUud{x}{} = 0,~\TUud{p}{} \in \TUspace{R}{d+1}\f$
  によって表される．
*/
template <class T, size_t D=0>
class HyperPlane : public std::conditional_t<D==0, Array<T>, Array<T, D+1> >
{
  public:
    using base_type	= std::conditional_t<D==0, Array<T>, Array<T, D+1> >;
    using		typename base_type::element_type;

  public:
  //! 空間の次元を指定して超平面オブジェクトを生成する．
  /*!
    無限遠超平面([0, 0,..., 0, 1])に初期化される．
    \param d	この超平面が存在する射影空間の次元
  */
    explicit	HyperPlane(size_t d=D)	:base_type(d+1)	{ (*this)[d] = 1; }

  //! 与えられた点列の非同次座標に当てはめられた超平面オブジェクトを生成する．
  /*!
    \param begin			点列の先頭を示す反復子
    \param end				点列の末尾を示す反復子
    \throw std::invalid_argument	点の数が ndataMin() に満たない場合に送出
  */
    template <class ITER_>
    HyperPlane(ITER_ begin, ITER_ end)			{ fit(begin, end); }

    template <class E_, std::enable_if_t<TU::rank<E_>() == 1>* = nullptr>
    HyperPlane(const E_& expr)	:base_type(expr)	{}

  //! 超平面を表す同次座標ベクトルを指定する．
  /*!
    \param expr	(d+1)次元ベクトル(dは空間の次元)
  */
    template <class E_>
    std::enable_if_t<rank<E_>() == 1, HyperPlane&>
			operator =(const E_& expr)
			{
			    base_type::operator =(expr);
			    return *this;
			}

    using		base_type::size;

    template <class ITER_>
    void		fit(ITER_ begin, ITER_ end)			;

  //! この超平面が存在する射影空間の次元を返す．
  /*!
    \return	射影空間の次元(同次座標のベクトルとしての次元は spaceDim()+1)
  */
    size_t		spaceDim()		const	{return size()-1;}

  //! 超平面を求めるために必要な点の最小個数を返す．
  /*!
    現在設定されている射影空間の次元をもとに計算される．
    \return	必要な点の最小個数すなわち入力空間の次元 spaceDim()
  */
    size_t		ndataMin()		const	{return spaceDim();}

    template <class T_, size_t D_>
    element_type	square_distance(const Array<T_, D_>& x)	const	;
    template <class T_, size_t D_>
    element_type	distance(const Array<T_, D_>& x)	const	;
};

//! 与えられた点列の非同次座標に超平面を当てはめる．
/*!
  \param begin			点列の先頭を示す反復子
  \param end			点列の末尾を示す反復子
  \throw std::invalid_argument	点の数が ndataMin() に満たない場合に送出
*/
template <class T, size_t D> template <class ITER_> void
HyperPlane<T, D>::fit(ITER_ begin, ITER_ end)
{
  // 点列の正規化
    const Normalize<element_type, D>	normalize(begin, end);

  // 充分な個数の点があるか？
    const size_t	ndata = std::distance(begin, end);
    const auto		d     = normalize.spaceDim();
    if (ndata < d)	// Vのサイズが未定なのでndataMin()は無効
	throw std::invalid_argument("Hyperplane::initialize(): not enough input data!!");

  // データ行列の計算
    Matrix<element_type, D, D>	A(d, d);
    for (; begin != end; ++begin)
    {
	const auto	x = normalize(*begin);
	A += x % x;
    }

  // データ行列の最小固有値に対応する固有ベクトルから法線ベクトルを計算し，
  // さらに点列の重心より原点からの距離を計算する．
    Vector<element_type, D>	eval;
    const auto	Ut = eigen(A, eval);
    base_type::resize(d+1);
    (*this)(0, d) = Ut[Ut.nrow()-1];
    (*this)[d] = -((*this)(0, d)*normalize.centroid());
    if ((*this)[d] > 0)
	*this *= -1;
}

//! 与えられた点と超平面の距離の2乗を返す．
/*!
  \param x	点の非同次座標(spaceDim() 次元)または同次座標
		(spaceDim()+1次元)
  \return	点と超平面の距離の2乗
*/
template <class T, size_t D> template <class T_, size_t D_> inline auto
HyperPlane<T, D>::square_distance(const Array<T_, D_>& x) const -> element_type
{
    const auto	d = distance(x);
    return d*d;
}

//! 与えられた点と超平面の距離を返す．
/*!
  \param x			点の非同次座標(spaceDim() 次元)または
				同次座標(spaceDim()+1次元)
  \return			点と超平面の距離(非負)
  \throw std::invalid_argument	点のベクトルとしての次元が spaceDim()，
				spaceDim()+1のいずれでもない場合，もしくは
				この点が無限遠点である場合に送出．
*/
template <class T, size_t D> template <class T_, size_t D_> auto
HyperPlane<T, D>::distance(const Array<T_, D_>& x) const -> element_type
{
    const auto	p = (*this)(0, spaceDim());
    if (x.size() == spaceDim())
	return (p * x + (*this)[spaceDim()]) / length(p);
    else if (x.size() == spaceDim() + 1)
    {
	if (x[spaceDim()] == element_type(0))
	    throw std::invalid_argument("HyperPlane::distance(): point at infinitiy!!");
	return (*this * x) / (length(p) * x[spaceDim()]);
    }
    else
	throw std::invalid_argument("HyperPlane::distance(): dimension mismatch!!");
}

template <class T> using LineP	= HyperPlane<T, 2>;	//!< 2次元空間中の直線
template <class T> using PlaneP = HyperPlane<T, 3>;	//!< 3次元空間中の平面

using LineP2f	= LineP<float>;			//!< 2次元空間中のfloat型直線
using LineP2d	= LineP<double>;		//!< 2次元空間中のdouble型直線
using PlaneP3f	= PlaneP<float>;		//!< 3次元空間中のfloat型平面
using PlaneP3d	= PlaneP<double>;		//!< 3次元空間中のdouble型平面

/************************************************************************
*  class Projectivity<T, DO, DI>					*
************************************************************************/
//! 射影変換を行うクラス
/*!
  \f$\TUvec{T}{} \in \TUspace{R}{(n+1)\times(m+1)}\f$を用いてm次元空間の点
  \f$\TUud{x}{} \in \TUspace{R}{m+1}\f$をn次元空間の点
  \f$\TUud{y}{} \simeq \TUvec{T}{}\TUud{x}{} \in \TUspace{R}{n+1}\f$
  に写す(\f$m \neq n\f$でも構わない)．
*/
template <class T, size_t DO=0, size_t DI=0>
class Projectivity : public std::conditional_t<(DO==0 || DI==0),
					       Array2<T>,
					       Array2<T, DO+1, DI+1> >
{
  public:
    constexpr static size_t	DO1	= (DO == 0	? 0 : DO + 1);
    constexpr static size_t	DI1	= (DI == 0	? 0 : DI + 1);
    constexpr static size_t	NPARAMS = DO1*DI1;
    constexpr static size_t	DOF	= (NPARAMS == 0 ? 0 : NPARAMS - 1);

    using base_type		= std::conditional_t<(DO==0 || DI==0),
						     Array2<T>,
						     Array2<T, DO+1, DI+1> >;
    using			typename base_type::element_type;
    using point_type		= Array<element_type, DO>;
    using ppoint_type		= Array<element_type, DO1>;
    using vector_type		= Array<element_type>;
    using derivative_type	= Array2<element_type, DO, NPARAMS>;
    using derivative0_type	= Array2<element_type, DO, DOF>;
    using jacobian_type		= Array2<element_type, DO, DI>;

  public:
    Projectivity(size_t outDim=DO, size_t inDim=DI)			;
    template <class ITER_>
    Projectivity(ITER_ begin, ITER_ end, bool refine=false)		;

    template <class E_, std::enable_if_t<rank<E_>() == 2>* = nullptr>
    Projectivity(const E_& expr)	:base_type(expr)		{}

  //! 変換行列を指定する．
  /*!
    \param expr	(m+1)x(n+1)行列(m, nは入力／出力空間の次元)
  */
    template <class E_>
    std::enable_if_t<rank<E_>() == 2, Projectivity&>
			operator =(const E_& expr)
			{
			    base_type::operator =(expr);
			    return *this;
			}

    using		base_type::nrow;
    using		base_type::ncol;

    template <class ITER_>
    void		fit(ITER_ begin, ITER_ end, bool refine=false)	;

  //! この射影変換の入力空間の次元を返す．
  /*!
    \return	入力空間の次元(同次座標のベクトルとしての次元は inDim()+1)
  */
    size_t		inDim()			const	{return ncol()-1;}

  //! この射影変換の出力空間の次元を返す．
  /*!
    \return	出力空間の次元(同次座標のベクトルとしての次元は outDim()+1)
  */
    size_t		outDim()		const	{return nrow()-1;}

    size_t		nparams()				 const	;
    size_t		ndataMin()				 const	;
    Projectivity	inverse()				 const	;
    template <class T_, size_t D_>
    point_type		operator ()(const Array<T_, D_>& x)	 const	;
    template <size_t DO_=DO, size_t DI_=DI>
    std::enable_if_t<DO_ == 2 && DI_ == 2, point_type>
			operator ()(element_type u,
				    element_type v)		 const	;
    template <class T_, size_t D_>
    ppoint_type		mapP(const Array<T_, D_>& x)		 const	;
    template <class IN, class OUT>
    element_type	square_distance(
			    const std::pair<IN, OUT>& pair)	 const	;
    template <class IN, class OUT>
    element_type	distance(const std::pair<IN, OUT>& pair) const	;
    template <class T_, size_t D_>
    jacobian_type	jacobian(const Array<T_, D_>& x)	 const	;
    template <class T_, size_t D_>
    derivative_type	derivative(const Array<T_, D_>& x)	 const	;
    template <size_t DO_=DO, size_t DI_=DI>
    static std::enable_if_t<DO_ == 2 && DI_ == 2, derivative0_type>
			derivative0(element_type u, element_type v)
			{
			    constexpr element_type	_0 = 0;
			    constexpr element_type	_1 = 1;

			    return {{ u,  v, _1, _0, _0, _0, -u*u, -u*v},
				    {_0, _0, _0,  u,  v, _1, -u*v, -v*v}};
			}
    void		update(const Array<element_type, NPARAMS>& dt)	;
    template <size_t DO_=DO, size_t DI_=DI>
    std::enable_if_t<DO_ == 2 && DI_ == 2>
			compose(const Array<element_type, DOF>& dt)	;
    template <class ITER_>
    element_type	rmsError(ITER_ begin, ITER_ end)	 const	;

  protected:
  //! 射影変換行列の最尤推定のためのコスト関数
    template <class MAP_, class ITER_>
    class Cost
    {
      public:
	using element_type	= typename MAP_::element_type;
	using vector_type	= Array<element_type>;
	using matrix_type	= Array2<element_type>;

      public:
	Cost(ITER_ begin, ITER_ end)					;

	vector_type	operator ()(const MAP_& map)		const	;
	matrix_type	derivative(const MAP_& map)		const	;
	static void	update(MAP_& map, const vector_type& dm)	;

      private:
	const ITER_	_begin, _end;
	const size_t	_npoints;
    };
};

//! 入力空間と出力空間の次元を指定して射影変換オブジェクトを生成する．
/*!
  恒等変換として初期化される．
  \param outDim	出力空間の次元
  \param inDim	入力空間の次元
*/
template <class T, size_t DO, size_t DI>
Projectivity<T, DO, DI>::Projectivity(size_t outDim, size_t inDim)
    :base_type(outDim + 1, inDim + 1)
{
    const auto	n = std::min(inDim, outDim);
    for (size_t i = 0; i < n; ++i)
	(*this)[i][i] = 1;
    (*this)[outDim][inDim] = 1;
}

//! 与えられた点対列の非同次座標から射影変換オブジェクトを生成する．
/*!
  \param begin			点対列の先頭を示す反復子
  \param end			点対列の末尾を示す反復子
  \param refine			非線型最適化の有(true)／無(false)を指定
  \throw std::invalid_argument	点対の数が ndataMin() に満たない場合に送出
*/
template <class T, size_t DO, size_t DI> template <class ITER_> inline
Projectivity<T, DO, DI>::Projectivity(ITER_ begin, ITER_ end, bool refine)
{
    fit(begin, end, refine);
}

//! 与えられた点対列の非同次座標から射影変換を計算する．
/*!
  \param begin			点対列の先頭を示す反復子
  \param end			点対列の末尾を示す反復子
  \param refine			非線型最適化の有(true)／無(false)を指定
  \throw std::invalid_argument	点対の数が ndataMin() に満たない場合に送出
*/
template <class T, size_t DO, size_t DI> template <class ITER_> void
Projectivity<T, DO, DI>::fit(ITER_ begin, ITER_ end, bool refine)
{
  // 点列の正規化
    const Normalize<element_type, DI>
			xNormalize(make_first_iterator(begin),
				   make_first_iterator(end));
    const Normalize<element_type, DO>
			yNormalize(make_second_iterator(begin),
				   make_second_iterator(end));

  // 充分な個数の点対があるか？
    const auto	ndata = std::distance(begin, end);
    const auto	xdim1 = xNormalize.spaceDim() + 1;
    const auto	ydim  = yNormalize.spaceDim();
    if (ndata*ydim < xdim1*(ydim + 1) - 1)	// 行列のサイズが未定なので
						// ndataMin()は使えない
	throw std::invalid_argument("Projectivity::fit(): not enough input data!!");

  // データ行列の計算
    Matrix<element_type, DO1*DI1, DO1*DI1>	A(xdim1*(ydim + 1),
						  xdim1*(ydim + 1));
    for (auto iter = begin; iter != end; ++iter)
    {
	const auto	x  = xNormalize.normalizeP(iter->first);
	const auto	y  = yNormalize(iter->second);
	const auto	xx = evaluate(x % x);
	A(0, xdim1, 0, xdim1) += xx;
	for (size_t j = 0; j < ydim; ++j)
	    A(ydim*xdim1, xdim1, j*xdim1, xdim1) -= y[j] * xx;
	A(ydim*xdim1, xdim1, ydim*xdim1, xdim1) += (y*y) * xx;
    }
    for (size_t j = 1; j < ydim; ++j)
	A(j*xdim1, xdim1, j*xdim1, xdim1) = A(0, xdim1, 0, xdim1);
    symmetrize(A);

  // データ行列の最小固有値に対応する固有ベクトルから変換行列を計算し，
  // 正規化をキャンセルする．
    Vector<element_type, DI1*DO1>	eval;
    const auto	Ut = eigen(A, eval);
    operator =(yNormalize.Tinv() *
	       make_dense_range(Ut[Ut.nrow()-1].begin(),
				ydim + 1, xdim1) * xNormalize.T());

  // 変換行列が正方ならば，その行列式が１になるように正規化する．
    if (nrow() == ncol())
    {
	const auto	d = det(*this);
	if (d > 0)
	    *this /=  pow( d, element_type(1)/nrow());
	else
	    *this /= -pow(-d, element_type(1)/nrow());
    }

  // 非線型最適化を行う．
    if (refine)
    {
	Cost<Projectivity, ITER_>		cost(begin, end);
	ConstNormConstraint<Projectivity>	constraint(*this);
	minimizeSquare(cost, constraint, *this);
    }
}

//! この射影変換の逆変換を返す．
/*!
  \return	逆変換
*/
template <class T, size_t DO, size_t DI> inline Projectivity<T, DO, DI>
Projectivity<T, DO, DI>::inverse() const
{
    return TU::inverse(*this);
}

//! この射影変換のパラメータ数を返す．
/*!
  射影変換行列の要素数であり，変換の自由度数とは異なる．
  \return	射影変換のパラメータ数((outDim()+1) x (inDim()+1))
*/
template <class T, size_t DO, size_t DI> inline size_t
Projectivity<T, DO, DI>::nparams() const
{
    return (outDim() + 1)*(inDim() + 1);
}

//! 射影変換を求めるために必要な点対の最小個数を返す．
/*!
  現在設定されている入出力空間の次元をもとに計算される．
  \return	必要な点対の最小個数すなわち入力空間の次元m，出力空間の次元n
		に対して m + 1 + m/n
*/
template <class T, size_t DO, size_t DI> inline size_t
Projectivity<T, DO, DI>::ndataMin() const
{
    return inDim() + 1
	 + size_t(std::ceil(element_type(inDim()) / element_type(outDim())));
}

//! 与えられた点に射影変換を適用してその非同次座標を返す．
/*!
  \param x	点の非同次座標(inDim()次元)または同次座標(inDim()+1 次元)
  \return	射影変換された点の非同次座標(outDim() 次元)
*/
template <class T, size_t DO, size_t DI> template <class T_, size_t D_>
inline auto
Projectivity<T, DO, DI>::operator ()(const Array<T_, D_>& x) const -> point_type
{
    if (x.size() == inDim())
	return inhomogeneous(*this * homogeneous(x));
    else
	return inhomogeneous(*this * x);
}

template <class T, size_t DO, size_t DI> template <size_t DO_, size_t DI_>
inline std::enable_if_t<DO_ == 2 && DI_ == 2,
			typename Projectivity<T, DO, DI>::point_type>
Projectivity<T, DO, DI>::operator ()(element_type u, element_type v) const
{
    const auto	w = element_type(1)
		  / ((*this)[2][0]*u + (*this)[2][1]*v + (*this)[2][2]);
    return {{w * ((*this)[0][0]*u + (*this)[0][1]*v + (*this)[0][2]),
	     w * ((*this)[1][0]*u + (*this)[1][1]*v + (*this)[1][2])}};
}

//! 与えられた点に射影変換を適用してその同次座標を返す．
/*!
  \param x	点の非同次座標(inDim() 次元)または同次座標(inDim()+1 次元)
  \return	射影変換された点の同次座標(outDim()+1 次元)
*/
template <class T, size_t DO, size_t DI> template <class T_, size_t D_>
inline auto
Projectivity<T, DO, DI>::mapP(const Array<T_, D_>& x) const -> ppoint_type
{
    if (x.size() == inDim())
	return *this * homogeneous(x);
    else
	return *this * x;
}

//! 入力点に射影変換を適用した点と出力点の距離の2乗を返す．
/*!
  \param pair	入力点の非同次座標(inDim() 次元)と出力点の非同次座標
		(outDim() 次元)の対
  \return	変換された入力点と出力点の距離の2乗
*/
template <class T, size_t DO, size_t DI> template <class IN, class OUT>
inline auto
Projectivity<T, DO, DI>::square_distance(const std::pair<IN, OUT>& pair) const
    -> element_type
{
    return TU::square_distance((*this)(pair.first), pair.second);
}

//! 入力点に射影変換を適用した点と出力点の距離を返す．
/*!
  \param pair	入力点の非同次座標(inDim() 次元)と出力点の非同次座標
		(outDim() 次元)の対
  \return	変換された入力点と出力点の距離
*/
template <class T, size_t DO, size_t DI> template <class IN, class OUT>
inline auto
Projectivity<T, DO, DI>::distance(const std::pair<IN, OUT>& pair) const
    -> element_type
{
    return sqrt(square_distance(pair));
}

//! 与えられた点においてその点の座標に関するヤコビ行列を返す．
/*!
  \param x	点の非同次座標(inDim() 次元)または同次座標(inDim()+1 次元)
  \return	outDim() x inDim() ヤコビ行列
*/
template <class T, size_t DO, size_t DI> template <class T_, size_t D_> auto
Projectivity<T, DO, DI>::jacobian(const Array<T_, D_>& x) const -> jacobian_type
{
    const auto		y = mapP(x);
    jacobian_type	J(outDim(), inDim());
    for (size_t i = 0; i < J.nrow(); ++i)
	J[i] = slice((*this)[i], 0, J.ncol())
	     - (y[i]/y[outDim()]) * slice((*this)[outDim()], 0, J.ncol());
    J /= y[outDim()];

    return J;
}

//! 与えられた点における1階微分行列を返す．
/*!
  変換された点の射影変換行列成分に関する1階微分を計算する．
  \param x	点の非同次座標(inDim() 次元)または同次座標(inDim()+1 次元)
  \return	outDim() x ((outDim()+1)x(inDim()+1)) 1階微分行列
*/
template <class T, size_t DO, size_t DI> template <class T_, size_t D_> auto
Projectivity<T, DO, DI>::derivative(const Array<T_, D_>& x) const
    -> derivative_type
{
    Array<T, DI1>	xP;
    if (x.size() == inDim())
	xP = homogeneous(x);
    else
	xP = x;
    const auto		y  = mapP(xP);
    derivative_type	J(outDim(), (outDim() + 1)*xP.size());
    for (size_t i = 0; i < J.nrow(); ++i)
    {
	slice(J[i], i*xP.size(), xP.size()) = xP;
	slice(J[i], outDim()*xP.size(), xP.size()) = xP *(-y[i]/y[outDim()]);
    }
    J /= y[outDim()];

    return J;
}

//! 射影変換行列を与えられた量だけ修正する．
/*!
  \param dt	修正量を表すベクトル((outDim()+1) x (inDim()+1) 次元)
*/
template <class T, size_t DO, size_t DI> inline void
Projectivity<T, DO, DI>::update(const Array<element_type, NPARAMS>& dt)
{
    auto	t = make_range(base_type::data(), nparams());
    const auto	norm = length(t);
    t -= dt;
    t *= (norm / length(t));	// 修正の前後で射影変換行列のノルムは不変
}

template <class T, size_t DO, size_t DI> template <size_t DO_, size_t DI_>
inline std::enable_if_t<DO_ == 2 && DI_ == 2>
Projectivity<T, DO, DI>::compose(const Array<element_type, DOF>& dt)
{
    auto	t0 = (*this)[0][0];
    auto	t1 = (*this)[0][1];
    auto	t2 = (*this)[0][2];

    (*this)[0][0] -= (t0*dt[0] + t1*dt[3] + t2*dt[6]);
    (*this)[0][1] -= (t0*dt[1] + t1*dt[4] + t2*dt[7]);
    (*this)[0][2] -= (t0*dt[2] + t1*dt[5]);

    t0 = (*this)[1][0];
    t1 = (*this)[1][1];
    t2 = (*this)[1][2];
    (*this)[1][0] -= (t0*dt[0] + t1*dt[3] + t2*dt[6]);
    (*this)[1][1] -= (t0*dt[1] + t1*dt[4] + t2*dt[7]);
    (*this)[1][2] -= (t0*dt[2] + t1*dt[5]);

    t0 = (*this)[2][0];
    t1 = (*this)[2][1];
    t2 = (*this)[2][2];
    (*this)[2][0] -= (t0*dt[0] + t1*dt[3] + t2*dt[6]);
    (*this)[2][1] -= (t0*dt[1] + t1*dt[4] + t2*dt[7]);
    (*this)[2][2] -= (t0*dt[2] + t1*dt[5]);
}

//! 与えられた点対列の平均変換誤差を返す．
/*!
  \param begin	点対列の先頭を示す反復子
  \param end	点対列の末尾を示す反復子
  \return	平均変換誤差
*/
template <class T, size_t DO, size_t DI> template <class ITER_> auto
Projectivity<T, DO, DI>::rmsError(ITER_ begin, ITER_ end) const -> element_type
{
    element_type	sqrerr_sum = 0;
    size_t		npoints = 0;
    for (auto iter = begin; iter != end; ++iter)
    {
	sqrerr_sum += square((*this)(iter->first) - iter->second);
	++npoints;
    }

    return (npoints > 0 ? std::sqrt(sqrerr_sum / npoints) : 0);
}

template <class T, size_t DO, size_t DI> template <class MAP_, class ITER_>
Projectivity<T, DO, DI>::Cost<MAP_, ITER_>::Cost(ITER_ begin, ITER_ end)
    :_begin(begin), _end(end), _npoints(std::distance(_begin, _end))
{
}

template <class T, size_t DO, size_t DI> template <class MAP_, class ITER_> auto
Projectivity<T, DO, DI>::Cost<MAP_, ITER_>::operator ()(const MAP_& map) const
    -> vector_type
{
    const auto	outDim = map.outDim();
    vector_type	val(_npoints*outDim);
    size_t	n = 0;
    for (auto iter = _begin; iter != _end; ++iter)
    {
	val(n, outDim) = map(iter->first) - iter->second;
	n += outDim;
    }

    return val;
}

template <class T, size_t DO, size_t DI> template <class MAP_, class ITER_> auto
Projectivity<T, DO, DI>::Cost<MAP_, ITER_>::derivative(const MAP_& map) const
    -> matrix_type
{
    const auto	outDim = map.outDim();
    matrix_type	J(_npoints*outDim, map.nparams());
    size_t	n = 0;
    for (auto iter = _begin; iter != _end; ++iter)
    {
	J(n, outDim, 0, J.ncol()) = map.derivative(iter->first);
	n += outDim;
    }

    return J;
}

template <class T, size_t DO, size_t DI> template <class MAP_, class ITER_>
inline void
Projectivity<T, DO, DI>::Cost<MAP_, ITER_>::update(MAP_& map,
						   const vector_type& dm)
{
    map.update(dm);
}

template <class T> using Homography	= Projectivity<T, 2, 2>;
template <class T> using Projectivity22	= Projectivity<T, 2, 2>;
template <class T> using Projectivity23	= Projectivity<T, 2, 3>;
template <class T> using Projectivity33	= Projectivity<T, 3, 3>;

/************************************************************************
*  class Affinity<T, DO, DI>						*
************************************************************************/
//! アフィン変換を行うクラス
/*!
  \f$\TUvec{A}{} \in \TUspace{R}{n\times m}\f$と
  \f$\TUvec{b}{} \in \TUspace{R}{n}\f$を用いてm次元空間の点
  \f$\TUvec{x}{} \in \TUspace{R}{m}\f$をn次元空間の点
  \f$\TUvec{y}{} \simeq \TUvec{A}{}\TUvec{x}{} + \TUvec{b}{}
  \in \TUspace{R}{n}\f$に写す(\f$m \neq n\f$でも構わない)．
*/
template <class T, size_t DO, size_t DI>
class Affinity : public Projectivity<T, DO, DI>
{
  public:
    using base_type		= Projectivity<T, DO, DI>;
    using			base_type::DO1;
    using			base_type::DI1;
    constexpr static size_t	NPARAMS	= DO*DI1;
    constexpr static size_t	DOF	= NPARAMS;

    using			typename base_type::element_type;
    using			typename base_type::point_type;
    using			typename base_type::vector_type;
    using derivative_type	= Array2<element_type, DO, NPARAMS>;
    using derivative0_type	= Array2<element_type, DO, DOF>;


  //! 入力空間と出力空間の次元を指定してアフィン変換オブジェクトを生成する．
  /*!
    恒等変換として初期化される．
    \param outDim	出力空間の次元
    \param inDim	入力空間の次元
  */
			Affinity(size_t outDim=DO, size_t inDim=DI)
			    :base_type(outDim, inDim)			{}

  //! 与えられた点対列の非同次座標からアフィン変換オブジェクトを生成する．
  /*!
    \param begin			点対列の先頭を示す反復子
    \param end				点対列の末尾を示す反復子
    \throw std::invalid_argument	点対の数が ndataMin() に満たない場合に送出
  */
    template <class ITER_>
			Affinity(ITER_ begin, ITER_ end)
			{
			    fit(begin, end);
			}

  //! 変換行列を指定してアフィン変換オブジェクトを生成する．
  /*!
    変換行列の下端行は強制的に 0,0,...,0,1 に設定される．
    \param T	(m+1) x (n+1) 行列(m, nは入力／出力空間の次元)
  */
    template <class E_, std::enable_if_t<rank<E_>() == 2>* = nullptr>
			Affinity(const E_& expr)
			    :base_type(expr)
			{
			    (*this)[outDim()] = 0;
			    (*this)[outDim()][inDim()] = 1;
			}

  //! 変換行列を指定する．
  /*!
    変換行列の下端行は強制的に 0,0,...,0,1 に設定される．
    \param T	(m+1) x (n+1) 行列(m, nは入力／出力空間の次元)
  */
    template <class E_>
    std::enable_if_t<rank<E_>() == 2, Affinity&>
			operator =(const E_& expr)
			{
			    base_type::operator =(expr);
			    (*this)[outDim()] = 0;
			    (*this)[outDim()][inDim()] = 1;
			    return *this;
			}

    using		base_type::inDim;
    using		base_type::outDim;
    using		base_type::operator ();

    template <class ITER_>
    void		fit(ITER_ begin, ITER_ end)			;
    size_t		nparams()				const	;
    size_t		ndataMin()				const	;
    Affinity		inverse()				const	;
    template <size_t DO_=DO, size_t DI_=DI>
    std::enable_if_t<DO_ == 2 && DI_ == 2, point_type>
			operator ()(element_type u,
				    element_type v)		const	;
    template <class T_, size_t D_>
    derivative_type	derivative(const Array<T_, D_>& x)	const	;
    template <size_t DO_=DO, size_t DI_=DI>
    static std::enable_if_t<DO_ == 2 && DI_ == 2, derivative0_type>
			derivative0(element_type u, element_type v)
			{
			    constexpr element_type	_0 = 0;
			    constexpr element_type	_1 = 1;

			    return {{ u,  v, _1, _0, _0, _0},
				    {_0, _0, _0,  u,  v, _1}};
			}
    void		update(const Array<element_type, NPARAMS>& dt)	;
    template <size_t DO_=DO, size_t DI_=DI>
    std::enable_if_t<DO_ == 2 && DI_ == 2>
			compose(const Array<element_type, DOF>& dt)	;

  //! このアフィン変換の変形部分を表現する行列を返す．
  /*!
    \return	outDim() x inDim() 行列
  */
    auto	A()	const	{return slice(*this, 0, outDim(), 0, inDim());}
    auto	b()	const	;
};

//! 与えられた点対列の非同次座標からアフィン変換を計算する．
/*!
  \param begin			点対列の先頭を示す反復子
  \param end			点対列の末尾を示す反復子
  \throw std::invalid_argument	点対の数が ndataMin() に満たない場合に送出
*/
template<class T, size_t DO, size_t DI> template <class ITER_> void
Affinity<T, DO, DI>::fit(ITER_ begin, ITER_ end)
{
  // 入力点列の正規化
    const Normalize<element_type, DI>	normalize(make_first_iterator(begin),
						  make_first_iterator(end));

  // 充分な個数の点対があるか？
    const size_t			ndata = std::distance(begin, end);
    if (ndata == 0)		// beginが有効か？
	throw std::invalid_argument("Affinity::fit(): 0-length input data!!");
    const auto				xdim = normalize.spaceDim();
    if (ndata < xdim + 1)	// 行列のサイズが未定なのでndataMin()は無効
	throw std::invalid_argument("Affinity::fit(): not enough input data!!");

  // データ行列の計算
    const auto				ydim = begin->second.size();
    Matrix<element_type, DI1, DI1>	M(xdim + 1, xdim + 1);
    Matrix<element_type, DO,  DI1>	N(ydim,     xdim + 1);
    for (auto iter = begin; iter != end; ++iter)
    {
	const auto	x = normalize.normalizeP(iter->first);
	M += x % x;
	N += iter->second % x;
    }
    const auto	Minv = TU::inverse(M);

  // 変換行列をセットする．
    for (size_t j = 0; j < ydim; ++j)
    {
	(*this)[j] = Minv * N[j];
	slice((*this)[j], 0, xdim) /= normalize.scale();
	(*this)[j][xdim] -= slice((*this)[j], 0, xdim) * normalize.centroid();
    }
}

//! このアフィン変換の並行移動部分を表現するベクトルを返す．
/*!
  \return	outDim() 次元ベクトル
*/
template <class T, size_t DO, size_t DI> auto
Affinity<T, DO, DI>::b() const
{
    Vector<element_type, DO>	bb(outDim());
    for (size_t j = 0; j < bb.size(); ++j)
	bb[j] = (*this)[j][inDim()];

    return bb;
}

//! このアフィン変換の独立なパラメータ数を返す．
/*!
  アフィン変換行列の最初のoutDim()行の要素数であり，変換の自由度数と一致する．
  \return	アフィン変換のパラメータ数(outDim() x (inDim()+1))
*/
template <class T, size_t DO, size_t DI> inline size_t
Affinity<T, DO, DI>::nparams() const
{
    return outDim()*(inDim() + 1);
}

//! アフィン変換を求めるために必要な点対の最小個数を返す．
/*!
  現在設定されている入出力空間の次元をもとに計算される．
  \return	必要な点対の最小個数すなわち入力空間の次元mに対して m + 1
*/
template<class T, size_t DO, size_t DI> inline size_t
Affinity<T, DO, DI>::ndataMin() const
{
    return inDim() + 1;
}

//! このアフィン変換の逆変換を返す．
/*!
  \return	逆変換
*/
template <class T, size_t DO, size_t DI> inline Affinity<T, DO, DI>
Affinity<T, DO, DI>::inverse() const
{
    return TU::inverse(*this);
}

template <class T, size_t DO, size_t DI> template <size_t DO_, size_t DI_>
inline std::enable_if_t<DO_ == 2 && DI_ == 2,
			typename Affinity<T, DO, DI>::point_type>
Affinity<T, DO, DI>::operator ()(element_type u, element_type v) const
{
    return point_type({(*this)[0][0]*u + (*this)[0][1]*v + (*this)[0][2],
		       (*this)[1][0]*u + (*this)[1][1]*v + (*this)[1][2]});
}

//! 与えられた点における1階微分行列を返す．
/*!
  変換された点のアフィン変換行列成分に関する1階微分を計算する．
  \param x	点の非同次座標(inDim() 次元)または同次座標(inDim()+1次元)
  \return	outDim() x (outDim()x(inDim()+1)) 1階微分行列
*/
template <class T, size_t DO, size_t DI> template <class T_, size_t D_> auto
Affinity<T, DO, DI>::derivative(const Array<T_, D_>& x) const -> derivative_type
{
    vector_type	xP;
    if (x.size() == inDim())
	xP = homogeneous(x);
    else
	xP = x;
    const vector_type&	y = mapP(xP);
    derivative_type	J(outDim(), outDim()*xP.size());
    for (size_t i = 0; i < J.nrow(); ++i)
	slice(J[i], i*xP.size(), xP.size()) = xP;
    J /= y[outDim()];

    return J;
}

//! アフィン変換行列を与えられた量だけ修正する．
/*!
  \param dt	修正量を表すベクトル(outDim() x (inDim()+1) 次元)
*/
template <class T, size_t DO, size_t DI> inline void
Affinity<T, DO, DI>::update(const Array<element_type, NPARAMS>& dt)
{
    make_range(base_type::data(), nparams()) -= dt;
}

template <class T, size_t DO, size_t DI> template <size_t DO_, size_t DI_>
inline std::enable_if_t<DO_ == 2 && DI_ == 2>
Affinity<T, DO, DI>::compose(const Array<element_type, DOF>& dt)
{
    auto	t0 = (*this)[0][0];
    auto	t1 = (*this)[0][1];
    (*this)[0][0] -= (t0*dt[0] + t1*dt[3]);
    (*this)[0][1] -= (t0*dt[1] + t1*dt[4]);
    (*this)[0][2] -= (t0*dt[2] + t1*dt[5]);

    t0 = (*this)[1][0];
    t1 = (*this)[1][1];
    (*this)[1][0] -= (t0*dt[0] + t1*dt[3]);
    (*this)[1][1] -= (t0*dt[1] + t1*dt[4]);
    (*this)[1][2] -= (t0*dt[2] + t1*dt[5]);
}

template <class T> using Affinity11	= Affinity<T, 1, 1>;
template <class T> using Affinity12	= Affinity<T, 1, 2>;
template <class T> using Affinity22	= Affinity<T, 2, 2>;
template <class T> using Affinity23	= Affinity<T, 2, 3>;
template <class T> using Affinity33	= Affinity<T, 3, 3>;

/************************************************************************
*  class Similarity<T, D>						*
************************************************************************/
//! 相似変換を行うクラス
/*!
  回転行列\f$\TUvec{R}{} \in \TUspace{SO}{n}\f$と
  \f$\TUvec{t}{} \in \TUspace{R}{n}\f$を用いてm次元空間の点
  \f$\TUvec{x}{} \in \TUspace{R}{n}\f$をn次元空間の点
  \f$\TUvec{y}{} = s\TUvec{R}{}\TUvec{x}{} + \TUvec{t}{}
  \in \TUspace{R}{n}\f$に写す．
*/
template <class T, size_t D>
class Similarity : public Affinity<T, D, D>
{
  public:
    using			base_type = Affinity<T, D, D>;

    constexpr static size_t	NPARAMS = D*(D+1)/2 + 1;
    constexpr static size_t	DOF	= NPARAMS;

    using			typename base_type::element_type;
    using derivative_type	= Array2<element_type, D, NPARAMS>;
    using derivative0_type	= Array2<element_type, D, DOF>;

  //! 入力空間と出力空間の次元を指定して相似変換オブジェクトを生成する．
  /*!
    恒等変換として初期化される．
    \param d	入力/出力空間の次元
  */
    explicit	Similarity(size_t d=D) :base_type(d, d)	{}

  //! 与えられた点対列の非同次座標から相似変換オブジェクトを生成する．
  /*!
    \param begin			点対列の先頭を示す反復子
    \param end				点対列の末尾を示す反復子
    \throw std::invalid_argument	点対の数が ndataMin() に満たない場合に送出
  */
    template <class ITER_>
			Similarity(ITER_ begin, ITER_ end)
			{
			    fit(begin, end);
			}

  //! 変換行列を指定して相似変換オブジェクトを生成する．
  /*!
    \param T	(d+1) x (d+1)行列(dは入力/出力空間の次元)
  */
    template <class E_, std::enable_if_t<rank<E_>() == 2>* = nullptr>
			Similarity(const E_& expr)
			    :base_type(expr)
			{
			}

  //! 変換行列を指定する．
  /*!
    \param T	(d+1) x (d+1) 行列(dは入力/出力空間の次元)
  */
    template <class E_> std::enable_if_t<rank<E_>() == 2, Similarity&>
			operator =(const E_& expr)
			{
			    if (TU::size<0>(expr) != TU::size<1>(expr))
				throw std::invalid_argument("Similarity::set(): non-square matrix!!");
			    base_type::operator =(expr);
			    return *this;
			}

    using		base_type::inDim;
    using		base_type::outDim;
    using		base_type::operator ();

  //! この相似変換の入力/出力空間の次元を返す．
  /*!
    \return	入力/出力空間の次元(同次座標のベクトルとしての次元は dim()+1)
  */
    size_t		dim()		const	{ return inDim(); };

    auto		s()		const	;
    auto		R()		const	;

  //! この相似変換の回転部分を表現する回転行列を返す．
  /*!
    \return	dim() x dim() 行列
  */
    auto		sR()		const	{ return base_type::A(); }

  //! この相似変換の並行移動部分を表現するベクトルを返す．
  /*!
    \return	dim() 次元ベクトル
  */
    auto		t()		const	{ return base_type::b(); }

    template <class ITER_>
    element_type	fit(ITER_ begin, ITER_ end)			;
    Similarity		inverse()				const	;
    size_t		nparams()				const	;
    size_t		ndataMin()				const	;
    template <class T_, size_t D_>
    derivative_type	derivative(const Array<T_, D_>& x)	const	;
    template <size_t D_=D>
    static std::enable_if_t<D_ == 2, derivative0_type>
			derivative0(element_type u, element_type v)
			{
			    constexpr element_type	_0 = 0;
			    constexpr element_type	_1 = 1;

			    return {{_1, _0, -v}, {_0, _1,  u}};
			}
    void		update(const Array<element_type, NPARAMS>& dt)	;
    template <size_t D_=D>
    std::enable_if_t<D_ == 2>
			compose(const Array<element_type, DOF>& dt)	;
};

//! この相似変換のスケーリング係数を返す．
/*!
  \return	スケーリング係数
*/
template<class T, size_t D> inline auto
Similarity<T, D>::s() const
{
    return length(slice((*this)[0], 0, dim()));
}

//! この相似変換の回転部分を表現する回転行列を返す．
/*!
  \return	dim() x dim() 行列
*/
template<class T, size_t D> inline auto
Similarity<T, D>::R() const
{
    Matrix<T, D, D>	RR = sR();
    RR /= s();

    return RR;
}


//! 与えられた点対列の非同次座標から相似変換を計算する．
/*!
  \param begin			点対列の先頭を示す反復子
  \param end			点対列の末尾を示す反復子
  \throw std::invalid_argument	点対の数が ndataMin() に満たない場合に送出
*/
template<class T, size_t D> template <class ITER_>
typename Similarity<T, D>::element_type
Similarity<T, D>::fit(ITER_ begin, ITER_ end)
{
  // 充分な個数の点対があるか？
    const size_t	ndata = std::distance(begin, end);
    if (ndata == 0)		// beginが有効か？
	throw std::invalid_argument("Similarity::fit(): 0-length input data!!");
    const auto		d = begin->first.size();
    if (begin->second.size() != d)
	throw std::invalid_argument("Similarity::fit(): input data contains a pair of different dimensions!!");
    if (ndata < d)		// 行列のサイズが未定なのでndataMin()は無効
	throw std::invalid_argument("Similarity::fit(): not enough input data!!");

  // 重心の計算
    Array<element_type, D>	xc(d), yc(d);
    for (auto corres = begin; corres != end; ++corres)
    {
	xc += corres->first;
	yc += corres->second;
    }
    xc /= ndata;
    yc /= ndata;

  // モーメント行列の計算
    element_type		sqr_dx = 0;
    element_type		sqr_dy = 0;
    Matrix<element_type, D, D>	M(d, d);
    for (auto corres = begin; corres != end; ++corres)
    {
	const auto	dx = corres->first  - xc;
	const auto	dy = corres->second - yc;

	sqr_dx += square(dx);
	sqr_dy += square(dy);
	M      += dx % dy;
    }

  // モーメント行列の特異値がすべて正であることを確認
    SVDecomposition<element_type>	svd(M);
    for (auto sigma : svd.diagonal())
	if (sigma <= 0)
	    throw std::runtime_error("Similarity::fit(): non-positive singula value!!");

  // スケールの計算
    element_type	scale = 0;
    for (auto sigma : svd.diagonal())
	scale += sigma;
    scale /= sqr_dx;

  // 点群間の相似変換の計算
    base_type::resize(d + 1, d + 1);
    slice(*this, 0, d, 0, d) = scale * transpose(svd.Ut()) * svd.Vt();
    for (size_t i = 0; i < d; ++i)
	(*this)[i][d] = yc[i] - slice((*this)[i], 0, d) * xc;
    (*this)[d][d] = 1;

    return std::sqrt(std::abs(sqr_dy - square(scale)*sqr_dx)/ndata);
}

//! この相似変換の逆変換を返す．
/*!
  \return	逆変換
*/
template <class T, size_t D> inline Similarity<T, D>
Similarity<T, D>::inverse() const
{
    Similarity	Tinv(inDim());

    const auto	sqinv_s = 1 / square(slice((*this)[0], 0, dim()));
    for (size_t i = 0; i < dim(); ++i)
	for (size_t j = 0; j < dim(); ++j)
	    Tinv[j][i] = sqinv_s * (*this)[i][j];

    Vector<element_type, D>	tt = t();
    for (size_t j = 0; j < dim(); ++j)
	Tinv[j][dim()] = -(slice(Tinv[j], 0, dim()) * tt);

    return Tinv;
}

//! この相似変換の独立なパラメータ数を返す．
/*!
  相似変換の独立なパラメータ数すなわち変換の自由度数に一致する．
  \return	相似変換のパラメータ数(dim() x (dim()+1))/2
*/
template <class T, size_t D> inline size_t
Similarity<T, D>::nparams() const
{
    return (dim()*(dim() + 1))/2 + 1;
}

//! 相似変換を求めるために必要な点対の最小個数を返す．
/*!
  現在設定されている空間の次元をもとに計算される．
  \return	必要な点対の最小個数すなわち空間の次元mに対してm
*/
template<class T, size_t D> inline size_t
Similarity<T, D>::ndataMin() const
{
    return dim();
}

//! 与えられた点における1階微分行列を返す．
/*!
  変換された点の並進/回転/スケーリングパラメータに関する1階微分を計算する．
  \param x	点の非同次座標(dim() 次元)または同次座標(dim()+1 次元)
  \return	dim() x ((dim()x(dim()+1))/2 + 1)1階微分行列
*/
template <class T, size_t D> template <class T_, size_t D_> auto
Similarity<T, D>::derivative(const Array<T_, D_>& x) const -> derivative_type
{
    Array<element_type, D>	xx;
    if (x.size() == dim())
	xx = x;
    else
	xx = inhomogeneous(x);

    derivative_type	J(dim(), nparams());

    switch (dim())
    {
      case 2:
	J[0][0] = J[1][1] = 1;
	J[0][2] = -(slice<2>((*this)[1], 0) * xx);
	J[1][2] =   slice<2>((*this)[0], 0) * xx;
	break;
      case 3:
      {
	J[0][0] = J[1][1] = J[2][2] = 1;
	const Vector<element_type, D>	sRx = sR() * xx;
	slice<3, 3>(J, 0, 3) = skew(sRx);
	const auto	scale = s();
	J[0][6] = sRx[0] / scale;
	J[1][6] = sRx[1] / scale;
	J[2][6] = sRx[2] / scale;
	break;
      }
      default:
	throw std::runtime_error("Similarity<T, D>::jacobian(): sorry, not implemented yet...");
    }

    return J;
}

//! 相似変換行列を与えられた量だけ修正する．
/*!
  \param dt	修正量を表すベクトル(dim() x (dim()+1)/2 次元)
*/
template <class T, size_t D> void
Similarity<T, D>::update(const Array<element_type, NPARAMS>& dt)
{
    const auto	scale = s();

    switch (dim())
    {
      case 2:
	(*this)[0][dim()] -= dt[0];
	(*this)[1][dim()] -= dt[1];
	slice<2, 2>(*this, 0, 0) = evaluate((1 - dt[3]/scale) *
					    rotation(-dt[2]) *
					    slice<2, 2>(*this, 0, 0));
	break;
      case 3:
	(*this)[0][dim()] -= dt[0];
	(*this)[1][dim()] -= dt[1];
	(*this)[2][dim()] -= dt[2];
	slice<3, 3>(*this, 0, 0) = evaluate((1 - dt[6]/scale) *
					    rotation(-dt(3, 3)) *
					    slice<3, 3>(*this, 0, 0));
	break;
      default:
	throw std::runtime_error("Similarity<T, D>::update(): sorry, not implemented yet...");
    }
}

template <class T, size_t D> template <size_t D_>
inline std::enable_if_t<D_ == 2>
Similarity<T, D>::compose(const Array<element_type, DOF>& dt)
{
    const auto	Rt = rotation(dt[2]);
    const auto	k  = (1 - dt[3]/s());

    auto	r0 = (*this)[0][0];
    auto	r1 = (*this)[0][1];
    (*this)[0][0]  = k * (r0*Rt[0][0] + r1*Rt[1][0]);
    (*this)[0][1]  = k * (r0*Rt[0][1] + r1*Rt[1][1]);
    (*this)[0][2] -= ((*this)[0][0]*dt[0] + (*this)[0][1]*dt[1]);

    r0 = (*this)[1][0];
    r1 = (*this)[1][1];
    (*this)[1][0]  = k * (r0*Rt[0][0] + r1*Rt[1][0]);
    (*this)[1][1]  = k * (r0*Rt[0][1] + r1*Rt[1][1]);
    (*this)[1][2] -= ((*this)[1][0]*dt[0] + (*this)[1][1]*dt[1]);
}

template <class T> using Similarity2	= Similarity<T, 2>;
template <class T> using Similarity3	= Similarity<T, 3>;

/************************************************************************
*  class Rigidity<T, D>							*
************************************************************************/
//! 剛体変換を行うクラス
/*!
  回転行列\f$\TUvec{R}{} \in \TUspace{SO}{n}\f$と
  \f$\TUvec{t}{} \in \TUspace{R}{n}\f$を用いてm次元空間の点
  \f$\TUvec{x}{} \in \TUspace{R}{n}\f$をn次元空間の点
  \f$\TUvec{y}{} = \TUvec{R}{}\TUvec{x}{} + \TUvec{t}{}
  \in \TUspace{R}{n}\f$に写す．
*/
template <class T, size_t D>
class Rigidity : public Affinity<T, D, D>
{
  public:
    using			base_type = Affinity<T, D, D>;

    constexpr static size_t	NPARAMS = D*(D+1)/2;
    constexpr static size_t	DOF	= NPARAMS;

    using			typename base_type::element_type;
    using derivative_type	= Array2<element_type, D, NPARAMS>;
    using derivative0_type	= Array2<element_type, D, DOF>;

  //! 入力空間と出力空間の次元を指定して剛体変換オブジェクトを生成する．
  /*!
    恒等変換として初期化される．
    \param d	入力/出力空間の次元
  */
    explicit	Rigidity(size_t d=D) :base_type(d, d)	{}

  //! 与えられた点対列の非同次座標から剛体変換オブジェクトを生成する．
  /*!
    \param begin			点対列の先頭を示す反復子
    \param end				点対列の末尾を示す反復子
    \throw std::invalid_argument	点対の数が ndataMin() に満たない場合に送出
  */
    template <class ITER_>
			Rigidity(ITER_ begin, ITER_ end)
			{
			    fit(begin, end);
			}

  //! 変換行列を指定して剛体変換オブジェクトを生成する．
  /*!
    \param T	(d+1) x (d+1)行列(dは入力/出力空間の次元)
  */
    template <class E_, std::enable_if_t<rank<E_>() == 2>* = nullptr>
			Rigidity(const E_& expr)
			    :base_type(expr)
			{
			}

  //! 変換行列を指定する．
  /*!
    \param T	(d+1) x (d+1) 行列(dは入力/出力空間の次元)
  */
    template <class E_> std::enable_if_t<rank<E_>() == 2, Rigidity&>
			operator =(const E_& expr)
			{
			    if (TU::size<0>(expr) != TU::size<1>(expr))
				throw std::invalid_argument("Rigidity::set(): non-square matrix!!");
			    base_type::operator =(expr);
			    return *this;
			}

    using		base_type::inDim;
    using		base_type::outDim;

  //! この剛体変換の入力/出力空間の次元を返す．
  /*!
    \return	入力/出力空間の次元(同次座標のベクトルとしての次元は dim()+1)
  */
    size_t		dim()		const	{ return inDim(); };

  //! この剛体変換の回転部分を表現する回転行列を返す．
  /*!
    \return	dim() x dim() 行列
  */
    auto		R()		const	{ return base_type::A(); }

  //! この剛体変換の並行移動部分を表現するベクトルを返す．
  /*!
    \return	dim() 次元ベクトル
  */
    auto		t()		const	{ return base_type::b(); }

    template <class ITER_>
    element_type	fit(ITER_ begin, ITER_ end)			;
    Rigidity		inverse()				const	;
    size_t		nparams()				const	;
    size_t		ndataMin()				const	;
    template <class T_, size_t D_>
    derivative_type	derivative(const Array<T_, D_>& x)	const	;
    template <size_t D_=D>
    static std::enable_if_t<D_ == 2, derivative0_type>
			derivative0(element_type u, element_type v)
			{
			    constexpr element_type	_0 = 0;
			    constexpr element_type	_1 = 1;

			    return {{_1, _0, -v}, {_0, _1,  u}};
			}
    void		update(const Array<element_type, NPARAMS>& dt)	;
    template <size_t D_=D>
    std::enable_if_t<D_ == 2>
			compose(const Array<element_type, DOF>& dt)	;
};

//! 与えられた点対列の非同次座標から剛体変換を計算する．
/*!
  \param begin			点対列の先頭を示す反復子
  \param end			点対列の末尾を示す反復子
  \throw std::invalid_argument	点対の数が ndataMin() に満たない場合に送出
*/
template<class T, size_t D> template <class ITER_>
typename Rigidity<T, D>::element_type
Rigidity<T, D>::fit(ITER_ begin, ITER_ end)
{
  // 充分な個数の点対があるか？
    const size_t	ndata = std::distance(begin, end);
    if (ndata == 0)		// beginが有効か？
	throw std::invalid_argument("Rigidity::fit(): 0-length input data!!");
    const auto		d = begin->first.size();
    if (begin->second.size() != d)
	throw std::invalid_argument("Rigidity::fit(): input data contains a pair of different dimensions!!");
    if (ndata < d)		// 行列のサイズが未定なのでndataMin()は無効
	throw std::invalid_argument("Rigidity::fit(): not enough input data!!");

  // 重心の計算
    Array<element_type, D>	xc(d), yc(d);
    for (auto corres = begin; corres != end; ++corres)
    {
	xc += corres->first;
	yc += corres->second;
    }
    xc /= ndata;
    yc /= ndata;

  // モーメント行列の計算
    element_type	sqr_d = 0;
    Matrix<element_type, D, D>	M(d, d);
    for (auto corres = begin; corres != end; ++corres)
    {
	const auto	dx = corres->first - xc;
	const auto	dy = corres->second - yc;

	sqr_d += (square(dx) + square(dy));
	M     += dx % dy;
    }

  // モーメント行列の特異値がすべて正であることを確認
    SVDecomposition<element_type>	svd(M);
    for (auto sigma : svd.diagonal())
    {
	if (sigma <= 0)
	    throw std::runtime_error("Similarity::fit(): non-positive singula value!!");
	sqr_d -= 2*sigma;
    }

  // 点群間の剛体変換の計算
    base_type::resize(d + 1, d + 1);
    (*this)(0, d, 0, d) = transpose(svd.Ut()) * svd.Vt();
    for (size_t i = 0; i < d; ++i)
	(*this)[i][d] = yc[i] - slice((*this)[i], 0, d) * xc;
    (*this)[d][d] = 1;

    return std::sqrt(std::abs(sqr_d)/ndata);
}

//! この剛体変換の逆変換を返す．
/*!
  \return	逆変換
*/
template <class T, size_t D> inline Rigidity<T, D>
Rigidity<T, D>::inverse() const
{
    Rigidity	Tinv(inDim());

    for (size_t i = 0; i < dim(); ++i)
	for (size_t j = 0; j < dim(); ++j)
	    Tinv[j][i] = (*this)[i][j];

    Vector<element_type, D>	tt = t();
    for (size_t j = 0; j < dim(); ++j)
	Tinv[j][dim()] = -(slice(Tinv[j], 0, dim()) * tt);

    return Tinv;
}

//! この剛体変換の独立なパラメータ数を返す．
/*!
  剛体変換の独立なパラメータ数すなわち変換の自由度数に一致する．
  \return	剛体変換のパラメータ数(dim() x (dim()+1))/2
*/
template <class T, size_t D> inline size_t
Rigidity<T, D>::nparams() const
{
    return (dim()*(dim() + 1))/2;
}

//! 剛体変換を求めるために必要な点対の最小個数を返す．
/*!
  現在設定されている空間の次元をもとに計算される．
  \return	必要な点対の最小個数すなわち空間の次元mに対してm
*/
template<class T, size_t D> inline size_t
Rigidity<T, D>::ndataMin() const
{
    return dim();
}

//! 与えられた点における1階微分行列を返す．
/*!
  1階微分行列とは並進/回転パラメータに関する1階微分のことである．
  \param x	点の非同次座標(dim() 次元)または同次座標(dim()+1 次元)
  \return	dim() x (dim()x(dim()+1))/2 1階微分行列
*/
template <class T, size_t D> template <class T_, size_t D_> auto
Rigidity<T, D>::derivative(const Array<T_, D_>& x) const -> derivative_type
{
    Array<element_type, D>	xx;
    if (x.size() == dim())
	xx = x;
    else
	xx = inhomogeneous(x);

    derivative_type	J(dim(), nparams());

    switch (dim())
    {
      case 2:
	J[0][0] = J[1][1] = 1;
	J[0][2] = -(slice<2>((*this)[1], 0) * xx);
	J[1][2] =   slice<2>((*this)[0], 0) * xx;
	break;
      case 3:
	J[0][0] = J[1][1] = J[2][2] = 1;
	slice<3, 3>(J, 0, 3) = skew(R() * xx);
	break;
      default:
	throw std::runtime_error("Rigidity<T, D>::jacobian(): sorry, not implemented yet...");
    }

    return J;
}

//! 剛体変換行列を与えられた量だけ修正する．
/*!
  \param dt	修正量を表すベクトル(dim() x (dim()+1)/2 次元)
*/
template <class T, size_t D> void
Rigidity<T, D>::update(const Array<element_type, NPARAMS>& dt)
{
    switch (dim())
    {
      case 2:
	(*this)[0][dim()] -= dt[0];
	(*this)[1][dim()] -= dt[1];
	slice<2, 2>(*this, 0, 0) = evaluate(rotation(-dt[2]) *
					    slice<2, 2>(*this, 0, 0));
	break;
      case 3:
	(*this)[0][dim()] -= dt[0];
	(*this)[1][dim()] -= dt[1];
	(*this)[2][dim()] -= dt[2];
	slice<3, 3>(*this, 0, 0) = evaluate(rotation(-dt(3, 3)) *
					    slice<3, 3>(*this, 0, 0));
	break;
      default:
	throw std::runtime_error("Rigidity<T, D>::update(): sorry, not implemented yet...");
    }
}

template <class T, size_t D> template <size_t D_>
inline std::enable_if_t<D_ == 2>
Rigidity<T, D>::compose(const Array<element_type, DOF>& dt)
{
    const auto	Rt = rotation(dt[2]);

    auto	r0 = (*this)[0][0];
    auto	r1 = (*this)[0][1];
    (*this)[0][0]  = r0*Rt[0][0] + r1*Rt[1][0];
    (*this)[0][1]  = r0*Rt[0][1] + r1*Rt[1][1];
    (*this)[0][2] -= ((*this)[0][0]*dt[0] + (*this)[0][1]*dt[1]);

    r0 = (*this)[1][0];
    r1 = (*this)[1][1];
    (*this)[1][0]  = r0*Rt[0][0] + r1*Rt[1][0];
    (*this)[1][1]  = r0*Rt[0][1] + r1*Rt[1][1];
    (*this)[1][2] -= ((*this)[1][0]*dt[0] + (*this)[1][1]*dt[1]);
}

template <class T> using Rigidity2	= Rigidity<T, 2>;
template <class T> using Rigidity3	= Rigidity<T, 3>;

/************************************************************************
*   class BoundingBox<P>						*
************************************************************************/
//! P型の点に対するbounding boxを表すクラス
/*!
  \param P	点の型(次元は自由)
*/
template <class P>
class BoundingBox
{
  public:
    using point_type	= P;				//!< 点の型
    using element_type	= typename P::element_type;	//!< 点の要素の型

  public:
    BoundingBox()				;
    explicit BoundingBox(size_t d)		;

    bool		operator !()	const	;
  //! このbounding boxが属する空間の次元を返す．
  /*!
    \return	空間の次元
  */
    size_t		dim()		const	{return _min.size();}

  //! このbounding boxの最小点を返す．
  /*!
    \return	最小点
  */
    const point_type&	min()		const	{return _min;}

  //! このbounding boxの最大点を返す．
  /*!
    \return	最大点
  */
    const point_type&	max()		const	{return _max;}

  //! このbounding boxの最小点の指定された軸の座標値を返す．
  /*!
    \param i	軸を指定するindex
    \return	軸の座標値
  */
    element_type	min(int i)	const	{return _min[i];}

  //! このbounding boxの最大点の指定された軸の座標値を返す．
  /*!
    \param i	軸を指定するindex
    \return	軸の座標値
  */
    element_type	max(int i)	const	{return _max[i];}

  //! このbounding boxの指定された軸に沿った長さを返す．
  /*!
    \param i	軸を指定するindex
    \return	軸に沿った長さ
  */
    element_type	length(int i)	const	{return _max[i] - _min[i];}

  //! このbounding boxの幅を返す．
  /*!
    \return	幅 (TU::BoundingBox::length (0)に等しい)
  */
    element_type	width()		const	{return length(0);}

  //! このbounding boxの高さを返す．
  /*!
    \return	高さ (TU::BoundingBox::length (1)に等しい)
  */
    element_type	height()	const	{return length(1);}

  //! このbounding boxの奥行きを返す．
  /*!
    \return	奥行き (TU::BoundingBox::length (2)に等しい)
  */
    element_type	depth()		const	{return length(2);}

    template <class T_, size_t D_>
    bool		include(const Array<T_, D_>& p)		;
    BoundingBox&	clear()					;
    template <class T_, size_t D_>
    BoundingBox&	expand(const Array<T_, D_>& p)		;
    template <class T_, size_t D_>
    BoundingBox&	operator +=(const Array<T_, D_>& dt)	;
    template <class T_, size_t D_>
    BoundingBox&	operator -=(const Array<T_, D_>& dt)	;
    template <class S>
    BoundingBox&	operator *=(S c)			;
    BoundingBox&	operator |=(const BoundingBox& bbox)	;
    BoundingBox&	operator &=(const BoundingBox& bbox)	;

  private:
  //! 入力ストリームからbounding boxを成す2つの点の座標を入力する(ASCII)．
  /*!
    \param in	入力ストリーム
    \param bbox	bounding box
    \return	inで指定した入力ストリーム
  */
    friend std::istream&
    operator >>(std::istream& in, BoundingBox<P>& bbox)
    {
	return in >> bbox._min >> bbox._max;
    }

    point_type	_min;
    point_type	_max;
};

//! 空のbounding boxを作る．
template <class P> inline
BoundingBox<P>::BoundingBox()
    :_min(), _max()
{
    clear();
}

//! 指定した次元の空間において空のbounding boxを作る．
/*!
  \param d	空間の次元
*/
template <class P> inline
BoundingBox<P>::BoundingBox(size_t d)
    :_min(d), _max(d)
{
    clear();
}

//! bounding boxが空であるか調べる．
/*!
  \return	空であればtrue, そうでなければfalse
*/
template <class P> bool
BoundingBox<P>::operator !() const
{
    for (size_t i = 0; i < dim(); ++i)
	if (_min[i] > _max[i])
	    return true;
    return false;
}

//! bounding boxが与えられた点を含むか調べる．
/*!
  \param p	点の座標
  \return	含めばtrue, そうでなければfalse
*/
template <class P> template <class T_, size_t D_> bool
BoundingBox<P>::include(const Array<T_, D_>& p)
{
    for (size_t i = 0; i < dim(); ++i)
	if (p[i] < _min[i] || p[i] > _max[i])
	    return false;
    return true;
}

//! bounding boxを空にする．
/*!
  \return	空にされたこのbounding box
*/
template <class P> BoundingBox<P>&
BoundingBox<P>::clear()
{
    using Limits = std::numeric_limits<element_type>;

    for (size_t i = 0; i < dim(); ++i)
    {
	_min[i] = Limits::max();
	_max[i] = (Limits::is_integer ? Limits::min() : -Limits::max());
    }
    return *this;
}

//! bounding boxを与えられた点を含むように拡張する．
/*!
  \param p	点の座標
  \return	拡張されたこのbounding box
*/
template <class P> template <class T_, size_t D_> BoundingBox<P>&
BoundingBox<P>::expand(const Array<T_, D_>& p)
{
    for (size_t i = 0; i < dim(); ++i)
    {
	_min[i] = std::min(_min[i], p[i]);
	_max[i] = std::max(_max[i], p[i]);
    }
    return *this;
}

//! bounding boxを与えられた変位だけ正方向に平行移動する．
/*!
  \param dt	変位
  \return	平行移動されたこのbounding box
*/
template <class P> template <class T_, size_t D_>
inline BoundingBox<P>&
BoundingBox<P>::operator +=(const Array<T_, D_>& dt)
{
    _min += dt;
    _max += dt;
    return *this;
}

//! bounding boxを与えられた変位だけ負方向に平行移動する．
/*!
  \param dt	変位
  \return	平行移動されたこのbounding box
*/
template <class P> template <class T_, size_t D_>
inline BoundingBox<P>&
BoundingBox<P>::operator -=(const Array<T_, D_>& dt)
{
    _min -= dt;
    _max -= dt;
    return *this;
}

//! bounding boxを与えられたスケールだけ拡大／縮小する．
/*!
  負のスケールを与えるとbounding boxが反転する．
  \param c	スケール
  \return	平行移動されたこのbounding box
*/
template <class P> template <class S> inline BoundingBox<P>&
BoundingBox<P>::operator *=(S c)
{
    if (c < S(0))
	std::swap(_min, _max);
    _min *= c;
    _max *= c;
    return *this;
}

//! このbounding boxと指定されたbounding boxとの結びをとる．
/*!
  \param bbox	bounding box
  \return	結びをとった後のこのbounding box
*/
template <class P> inline BoundingBox<P>&
BoundingBox<P>::operator |=(const BoundingBox<P>& bbox)
{
    return expand(bbox.min()).expand(bbox.max());
}

//! このbounding boxと指定されたbounding boxとの交わりをとる．
/*!
  与えられたbounding boxとの間に共通部分がなければ空のbounding boxとなる．
  \param bbox	bounding box
  \return	交わりをとった後のこのbounding box
*/
template <class P> BoundingBox<P>&
BoundingBox<P>::operator &=(const BoundingBox<P>& bbox)
{
    for (int i = 0; i < dim(); ++i)
    {
	_min[i] = std::max(_min[i], bbox.min(i));
	_max[i] = std::min(_max[i], bbox.max(i));
    }
    return *this;
}

//! 2つのbounding boxの結びをとる．
/*!
  \param a	bounding box
  \param b	bounding box
  \return	aとbの結びとなるbounding box
*/
template <class P> inline BoundingBox<P>
operator |(const BoundingBox<P>& a, const BoundingBox<P>& b)
{
    BoundingBox<P>	c(a);
    return c |= b;
}

//! 2つのbounding boxの交わりをとる．
/*!
  与えられたbounding boxに共通部分がなければ空のbounding boxを返す．
  \param a	bounding box
  \param b	bounding box
  \return	aとbの交わりとなるbounding box
*/
template <class P> inline BoundingBox<P>
operator &(const BoundingBox<P>& a, const BoundingBox<P>& b)
{
    BoundingBox<P>	c(a);
    return c &= b;
}

//! 出力ストリームにbounding boxを成す2つの点の座標を出力する(ASCII)．
/*!
  \param out	出力ストリーム
  \param bbox	bounding box
  \return	outで指定した出力ストリーム
*/
template <class P> std::ostream&
operator <<(std::ostream& out, const BoundingBox<P>& bbox)
{
#ifdef TU_DEBUG
    for (size_t i = 0; i < bbox.dim(); ++i)
    {
	if (i != 0)
	    out << 'x';
	out << '[' << bbox.min(i) << ", " << bbox.max(i) << ']';
    }
    return out << std::endl;
#else
    return out << bbox.min() << bbox.max() << std::endl;
#endif
}

}
#endif	// !TU_GEOMETRYPP_H
