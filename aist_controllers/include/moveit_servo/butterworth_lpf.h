#pragma once

#include <cmath>

namespace moveit_servo
{
template <class S, class T=S>
class ButterworthLPF
{
  public:
    using element_type	= S;
    using value_type	= T;

  public:
  /*!
    バタワースフィルタを生成
    \param half_order	フィルタの次数の半分
    \param cutoff	カットオフ周波数fcをサンプリング周波数fsで正規化した値
			すなわちfc/fs
   */
    explicit	ButterworthLPF(size_t half_order, element_type cutoff)
		{
		    initialize(half_order, cutoff);
		}

    void	initialize(size_t half_order, element_type cutoff)
		{
		    _cutoff = cutoff;

		    _A.resize(half_order);
		    _d1.resize(_A.size());
		    _d2.resize(_A.size());
		    _y0.resize(_A.size());
		    _y1.resize(_A.size());
		    _y2.resize(_A.size());

		    const auto	a  = std::tan(M_PI*_cutoff);
		    const auto	a2 = a*a;

		    for (size_t k = 0; k < _A.size(); ++k)
		    {
			const auto	n = _A.size();
			const auto	s = std::sin(M_PI*(2*k + 1)/(4*n));
			const auto	t = 1 + 2*a*s + a2;
			_A[k]  = a2/t;
			_d1[k] = 2*(1 - a2)/t;
			_d2[k] = -(1 - 2*a*s + a2)/t;
		    }
		}

    size_t	half_order() const
		{
		    return _A.size();
		}

    double	cutoff() const
		{
		    return _cutoff;
		}

    value_type	filter(value_type x) const
		{
		  // 2次フィルタをカスケード接続
		    for (size_t k = 0; k < _A.size(); ++k)
		    {
			_y0[k] = _d1[k]*_y1[k] + _d2[k]*_y2[k] + x;
			x      = _A[k]*(_y0[k] + 2*_y1[k] + _y2[k]);
			_y2[k] = _y1[k];
			_y1[k] = _y0[k];
		    }

		    return x;
		}

    void	reset(const value_type& x)
		{
		    _y1[0] = _y2[0] = x / (1 - _d1[0] - _d2[0]);
		}

  private:
    element_type			_cutoff;
    std::vector<element_type>		_A;
    std::vector<element_type>		_d1;
    std::vector<element_type>		_d2;
    mutable std::vector<value_type>	_y0;
    mutable std::vector<value_type>	_y1;
    mutable std::vector<value_type>	_y2;
};

}  // namespace moveit_servo
