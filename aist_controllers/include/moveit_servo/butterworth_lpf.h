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
    explicit	ButterworthLPF(size_t half_order, element_type cutoff)
		{
		    initialize(half_order, cutoff);
		}
    
    void	initialize(size_t half_order, element_type cutoff)
		{
		    _A.resize(half_order);
		    _d1.resize(_A.size());
		    _d2.resize(_A.size());
		    _y0.resize(_A.size());
		    _y1.resize(_A.size());
		    _y2.resize(_A.size());
		    
		    const auto	a  = std::tan(M_PI*cutoff);
		    const auto	a2 = a*a;
		    
		    for (size_t i = 0; i < _A.size(); ++i)
		    {
			const auto	n = _A.size();
			const auto	s = std::sin(M_PI*(2*i + 1)/(4*n));
			const auto	t = 1 + 2*a*s + a2;
			_A[i]  = a2/t;
			_d1[i] = 2*(1 - a2)/t;
			_d2[i] = -(1 - 2*a*s + a2)/t;
		    }
		}
    
    size_t	half_order() const
		{
		    return _A.size();
		}
    
    value_type	filter(value_type x) const
		{
		    for (size_t i = 0; i < _A.size(); ++i)
		    {
			_y0[i] = _d1[i]*_y1[i] + _d2[i]*_y2[i] + x;
			x      = _A[i]*(_y0[i] + 2*_y1[i] + _y2[i]);
			_y2[i] = _y1[i];
			_y1[i] = _y0[i];
		    }

		    return x;
		}
    
    void	reset(const value_type& x)
		{
		    _y1[0] = _y2[0] = x / (1 - _d1[0] - _d2[0]);
		}

  private:
    std::vector<element_type>		_A;
    std::vector<element_type>		_d1;
    std::vector<element_type>		_d2;
    mutable std::vector<value_type>	_y0;
    mutable std::vector<value_type>	_y1;
    mutable std::vector<value_type>	_y2;
};
    
}  // namespace moveit_servo
