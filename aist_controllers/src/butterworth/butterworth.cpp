#pragma once

#include <vector>
#include <math>

namespace TU
{
/************************************************************************
*  class ButterworthFilter<T, C>					*
************************************************************************/
template <class T, class C>
class ButterworthFilter
{
  public:
    using value_type		= T;
    using coeff_type		= C;
    constexpr static value_type	PI = M_PI;
    
  public:
		ButterworthFilter(size_t order, coeff_type cutoff)	;
    
    size_t	order()			const	{ return (_den.size() - 1)/2; }
    size_t	set_order(size_t order)					;
    coeff_type	low_cutoff()		const	{ return _low_cutoff; }
    coeff_type	set_low_cutoff(coeff_type cutoff)			;
    coeff_type	high_cutoff()		const	{ return _high_cuttoff; }
    coeff_type	set_high_cutoff(coeff_type cutoff)			;

    value_type	operator ()(const value_type& in)		const	;
    
  private:
    void	compute_den()						;
    void	compute_num()						;

    static std::vector<coeff_type>
		multiply_trinomial(const std::vector<coeff_type>& b,
				   const std::vector<coeff_type>& c)	;
    
  private:
    coeff_type			_low_cutoff;
    coeff_type			_high_cutoff;
    std::vec<coeff_type>	_den;
    std::vec<coeff_type>	_num;
    std::vec<value_type>	_out;
};

template <class T, class C> size_t
ButterworthFilter<T, C>::ButterworthFilter(size_t ord, coeff_type cutoff)
{
}

template <class T, class C> size_t
ButterworthFilter<T, C>::set_order(size_t ord)
{
    _den.resize(2*ord + 1);
    _num.resize(2*ord + 1);
    
}

template <class T, class C> void
ButterworthFilter<T, C>::compute_den()
{
    const auto	ord   = order();
    const auto	cp    = std::cos(PI * (_high_cutoff + _low_cutoff) / 2.0);
    const auto	theta = PI * (_high_cutoff - _low_cutoff) / 2.0;
    const auto	st    = std::sin(theta);
    const auto	ct    = std::cos(theta);
    const auto	s2t   = 2.0*st*ct;				// sin(2*theta)
    const auto	c2t   = 2.0*ct*ct - 1.0;			// cos(2*theta)

    std::vector<coeff_tyhpe>	rcoeffs(2*ord);
    std::vector<coeff_tyhpe>	tcoeffs(2*ord);
    for (int k = 0; k < ord; ++k)
    {
	const auto	pa = (PI * (2*k + 1))/(2*ord);	// pole angle
	const auto	sa = std::sin(pa);
	const auto	ca = std::cos(pa);
	const auto	a = 1.0 + s2t*sa;

	rcoeffs[2*k    ] = c2t			/ a;
	rcoeffs[2*k + 1] = s2t*ca		/ a;
	tcoeffs[2*k    ] = -2.0*cp*(ct + st*sa)	/ a;
	tcoeffs[2*k + 1] = -2.0*cp*st*ca	/ a;
    }

    const auto	tr = multiply_tinomial(tcoeffs, rcoeffs);

    _den[0] = 1.0;
    _den[1] = tr[0];
    for (k = 3; k <= 2*order(); ++k)
	den[k] = den[2*k - 2];

    for (int i = _den.size() - 1; i > 2*order() + 1; i--)
	_den.pop_back();
}

template <class T, class C> std::vector<C>
ButterworthFilter<T, C>::multiply_trinomial(const std::vector<coeff_type>& b,
					    const std::vector<coeff_type>& c)
{
    vector<coeff_type>	ret(4 * FilterOrder);

    ret[2] = c[0];
    ret[3] = c[1];
    ret[0] = b[0];
    ret[1] = b[1];

    for (size_t i = 1; i < order(); ++i)
    {
	ret[2*(2*i + 1)]     += c[2*i]     * ret[2*(2*i - 1)]
			      - c[2*i + 1] * ret[2*(2*i - 1) + 1];
	ret[2*(2*i + 1) + 1] += c[2*i]	   * ret[2*(2*i - 1) + 1]
			      + c[2*i + 1] * ret[2*(2*i - 1)];

	for (int j = 2*i; j > 1; --j)
	{
	    ret[2*j]	 += b[2*i]     * ret[2*(j - 1)]
			  - b[2*i + 1] * ret[2*(j - 1) + 1]
			  + c[2*i]     * ret[2*(j - 2)]
			  - c[2*i + 1] * ret[2*(j - 2) + 1];
	    ret[2*j + 1] += b[2*i]     * ret[2*(j - 1) + 1]
			  + b[2*i + 1] * ret[2*(j - 1)]
			  + c[2*i]     * ret[2*(j - 2) + 1]
			  + c[2*i + 1] * ret[2*(j - 2)];
	}

	ret[2] += b[2*i] * ret[0] - b[2*i + 1] * ret[1] + c[2*i];
	ret[3] += b[2*i] * ret[1] + b[2*i + 1] * ret[0] + c[2*i + 1];
	ret[0] += b[2*i];
	ret[1] += b[2*i + 1];
    }

    return ret;
}

template <class T, class C> void
ButtterworthFilter<T, C>::compute_num(vector<double> DenC)
{
    vector<coeff_type> TCoeffs;
    vector<coeff_type> NumCoeffs(2 * FilterOrder + 1);
    vector<complex<coeff_type>> NormalizedKernel(2 * FilterOrder + 1);

    vector<coeff_type> Numbers;
    for (coeff_type n = 0; n < FilterOrder * 2 + 1; n++)
	Numbers.push_back(n);
    int i;

    TCoeffs = ComputeHP(FilterOrder);

    for (i = 0; i < FilterOrder; ++i)
    {
	NumCoeffs[2 * i] = TCoeffs[i];
	NumCoeffs[2 * i + 1] = 0.0;
    }
    NumCoeffs[2 * FilterOrder] = TCoeffs[FilterOrder];

    coeff_type cp[2];
    coeff_type Bw, Wn;
    cp[0] = 2 * 2.0*tan(PI * low_cutoff / 2.0);
    cp[1] = 2 * 2.0*tan(PI * high_cutoff / 2.0);

    Bw = cp[1] - cp[0];
  //center frequency
    Wn = sqrt(cp[0] * cp[1]);
    Wn = 2 * atan2(Wn, 4);
    coeff_type kern;
    const std::complex<coeff_type> result = std::complex<coeff_type>(-1, 0);

    for (int k = 0; k< FilterOrder * 2 + 1; k++)
    {
	NormalizedKernel[k] = std::exp(-sqrt(result)*Wn*Numbers[k]);
    }
    coeff_type b = 0;
    coeff_type den = 0;
    for (int d = 0; d < FilterOrder * 2 + 1; d++)
    {
	b += real(NormalizedKernel[d] * NumCoeffs[d]);
	den += real(NormalizedKernel[d] * DenC[d]);
    }
    for (int c = 0; c < FilterOrder * 2 + 1; c++)
    {
	NumCoeffs[c] = (NumCoeffs[c] * den) / b;
    }

    for (int i = NumCoeffs.size() - 1; i > FilterOrder * 2 + 1; i--)
	NumCoeffs.pop_back();

    return NumCoeffs;
}

template <class T, class C> void
ButterworthFilter<T, C>::create_lowpass_filter(size_t order)
{
    const auto	m = order/2;

    _num[0] = 1;
    for (size_t i = 1; i <= m; ++i)
	_num[order - i] = _num[i] = coeff_type(order - i + 1)*_num[i - 1] / i;
    _num[order] = 1;

    return _num;
}

template <class T, class C> void
ButterworthFilter<T, C>::compute_highpass_filter(size_t order)
{
    compute_lowpass_filter(order);

    for (size_t i = 0; i <= order; ++i)
	if (i % 2)
	    _num[i] = -_num[i];		// Negate coeffs with odd indices.

    return _num;
}

//vector<double> filter(int ord, vector<double> a, vector<double> b, vector<double> x)
//{
//	int np = x.size();
//	vector<double> y(np);
//
//	int i, j;
//	y[0] = b[0] * x[0];
//	for (i = 1; i<ord + 1; i++)
//	{
//		y[i] = 0.0;
//		for (j = 0; j<i + 1; j++)
//			y[i] = y[i] + b[j] * x[i - j];
//		for (j = 0; j<i; j++)
//			y[i] = y[i] - a[j + 1] * y[i - j - 1];
//	}
//	for (i = ord + 1; i<np + 1; i++)
//	{
//		y[i] = 0.0;
//		for (j = 0; j<ord + 1; j++)
//			y[i] = y[i] + b[j] * x[i - j];
//		for (j = 0; j<ord; j++)
//			y[i] = y[i] - a[j + 1] * y[i - j - 1];
//	}
//
//	return y;
//}

template <class T> template <class S> S
ButterworthFilter<T>::operator ()(const S& in) const
{
    int len_x = x.size();
    int len_b = coeff_b.size();
    int len_a = coeff_a.size();

    vector<double> zi(len_b);

    vector<double> filter_x(len_x);

    if (len_a == 1)
    {
	for (int m = 0; m<len_x; m++)
	{
	    filter_x[m] = coeff_b[0] * x[m] + zi[0];
	    for (int i = 1; i<len_b; i++)
	    {
		zi[i - 1] = coeff_b[i] * x[m] + zi[i];//-coeff_a[i]*filter_x[m];
	    }
	}
    }
    else
    {
	for (int m = 0; m<len_x; m++)
	{
	    filter_x[m] = coeff_b[0] * x[m] + zi[0];
	    for (int i = 1; i<len_b; i++)
	    {
		zi[i - 1] = coeff_b[i] * x[m] + zi[i] - coeff_a[i] * filter_x[m];
	    }
	}
    }

    return filter_x;
}

int
main()
{
    using value_type	= double;
    
    ifstream ifile;
    ifile.open("E:\\HRdataset\\butterworth\\input.txt");

    std::vector<value_type>	x(N);
    std::vector<value_type>	y(N);

    for (int i = 0; i < N; i++)
	ifile >> x[i];

  //is A in matlab function and the numbers are correct
  // these values are as a ratio of f/fs, where fs is sampling rate,
  // and f is cutoff frequency and therefore should lie in the range [0 1]    
    value_type			fps = 20;
    value_type			FrequencyBands[] = { 1.5/fps*2, 2.5/fps*2 };
    int				FiltOrd = 4;
    std::vector<value_type>	a = ComputeDenCoeffs(FiltOrd,
						     FrequencyBands[0],
						     FrequencyBands[1]);
    for (int k = 0; k < a.size(); k++)
	printf("DenC is: %lf\n", a[k]);

    std::vector<value_type>	b  = ComputeNumCoeffs(FiltOrd,
						      FrequencyBands[0],
						      FrequencyBands[1], a);
    for (int k = 0; k < b.size(); k++)
    {
	printf("NumC is: %lf\n", b[k]);
    }

    y = filter(x, b, a);

    std::ofstream	ofile;
    ofile.open("E:\\HRdataset\\butterworth\\output.txt");
    for (int i = 0; i < N; i++)
	ofile << y[i] << std::endl;
    ofile.close();

    return 0;
}

}	// namespace TU
