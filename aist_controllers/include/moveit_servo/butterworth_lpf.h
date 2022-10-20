#pragma once

#include <cstddef>

namespace moveit_servo
{
template <class T>
class ButterworthLPF
{
  public:
    using value_type	= T;
    
  public:
    explicit	ButterworthLPF(size_t order,
			       value_type sampling_frequency,
			       value_type half_power_frequency)
		  :_A(order/2),    _d1(_A.size()), _d2(_A.size()),
		   _y0(_A.size()), _y1(_A.size()), _y2(_A.size())
		     
		{
		    const auto	a  = std::tan(M_PI*(half_power_frequency/
						    sampling_frequency));
		    const auto	a2 = a*a;
		    
		    for (size_t i = 0; i < _A.size(); ++i)
		    {
			const auto	r = std::sin(M_PI*(2*i+1)/(4*n));
			const auto	s = 1 + 2*a*r + a2;
			_A[i]  = a2/s;
			_d1[i] = 2*(1-a2)/s;
			_d2[i] = -(1 - 2*a*r + a2)/s;
		    }
		}
    

    size_t	order()				const	{ return 2*_A.size(); }
    
    value_type	filter(value_type x)
		{
		    for (size_t i = 0; i < _A.size(); ++i)
		    {
			_y0[i] = x + _d1[i]*_y1[i] + _d2[i]*_y2[i];
			x      = _A[i]*(_y0[i] + 2*_y1[i] + _y2[i]);
			_y2[i] = _y1[i];
			_y1[i] = _y0[i];
		    }

		    return _x;
		}
    
    void	reset(value_type x)
		{
		    
		}

  private:
    std::vector<value_type>	_A;
    std::vector<value_type>	_d1;
    std::vector<value_type>	_d2;
    std::vector<value_type>	_y0;
    std::vector<value_type>	_y1;
    std::vector<value_type>	_y2;
};
    
}  // namespace moveit_servo

int
main(int argc, char *argv[])
{
    if (argc < 4)
    {
	printf("Usage: %s n s f\n", argv[0]);
	printf("Butterworth lowpass filter.\n");
	printf("  n = filter order 2,4,6,...\n");
	printf("  s = sampling frequency\n");
	printf("  f = half power frequency\n");
	return(-1);
    }

    int i, n = (int)strtol(argv[1], NULL, 10);
    n = n/2;
    double s = strtod(argv[2], NULL);
    double f = strtod(argv[3], NULL);
    double a = tan(M_PI*f/s);
    double a2 = a*a;
    double r;
    double *A = (double *)malloc(n*sizeof(double));
    double *d1 = (double *)malloc(n*sizeof(double));
    double *d2 = (double *)malloc(n*sizeof(double));
    double *w0 = (double *)calloc(n, sizeof(double));
    double *w1 = (double *)calloc(n, sizeof(double));
    double *w2 = (double *)calloc(n, sizeof(double));
    double x;

    for (i = 0; i < n; ++i)
    {
	r = sin(M_PI*(2.0*i+1.0)/(4.0*n));
	s = a2 + 2.0*a*r + 1.0;
	A[i] = a2/s;
	d1[i] = 2.0*(1-a2)/s;
	d2[i] = -(a2 - 2.0*a*r + 1.0)/s;
    }

    while (scanf("%lf", &x) != EOF)
    {
	for(i = 0; i < n; ++i)
	{
	    w0[i] = d1[i]*w1[i] + d2[i]*w2[i] + x;
	    x = A[i]*(w0[i] + 2.0*w1[i] + w2[i]);
	    w2[i] = w1[i];
	    w1[i] = w0[i];
	}
	printf("%lf\n", x);
    }

    return(0);
}
