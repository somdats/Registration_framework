#include"pch.h"

// C++ STL
#include <cmath>

// Implemented header
#include "wfuncs.h"

//////
//
// Class implementation
//

// CTruncatedGaussianWF
//

template <class flt_type>
typename CTruncatedGaussianWF<flt_type>::real CTruncatedGaussianWF<flt_type>::operator()(
	real d, real h
) const
{
	return d < h*real(1.55) ?
		std::exp(-(d*d*real(9)) / (h*h)) :
		0;
}


// CWendlandWF
//

template <class flt_type>
typename CWendlandWF<flt_type>::real CWendlandWF<flt_type>::operator() (real d, real h)
	const
{
	real norm = h*h*h*h*h,
	     base = h - d;

	return d <= h ?
		(base*base*base*base * (h + real(4)*d)) / norm :
		0;
}



//////
//
// Explicit template instantiations
//

// Only floating point variants are intended
template CTruncatedGaussianWF<float>;
template CTruncatedGaussianWF<double>;
template CWendlandWF<float>;
template CWendlandWF<double>;
