#pragma once
#include"Config.h"


//////
//
// Interfaces
//

/**
 * @brief Weighting function interface.
 */
template <class flt_type>
class REG3D_API IWeightingFunction
{

public:

	////
	// Types

	/** @brief Real number type. */
	typedef flt_type real;


	////
	// Methods

	/**
	* @brief
	*		Function call oeprator; should return the weight for given distance @a d
	*		using the kernel size @a h.
	*/
	virtual real operator () (real d, real h) const = 0;
};



//////
//
// Classes
//

/**
 * @brief Truncated gaussian weighting function.
 */
template <class flt_type>
class REG3D_API CTruncatedGaussianWF : public IWeightingFunction<flt_type>
{

public:

	////
	// Types

	/** @brief Real number type. */
	typedef typename IWeightingFunction<flt_type>::real real;


	////
	// Methods

	/**
	* @brief
	*		Function call oeprator; evaluates the truncated Guassian with kernel size @a
	*		h at distance @a d.
	*/
	virtual real operator () (real d, real h) const;
};


/**
 * @brief Wendland polynomial
 */
template <class flt_type>
class REG3D_API CWendlandWF : public IWeightingFunction<flt_type>
{

public:

	////
	// Types

	/** @brief Real number type. */
	typedef typename IWeightingFunction<flt_type>::real real;


	////
	// Methods

	/**
	* @brief
	*		Function call oeprator; evaluates the Wendland RBF with kernel size @a h at
	*		distance @a d.
	*/
	virtual real operator () (real d, real h) const;
};

