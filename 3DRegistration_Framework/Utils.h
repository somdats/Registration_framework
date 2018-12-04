

//////
//
// Includes
//

// Local config
#include "Config.h"



//////
//
// Classes
//

/** @brief Collection of templated math constants. */
template <class flt_type>
struct REG3D_API Constants
{
};
/** @brief Single precision specialization of @ref #Constants . */
template<>
struct REG3D_API Constants<float>
{
	static const float pi;
	static const float eps;
	static const float inf;
	static const float max;
    static const float zero;
    static const float one;
};

/** @brief Double precision specialization of @ref #Constants . */
template<>
struct REG3D_API Constants<double>
{
	static const double pi;
	static const double eps;
	static const double inf;
	static const double max;
    static const double zero;
    static const double one;
};


