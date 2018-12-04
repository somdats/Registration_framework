
#pragma once

//////
//
// Includes
//

// Eigen library
#include <Eigen/Dense>

// Local config
#include "Config.h"



//////
//
// Classes
//

/**
 * @brief
 *		Templated abstraction to both single and double precision versions of Eigen's
 *		data type shortcuts.
 *
 * This might be useful if the internal structure of the Eigen library changes some day
 * due to some major overhaul. When that happens, the shortcut names are more likely to
 * stay unchanged and still be located in their usual place than the template classes
 * themselves.
 */
template <class flt_type>
struct REG3D_API EigenTypes
{
};
/** @brief Single precision specialization of @ref #EigenTypes . */
template<>
struct REG3D_API EigenTypes<float>
{
	typedef Eigen::Vector2f Vec2;
	typedef Eigen::Vector3f Vec3;
	typedef Eigen::Vector4f Vec4;
    typedef Eigen::Matrix3f Mat3;
	typedef Eigen::Matrix4f Mat4;
    typedef Eigen::Affine3f affine4;
    typedef Eigen::Quaternionf Quat;
 
};
/** @brief Double precision specialization of @ref #EigenTypes . */
template<>
struct REG3D_API EigenTypes<double>
{
	typedef Eigen::Vector2d Vec2;
	typedef Eigen::Vector3d Vec3;
	typedef Eigen::Vector4d Vec4;
	typedef Eigen::Matrix4d Mat4;
    typedef Eigen::Matrix3d Mat3;
    typedef Eigen::Affine3d affine4;
    typedef Eigen::Quaterniond Quat;
};
/** @brief Signed integer specialization of @ref #EigenTypes . */
template<>
struct REG3D_API EigenTypes<signed>
{
	typedef Eigen::Vector2i Vec2;
	typedef Eigen::Vector3i Vec3;
	typedef Eigen::Vector4i Vec4;
	typedef Eigen::Matrix4i Mat4;
    typedef Eigen::Matrix3i Mat3;
   
};
/** @brief Unsigned integer specialization of @ref #EigenTypes . */
template<>
struct REG3D_API EigenTypes<unsigned>
{
	typedef Eigen::Matrix<unsigned, 2, 1> Vec2;
	typedef Eigen::Matrix<unsigned, 3, 1> Vec3;
	typedef Eigen::Matrix<unsigned, 4, 1> Vec4;
    typedef Eigen::Matrix<unsigned, 4, 4> Mat4;
	typedef Eigen::Matrix<unsigned, 3, 3> Mat3;
   
};


/**
 * @brief
 *		Templated collection of helper functions that make calls to Eigen's API and inject
 *		some useful post-processing before reporting the results.
 */
template <class flt_type>
struct REG3D_API EigenHelper
{
	static typename EigenTypes<flt_type>::Mat4 rotate44 (
		const typename EigenTypes<flt_type>::Vec3 &axis, flt_type angle
	);
	static typename EigenTypes<flt_type>::Mat4 translate44(
		const typename EigenTypes<flt_type>::Vec3 &vector
	);
	static typename EigenTypes<flt_type>::Mat4 translate44(
		flt_type x, flt_type y, flt_type z
	);
};

