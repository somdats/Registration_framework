#ifndef _DUALQUAT_H_
#define _DUALQUAT_H_

// Eigen include
#include<Eigen/Geometry>

// API include for import

#include"Config.h"
#include "Utils.h"
#include "EigenTools.h"

template<class real>
class REG3D_API cDualQauternion
{
    // define quaternion type
     typedef typename EigenTypes<real>::Quat Quat;
     typedef typename EigenTypes<real>::Vec3 Vec3;
     typedef typename EigenTypes<real>::Mat4 Mat4;
     typedef typename EigenTypes<real>::Mat3 Mat3;
     typedef typename EigenTypes<real>::affine4 affine4;
public:
   
    cDualQauternion() : m_real(Constants<real>::zero, Constants<real>::zero, Constants<real>::zero, Constants<real>::one),
        m_dual(Constants<real>::zero, Constants<real>::zero, Constants<real>::zero, Constants<real>::zero)
    {}

   
    cDualQauternion(Quat &real_part,  Quat &dual_part);
    cDualQauternion( Quat &real_part,  Vec3 &t);
    cDualQauternion(Mat4 TransformationMatrix);

    cDualQauternion setZero();
    real Dot(cDualQauternion &a, cDualQauternion &b);
    cDualQauternion operator * (real scale);
    cDualQauternion Normalize();
  cDualQauternion operator + ( cDualQauternion &rhs);
    cDualQauternion operator * ( cDualQauternion &rs);
    cDualQauternion Conjugate();
    cDualQauternion Inverse();
    Quat GetRotation()const;
    Vec3 GetTranslation()const;
    Mat4 DualQuaternionToMatrix();
    cDualQauternion MatrixToDualQuaternion(Mat4 &TransformationMatrix);
    Quat GetReal()const;
    Quat GetDual()const;


protected:
    Quat m_real;
    Quat m_dual;

};
#endif
