#include"pch.h"
#include<iostream>
#include"dualQuaternion.h"

template <class real>
cDualQauternion<real> ::cDualQauternion(Mat4 TransformationMatrix)
{
    cDualQauternion<real> dual_quat = MatrixToDualQuaternion(TransformationMatrix);
    m_real = dual_quat.m_real;
    m_dual = dual_quat.m_dual;
   // SANITY CHECK
   
}
template <class real>
cDualQauternion<real> ::cDualQauternion( Quat &real_part,  Quat &dual_part)
{
    real_part.normalize();
    m_real =  real_part;
    m_dual = dual_part;
}

template <class real>
cDualQauternion<real> ::cDualQauternion( Quat &real_part,  Vec3 &t)
{
    
    m_real =  real_part.normalized();
    Quat trans(Constants<real>::zero, t(0), t(1), t(2));
    m_dual =  trans * m_real;
    m_dual = real(0.5) * m_dual.coeffs();
    
}

template <class real>
real cDualQauternion<real> ::Dot(cDualQauternion &a, cDualQauternion &b)
{
    Quat A = a.m_real;
    Quat B = b.m_real;
    real product = A.dot(B);
    return product;
}

template <class real>
cDualQauternion<real> cDualQauternion<real> :: setZero()
{
    cDualQauternion<real> ret(Quat(0.0,0.0,0.0,0.0),
        Quat(0.0, 0.0, 0.0, 0.0));
    return ret;
}
template <class real>
cDualQauternion<real> cDualQauternion<real> :: operator *( real scale)
{
    cDualQauternion<real> ret(Quat(scale * m_real.coeffs()),
        Quat(scale * m_dual.coeffs()));
    return ret;
}
template <class real>
cDualQauternion<real> cDualQauternion<real> :: operator + (cDualQauternion &rhs)
{

    cDualQauternion ret;
    ret.m_real.w() = (*this).m_real.w() + rhs.m_real.w();
    ret.m_real.vec() = (*this).m_real.vec() + rhs.m_real.vec();

    ret.m_dual.w() = (*this).m_dual.w() + rhs.m_dual.w();
   ret.m_dual.vec() = (*this).m_dual.vec()  + rhs.m_dual.vec();
   ret = ret.Normalize();
    return ret;
}

template <class real>
cDualQauternion<real> cDualQauternion<real> :: operator * (cDualQauternion &rs)
{
    cDualQauternion ret((*this).m_real * rs.m_real, (*this).m_dual * rs.m_dual);
    return ret;
}

template <class real>
cDualQauternion<real> cDualQauternion<real> ::Normalize()
{
    Quat real_part = (*this).m_real;
    real mag = real_part.norm();
    real magSqr = real_part.squaredNorm();
    cDualQauternion<real> ret;
    real scale = real(1) / mag;

    // real part is of unit length
    ret.m_real = scale * m_real.coeffs();
    

    // real and dual parts are orthogonal
    m_dual.coeffs() /= mag;
    m_dual.coeffs() -= (m_real.coeffs().dot(m_dual.coeffs()) * magSqr) * m_real.coeffs();
   
    return ret;
}

template <class real>
cDualQauternion<real> cDualQauternion<real> ::Conjugate()
{
    cDualQauternion ret;
    ret.m_real = (*this).m_real.conjugate();
    ret.m_dual = (*this).m_dual.conjugate();
    return ret;
}

template <class real>
typename EigenTypes<real>::Quat cDualQauternion<real>::GetRotation()const
 {
    Quat rot = (*this).m_real;
    return rot;
}

template <class real>
 typename EigenTypes<real>::Vec3 cDualQauternion<real>:: GetTranslation()const
{
    // q_dual = 0.5 * trans * rot;
   Quat t(2.0 * (m_dual * m_real.conjugate()).coeffs());
    Vec3 translationVector(t.x(), t.y(), t.z());
    return translationVector;

}
 template <class real>
 typename EigenTypes<real>::Mat4 cDualQauternion<real>::DualQuaternionToMatrix()
 {
     cDualQauternion<real> ret = (*this).Normalize();
     Mat4 transformationMatrix = Mat4::Identity();
    
     real w = ret.m_real.w() ;
     real x = ret.m_real.x();
     real y = ret.m_real.y();
     real z = ret.m_real.z();

    //Extract rotation
     transformationMatrix(0, 0) = w * w + x * x - y * y - z * z;
     transformationMatrix(1, 0) = 2 * x * y + 2 * w* z;
     transformationMatrix(2, 0) = 2 * x * z - 2 * w * y;

     transformationMatrix(0, 1) = 2 * x* y - 2 * w * z;
     transformationMatrix(1, 1) = w*w + y*y - x*x - z*z;
     transformationMatrix(2, 1) = 2 * y*z + 2 * w*x;

     transformationMatrix(0, 2) = 2 * x*z + 2 * w*y;
     transformationMatrix(1, 2) = 2 * y*z - 2 * w*x;
     transformationMatrix(2, 2) = w*w + z*z - x*x - y*y;

     //Extract translation
  /*   real scale = 2;
     Quat A;
     A.w() = scale * (*this).m_dual.w();
     A.vec() = scale * (*this).m_dual.vec();
     Quat B = (*this).m_real;
     B = B.conjugate();*/
     Quat trans(2.0 * (m_dual * m_real.conjugate()).coeffs());
     transformationMatrix(0, 3) = trans.x();
     transformationMatrix(1, 3) = trans.y();
     transformationMatrix(2, 3) = trans.z();
     
     return transformationMatrix;
 }
 template <class real>
 cDualQauternion<real> cDualQauternion<real> ::Inverse()
 {
     real sqrLen0 = m_real.squaredNorm();
     real sqrLenE = 2.0 * (m_real.coeffs().dot(m_dual.coeffs()));

     if (sqrLen0 > 0.0)
     {
         real invSqrLen0 = 1.0 / sqrLen0;
         real invSqrLenE = -sqrLenE / (sqrLen0 * sqrLen0);

         cDualQauternion<real> conj = Conjugate();
         conj.m_real.coeffs() = invSqrLen0 * conj.m_real.coeffs();
         conj.m_dual.coeffs() = invSqrLen0 * conj.m_dual.coeffs() + invSqrLenE * conj.m_real.coeffs();

         return conj;
     }
     else
     {
         return cDualQauternion<real>::setZero();
     }
 }
 template <class real>
 cDualQauternion<real> cDualQauternion<real> ::MatrixToDualQuaternion(Mat4 &TransformationMatrix)
 {
   
     Mat3 rotationMatrix = TransformationMatrix.block(0,0,3,3);
     Vec3 translation(TransformationMatrix.col(3).head<3>());
     cDualQauternion<real> dual_quat;
    
     // real part
     Quat  rotationQuaternion(rotationMatrix);
     dual_quat.m_real = rotationQuaternion.normalized();

     // dual part
     Quat translationQuaternion(real(0),translation(0), translation(1), translation(2));
     dual_quat.m_dual =  translationQuaternion * (dual_quat.m_real);
     dual_quat.m_dual = real(0.5) *  dual_quat.m_dual.coeffs();
   
     return dual_quat;
  
 }
 template <class real>
 typename EigenTypes<real>::Quat cDualQauternion<real> ::GetReal()const
 {
     return m_real;
 }

 template <class real>
 typename EigenTypes<real>::Quat cDualQauternion<real> ::GetDual()const
 {
     return m_dual;
 }
 //////
 //
 // Explicit template instantiations
 //
template REG3D_API cDualQauternion<float>;
template REG3D_API cDualQauternion<double>;