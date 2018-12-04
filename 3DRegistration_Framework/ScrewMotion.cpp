#include"pch.h"
#include"ScrewMotion.h"
#include<ostream>
#include<iostream>

#define EPSILON 1e-5

//Computes the angle between two vectors
template <class real>
real cScrewTransformation<real>::Angle(Vec3 &vecA, Vec3 &vecB)
{
    real dtPdt = vecA.dot(vecB);
    real dtPDt1 = sqrt(vecA.dot(vecA));
    real dtPDt2 = sqrt(vecB.dot(vecB));
    real cosTheta = dtPdt / (dtPDt1 * dtPDt2);
    return acos(cosTheta);
}

// prints all the parameter of screw motion
template <class real>
std::string cScrewTransformation<real>::print()
{
    std::stringstream ss;
    ss << "angle:" << rotation_angle << "\n";
    ss << "translation magnitude:" << normTranslation << "\n";
    ss << "rotation axis:" << Coord3D << "\n";
   // ss << "axis point:" << anchor_point << "\n";
    return ss.str();
 }

// Converts a 4 X 4 rigid transformation matrix to screw motion parameters
template <class real>
cScrewTransformation <real> cScrewTransformation<real>::ConvertMatTransformToScrew(Mat4 & TranformationMatrix)
{
    Vec3 Trans(TranformationMatrix.col(3).head<3>());
    Mat3 rotation_matrix;
    // extracting rotation matrix from 4 X 4 matrix
    rotation_matrix = TranformationMatrix.block(0, 0, 3, 3);
    cScrewTransformation <real> screwTransform;
    Vec3 point, X1, X2, X3;
    X1 = rotation_matrix.col(0).head<3>();
    X2 = rotation_matrix.col(1).head<3>();
    X3 = rotation_matrix.col(2).head<3>();

    real a = rotation_matrix(0, 0);
    real b = rotation_matrix(1, 1);
    real c = rotation_matrix(2, 2);
    if (fabs(static_cast<real>(1) + a - b - c) > EPSILON)
    {
        point(0) = X1(0) + static_cast<real>(1) - X2(1) - X3(2);
        point(1) = X1(1) + X2(0);
        point(2) = X1(2) + X3(0);
        screwTransform.Coord3D = point / point.norm();
        screwTransform.normTranslation = Trans.dot(screwTransform.Coord3D);
        Vec3 s = Trans - screwTransform.normTranslation *  screwTransform.Coord3D;
        screwTransform.anchor_point(0) = 0;
        screwTransform.anchor_point(1) = s(2) * X3(1) + s(1) * (static_cast<real>(1) - X3(2));
        screwTransform.anchor_point(2) = s(1) * X2(2) + s(2) * (static_cast<real>(1) - X2(1));
        screwTransform.anchor_point = screwTransform.anchor_point / (static_cast<real>(1) + X1(0) - X2(1) - X3(2));
    }
    else if (fabs(static_cast<real>(1) - a + b - c) > EPSILON)
    {
        point(0) = X2(0) + X1(1);
        point(1) = X2(1) + static_cast<real>(1) - X1(0) - X3(2);
        point(2) = X2(2) + X3(1);
        screwTransform.Coord3D = point / point.norm();
        screwTransform.normTranslation = Trans.dot(screwTransform.Coord3D);
        Vec3 s = Trans - screwTransform.normTranslation *  screwTransform.Coord3D;
        screwTransform.anchor_point(0) = s(2)* X3(0) + s(0) * (static_cast<real>(1) - X3(2));
        screwTransform.anchor_point(1) = 0;
        screwTransform.anchor_point(2) = s(0) * X1(2) + s(2) * (static_cast<real>(1) - X1(0));
        screwTransform.anchor_point = screwTransform.anchor_point / (static_cast<real>(1) - X1(0) + X2(1) - X3(2));
    }

    else if (fabs(static_cast<real>(1) - a - b + c) > EPSILON)
    {
        point(0) = X3(0) + X1(2);
        point(1) = X3(1) + X2(2);
        point(2) = X3(2) + static_cast<real>(1) - X1(0) - X2(1);
        screwTransform.Coord3D = point / point.norm();
        screwTransform.normTranslation = Trans.dot(screwTransform.Coord3D);
        Vec3 s = Trans - screwTransform.normTranslation *  screwTransform.Coord3D;
        screwTransform.anchor_point(0) = s(1)*X2(0) + s(0) * (static_cast<real>(1) - X2(1));
        screwTransform.anchor_point(1) = s(0) * X1(1) + s(1) * (static_cast<real>(1) - X1(0));
        screwTransform.anchor_point(2) = 0;
        screwTransform.anchor_point = screwTransform.anchor_point / (static_cast<real>(1) - X1(0) + X2(1) - X3(2));
    }
    else
    {
        Vec3 temp(0, 0, 0);
        screwTransform.Coord3D = temp ;
        if (Trans.norm() != 0)
        {
            screwTransform.Coord3D = Trans / Trans.norm();
        }
        else
        {
            Vec3 temp(0, 0, static_cast<real>(1));
            screwTransform.Coord3D = temp;
        }
        screwTransform.normTranslation = Trans.norm();
        screwTransform.rotation_angle = 0;
        return screwTransform;
    }
    Vec3 v(static_cast<real>(1), 0, 0);
    if (fabs(Angle(screwTransform.Coord3D, v)) < 0.1)
        v = Vec3(0, 0, static_cast<real>(1));
    Vec3 u = v - (v.dot(screwTransform.Coord3D)) * screwTransform.Coord3D;
    u = u.normalized();
    Vec3 uprime;
    uprime =  rotation_matrix * u;
    real cost = u.dot(uprime);

    Vec3 usec;
    usec = screwTransform.Coord3D.cross(u);
    real sint = usec.dot(uprime);
    if (cost < static_cast<real>(-1))
    {
        cost = static_cast<real>(-1);
    }
    if (cost > static_cast<real>(1))
    {
        cost = static_cast<real>(1);
    }
    screwTransform.rotation_angle = acos(cost);
    if (sint < 0)
        screwTransform.rotation_angle = -screwTransform.rotation_angle;

    screwTransform.rotation_angle *= 1;

    //real mag = screwTransform.Coord3D.dot(screwTransform.anchor_point.cross(screwTransform.Coord3D));
    //if (mag == 0.0)
    //    std::cout << "Only Rotation is true " << std::endl;
    //else
    //    std::cout << "Error in Computation" << std::endl;
   /* Vec3 transform = (screwTransform.rotation_angle * screwTransform.anchor_point).cross(screwTransform.Coord3D) + screwTransform.normTranslation *
        screwTransform.Coord3D;
    std::cout << "transform:" << transform << std::endl;*/
    return screwTransform;
    
}
template <class real>
real cScrewTransformation<real>::GetScrewRotationAngle()const
{
    return rotation_angle;
}

template <class real>
real cScrewTransformation<real>::GetTranslationMagnitude()const
{
    return normTranslation;
}
//////
//
// Explicit template instantiations
//
template REG3D_API cScrewTransformation<float>;
template REG3D_API cScrewTransformation<double>;
