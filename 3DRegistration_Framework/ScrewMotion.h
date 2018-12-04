#ifndef _SCREW_H_
#define _SCREW_H_

#include<string>
#include"Config.h"
#include"EigenTools.h"

#define PI 3.1421
 // c++ class to represent transformation matrix as a screw motion
template<class real>
class REG3D_API  cScrewTransformation
{
    typedef typename EigenTypes<real>::Vec3 Vec3;
    typedef typename EigenTypes<real>::Mat4 Mat4;
    typedef typename EigenTypes<real>::Mat3 Mat3;
public:
    cScrewTransformation() {};
    cScrewTransformation( real translation, Vec3 point, Vec3 axis, real angle):normTranslation(translation),
        rotation_angle(angle * PI/static_cast<real>(180)),
        anchor_point(point),
        Coord3D(axis)
    {}
    std::string print();
    cScrewTransformation ConvertMatTransformToScrew(Mat4 &TranformationMatrix);
    real GetScrewRotationAngle()const;
    real GetTranslationMagnitude()const;


protected:
    Vec3 Coord3D;  // unit vector of translation
    real normTranslation;  // translation norm (magnitude of translation)
    real rotation_angle; // angle of rotation (in radian)
    Vec3  anchor_point;  // point on rotation axis(to generate the rotation axis)
    real Angle(Vec3 &vecA, Vec3 &vecB);
    
};

#endif
