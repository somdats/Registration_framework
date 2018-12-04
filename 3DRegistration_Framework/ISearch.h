#pragma once
#include"Datatypes.h"
#include"Config.h"


namespace search
{
    
    class REG3D_API ISearch
    {
    public:
        virtual void PrepareSample(CloudWithoutType &targetCloud) = 0;
        virtual int findIndices(PointRGBNormalType &querypoint) = 0;
        virtual void findPoint(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint)= 0;
       

    };
}