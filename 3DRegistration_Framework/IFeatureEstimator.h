#pragma once

#include"Datatypes.h"
#include"pcl/point_cloud.h"
#include"Config.h"


namespace FeatureEstimators
{

    class REG3D_API IFeature
    {
    public:
        virtual void ComputeFeature() = 0;
    };
}
