#pragma once
#include"Datatypes.h"

namespace sampling
{
   
    class ISample
    {
        virtual void PrepareSamplingData(CloudWithoutType &inputCloud) = 0;
        virtual void SamplePointCloud(CloudWithoutType &outputCloud) = 0;
    };
}