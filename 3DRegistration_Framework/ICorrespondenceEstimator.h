#pragma once

#include"Datatypes.h"
#include"ISearch.h"
#include"Config.h"

// base class for correspondence estimation
namespace CorrespondenceEstimator
{
    class REG3D_API ICorrespondenceEstimator
    {
    public:
        virtual ~ICorrespondenceEstimator() {};
        virtual void PrepareDataForCorrespondenceEstimation(CloudWithoutType &srcCloud, CloudWithoutType &tarCloud) = 0;
        virtual search::ISearch* getSearchStrategy()const = 0;

 };
}
