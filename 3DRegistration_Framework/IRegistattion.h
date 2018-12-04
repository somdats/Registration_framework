#pragma once
#include"Datatypes.h"
#include"Config.h"
#include"ICorrespondenceEstimator.h"

namespace Registration
{
    class REG3D_API iRegsitration
    {
        virtual  void PrepareSample(CloudWithoutType &cloudSrc, CloudWithoutType &cloudTarget) = 0;
        virtual std::vector<std::pair<double, double>> estimateRigidRegistration(CorrespondenceEstimator::ICorrespondenceEstimator *Estimators, Eigen::Matrix4f &Transformation_matrix) = 0;
        virtual Eigen::Affine3f getFinalTransformation()const = 0;

    };
    
}
