#pragma once
#include"IRegistattion.h"
#include"CirconDescriptor.h"
#include"CorrespondenceEstimatioBySimilarity.h"

class REG3D_API  RegistrationSimilarity : public Registration::iRegsitration
{
public:
    RegistrationSimilarity(std::string A, std::string B):
        file_name_source(A),
        file_name_target(B)
    {};
    void PrepareSample(CloudWithoutType &cloudSrc, CloudWithoutType &cloudTarget);
    std::vector<std::pair<double, double>> estimateRigidRegistration(CorrespondenceEstimator::ICorrespondenceEstimator *Estimators,
        Eigen::Matrix4f &Transformation_matrix);
    Eigen::Affine3f getFinalTransformation()const;
protected:
    CloudWithoutType srcCloudCorrespondence_;
    CloudWithoutType targetCloudCorrespondence_;
    Eigen::Matrix4f m_TransformationMatrix;
    int num_resolution_level;
    std::string file_name_source;
    std::string file_name_target;
};
