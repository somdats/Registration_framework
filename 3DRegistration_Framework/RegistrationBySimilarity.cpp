#include"pch.h"
#include"RegistrationBySimilarity.h"
#include"CorrespondenceEstimatioBySimilarity.h"



void RegistrationSimilarity::PrepareSample(CloudWithoutType &cloudSrc, CloudWithoutType &cloudTarget)
{
    srcCloudCorrespondence_ = cloudSrc;
    targetCloudCorrespondence_ = cloudTarget;
}

std::vector<std::pair<double, double>> RegistrationSimilarity::estimateRigidRegistration(CorrespondenceEstimator::ICorrespondenceEstimator *Estimators,
    Eigen::Matrix4f &Transformation_matrix)
{
    Estimators->PrepareDataForCorrespondenceEstimation(srcCloudCorrespondence_, targetCloudCorrespondence_);
    CirconCorrespondence *ptr = dynamic_cast<CirconCorrespondence*>(Estimators);
    if (ptr)
    {
        ptr->ReadCorrespondenceIndex(file_name_source, file_name_target);
        Transformation_matrix = ptr->ComputeTransformation();
       
    }
    std::vector<std::pair<double, double>> error;
    return error;
   
}

Eigen::Affine3f RegistrationSimilarity::getFinalTransformation()const
{
    Eigen::Affine3f new_matrix(m_TransformationMatrix);
    return new_matrix;
}