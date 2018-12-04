#pragma once

#include"ICorrespondenceEstimator.h"
#include"Config.h"
#include"Datatypes.h"
#include"MLSSearch.h"
// pcl includes
#include <pcl/kdtree/kdtree_flann.h>

class REG3D_API cMLSCorrespondence : public CorrespondenceEstimator::ICorrespondenceEstimator
{
public:
  
    cMLSCorrespondence(CloudWithoutType &CloudB, IWeightingFunction<float> &WeightFunc,
        float scalefactor = 1.0f, int polynomial_order = 1, int itr_max = 20, float Resolution = 0.0f, bool _create = true) :
        inputTargetCloud(CloudB),
        newsearch_create(_create),
        mlsSearch(new cMLSearch(CloudB, WeightFunc, scalefactor, polynomial_order, itr_max, Resolution ))
    {}
    cMLSCorrespondence(cMLSearch &mls_Search) :
        /*inputTargetCloud(mls_Search.getTargetCloudForMLSSurface()),*/
        mlsSearch(&mls_Search)
    {}
    ~cMLSCorrespondence();
    cMLSCorrespondence(cMLSCorrespondence &mlscorres);
    void PrepareDataForCorrespondenceEstimation(CloudWithoutType &srcCloud, CloudWithoutType &tarCloud);
 
    search::ISearch* getSearchStrategy()const;
    CloudWithoutType ComputeMLSSurface();
    std::vector<Eigen::Matrix3f> GetPrincipalFrame();
    std::vector<Eigen::Vector3f> GetEigenValue();
    CloudWithoutType GetInputCloud();
  
protected:
    CloudWithoutType inputTargetCloud;
    CloudWithoutType inputSourceCloud;
    cMLSearch *mlsSearch = nullptr;
    bool newsearch_create = false;
    std::vector<Eigen::Matrix3f> Principal_frame;
    std::vector<Eigen::Vector3f> eigen_val;
   
  
};
