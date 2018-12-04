#pragma once
#include"ICorrespondenceEstimator.h"
#include"KdTreeSearch.h"
#include"ISearch.h"
#include"Datatypes.h"
#include"Config.h"

class REG3D_API cCorrespondenceKdtree : public CorrespondenceEstimator::ICorrespondenceEstimator
{
public:
    cCorrespondenceKdtree() :
        sourceCorrespondence_(),
        targetCorrespondence_(new pcl::PCLPointCloud2),
        targetCloud_(new pcl::PCLPointCloud2),
        newsearch(new cKDTreeSearch),
        sourceCorrespIndices_(),
        targetCorrespIndices_(),
        newsearch_create(true)

    {}
    cCorrespondenceKdtree(cKDTreeSearch &kdtreeSearch) :newsearch(&kdtreeSearch) {}
    ~cCorrespondenceKdtree();
    cCorrespondenceKdtree& operator=(cCorrespondenceKdtree &rhs);
    cCorrespondenceKdtree(cCorrespondenceKdtree &rhs);
    void PrepareDataForCorrespondenceEstimation(CloudWithoutType &srcCloud, CloudWithoutType &tarCloud);
    void findCorrespondence(CloudWithoutType &srcCloud);
    std::vector<int> getSourceCorrespondenceIndices()const;
    CloudWithoutType getTargetCorrespondence()const;
    search::ISearch* getSearchStrategy()const;


protected:
    cKDTreeSearch *newsearch = nullptr;
    CloudWithoutType sourceCorrespondence_;
    CloudWithoutType targetCorrespondence_;
    std::vector<int>sourceCorrespIndices_;
    std::vector<int>targetCorrespIndices_;
    CloudWithoutType targetCloud_;
    bool newsearch_create = false;
};
