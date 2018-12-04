#pragma once
#include"ICorrespondenceEstimator.h"
#include"GridSearch.h"
#include"ISearch.h"
#include"InterpolationMethod.h"
#include"Datatypes.h"
#include"Config.h"


class REG3D_API cCorrespondenceDepth : public CorrespondenceEstimator::ICorrespondenceEstimator
{
public:
  
    cCorrespondenceDepth(cGridSearch &gridSearch) :
        TargetPixelCoordinates_(gridSearch.getTargetPixelCoordinate()),
        sourceCorrespondence_(),
        targetCorrespondence_(new pcl::PCLPointCloud2),
        newsearch(&gridSearch),
        sourceCorrespIndices_(),
        targetCorrespIndices_()

    {}
    ~cCorrespondenceDepth();
    cCorrespondenceDepth& operator=(cCorrespondenceDepth &rhs);
    cCorrespondenceDepth(cCorrespondenceDepth &rhs);

    void PrepareDataForCorrespondenceEstimation(CloudWithoutType &srcCloud, CloudWithoutType &tarCloud);
    void findCorrespondence(CloudWithoutType &srcCloud);
    std::vector<int> getSourceCorrespondenceIndices()const;
    CloudWithoutType getTargetCorrespondence()const;
    search::ISearch* getSearchStrategy()const;
    void SetTargetPixelCoordinate(std::vector<UVData<float>>pixelCoordinates);
    std::vector<UVData<float>> getTargetPixelCoordinate();
    
    protected:
        cGridSearch *newsearch = nullptr;
        CloudWithoutType sourceCorrespondence_;
        CloudWithoutType targetCorrespondence_;
        std::vector<int>sourceCorrespIndices_;
        std::vector<int>targetCorrespIndices_;
        std::vector<UVData<float>>TargetPixelCoordinates_;

};
