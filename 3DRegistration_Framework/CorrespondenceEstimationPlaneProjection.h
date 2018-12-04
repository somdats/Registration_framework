#pragma once
#include"ICorrespondenceEstimator.h"
#include"GridSearch.h"
#include"ISearch.h"
#include"PlaneProjectionMethod.h"
#include"Datatypes.h"
#include"Config.h"


// Derived class for correspondence estimation : Method Correspondence Estimation by weighted plane projection(Phong projection)
class REG3D_API cCorrespondencePlaneProjection : public CorrespondenceEstimator::ICorrespondenceEstimator
{
public:
    cCorrespondencePlaneProjection(cGridSearch &gridSearch) :
        TargetPixelCoordinates_(gridSearch.getTargetPixelCoordinate()),
        sourceCorrespondence_(),
        targetCorrespondence_(new pcl::PCLPointCloud2),
        newsearch(&gridSearch)
    {}
    ~cCorrespondencePlaneProjection();
    cCorrespondencePlaneProjection& operator=(cCorrespondencePlaneProjection &rhs);
    cCorrespondencePlaneProjection(cCorrespondencePlaneProjection &rhs);

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