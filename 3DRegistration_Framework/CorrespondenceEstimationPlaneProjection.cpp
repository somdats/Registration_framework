#include"pch.h"
#include"CorrespondenceEstimationPlaneProjection.h"

//destructor
cCorrespondencePlaneProjection::~cCorrespondencePlaneProjection()
{
   // delete newsearch;
    sourceCorrespIndices_.clear(),
    targetCorrespIndices_.clear();
   /* if (NULL != newsearch)
    {
        delete[] newsearch;
        newsearch = NULL;
    }*/
    TargetPixelCoordinates_.clear();
}

// The function is assignment operator for the class
cCorrespondencePlaneProjection& cCorrespondencePlaneProjection :: operator=(cCorrespondencePlaneProjection &rhs)
{
    if (this == &rhs)
        return *this;
    sourceCorrespondence_ = rhs.sourceCorrespondence_;
    targetCorrespondence_ = rhs.targetCorrespondence_;
    sourceCorrespIndices_ = rhs.sourceCorrespIndices_;
    targetCorrespIndices_ = rhs.targetCorrespIndices_;
    newsearch = rhs.newsearch;
    return *this;
}

// copy constructor
cCorrespondencePlaneProjection::cCorrespondencePlaneProjection(cCorrespondencePlaneProjection &rhs)
{
    sourceCorrespondence_ = rhs.sourceCorrespondence_;
    targetCorrespondence_ = rhs.targetCorrespondence_;
    sourceCorrespIndices_ = rhs.sourceCorrespIndices_;
    targetCorrespIndices_ = rhs.targetCorrespIndices_;
    newsearch = rhs.newsearch;
}
// determine correspondence for a source cloud
void cCorrespondencePlaneProjection::findCorrespondence(CloudWithoutType &srcCloud)
{
    CloudWithRGBNormalPtr pTargetCorrespondence(new pcl::PointCloud<PointRGBNormalType>);
    CloudWithRGBNormalPtr pSrc(new pcl::PointCloud <PointRGBNormalType>);
    pcl::fromPCLPointCloud2(*srcCloud, *pSrc);
    PointRGBNormalType targetPoint;
    int count = 0;
    for (int i = 0; i < pSrc->points.size(); i++)
    {
        if (targetPoint.x != NAN)
        {
            pTargetCorrespondence->points.push_back(targetPoint);
            sourceCorrespIndices_.push_back(i);
            count++;
        } 
    }

    pTargetCorrespondence->width = count;
    pTargetCorrespondence->height = 1;
    pcl::toPCLPointCloud2(*pTargetCorrespondence, *targetCorrespondence_);
}

// Intialize Data for correspondenc  computation
void cCorrespondencePlaneProjection::PrepareDataForCorrespondenceEstimation(CloudWithoutType &srcCloud, CloudWithoutType &tarCloud)
{
    newsearch->createMap(tarCloud);
    newsearch->PrepareSample(tarCloud);
  /*  newsearch->setUVDataList(getTargetPixelCoordinate());
    newsearch->createMap(tarCloud);*/
}

// returns the search strategy for a correspondence estimation
search::ISearch* cCorrespondencePlaneProjection::getSearchStrategy()const
{
    return newsearch;
}

// returns all the correspondence as a cloud
CloudWithoutType cCorrespondencePlaneProjection::getTargetCorrespondence()const
{
    CloudWithoutType corresps(new pcl::PCLPointCloud2);
    pcl::copyPointCloud(*targetCorrespondence_, *corresps);
    return corresps;
}

//returns a  vector of source correspondene indices
std::vector<int>cCorrespondencePlaneProjection::getSourceCorrespondenceIndices()const
{
    return sourceCorrespIndices_;
}
// sets pixel coordinates for correspondence estimation
void  cCorrespondencePlaneProjection::SetTargetPixelCoordinate(std::vector<UVData<float>>pixelCoordinates)
{
    TargetPixelCoordinates_.reserve(pixelCoordinates.size());
    TargetPixelCoordinates_ = pixelCoordinates;
}

// returns  a  vector of target pixel coordinates
std::vector<UVData<float>>cCorrespondencePlaneProjection::getTargetPixelCoordinate()
{
    return TargetPixelCoordinates_;
}