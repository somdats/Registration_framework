#include"pch.h"
#include"CorrespondenceEstimationDepthMap.h"

cCorrespondenceDepth::~cCorrespondenceDepth()
{
    //delete newsearch;
    sourceCorrespIndices_.clear(),
    targetCorrespIndices_.clear();
    /*if (NULL != newsearch)
    {
        delete[] newsearch;
        newsearch = NULL;
    }*/
    TargetPixelCoordinates_.clear();
}

cCorrespondenceDepth& cCorrespondenceDepth :: operator=(cCorrespondenceDepth &rhs)
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

cCorrespondenceDepth::cCorrespondenceDepth(cCorrespondenceDepth &rhs)
{
    sourceCorrespondence_ = rhs.sourceCorrespondence_;
    targetCorrespondence_ = rhs.targetCorrespondence_;
    sourceCorrespIndices_ = rhs.sourceCorrespIndices_;
    targetCorrespIndices_ = rhs.targetCorrespIndices_;
    newsearch = rhs.newsearch;
}
void cCorrespondenceDepth::findCorrespondence(CloudWithoutType &srcCloud)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    CloudWithRGBNormalPtr pTargetCorrespondence( new pcl::PointCloud<PointRGBNormalType>);
    CloudWithRGBNormalPtr pSrc(new pcl::PointCloud <PointRGBNormalType>);
    pcl::fromPCLPointCloud2(*srcCloud, *pSrc);
    PointRGBNormalType targetPoint;
    int count = 0;
    for (int i = 0; i < pSrc->points.size(); i++)
    {
        newsearch->findPoint(pSrc->points[i], transform, targetPoint);
        if (targetPoint.x != NAN)
        {
            pTargetCorrespondence->points.push_back(targetPoint);
            sourceCorrespIndices_.push_back(i);
            count++;
        }
        else
            continue;
        
    }
  
    pTargetCorrespondence->width = count;
    pTargetCorrespondence->height = 1;
    pcl::toPCLPointCloud2(*pTargetCorrespondence, *targetCorrespondence_);
}

search::ISearch* cCorrespondenceDepth::getSearchStrategy()const
{
    return newsearch;
}
void cCorrespondenceDepth::PrepareDataForCorrespondenceEstimation(CloudWithoutType &srcCloud, CloudWithoutType &tarCloud)
{
    newsearch->createMap(tarCloud);
    newsearch->PrepareSample(tarCloud);
    //newsearch->setUVDataList(getTargetPixelCoordinate());
   
}
CloudWithoutType cCorrespondenceDepth::getTargetCorrespondence()const
{
     CloudWithoutType corresps(new pcl::PCLPointCloud2);
     pcl::copyPointCloud(*targetCorrespondence_, *corresps);
     return corresps;
}

std::vector<int> cCorrespondenceDepth:: getSourceCorrespondenceIndices()const
{
    return sourceCorrespIndices_;
}
void  cCorrespondenceDepth::SetTargetPixelCoordinate(std::vector<UVData<float>>pixelCoordinates)
{
    TargetPixelCoordinates_.reserve(pixelCoordinates.size());
    TargetPixelCoordinates_ = pixelCoordinates;
}

std::vector<UVData<float>>cCorrespondenceDepth::getTargetPixelCoordinate()
{
    return TargetPixelCoordinates_;
}