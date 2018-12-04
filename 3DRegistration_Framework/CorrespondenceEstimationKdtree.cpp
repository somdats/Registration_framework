#include"pch.h"
#include"CorrespondenceEstimationKdtree.h"
#include<type_traits>

cCorrespondenceKdtree::~cCorrespondenceKdtree()
{
   // delete newsearch;
    sourceCorrespIndices_.clear(),
    targetCorrespIndices_.clear();
    if (NULL != newsearch && newsearch_create == true)
    {
       delete newsearch;
        newsearch = NULL;
    }
}
cCorrespondenceKdtree& cCorrespondenceKdtree :: operator=(cCorrespondenceKdtree &rhs)
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

cCorrespondenceKdtree::cCorrespondenceKdtree(cCorrespondenceKdtree &rhs)
{
    sourceCorrespondence_ = rhs.sourceCorrespondence_;
    targetCorrespondence_ = rhs.targetCorrespondence_;
    sourceCorrespIndices_ = rhs.sourceCorrespIndices_;
    targetCorrespIndices_ = rhs.targetCorrespIndices_;
    newsearch = rhs.newsearch;
}
 void cCorrespondenceKdtree :: PrepareDataForCorrespondenceEstimation(CloudWithoutType &srcCloud, CloudWithoutType &tarCloud)
{
     newsearch->PrepareSample(tarCloud);
     targetCloud_ = tarCloud;
}

 void cCorrespondenceKdtree::findCorrespondence(CloudWithoutType &srcCloud)
 {
     CloudWithRGBNormalPtr pTargetCorrespondence(new pcl::PointCloud<PointRGBNormalType>);
     CloudWithRGBNormalPtr pSrc(new pcl::PointCloud <PointRGBNormalType>);
     CloudWithRGBNormalPtr pTar(new pcl::PointCloud <PointRGBNormalType>);
     pcl::fromPCLPointCloud2(*srcCloud, *pSrc);
     pcl::fromPCLPointCloud2(*targetCloud_, *pTar);  //*srcCloud
     PointRGBNormalType targetPoint;
     int count = 0;
     for (int i = 0; i < pSrc->points.size(); i++)
     {
      int idx  =   newsearch->findIndices(pSrc->points[i]);
         if (idx != INFINITY)
         {
             pTargetCorrespondence->points.push_back(pTar->points[idx]);
             sourceCorrespIndices_.push_back(i);
             targetCorrespIndices_.push_back(idx);
             count++;
         }
         else
             continue;
     }

     pTargetCorrespondence->width = count;
     pTargetCorrespondence->height = 1;
     pcl::toPCLPointCloud2(*pTargetCorrespondence, *targetCorrespondence_);
 }

 search::ISearch* cCorrespondenceKdtree :: getSearchStrategy()const
 {
     return newsearch;
 }
 CloudWithoutType cCorrespondenceKdtree::getTargetCorrespondence()const
 {
     CloudWithoutType corresps(new pcl::PCLPointCloud2);
     pcl::copyPointCloud(*targetCorrespondence_, *corresps);
     return corresps;
 }

 std::vector<int> cCorrespondenceKdtree::getSourceCorrespondenceIndices()const
 {
     return sourceCorrespIndices_;
 }
