#pragma once
#include"IFeatureEstimator.h"
#include"Normal.h"
#include"MLSSearch.h"
#include<pcl/kdtree/kdtree_flann.h>
#include"Config.h"

class REG3D_API cNormalEstimationPointCloud : public FeatureEstimators::IFeature
{
public:
    cNormalEstimationPointCloud(CloudWithoutType &inputCloud, int neigborhoodSize = 5, float scalefactor = 1.0f, int algorithm = 1, bool orient = false) :
        inputCloud(inputCloud), neighborSize(neigborhoodSize), algorithm_type(algorithm), 
        use_mst_to_orient(false),
        scaleFactor(scalefactor),
        PointWithNormal(new pcl::PCLPointCloud2)
      
    {};
   
    ~cNormalEstimationPointCloud();
    void ComputeFeature();
    CloudWithoutType getOutputCloudWithNormal();
    void PrepareSample();
    typedef pcl::KdTreeFLANN<PointType>KdTreeType;

protected:
    KdTreeType *kdTree = nullptr;
    CloudWithoutType inputCloud;
    CloudWithoutType PointWithNormal;
    NormalType Normals;
    int neighborSize;
    int algorithm_type;
    bool use_mst_to_orient;
    float scaleFactor;
   
};
