#pragma once

#include"Config.h"
#include"Datatypes.h"

#include<tuple>
#include<vector>

template <class flt_type>
class REG3D_API cClusterEuclidean
{
public:
    typedef flt_type real;
    typedef std::vector<std::tuple<int, int>> cluster_indices_pair;
    size_t minClusterSize;
    size_t maxClusterSize;
    real clusterTolerance;
    real ls;
    CloudPtr UnsegmentedCloud;
    CloudWithNormalPtr UnsegmentedNormalCloud;
    size_t numCluster;
    std::vector<pcl::PointIndices> cluster_indices;
    cClusterEuclidean();
    cClusterEuclidean(const CloudWithoutType &InputCloud, const size_t &min_size, const size_t &max_size, const real &tolerance, const real &leaf_size = 0.05);

protected:
    // pcl cluster-euclidean class
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    cluster_indices_pair cip;
    // // Create the filtering object: downsample the dataset using a leaf size
   // pcl::VoxelGrid<pcl::PointXYZ> vg;
  

public:
    void PerformEuclideanClustering();
    std::vector<CloudWithoutType> RetrieveIndividualClusterAsPointCloud();
    size_t GetNumCluster();
     

};
