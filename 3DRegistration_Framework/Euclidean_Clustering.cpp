#include "pch.h"
#include"Euclidean_Clustering.h"


template <class flt_type>
cClusterEuclidean<flt_type>::cClusterEuclidean(){}

template <class flt_type>
cClusterEuclidean<flt_type>::cClusterEuclidean(const CloudWithoutType &InputCloud, const size_t &min_size, const size_t &max_size, const real &tolerance,
    const real &leaf_size): UnsegmentedCloud(new pcl::PointCloud<PointType>),UnsegmentedNormalCloud(new pcl::PointCloud<PointNormalType>)
{
    pcl::fromPCLPointCloud2(*InputCloud, *UnsegmentedCloud);
    pcl::fromPCLPointCloud2(*InputCloud, *UnsegmentedNormalCloud);
    minClusterSize= min_size;
    maxClusterSize= max_size;
    clusterTolerance= tolerance;
    ls= leaf_size;
 
}

template <class flt_type>
void cClusterEuclidean<flt_type>::PerformEuclideanClustering()
{
    //// perform voxel-grid downsampling
    //vg.setInputCloud(UnsegmentedCloud);
    //vg.setLeafSize(ls, ls, ls);
    //vg.filter(UnsegmentedFilteredCloud);
   
    // set up kdtree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kTree(new pcl::search::KdTree<pcl::PointXYZ>);
    kTree->setInputCloud(UnsegmentedCloud);

    // peform clustering
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(kTree);
    ec.setInputCloud(UnsegmentedCloud);
    ec.extract(cluster_indices);
    numCluster = cluster_indices.size();
}

template <class flt_type>
size_t cClusterEuclidean<flt_type>::GetNumCluster()
{
    return numCluster;
}

template <class flt_type>
std::vector<CloudWithoutType>cClusterEuclidean<flt_type>::RetrieveIndividualClusterAsPointCloud()
{
    std::vector<CloudWithoutType>vec_clouds;
    vec_clouds.reserve(numCluster);
    CloudWithoutType cloud(new pcl::PCLPointCloud2);
    pcl::PLYWriter writer;
    int j = 0;
 
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<PointNormalType>::Ptr cloud_cluster(new pcl::PointCloud<PointNormalType>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cip.push_back(std::tuple<int, int>(j, *pit));
            cloud_cluster->points.push_back(UnsegmentedNormalCloud->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".ply";
        writer.write<PointNormalType>(ss.str(), *cloud_cluster, false); //*

        cloud.reset(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cloud_cluster, *cloud);
        vec_clouds.push_back(cloud);
        j++;
    }
    return vec_clouds;
}
//////
//
// Explicit template instantiations
//
template REG3D_API cClusterEuclidean<float>;
template REG3D_API cClusterEuclidean<double>;