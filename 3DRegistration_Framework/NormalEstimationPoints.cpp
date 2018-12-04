#include"pch.h"
#include"NormalEstimationPoints.h"
#include"ErrorMetric.h"


cNormalEstimationPointCloud::~cNormalEstimationPointCloud()
{
    if (NULL != kdTree)
    {
        delete kdTree;
        kdTree = NULL;
    }
}
 void cNormalEstimationPointCloud ::ComputeFeature()
{
     CloudPtr input_cloud(new pcl::PointCloud<PointType>);
     pcl::fromPCLPointCloud2(*inputCloud, *input_cloud);

   float avgDistanceTargetCloud = metric::EstimatePointAvgDistance(inputCloud);
    
    // initialize normalptr

     kdTree = new(pcl::KdTreeFLANN<PointType>);
     kdTree->setInputCloud(input_cloud);
     float radius = scaleFactor * avgDistanceTargetCloud;
     CWendlandWF<float> wendLandWeightF;
     cMLSearch mls(inputCloud, wendLandWeightF, scaleFactor, 3.0f, 20);
    // mls.PrepareSample(inputCloud);
     int itr = 0;
     
     NormalPtr normals(new pcl::PointCloud<NormalType>);
    
     normals->width = input_cloud->width;
     normals->height = input_cloud->height;
     normals->points.resize(normals->width * normals->height);
     Eigen::Vector3f dummyNormal(0.0, 0.0, 0.0);
     bool  _WithDummyNormal = false;
//#pragma omp parallel for 
     for (int i = 0; i < input_cloud->points.size(); i++)
     {
         std::vector<int> pointIdxRadiusSearch;
         std::vector<float> pointRadiusSquaredDistance;
         std::vector<float>weight;
         PointType point = input_cloud->points[i];
         if (kdTree->radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 2)
         {
             PointNormalType pnOld;
             pnOld.getVector3fMap() = input_cloud->points[i].getVector3fMap();
             pnOld.getNormalVector3fMap() = Eigen::Vector3f(0.0f,0.0f,0.0f);
             weight = mls.ComputeWeightFromLocalNeighborhood(pnOld, input_cloud, pointIdxRadiusSearch, wendLandWeightF, radius);

             Eigen::Vector3f meanOfNeighborhood(0.0f, 0.0f, 0.0f);
             Eigen::MatrixXf cov_matrix;
             std::vector<Eigen::Vector3f>eigen_vectors;
             eigen_vectors.clear();
             eigen_vectors.resize(3);
     
             // compute weighted covariance matrix of local neighborhood
           mls.ComputeWeightedCovariancematrix(input_cloud, weight, pointIdxRadiusSearch,
                 meanOfNeighborhood, eigen_vectors, cov_matrix);
           normals->points[i].getNormalVector3fMap() = eigen_vectors[0];
           Eigen::Vector3f viewDirection = pnOld.getVector3fMap() - Eigen::Vector3f(0.0f, 0.0f, 0.0f);
          // viewDirection.normalize();
           float dotProduct = normals->points[i].getNormalVector3fMap().dot(viewDirection);
         
           if (dotProduct > 0.0f)
           {
               normals->points[i].getNormalVector3fMap() *= -1.0f;
           }
         }
         if (normals->points[i].getNormalVector3fMap() == dummyNormal)
         {
             _WithDummyNormal = true;
         }
     }
     std::vector<PointType>refined_output_points;
     std::vector<NormalType>refined_normals;
     refined_output_points.reserve(normals->points.size());
     refined_normals.reserve(normals->points.size());
     CloudWithNormalPtr  resultant_cloud(new pcl::PointCloud<PointNormalType>);
     if (_WithDummyNormal == true)
     {
         for (size_t i = 0; i < normals->points.size(); i++)
         {
             if (normals->points[i].getNormalVector3fMap() != dummyNormal)
             {
                 refined_output_points.push_back(input_cloud->points[i]);
                 refined_normals.push_back(normals->points[i]);
             }
             
         }
         input_cloud->points.assign(refined_output_points.begin(), refined_output_points.end());
         input_cloud->width = static_cast<uint32_t>(refined_output_points.size());
         input_cloud->height = 1;
         normals->points.assign(refined_normals.begin(), refined_normals.end());
         normals->width = static_cast<uint32_t>(refined_output_points.size());
         normals->height = 1;
         pcl::concatenateFields(*input_cloud, *normals, *resultant_cloud);
         pcl::toPCLPointCloud2(*resultant_cloud, *PointWithNormal);
         return;
     }
     pcl::concatenateFields(*input_cloud, *normals, *resultant_cloud);
     pcl::toPCLPointCloud2(*resultant_cloud, *PointWithNormal);
}

 CloudWithoutType cNormalEstimationPointCloud::getOutputCloudWithNormal()
 {
     return PointWithNormal;
 }
 
