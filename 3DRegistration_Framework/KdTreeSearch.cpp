#include"pch.h"
#include"KdTreeSearch.h"
#include "nanoflann.hpp"
#include"ErrorMetric.h"


cKDTreeSearch::~cKDTreeSearch()
{
    if (NULL != kdTree)
    {
        delete kdTree;
        kdTree = NULL;
    }
}

void cKDTreeSearch::PrepareSample(CloudWithoutType &targetCloud)
{
    CloudPtr pTarget(new pcl::PointCloud <PointType>);
   
    inputCloud = targetCloud;
    pcl::fromPCLPointCloud2(*targetCloud, *pTarget);
    kdTree = new(pcl::KdTreeFLANN<PointType>);
    kdTree->setInputCloud(pTarget);
    targetCloudSize = pTarget->points.size();
    std::string normal = "normal_x";
    auto it = find_if(begin(targetCloud->fields), end(targetCloud->fields), [=](pcl::PCLPointField const& f) {
        return (f.name == normal);
    });
    if (it != end(targetCloud->fields))
    {
        with_normals = true;
        CloudWithNormalPtr pNew(new pcl::PointCloud <PointNormalType>);
        pcl::fromPCLPointCloud2(*targetCloud, *pNew);
        inputTargetPointSet = pNew;
    }
    avgDistanceTargetCloud = metric::EstimatePointAvgDistance(targetCloud);
  
}

int cKDTreeSearch::findIndices(PointRGBNormalType &querypoint)
{
    PointType qPoint(querypoint.x, querypoint.y, querypoint.z);
    int nr_of_nearest_neighbor = 1;
    std::vector<int>k_indices(nr_of_nearest_neighbor);
    std::vector<float>k_distance(nr_of_nearest_neighbor);
    if (kdTree == nullptr)
    {
        throw std::runtime_error("Must create the kd tree before being able to query points.");
    }
    kdTree->nearestKSearch(qPoint, nr_of_nearest_neighbor, k_indices, k_distance);
    if (k_indices.size() > 0)
    {
        if ((0 <= k_indices[0]) && (k_indices[0] < targetCloudSize))
            return k_indices[0];
        else
            return INFINITY;
    }
    else
    {
        return INFINITY;
    }

}

inline int cKDTreeSearch::KdTreeGetPointCount()const
{
    int size;
    if (kdTree == nullptr)
        throw std::runtime_error(" kd tree not created.");
    else
        size = targetCloudSize;

    return size;
}

void cKDTreeSearch::findPoint(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint)
{
   // PointType qPoint(querypoint.x, querypoint.y, querypoint.z);
    PointType qPoint;
    qPoint.getVector3fMap() =  transform * querypoint.getVector3fMap();  // transformed source
    Eigen::Vector3f tranformed_source_normal = transform.linear() * querypoint.getNormalVector3fMap();  // transformed source normal
    int nr_of_nearest_neighbor = 1;
   /* std::vector<int>k_indices(nr_of_nearest_neighbor);
    std::vector<float>k_distance(nr_of_nearest_neighbor);*/
    std::vector<int>k_indices;
    std::vector<float>k_distance;
    float minDistanceSq = std::numeric_limits<float>::infinity();
    int closestPoint = -1;
    double radius = scale_factor * avgDistanceTargetCloud;
    bool no_false_point_found = false;
    bool found = false;
   
    if (kdTree == nullptr)
    {
        throw std::runtime_error("Must create the kd tree before being able to query points.");
    }

    if (radius_query == true)
    {
        kdTree->radiusSearch(qPoint, radius, k_indices, k_distance);
        if (k_indices.size() > 0)
        {
            for (int i = 0; i < k_indices.size(); i++)
            {

                if (k_distance[i] < minDistanceSq && tranformed_source_normal.dot(inputTargetPointSet->points[k_indices[i]].getNormalVector3fMap()) <= 0.7f)
                {
                    Eigen::Vector3f s_eigen_value, t_eigen_value;
                    Eigen::Matrix3f s_frame, t_frame;
                    //bool crt = QueryPrincipalFrame(querypoint, k_indices[i], s_frame, t_frame);
                   bool crt = QueryEigenValues(querypoint, k_indices[i], s_eigen_value, t_eigen_value);
                    if (crt == false)
                    {
                        continue;
                    }
                    else
                    {
                        /*Eigen::Matrix3f transformed_s_frame;
                        transformed_s_frame.col(1) = transform.linear() *  s_frame.col(1);
                        transformed_s_frame.col(2) = transform.linear() *  s_frame.col(2);
                        if (transformed_s_frame.col(1).dot(t_frame.col(1)) <= 0.7f && transformed_s_frame.col(2).dot(t_frame.col(2)) <= 0.7f)*/
                        float comp1 = s_eigen_value(1) / t_eigen_value(1);
                        float comp2 = s_eigen_value(2) / t_eigen_value(2);
                        if (0.5f <= comp1 <= 1.5f && 0.5f <= comp2 <= 1.5f)
                        {
                            closestPoint = k_indices[i];
                            minDistanceSq = k_distance[i];
                            found = true;
                        }
                    }
                }
                else
                {
                    continue;
                }
              
            }
            if (found == true && closestPoint == -1)
            {
                std::cout << " false correspondence:" << std::endl;
            }
            if (found == true)
            {
               // std::cout << closestPoint << std::endl;
                targetPoint.x = inputTargetPointSet->points[closestPoint].x;
                targetPoint.y = inputTargetPointSet->points[closestPoint].y;
                targetPoint.z = inputTargetPointSet->points[closestPoint].z;
                targetPoint.normal_x = inputTargetPointSet->points[closestPoint].normal_x;
                targetPoint.normal_y = inputTargetPointSet->points[closestPoint].normal_y;
                targetPoint.normal_z = inputTargetPointSet->points[closestPoint].normal_z;
                return;
            }
            else
            {
                found = no_false_point_found;
            }
        }
        else
        {
           /* targetPoint.x = std::numeric_limits<float>::quiet_NaN();
            targetPoint.y = std::numeric_limits<float>::quiet_NaN();
            targetPoint.z = std::numeric_limits<float>::quiet_NaN();
            targetPoint.normal_x = std::numeric_limits<float>::quiet_NaN();
            targetPoint.normal_y = std::numeric_limits<float>::quiet_NaN();
            targetPoint.normal_z = std::numeric_limits<float>::quiet_NaN();*/
            found = no_false_point_found;
        
        }
        
    }
    //else
    //{
    //    kdTree->nearestKSearch(qPoint, nr_of_nearest_neighbor, k_indices, k_distance);
    //    if (k_indices.size() > 0)
    //    {
    //        if ((0 <= k_indices[0]) && (k_indices[0] < targetCloudSize))
    //        {
    //            if (with_normals == true)
    //            {

    //                targetPoint.x = inputTargetPointSet->points[k_indices[0]].x;
    //                targetPoint.y = inputTargetPointSet->points[k_indices[0]].y;
    //                targetPoint.z = inputTargetPointSet->points[k_indices[0]].z;
    //                targetPoint.normal_x = inputTargetPointSet->points[k_indices[0]].normal_x;
    //                targetPoint.normal_y = inputTargetPointSet->points[k_indices[0]].normal_y;
    //                targetPoint.normal_z = inputTargetPointSet->points[k_indices[0]].normal_z;
    //                found = true;
    //            }
    //            else
    //            {

    //                targetPoint.x = inputTargetPointSet->points[k_indices[0]].x;
    //                targetPoint.y = inputTargetPointSet->points[k_indices[0]].y;
    //                targetPoint.z = inputTargetPointSet->points[k_indices[0]].z;
    //                targetPoint.normal_x = 0.0f;
    //                targetPoint.normal_y = 0.0f;
    //                targetPoint.normal_z = 1.0f;
    //            }
    //            return;
    //        }

    //        else
    //        {
    //          /*  targetPoint.x = std::numeric_limits<float>::quiet_NaN();
    //            targetPoint.y = std::numeric_limits<float>::quiet_NaN();
    //            targetPoint.z = std::numeric_limits<float>::quiet_NaN();
    //            targetPoint.normal_x = std::numeric_limits<float>::quiet_NaN();
    //            targetPoint.normal_y = std::numeric_limits<float>::quiet_NaN();
    //            targetPoint.normal_z = std::numeric_limits<float>::quiet_NaN();*/
    //            found = no_false_point_found;
    //        }

    //    }
    //    else
    //    {
    //        throw std::runtime_error("Index out of Bound");
    //    }
    //}
    if (found == no_false_point_found)
    {
        targetPoint.x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.z = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_z = std::numeric_limits<float>::quiet_NaN();
        return;
    }
}

void cKDTreeSearch::SetPrincipalFrame(std::vector<Eigen::Matrix3f> &p_frame_source, std::vector<Eigen::Matrix3f> &p_frame_target)
{
    Principal_frame_source = p_frame_source;
    Principal_frame_target = p_frame_target;
}

Eigen::Matrix3f cKDTreeSearch::getPrincipalFrameForPoint(int index)
{
    if (index > -1)
    {
        return Principal_frame_source[index];
    }
    else
    {
        throw std::runtime_error("Index out of Bound");
    }
}

void cKDTreeSearch::CreateIndicesVsPointMap(CloudWithoutType & PointCloudA, CloudWithoutType & PointCloudB)
{
    CloudWithNormalPtr  temp_cloud(new pcl::PointCloud<PointNormalType>);
    pcl::fromPCLPointCloud2(*PointCloudA, *temp_cloud);
    Principal_frame_Index_source_map.clear();
  
    for (int i = 0; i < temp_cloud->points.size(); i++)
    {
        PointData<float>pt(temp_cloud->points[i].x, temp_cloud->points[i].y, temp_cloud->points[i].z,
            temp_cloud->points[i].normal_x, temp_cloud->points[i].normal_y, temp_cloud->points[i].normal_z);
       
        Principal_frame_Index_source_map.insert(std::pair<PointData<float>, int>(pt, i));
    }

    CloudWithNormalPtr  test_cloud(new pcl::PointCloud<PointNormalType>);
    pcl::fromPCLPointCloud2(*PointCloudB, *test_cloud);
    Principal_frame_Index_target_map.clear();
    for (int i = 0; i < test_cloud->points.size(); i++)
    {
        PointData<float>pt(test_cloud->points[i].x, test_cloud->points[i].y, test_cloud->points[i].z,
            test_cloud->points[i].normal_x, test_cloud->points[i].normal_y, test_cloud->points[i].normal_z);

        Principal_frame_Index_target_map.insert(std::pair<PointData<float>, int>(pt, i));
       
    }
}

bool cKDTreeSearch::QueryPrincipalFrame(PointRGBNormalType &query_sourcepoint, int query_targetindex, Eigen::Matrix3f & p_frame_source,
    Eigen::Matrix3f & p_frame_target)
{
    //if (query_targetindex < Principal_frame_target.size() && query_targetindex > -1)
    //    p_frame_target = Principal_frame_target[query_targetindex]; // principal frame for targetCloud-Points

    //else
    //{
    //    return false;
    //}

    //PointDataType pt(query_sourcepoint.x, query_sourcepoint.y, query_sourcepoint.z,
    //    query_sourcepoint.normal_x, query_sourcepoint.normal_y, query_sourcepoint.normal_z);
    //std::map<PointDataType, int>::iterator itr = Principal_frame_Index_source_map.find(pt);
    //if (itr != Principal_frame_Index_source_map.end())
    //{
    //    int index = itr->second;
    //    if (index < Principal_frame_source.size() && index > -1)
    //    {
    //        p_frame_source = Principal_frame_source[index];
    //    }
    //    else
    //    {
    //        return false;
    //    }
    //}
    //else
    //{
    //    return false;
    //}
    return false;
}
void cKDTreeSearch::EstimateCorrespondenceWithFiltering(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, Eigen::Matrix3f & principal_frame,
    PointRGBNormalType &targetPoint)
{
    PointType qPoint;
    qPoint.getVector3fMap() = transform * querypoint.getVector3fMap();  // transformed source
    Eigen::Vector3f tranformed_source_normal = transform * querypoint.getNormalVector3fMap();  // transformed source normal
    int nr_of_nearest_neighbor = 1;
 
    std::vector<int>k_indices;
    std::vector<float>k_distance;
    float minDistanceSq = std::numeric_limits<float>::infinity();
    size_t closestPoint = -1;
    double radius = scale_factor * avgDistanceTargetCloud;
    bool no_false_point_found = false;
    bool found;
    if (kdTree == nullptr)
    {
        throw std::runtime_error("Must create the kd tree before being able to query points.");
    }

    if (radius_query == true)
    {
        kdTree->radiusSearch(qPoint, radius, k_indices, k_distance);
        if (k_indices.size() > 0)
        {
            for (int i = 0; i < k_indices.size(); i++)
            {
                if (k_distance[i] < minDistanceSq && tranformed_source_normal.dot(principal_frame.col(0)) <= 0.7f)
                {
                    Eigen::Matrix3f s_frame, t_frame;
                    Eigen::Vector3f s_eigen_value, t_eigen_value;
                   // bool crt = QueryPrincipalFrame(querypoint, k_indices[i], s_frame, t_frame);
                    bool crt = QueryEigenValues(querypoint, k_indices[i], s_eigen_value, t_eigen_value);
                    if (crt == false)
                    {
                        continue;
                    }
                    else
                    {
                        if (s_frame.col(1).dot(principal_frame.col(1)) >= 0.5f && s_frame.col(2).dot(principal_frame.col(2)) >= 0.5f)
                        {
                            closestPoint = k_indices[i];
                            minDistanceSq = k_distance[i];
                            found = true;
                        }
                    }
                }

            }
            if (found == true)
            {
                targetPoint.x = inputTargetPointSet->points[closestPoint].x;
                targetPoint.y = inputTargetPointSet->points[closestPoint].y;
                targetPoint.z = inputTargetPointSet->points[closestPoint].z;
                targetPoint.normal_x = inputTargetPointSet->points[closestPoint].normal_x;
                targetPoint.normal_y = inputTargetPointSet->points[closestPoint].normal_y;
                targetPoint.normal_z = inputTargetPointSet->points[closestPoint].normal_z;
                return;
            }
        }
        else
        {
            found = no_false_point_found;
        }

    }
    else
    {
        kdTree->nearestKSearch(qPoint, nr_of_nearest_neighbor, k_indices, k_distance);
        if (k_indices.size() > 0)
        {
            if ((0 <= k_indices[0]) && (k_indices[0] < targetCloudSize))
            {
                if (with_normals == true)
                {

                    targetPoint.x = inputTargetPointSet->points[k_indices[0]].x;
                    targetPoint.y = inputTargetPointSet->points[k_indices[0]].y;
                    targetPoint.z = inputTargetPointSet->points[k_indices[0]].z;
                    targetPoint.normal_x = inputTargetPointSet->points[k_indices[0]].normal_x;
                    targetPoint.normal_y = inputTargetPointSet->points[k_indices[0]].normal_y;
                    targetPoint.normal_z = inputTargetPointSet->points[k_indices[0]].normal_z;
                    found = true;
                }
                else
                {

                    targetPoint.x = inputTargetPointSet->points[k_indices[0]].x;
                    targetPoint.y = inputTargetPointSet->points[k_indices[0]].y;
                    targetPoint.z = inputTargetPointSet->points[k_indices[0]].z;
                    targetPoint.normal_x = 0.0f;
                    targetPoint.normal_y = 0.0f;
                    targetPoint.normal_z = 1.0f;
                }
                return;
            }

            else
            {
                found = no_false_point_found;
            }

        }
        else
        {
            throw std::runtime_error("Index out of Bound");
        }
    }
    if (found == no_false_point_found)
    {
        targetPoint.x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.z = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_z = std::numeric_limits<float>::quiet_NaN();
        return;
    }
}
std::map<PointData<float>, int> cKDTreeSearch::GetIndicesVsPointMap()
{
     return Principal_frame_Index_source_map;
}
Eigen::Matrix3f cKDTreeSearch::getPrincipalFrameForPoint(PointRGBNormalType &querypoint)
{
    /*PointDataType pt(querypoint.x, querypoint.y, querypoint.z,
        querypoint.normal_x, querypoint.normal_y, querypoint.normal_z);
    Eigen::Matrix3f p_frame_source;
    std::map<PointDataType, int>::iterator itr = Principal_frame_Index_source_map.find(pt);
    if (itr != Principal_frame_Index_source_map.end())
    {
        int index = itr->second;
        if (index < Principal_frame_source.size() && index > -1)
        {
            p_frame_source = Principal_frame_source[index];
        }
    }
    return p_frame_source;*/
    Eigen::Matrix3f p_frame_source;
    return  p_frame_source;
}
void cKDTreeSearch::getEigenValuesForPoint(Eigen::VectorXf &querypointA, int id, Eigen::Vector3f &source_eig_value, Eigen::Vector3f &targ_eig_value)
{
    Eigen::Vector3f sx = querypointA.head(3);
    Eigen::Vector3f sn = querypointA.tail(3);
    PointDataType ptA(sx(0), sx(1), sx(2), sn(0), sn(1), sn(2));
    std::map<PointData<float>, int>::iterator itr = Principal_frame_Index_source_map.find(ptA);
    if (itr != Principal_frame_Index_source_map.end())
    {
        int index = itr->second;
        if (index < eigen_values_source.size() && index > -1)
        {
            source_eig_value = eigen_values_source[index];
        }
    }
    /*  Eigen::Vector3f tx = querypointB.head(3);
      Eigen::Vector3f tn = querypointB.tail(3);
      PointDataType ptB(tx(0), tx(1), tx(2), tn(0), tn(1), tn(2));
      std::map<PointData<float>, int>::iterator it = Principal_frame_Index_target_map.find(ptB);
      if (it != Principal_frame_Index_target_map.end())
      {
          int index = it->second;*/
    if (id < eigen_values_target.size() && id > -1)
    {
        targ_eig_value = eigen_values_target[id];
    }
    // }

}
bool cKDTreeSearch::QueryEigenValues(PointRGBNormalType &query_sourcepoint, int query_targetindex, Eigen::Vector3f & eigenval_source,
    Eigen::Vector3f & eigenval_target)
{
    //if (query_targetindex < eigen_values_target.size() && query_targetindex > -1)
    //    eigenval_target = eigen_values_target[query_targetindex]; // principal frame for targetCloud-Points

    //else
    //{
    //    return false;
    //}

    //PointDataType pt(query_sourcepoint.x, query_sourcepoint.y, query_sourcepoint.z,
    //    query_sourcepoint.normal_x, query_sourcepoint.normal_y, query_sourcepoint.normal_z);
    //std::map<PointDataType, int>::iterator itr = Principal_frame_Index_source_map.find(pt);
    //if (itr != Principal_frame_Index_source_map.end())
    //{
    //    int index = itr->second;
    //    if (index < eigen_values_source.size() && index > -1)
    //    {
    //        eigenval_source = eigen_values_source[index];
    //    }
    //    else
    //    {
    //        return false;
    //    }
    //}
    //else
    //{
    //    return false;
    //}
    return false;
}
void cKDTreeSearch::SetFeatureAngleForSourceAndTarget(std::vector<double>feature_angles_s, std::vector<double>feature_angles_t)
{
    Point_feature_angles_source = feature_angles_s;
    Point_feature_angles_target = feature_angles_t;
}

void cKDTreeSearch::getFeatureAngle(Eigen::VectorXf &querypointA, int id, double &Fa_source, double &Fa_target)
{
    Eigen::Vector3f sx = querypointA.head(3);
    Eigen::Vector3f sn = querypointA.tail(3);
    PointDataType ptA(sx(0), sx(1), sx(2), sn(0), sn(1), sn(2));
    std::map<PointData<float>, int>::iterator itr = Principal_frame_Index_source_map.find(ptA);
    if (itr != Principal_frame_Index_source_map.end())
    {
        int index = itr->second;
        if (index < Point_feature_angles_source.size() && index > -1)
        {
            Fa_source = Point_feature_angles_source[index];
        }
    }
  /*  Eigen::Vector3f tx = querypointA.head(3);
    Eigen::Vector3f tn = querypointA.tail(3);
    PointDataType ptB(tx(0), tx(1), tx(2), tn(0), tn(1), tn(2));
    std::map<PointData<float>, int>::iterator it = Principal_frame_Index_target_map.find(ptB);
    if (it != Principal_frame_Index_target_map.end())
    {*/
        //int index = it->second;
        if (id < Point_feature_angles_target.size() && id > -1)
        {
            Fa_target = Point_feature_angles_target[id];
        }
   // }
}