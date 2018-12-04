#include"pch.h"
#include"CorrespondenceEstimationMLS.h"
#include<chrono>
#include"Common.h"

// .cpp file
namespace
{
    bool IsZeroPoint(Eigen::Vector3f point)
    {
        bool isZero = false;
        Eigen::Vector3f dummypoint(0.0, 0.0, 0.0);
            if (point == dummypoint)
            {
                isZero = true;
                return isZero;
            }
        return isZero;
    }
}

cMLSCorrespondence::~cMLSCorrespondence()
{
    if (NULL != mlsSearch && newsearch_create == true)
    {
        delete mlsSearch;
        mlsSearch = NULL;
    }
}
cMLSCorrespondence::cMLSCorrespondence(cMLSCorrespondence &mlscorres)
{
    inputSourceCloud = mlscorres.inputSourceCloud;
    inputTargetCloud = mlscorres.inputTargetCloud;
    
}

void cMLSCorrespondence :: PrepareDataForCorrespondenceEstimation(CloudWithoutType &srcCloud, CloudWithoutType &tarCloud)
{
    mlsSearch->PrepareSample(tarCloud);
}


search::ISearch* cMLSCorrespondence::getSearchStrategy()const
{
    return mlsSearch;
}
CloudWithoutType cMLSCorrespondence::ComputeMLSSurface()
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    CloudWithoutType ApproximatedCloud(new pcl::PCLPointCloud2);
    CloudWithRGBNormalPtr input_cloud(new pcl::PointCloud<PointRGBNormalType>);
    pcl::fromPCLPointCloud2(*inputTargetCloud, *input_cloud);
    CloudWithRGBNormalPtr output_cloud(new pcl::PointCloud<PointRGBNormalType>);
    CloudWithRGBNormalPtr refined_input_cloud(new pcl::PointCloud<PointRGBNormalType>);
  
   // output_cloud = input_cloud;
    Eigen::Vector3i nullColor(0, 0, 0);
    Eigen::Vector3i assigned_color(255, 255, 0);
   
    mlsSearch->PrepareSample(inputTargetCloud);
   
    if (input_cloud->points.size() < 1)
    {
        std::cout << "No input Cloud :" << std::endl;
        abort();
    }
    
    std::vector<PointRGBNormalType>output_points;
    output_points.resize(input_cloud->points.size());
    std::vector<Eigen::Matrix3f> temp_principal_frame;
    temp_principal_frame.resize(input_cloud->points.size());
    std::vector<Eigen::Vector3f>temp_eigen_values;
    temp_eigen_values.resize(input_cloud->points.size());
    auto startItr = std::chrono::high_resolution_clock::now();
//#pragma omp parallel for
    for (int itr = 0; itr < input_cloud->points.size(); itr++)
    {
        PointRGBNormalType pointA = input_cloud->points[itr];
        PointRGBNormalType pointB;

        mlsSearch->findApproximatePointforMLSSmoothing(pointA,transform, pointB);  // findPoint() previously

        if (pointB.getRGBVector3i() == nullColor)
        {
            pointB.r = 255;
            pointB.g = 255;
            pointB.b = 0;
        }
        output_points[itr] = pointB;
        temp_principal_frame[itr] = mlsSearch->GetPrincipalFrame();
        temp_eigen_values[itr] = mlsSearch->GetEigenValues();

    }
    std::vector<PointRGBNormalType>refined_output_points;
    refined_output_points.reserve(output_points.size());
    std::vector<Eigen::Matrix3f>refined_temp_principal_frame;
    refined_temp_principal_frame.reserve(output_points.size());
    std::vector<Eigen::Vector3f>refined_temp_eigen_values;
    refined_temp_eigen_values.reserve(output_points.size());
    std::vector<PointRGBNormalType>refined_input_points;
    refined_input_points.reserve(input_cloud->points.size());
    for (int j = 0; j < output_points.size(); j++)
    {
        if (isPointFinite(output_points[j]) == true && IsZeroPoint(output_points[j].getVector3fMap()) == false)
        {
            refined_output_points.push_back(output_points[j]);
            refined_temp_principal_frame.push_back(temp_principal_frame[j]);
            refined_temp_eigen_values.push_back(temp_eigen_values[j]);

            refined_input_points.push_back(input_cloud->points[j]);
        }
    }
    if (refined_input_points.size() < input_cloud->points.size())
    {
        refined_input_cloud->width = static_cast<uint32_t>(refined_input_points.size());
        refined_input_cloud->height = 1;
        refined_input_cloud->points.assign(refined_input_points.begin(), refined_input_points.end());
        CloudWithoutType refined_inputCloud(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*refined_input_cloud, *inputTargetCloud);
       // inputTargetCloud = refined_inputCloud;
    }
    auto finishItr = std::chrono::high_resolution_clock::now();
    double executeTime = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
    executeTime = executeTime / double(1000);
    std::cout << "principal frame computation time:" << executeTime << "secs" << std::endl;
#ifdef LOGDATA
    error_log("principal frame computation time:%f\n", executeTime);
#endif
    output_cloud->width =  static_cast<uint32_t>(refined_output_points.size());
    output_cloud->height = 1;
    output_cloud->points.assign(refined_output_points.begin(), refined_output_points.end());
    pcl::toPCLPointCloud2(*output_cloud, *ApproximatedCloud);
    Principal_frame = refined_temp_principal_frame;
    eigen_val = refined_temp_eigen_values;
    pcl::io::savePLYFile("mls.ply", *output_cloud);
    
    return ApproximatedCloud;
}

std::vector<Eigen::Matrix3f> cMLSCorrespondence :: GetPrincipalFrame()
{
    return Principal_frame;
}
std::vector<Eigen::Vector3f>cMLSCorrespondence::GetEigenValue()
{
    return eigen_val;
}
CloudWithoutType cMLSCorrespondence::GetInputCloud()
{
    return inputTargetCloud;
}