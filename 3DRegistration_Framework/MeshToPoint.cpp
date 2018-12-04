#include"pch.h"
#include"MeshToPoint.h"
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/common/transforms.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/console/print.h>
//#include <pcl/console/parse.h>
#include <Eigen/Dense>

REG3D_API void Converter::ConvertMeshToPCD(std::string inputFileName, param parameter, std::vector<CloudPtr>&views, std::vector<Eigen::Matrix4f>& Camera_to_World,
    std::vector<float>&Enthropies)
{
    vtkSmartPointer<vtkPolyData> polydata1;

    if (inputFileName.substr(inputFileName.find_last_of(".") + 1) == "ply")
    {
        vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New();
        readerQuery->SetFileName(inputFileName.c_str());
        readerQuery->Update();
        polydata1 = readerQuery->GetOutput();
    }
    else if (inputFileName.substr(inputFileName.find_last_of(".") + 1) == "obj")
    {
        vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New();
        readerQuery->SetFileName(inputFileName.c_str());
        readerQuery->Update();
        polydata1 = readerQuery->GetOutput();
    }
    else
    {
        PCL_ERROR("Need an input file");
        abort();
    }
    pcl::visualization::PCLVisualizer vis;
    vis.addModelFromPolyData(polydata1, "mesh1", 0);
    vis.setRepresentationToSurfaceForAllActors();
    pcl::PointCloud<PointType>::CloudVectorType views_xyz;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;

    vis.renderViewTesselatedSphere(parameter.resolution_x, parameter.resolution_y, views_xyz, poses, Enthropies,
        parameter.default_tesselated_sphere_level, parameter.view_angle, parameter.radius_sphere);

    views.reserve(views_xyz.size());
    Camera_to_World.reserve(poses.size());

    for (size_t i = 0; i < views_xyz.size(); i++)
    {
       pcl:: PointCloud<PointType>::Ptr new_cloud(new pcl::PointCloud<PointType>());
       *new_cloud = views_xyz[i];
        views.push_back(new_cloud);
        Camera_to_World.push_back(poses[i]);
    }


}
REG3D_API std::vector<PointType> Converter:: ConvertCloudToDataVector(CloudPtr cloud)
{
    std::vector<PointType>vectors;
    vectors.reserve(cloud->points.size());
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        vectors.push_back(cloud->points[i]);
    }
    return vectors;
}
REG3D_API std::vector<UVData<float>> Converter::GenerateProjectedPoints(CloudPtr inputCloud, Eigen::Matrix4f transform, param parameter)
{
    UVData<float> uv;
    float aspectRatio = (parameter.resolution_x) / (parameter.resolution_y);
    std::vector<UVData<float>>imgData;
    imgData.reserve(inputCloud->points.size());
    float t, b, l, r;
  
    for (size_t i = 0; i < inputCloud->points.size(); i++)
    {
        Eigen::Vector4f p1 = transform * inputCloud->points[i].getVector4fMap();

        computeScreenCoordinate(parameter.view_angle, aspectRatio, parameter.nearPlane, parameter.farPlane, t, b, l, r);
        Eigen::Matrix4f perspectiveMatrix = computePerspectiveProjectionmatrix(parameter.nearPlane, parameter.farPlane, t, b, l, r);
        Eigen::Vector4f p2Prime = perspectiveMatrix * p1;

        p2Prime = p2Prime / p2Prime(3);
        uv.u = std::floor((p2Prime(0) + 1) * 0.5 * parameter.resolution_x);
        uv.v = std::floor((p2Prime(1) + 1) * 0.5 * parameter.resolution_y);
        imgData.push_back(uv);
    }
    return imgData;
}
REG3D_API void Converter::computeScreenCoordinate(const float AngleOfView, const float aspectRatio, const float n, const float f,
    float &t, float &b, float &l, float &r)
{
    float x = AngleOfView * 0.5 * M_PI / 180;
    float scale = tan(x) * n;
    t = scale;
    b = -t;
    r = aspectRatio * t;
    l = -r;
}
REG3D_API Eigen::Matrix4f Converter::computePerspectiveProjectionmatrix(float &n, float &f, float &t, float &b, float &r, float &l)
{
 
    Eigen::Matrix4f matrix;
    matrix.setIdentity();
    Eigen::Vector4f vec1(2 * (n / (r - l)), 0, 0, 0);
    Eigen::Vector4f vec2(0, 2 * (n / (t - b)), 0, 0);
    Eigen::Vector4f vec3((r + l) / (r - l), (t + b) / (t - b), -(f + n) / (f - n), -1);
    Eigen::Vector4f vec4(0, 0, -(2 * f * n) / (f - n), 0);
  
    matrix.col(0) = vec1;
    matrix.col(1) = vec2;
    matrix.col(2) = vec3;
    matrix.col(3) = vec4;
    return matrix;
}