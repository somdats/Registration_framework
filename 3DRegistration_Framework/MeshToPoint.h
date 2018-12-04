#pragma once
#include"Datatypes.h"
#include<pcl/conversions.h>
#include<pcl/common/transforms.h>
#include"Config.h"

namespace Converter
{
    struct param
    {
        int default_tesselated_sphere_level = 1;
        int default_resolution = 100;
        float default_leaf_size = 0.01f;
        float resolution_x = 640.0f;
        float resolution_y = 480.0f;
        float radius_sphere = 2.0f;
        float view_angle = 45.0f;
        float nearPlane = 0.1f;
        float farPlane = 100.0f;
    };
    REG3D_API void ConvertMeshToPCD(std::string inputFileName, param parameter,std::vector<CloudPtr>&views, std::vector<Eigen::Matrix4f>& Camera_to_World,
        std::vector<float>&Enthropies);
    REG3D_API std::vector<PointType> ConvertCloudToDataVector(CloudPtr cloud);
    REG3D_API  std::vector<UVData<float>> GenerateProjectedPoints(CloudPtr inputCloud, Eigen::Matrix4f transform, param parameter);
    REG3D_API void computeScreenCoordinate(const float AngleOfView, const float aspectRatio, const float n, const float f,
        float &t, float &b, float &l, float &r);
    REG3D_API Eigen::Matrix4f computePerspectiveProjectionmatrix(float &n, float &f, float &t, float &b, float &r, float &l);

}
