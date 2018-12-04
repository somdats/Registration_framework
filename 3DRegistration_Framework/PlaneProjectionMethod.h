#pragma once
#include"ISearch.h"
#include"GridSearch.h"
#include"Datatypes.h"



 class REG3D_API cPlaneProjectionMethod : public cGridSearch
{
public:
    cPlaneProjectionMethod(Eigen::Matrix4f &ProjectionMatrix, std::vector<UVData<float>>&pixelCoordinates) :cGridSearch(ProjectionMatrix, pixelCoordinates) {};
    void findPoint(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint);
    
 };
