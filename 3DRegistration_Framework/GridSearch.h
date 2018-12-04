#pragma once
#include<vector>
#include<map>
#include"ISearch.h"
#include"Datatypes.h"



class REG3D_API cGridSearch : public search::ISearch
{
public:
 
    cGridSearch();
    cGridSearch(Eigen::Matrix4f &ProjectionMatrix, std::vector< UVData<float>> &pixelList) :ProjectionMatrix_(ProjectionMatrix), uvDataList(pixelList){};
    ~cGridSearch() {};
   //virtual void findPoint(PointRGBNormalType &querypoint, PointRGBNormalType &targetPoint) const;
    void PrepareSample(CloudWithoutType &targetCloud);
    int findIndices(PointRGBNormalType &querypoint);
    void findPoint(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint);
    void createMap(CloudWithoutType &targetCloud);
    std::vector<UVData<float>>& getTargetGrid();
    bool FindUVNeigborsForQueryPoint(PointRGBNormalType queryPoint, std::vector<UVData<float>>&Targetgrid, std::vector<float>&PixelWeight,
        std::vector<UVData<float>>&PixelNeighbors)const;
    std::map <UVData<float>, PointData<float>>& getTargetMap();
    UVData<float> getProjectedPoint(PointRGBNormalType querypoint)const;
    void setUVDataList(std::vector< UVData<float>>pixelVertexList);
    std::vector< UVData<float>> GetUVDataList();
    std::vector<UVData<float>> getTargetPixelCoordinate();
    std::vector<int> PrepareGrid(std::vector<UVData<float>> pixelCoordinates,GridBound &BoundOfgrid);
    std::vector<PointType> ComputeEightNeigborsFromGrid(UVData<float>QueryPixelIndex, GridBound bounds, std::vector<int>&GridIndex,
        std::vector<PointType>&point_coordinate);
    void setSearchParameterExternal(Eigen::Matrix4f ProjectionMatrix, std::vector< UVData<float>>pixelList);
  
protected:
   
    std::map <UVData<float>, PointData<float>>UVPointMap;
    std::map<PointData<float>, UVData<float>>PointUVMap;
    std::vector<UVData<float>>Targetgrid;
    std::vector< UVData<float>>uvDataList;
    Eigen::Matrix4f ProjectionMatrix_;
    GridBound targetGridBound;
};