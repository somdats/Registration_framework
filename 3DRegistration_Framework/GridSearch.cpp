#include"pch.h"
#include"GridSearch.h"


cGridSearch::cGridSearch()
{
    UVPointMap.clear();
    PointUVMap.clear();
    Targetgrid.clear();
    uvDataList.clear();
}

//cGridSearch::~cGridSearch()
//{
//    if (UVPointMap.size()!= 0)
//    {
//        PointUVMap.clear();
//    }
//    if(PointUVMap.size() != 0)
//    {
//        UVPointMap.clear();
//    }
//    if(Targetgrid.size() != 0)
//    {
//        Targetgrid.clear();
//    }
//    if(uvDataList.size() != 0)
//    {
//        uvDataList.clear();
//    }
//  
//}

// createsMap between UV and Point 

void cGridSearch::createMap(CloudWithoutType &targetCloud)
{
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*targetCloud, *pTarget);
    int size = static_cast<int>(pTarget->points.size());
    for(int i = 0; i < size;i++)
    {
        auto var = pTarget->points[i];
        PointData<float> tempData(var.x, var.y, var.z, var.normal_x, var.normal_y, var.normal_z);
        UVData<float>UV(uvDataList[i]);
        UVPointMap.insert(std::pair<UVData<float>, PointData<float>>(UV, tempData));
        PointUVMap.insert(std::pair<PointData<float>, UVData<float>>(tempData, UV));
    }
}

void cGridSearch::setSearchParameterExternal(Eigen::Matrix4f ProjectionMatrix, std::vector< UVData<float>>pixelList)
{
    ProjectionMatrix_ = ProjectionMatrix;
    uvDataList = pixelList;
}
// Prepare grid for search of target correspondence

void cGridSearch::PrepareSample(CloudWithoutType &targetCloud)
{
   
    UVData<float>uvPoint(-1.0f,-1.0f);
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*targetCloud, *pTarget);
    int size = static_cast<int>(pTarget->points.size());
    std::vector<float> uCoord,vCoord;
    uCoord.reserve(size);
    vCoord.reserve(size);
    for (int j = 0; j < size; j++)
    {
        auto var = pTarget->points[j];
        PointData<float> tempData(var.x, var.y, var.z, var.normal_x, var.normal_y, var.normal_z);
        UVData<float>UV(uvDataList[j]);
        std::map<PointData<float>, UVData<float>>::iterator iterator = PointUVMap.find(tempData);
        if (iterator == PointUVMap.end())
            break;
        auto uvCoord = iterator->second;
        uCoord.push_back(uvCoord.u);
        vCoord.push_back(uvCoord.v);
    }
    targetGridBound.u_max = *std::max_element(uCoord.begin(), uCoord.end());
    targetGridBound.v_max = *std::max_element(vCoord.begin(), vCoord.end());
    targetGridBound.u_min = *std::min_element(uCoord.begin(), uCoord.end());
    targetGridBound.v_min = *std::min_element(vCoord.begin(), vCoord.end());
    int gridSize = (static_cast<int>(targetGridBound.u_max) + 1) * (static_cast<int>(targetGridBound.v_max) + 1);
    Targetgrid.resize(gridSize, uvPoint);
    for (int Itr = 0; Itr < uvDataList.size(); Itr++)
    {
        Targetgrid[uvDataList[Itr].u + uvDataList[Itr].v * (static_cast<int>(targetGridBound.u_max) + 1)] = uvDataList[Itr];
    }
    uCoord.shrink_to_fit();
    vCoord.shrink_to_fit();
}
std::vector<int> cGridSearch::PrepareGrid(std::vector<UVData<float>> pixelCoordinates, GridBound &BoundOfgrid)
{
    UVData<float>uvPoint(-1.0f, -1.0f);
    std::vector<int>targetGrid;
  
    int size = static_cast<int>(pixelCoordinates.size());
    std::vector<float> uCoord, vCoord;
    uCoord.reserve(size);
    vCoord.reserve(size);
    if (pixelCoordinates.size() > 0)
    {
        for (int j = 0; j < size; j++)
        {
            uCoord.push_back(pixelCoordinates[j].u);
            vCoord.push_back(pixelCoordinates[j].v);
        }
        float maxUCoord = *std::max_element(uCoord.begin(), uCoord.end());
        float maxVCoord = *std::max_element(vCoord.begin(), vCoord.end());
        float minUCoord = *std::min_element(uCoord.begin(), uCoord.end());
        float minVCoord = *std::min_element(vCoord.begin(), vCoord.end());

        GridBound bound(maxUCoord, maxVCoord, minUCoord, minVCoord);
        BoundOfgrid = bound;
        int gridSize = (maxUCoord + 1) * (maxVCoord + 1);
        targetGrid.resize(gridSize,-1);
        for (int Itr = 0; Itr < pixelCoordinates.size(); Itr++)
        {
            targetGrid[pixelCoordinates[Itr].u + pixelCoordinates[Itr].v * (maxUCoord + 1)] = Itr;
        }
        uCoord.clear();
        vCoord.clear();
    }
    return targetGrid;
}
std::vector<PointType>cGridSearch::ComputeEightNeigborsFromGrid(UVData<float>QueryPixelIndex, GridBound bounds,
    std::vector<int>&GridIndex, std::vector<PointType>&point_coordinate)
{
    int x0 = QueryPixelIndex.u;
    int y0 = QueryPixelIndex.v;

    int x1 = QueryPixelIndex.u + 1;
    int y1 = QueryPixelIndex.v;

    int x2 = QueryPixelIndex.u + 1;
    int y2 = QueryPixelIndex.v - 1;

    int x3 = QueryPixelIndex.u;
    int y3 = QueryPixelIndex.v - 1;

    int x4 = QueryPixelIndex.u - 1;
    int y4 = QueryPixelIndex.v - 1;

    int x5 = QueryPixelIndex.u - 1;
    int y5 = QueryPixelIndex.v;

    int x6 = QueryPixelIndex.u - 1;
    int y6 = QueryPixelIndex.v + 1;

    int x7 = QueryPixelIndex.u;
    int y7 = QueryPixelIndex.v + 1;

    int x8 = QueryPixelIndex.u + 1;
    int y8 = QueryPixelIndex.v + 1;
    std::vector<PointType> points;
    if (x0 != bounds.u_max)
    {
        if (x1 != -1 && y1 != -1 && GridIndex[x1 + y1 * (bounds.u_max + 1)] != -1)
        {
            points.push_back(point_coordinate[GridIndex[x1 + y1 * (bounds.u_max + 1)]]);
        }
    }
    if (x0 != bounds.u_max)
    {
        if (x2 != -1 && y2 != -1 && GridIndex[x2 + y2 * (bounds.u_max + 1)] != -1)
        {
            points.push_back(point_coordinate[GridIndex[x2 + y2 * (bounds.u_max + 1)]]);
        }
    }

    if (x3 != -1 && y3 != -1 && GridIndex[x3 + y3 * (bounds.u_max + 1)] != -1)
    {
        points.push_back(point_coordinate[GridIndex[x3 + y3 * (bounds.u_max + 1)]]);
    }

    if (x4 != -1 && y4 != -1 && GridIndex[x4 + y4 * (bounds.u_max + 1)] != -1)
    {
        points.push_back(point_coordinate[GridIndex[x4 + y4 * (bounds.u_max + 1)]]);
    }
    if (x5 != -1 && y5 != -1 && GridIndex[x5 + y5 * (bounds.u_max + 1)] != -1)
    {
        points.push_back(point_coordinate[GridIndex[x5 + y5 * (bounds.u_max + 1)]]);
    }
    if (y0 != bounds.v_max)
    {
        if (x6 != -1 && y6 != -1 && GridIndex[x6 + y6 * (bounds.u_max + 1)] != -1)
        {
            points.push_back(point_coordinate[GridIndex[x6 + y6 * (bounds.u_max + 1)]]);
        }
    }
    if (y0 != bounds.v_max)
    {
        if (x7 != -1 && y7 != -1 && GridIndex[x7 + y7 * (bounds.u_max + 1)]!= -1)
        {
            points.push_back(point_coordinate[GridIndex[x7 + y7 * (bounds.u_max + 1)]]);
        }
    }
    if (y0 != bounds.v_max && x8 < bounds.u_max && y8 < bounds.v_max)
    {
        if (x8 != -1 && y8 != -1 && GridIndex[x8 + y8 *  (bounds.u_max + 1)] != -1)
        {
            points.push_back(point_coordinate[GridIndex[x8 + y8 *  (bounds.u_max + 1)]]);
        }
    }
    return points;

}
 /* Compute neighborpixels (4-neighbor) in depth image
I/P: queryData(UvIndex obtained after projection)
O/P: Collection of interpolated pixelweight
O/P: Neighbor pixels on the grid*/


bool cGridSearch::FindUVNeigborsForQueryPoint(PointRGBNormalType queryPoint, std::vector<UVData<float>> &Targetgrid, std::vector<float> &PixelWeight,
    std::vector<UVData<float>>&PixelNeighbors)const
{
    PixelWeight.clear();
    PixelNeighbors.clear();
    UVData<float> uvIndex(-1.0, -1.0);
    bool status = false;
    /*std::vector<float> uCoord, vCoord;
    uCoord.resize(uvDataList.size());
    vCoord.resize(uvDataList.size());
    for (size_t i = 0; i < uvDataList.size(); i++)
    {
        uCoord[i] = uvDataList[i].u;
        vCoord[i] = uvDataList[i].v;

    }*/
    float maxUCoord = targetGridBound.u_max;
    float maxVCoord = targetGridBound.v_max;
    float minUCoord = targetGridBound.u_min;
    float minVCoord = targetGridBound.v_min;

    if (Targetgrid.size() == 0)
        throw std::runtime_error("Must create the grid before being able to determine correspondence.");

    UVData<float>queryData = getProjectedPoint(queryPoint);
    if (floor(queryData.u) > maxUCoord || floor(queryData.u) < minUCoord || floor(queryData.v) > maxVCoord || floor(queryData.v) < minVCoord)
        return status;

    float x = floor(queryData.u);
    float y = floor(queryData.v);
    float w1, w2, w3, w4;
    w1 = (queryData.u - floor(queryData.u)) * (queryData.v - floor(queryData.v));
    w2 = (ceil(queryData.u) - queryData.u) * (queryData.v - floor(queryData.v));
    w3 = (queryData.u - floor(queryData.u)) * (ceil(queryData.v) - queryData.v);
    w4 = (ceil(queryData.u) - queryData.u) * (ceil(queryData.v) - queryData.v);
    float x0 = x;
    float y0 = y + 1;

    float x1 = x + 1;
    float y1 = y;

    float x2 = x + 1;
    float y2 = y + 1;

    UVData<float> uv(x, y);
    PixelNeighbors.push_back(uv);
    PixelWeight.push_back(w1);

    if (x == maxUCoord && y != maxVCoord)
    {
        if (x0 != -1 && y0 != -1 && Targetgrid[x0 + y0 * (maxUCoord + 1)].u != uvIndex.u)
        {
           
            PixelNeighbors.push_back(Targetgrid[x0 + y0 * (maxUCoord + 1)]);
            PixelWeight.push_back(w3);
            
        }
    }
    else  if (y == maxVCoord && x != maxUCoord)
    {
        if (x1 != -1 && y1 != -1 && Targetgrid[x1 + y1 * (maxUCoord + 1)].u != uvIndex.u)
        {

            PixelNeighbors.push_back(Targetgrid[x1 + y1 *  (maxUCoord + 1)]);
            PixelWeight.push_back(w2);

        }
    }

    else if (x != maxUCoord && y != maxVCoord)
    {
        if (x0 != -1 && y0 != -1 && Targetgrid[x0 + y0 * (maxUCoord + 1)].u != uvIndex.u)
        {

            PixelNeighbors.push_back(Targetgrid[x0 + y0 * (maxUCoord + 1)]);
            PixelWeight.push_back(w3);

        }
        if (x1 != -1 && y1 != -1 && Targetgrid[x1 + y1 * (maxUCoord + 1)].u != uvIndex.u)
        {

            PixelNeighbors.push_back(Targetgrid[x1 + y1 *  (maxUCoord + 1)]);
            PixelWeight.push_back(w2);
        }
        if (x2 != -1 && y2 != -1 && Targetgrid[x2 + y2 * (maxUCoord + 1)].u != uvIndex.u)
        {

            PixelNeighbors.push_back(Targetgrid[x2 + y2 *  (maxUCoord + 1)]);
            PixelWeight.push_back(w4);

        }

    }
    if (PixelNeighbors.size() == 4)
    {
        status = true;
    }
    return status;

}
void cGridSearch::findPoint(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint)
 {
    //empty function
}

int cGridSearch:: findIndices(PointRGBNormalType &querypoint)
{
    return 0;
}

std::vector<UVData<float>>& cGridSearch::getTargetGrid()
{
   return Targetgrid;
}


std::map <UVData<float>, PointData<float>>& cGridSearch::getTargetMap()
{
   /* std::map <UVData<float>, PointData<float>> uvDataMap;
    uvDataMap = UVPointMap;*/
    return UVPointMap;
}


UVData<float> cGridSearch::getProjectedPoint(PointRGBNormalType querypoint)const
{
    UVData<float> screenCoord;
    Eigen::Vector3f  p1 = querypoint.getVector3fMap();
    Eigen::Vector4f p2(p1(0),p1(1),p1(2), 0);
    Eigen::Vector4f p2InCamCoord = ProjectionMatrix_ * p2;

    p2InCamCoord = p2InCamCoord / p2InCamCoord(3);  // perspective Divide
    screenCoord.u = (p2InCamCoord(0) + 1) * 0.5 * 640;
    screenCoord.v = (1 - (p2InCamCoord(1) + 1) * 0.5) * 480;
    /*screenCoord.u = temp(0) / temp(2);
    screenCoord.v = temp(1) / temp(2);*/
    return screenCoord;
 //The computation is specific to analytic surface
 //   float iDimension = 100;
 //   float jDimension = 100;
 //   float GridExtentX = 6.06f; //1.33
 //   float GridExtentY = 6.06f;  //1.0f
 //   float Xmin = -GridExtentX / 2.0;
 //   float Ymin = -GridExtentY / 2.0;
 //   uv.u = (((temp(0) - Xmin) * iDimension) / GridExtentX) - 0.5f;
 //   uv.v = (((temp(1) - Ymin) * jDimension) / GridExtentY) - 0.5f;

}
void cGridSearch :: setUVDataList(std::vector< UVData<float>>pixelVertexList)
{
    uvDataList.resize(pixelVertexList.size());
    uvDataList = pixelVertexList;
   
}
std::vector< UVData<float>> cGridSearch::GetUVDataList()
{
    return uvDataList;
}
std::vector<UVData<float>>cGridSearch::getTargetPixelCoordinate()
{
    return uvDataList;
}