#include"pch.h"
#include"InterpolationMethod.h"
#include"Datatypes.h"
#include<chrono>



void cInterPolation :: findPoint(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint)
{
    querypoint.getVector3fMap() = transform * querypoint.getVector3fMap();
    querypoint.getNormalVector3fMap() = transform.linear() * querypoint.getNormalVector3fMap();
    std::vector <UVData<float>>& grid = getTargetGrid();
    
    std::vector<float>PixelWeight;
    std::vector<UVData<float>>pixelNeighbors;

    bool correspStatus = false;
    correspStatus = this->FindUVNeigborsForQueryPoint(querypoint, grid, PixelWeight, pixelNeighbors);
    UVData<float> uvCoord;
    float depthSum = 0, sumX = 0, sumY = 0, normalX = 0, normalY = 0, normalZ = 0.0;
    PointData<float> tempPoint(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Eigen::Vector3f mV(0.0f, 0.0f, 0.0f), mN(0.0f, 0.0f, 0.0f);
    float weight = 0.0f;
    
    if (correspStatus == true)
    {
        std::map <UVData<float>, PointData<float>>&uvDataMap = getTargetMap();

        for (int itr = 0; itr < pixelNeighbors.size(); itr++)
        {
            uvCoord.u = pixelNeighbors[itr].u;
            uvCoord.v = pixelNeighbors[itr].v;
            std::map <UVData<float>, PointData<float>>::iterator iterator = uvDataMap.find(uvCoord);
            if (iterator == uvDataMap.end())
                break;
            PointData<float> pointCoord = iterator->second;
            Eigen::Vector3f cV(pointCoord.X, pointCoord.Y, pointCoord.Z), cN(pointCoord.nX, pointCoord.nY, pointCoord.nZ);
            mV = mV + PixelWeight[itr] * cV;
            mN = mN + PixelWeight[itr] * cN;
            weight += PixelWeight[itr];

        }
 
        if (weight != 0.0f)
        {
            mV = mV / weight;
            mN.normalize();
        }

        if (mV(0) != 0.0f && mV(1) != 0.0f && mV(2) != 0.0f)
        {
            if (isfinite(mV(0)) && isfinite(mV(1)) && isfinite(mV(2)))
            {
                // copypoint(tempPoint, targetPoint);
                targetPoint.getVector3fMap() = mV;
                targetPoint.getNormalVector3fMap() = mN;
            }
        }
       
    }
    else
    {
        targetPoint.x = NAN;
        targetPoint.y = NAN;
        targetPoint.z = NAN;
        targetPoint.normal_x = NAN;
        targetPoint.normal_y = NAN;
        targetPoint.normal_z = NAN;
    }
 
}