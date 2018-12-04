#include"pch.h"
#include"PlaneProjectionMethod.h"
#include"Datatypes.h"


void cPlaneProjectionMethod::findPoint(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint)
{
    querypoint.getVector3fMap() = transform * querypoint.getVector3fMap();
    querypoint.getNormalVector3fMap() = transform.linear() * querypoint.getNormalVector3fMap();
    std::vector <UVData<float>>& grid = getTargetGrid();
    std::vector<float>PixelWeight;
    std::vector<UVData<float>>pixelNeighbors;
    bool correspStatus = false;

    correspStatus = this->FindUVNeigborsForQueryPoint(querypoint, grid, PixelWeight, pixelNeighbors);
    std::map <UVData<float>, PointData<float>>& uvDataMap = getTargetMap();
    UVData<float> uvCoord;

    float weight = 0.0f;
    std::vector<float>tempVector(PixelWeight);
    std::sort(tempVector.begin(), tempVector.end(), std::greater<>());
  
    Eigen::Vector3f projectedPoint(0.0f, 0.0f, 0.0f), projectedNormal(0.0f, 0.0f, 0.0f);

    if (pixelNeighbors.size() == 4)
    {
        Eigen::Vector3f PlaneOrigin(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f PlaneNormal(0.0f, 0.0f, 0.0f);
        for (int itr = 0; itr < pixelNeighbors.size(); itr++)
        {
           //auto  pos = std::find(PixelWeight.begin(), PixelWeight.end(), tempVector[itr]) - PixelWeight.begin();
            uvCoord.u = pixelNeighbors[itr].u;
            uvCoord.v = pixelNeighbors[itr].v;
            std::map <UVData<float>, PointData<float>>::iterator iterator = uvDataMap.find(uvCoord);
            if (iterator == uvDataMap.end())
                break;
            PointData<float> pointCoord = iterator->second;
            if (pointCoord.isValidPoint())
            {
                Eigen::Vector3f toProject(querypoint.x, querypoint.y, querypoint.z);
                Eigen::Vector3f PlaneOrigin(pointCoord.X, pointCoord.Y, pointCoord.Z);
                Eigen::Vector3f PlaneNormal(pointCoord.nX, pointCoord.nY, pointCoord.nZ);
                pcl::geometry::project(toProject, PlaneOrigin, PlaneNormal, projectedPoint);
                projectedPoint = projectedPoint + PixelWeight[itr] * projectedPoint;
                projectedNormal = projectedNormal + PixelWeight[itr] * projectedNormal;
                weight += PixelWeight[itr];
            }

        }

        projectedPoint = projectedPoint / weight;
        projectedNormal = projectedNormal/projectedNormal.norm();
        if (projectedPoint(0) != 0.0f && projectedPoint(1) != 0.0f && projectedPoint(2) != 0.0f)
        {
            if (isfinite(projectedPoint(0)) && isfinite(projectedPoint(1)) && isfinite(projectedPoint(2)))
            {
                // copypoint(tempPoint, targetPoint);
                targetPoint.getVector3fMap() = projectedPoint;
                targetPoint.getNormalVector3fMap() = projectedNormal;
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