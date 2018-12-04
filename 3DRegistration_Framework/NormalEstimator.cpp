#include"pch.h"
#include"NormalEstimator.h"
#include"Datatypes.h"
//#include<pcl/point_types.h>


NormalEstimationFromImageGrid:: ~NormalEstimationFromImageGrid()
{
    /*if (NULL != newsearch)
    {
        delete[] newsearch;
        newsearch = NULL;
    }*/
}
std::vector<PointType>NormalEstimationFromImageGrid::createVectorOfPoints(CloudWithoutType inputCloud)
{
    CloudPtr pSrc(new pcl::PointCloud <PointType>);
    std::vector<PointType>points;
    points.reserve(pSrc->points.size());
    for (size_t i = 0; i < pSrc->points.size(); i++)
    {
        PointType pt;
        pt.getVector3fMap() = pSrc->points[i].getVector3fMap();
        points.push_back(pt);
    }
    return points;
}

void NormalEstimationFromImageGrid::ComputeFeature()
{
    GridBound bound;
    std::vector< UVData<float>>pixelList =  newsearch->GetUVDataList();
    std::vector<int>grid = newsearch->PrepareGrid(pixelList, bound);
    pcl::PointCloud<PointNormalType>::Ptr cloud_with_normals(new pcl::PointCloud<PointNormalType>);
    cloud_with_normals->width = input_points.size();
    cloud_with_normals->height = 1;
    cloud_with_normals->points.resize(cloud_with_normals->width * cloud_with_normals->height);
    Eigen::Matrix3Xf nn = Eigen::Matrix3Xf::Zero(3, cloud_with_normals->width);
    for (size_t i = 0; i < input_points.size(); i++)
    {
        std::vector<PointType>NeighborPoints =  newsearch->ComputeEightNeigborsFromGrid(pixelList[static_cast<int>(i)], 
            bound,grid, input_points);
        Eigen::Vector3f normal = EstimateNormals(NeighborPoints, input_points[i].getVector3fMap());
        if (flip_normals_viewpoint == true)
        {
            FlipNormals(input_points[i].getVector3fMap(), sensor_origin, normal);
        }
        cloud_with_normals->points[i].getNormalVector3fMap() = normal;
        cloud_with_normals->points[i].getVector3fMap() = input_points[i].getVector3fMap();
        nn.col(i) = normal;
    }

   pcl::toPCLPointCloud2(*cloud_with_normals, *outputCloud);
   output_normals = nn;
}
void NormalEstimationFromImageGrid:: FlipNormals(Eigen::Vector3f point, Eigen::Vector3f view_point, Eigen::Vector3f &Normal)
{
    Eigen::Vector3f view_direction = view_point - point;
    float theta = view_direction.dot(Normal);
    if (theta < 0)
        Normal *= -1.0f;
}
Eigen::Vector3f NormalEstimationFromImageGrid::EstimateNormals(std::vector<PointType>neigbors, Eigen::Vector3f seedPoint)
{
    int neighborSize = neigbors.size();
    Eigen::Vector3f normalVector(0, 0, 0);
    std::vector<Eigen::Vector3f>differenceVector;
    differenceVector.reserve(neighborSize);
    for (int i = 0; i < neighborSize; i++)
    {
        Eigen::Vector3f newVector = neigbors[i].getVector3fMap();
        differenceVector.push_back((seedPoint - newVector));

    }
    if (neighborSize > 1)
    {

        int diffSize = differenceVector.size();
        if (diffSize == 2)
        {
            Eigen::Vector3f temp = differenceVector[0].cross(differenceVector[1]);
            normalVector = normalVector + temp;
        }
        else
        {
            for (int j = 0; j < diffSize; j++)
            {
                if ((j + 1) != diffSize)
                {
                    Eigen::Vector3f temp = differenceVector[j].cross(differenceVector[j + 1]);
                    normalVector = normalVector + temp;
                }
                else if (j + 1 == diffSize)
                {
                    normalVector = normalVector + differenceVector[diffSize - 1].cross(differenceVector[0]);
                }
            }
        }
        // normalVector = normalVector + differenceVector[0].cross(differenceVector[differenceVector.size() - 1]);

    }

    else if (neighborSize == 1)
    {
        normalVector = seedPoint.cross(neigbors[0].getVector3fMap());
    }
    else
    {
        normalVector = Eigen::Vector3f::UnitZ();
    }

    if (normalVector.norm() != 0)
    {
        normalVector.normalize();
    }
    differenceVector.clear();
    return (normalVector);
}
CloudWithoutType NormalEstimationFromImageGrid::getOutputCloud()const
{
    return outputCloud;
}
Eigen::Matrix3Xf NormalEstimationFromImageGrid:: getNormalsinMatrixForm()const
{
    return output_normals;
}
void NormalEstimationFromImageGrid::SetStrategy(cGridSearch *Search, std::vector<PointType> inputCloud, Eigen::Vector3f sp , bool fp )
{
    newsearch = Search;
    input_points = inputCloud;
    sensor_origin = sp;
    flip_normals_viewpoint = fp;
}