#pragma once
#include"IFeatureEstimator.h"
#include"GridSearch.h"


class REG3D_API NormalEstimationFromImageGrid : public FeatureEstimators::IFeature
{
public:
    NormalEstimationFromImageGrid(cGridSearch &gridSearch, std::vector<PointType> inputCloud, Eigen::Vector3f vp = Eigen::Vector3f::Zero(), bool fp_normal = false) :
        input_points(inputCloud),
        newsearch(&gridSearch),
        outputCloud(new pcl::PCLPointCloud2),
        has_normals(false),
        flip_normals_viewpoint(fp_normal),
        sensor_origin(vp)
    {};
    ~NormalEstimationFromImageGrid();
    void ComputeFeature();
    std::vector<PointType> createVectorOfPoints(CloudWithoutType inputCloud);
    CloudWithoutType getOutputCloud()const;
    Eigen::Matrix3Xf getNormalsinMatrixForm()const;
    void SetStrategy(cGridSearch *Search, std::vector<PointType> inputCloud, Eigen::Vector3f sp = Eigen::Vector3f::Zero(), bool fp = false);
protected:
    bool has_normals;
    bool flip_normals_viewpoint ;
    Eigen::Vector3f sensor_origin;
    CloudWithoutType outputCloud;
    cGridSearch *newsearch = nullptr;
    Eigen::Matrix3Xf output_normals;
    std::vector<PointType> input_points;
    Eigen::Vector3f EstimateNormals(std::vector<PointType>neigbors, Eigen::Vector3f seedPoint);
    void FlipNormals(Eigen::Vector3f point, Eigen::Vector3f view_point, Eigen::Vector3f &Normal);

};
