#pragma once
#include"ISearch.h"
#include"Datatypes.h"



using namespace Eigen;

class REG3D_API cKDTreeSearch : public search::ISearch
{
public:
    cKDTreeSearch(bool use_flag = true, float scaleF = 0.0f, bool r_query = false):
    inputCloud(new pcl::PCLPointCloud2),
        with_normals(false),
        scale_factor(scaleF),
        radius_query(r_query),
        use_nanoFlann_for_search(true)
    {};

    cKDTreeSearch(std::vector<Eigen::Matrix3f>& Principal_frame_source, std::vector<Eigen::Matrix3f>& Principal_frame_target, 
        std::vector<Eigen::Vector3f> &eigen_values_source,
        std::vector<Eigen::Vector3f> &eigen_values_target,
        float scaleF = 0.0f, bool r_query = false) :
        inputCloud(new pcl::PCLPointCloud2),
        with_normals(false),
        scale_factor(scaleF),
        radius_query(r_query),
        use_nanoFlann_for_search(true),
        Principal_frame_source(Principal_frame_source),
        Principal_frame_target(Principal_frame_target),
        eigen_values_target(eigen_values_target),
        eigen_values_source(eigen_values_source)

    {};            // newly added constructor to additionally handle prinicpalFrame for source and targetcloud
    ~cKDTreeSearch();
    void PrepareSample(CloudWithoutType &targetCloud) override;
    inline int KdTreeGetPointCount() const;
    void findPoint(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint);
    int findIndices(PointRGBNormalType &querypoint);
    void SetPrincipalFrame(std::vector<Eigen::Matrix3f> &p_frame_source, std::vector<Eigen::Matrix3f> &p_frame_target);
    Eigen::Matrix3f getPrincipalFrameForPoint(int index);
    void CreateIndicesVsPointMap(CloudWithoutType &PointCloudA, CloudWithoutType &PointCloudB);
    bool QueryPrincipalFrame(PointRGBNormalType &query_sourcepoint, int query_targetindex, Eigen::Matrix3f & p_frame_source,
        Eigen::Matrix3f & p_frame_target);
    void EstimateCorrespondenceWithFiltering(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, Eigen::Matrix3f & principal_frame,
        PointRGBNormalType &targetPoint);
    Eigen::Matrix3f getPrincipalFrameForPoint(PointRGBNormalType &querypoint);
    typedef pcl::KdTreeFLANN<PointType>KdTreeType;
    typedef PointData<float> PointDataType;
    std::map<PointDataType, int> GetIndicesVsPointMap();
    void getEigenValuesForPoint(Eigen::VectorXf &querypointA, int id, Eigen::Vector3f &source_eig_value, Eigen::Vector3f &targ_eig_value);
    bool QueryEigenValues(PointRGBNormalType &query_sourcepoint, int query_targetindex, Eigen::Vector3f &eigenval_source,
        Eigen::Vector3f &eigenval_target);
    void SetFeatureAngleForSourceAndTarget(std::vector<double>feature_angles_s, std::vector<double>feature_angles_t);
    void getFeatureAngle(Eigen::VectorXf &querypointA, int id, double &Fa_source, double &Fa_target);

   
protected:
    KdTreeType *kdTree = nullptr;
    int targetCloudSize = 0;
    bool with_normals;
    CloudWithoutType inputCloud;
    bool use_nanoFlann_for_search;
    CloudWithNormalPtr inputTargetPointSet;
    float scale_factor;
    float avgDistanceTargetCloud;
    bool radius_query;
    std::vector<Eigen::Matrix3f> Principal_frame_source;
    std::vector<Eigen::Matrix3f> Principal_frame_target;
    std::map<PointDataType,int>Principal_frame_Index_source_map;
    std::map<PointDataType, int>Principal_frame_Index_target_map;
    std::vector<Eigen::Vector3f> eigen_values_source;
    std::vector<Eigen::Vector3f> eigen_values_target;
    std::vector<double>Point_feature_angles_source; // incoporrate new feature for the curvature analysis
    std::vector<double>Point_feature_angles_target;
    
};
