

#pragma once
#include"ISearch.h"
#include"Datatypes.h"
#include "wfuncs.h"
//pcl includes
#include<pcl/conversions.h>
#include"KdTreeSearch.h"

using namespace Eigen;

class REG3D_API cMLSearch : public search::ISearch
{
public:
    cMLSearch(CloudWithoutType &targetCloud, IWeightingFunction<float> &WeightFunct, float scalefactor = 1.0f, int polynomialOrder = 1, int itr_max = 20, float Resolution = 0.0f,
        bool with_filter = false, bool curv_filtering = false) :
        inputCloud(targetCloud),
        polynomialOrder_(polynomialOrder),
        scaleF(scalefactor),
        WeightFunc(WeightFunct),
        mls_itr_max(itr_max),
        m_resolution(Resolution),
        with_filtering(with_filter),
        _curvatureFiltering(curv_filtering)
        {};
    cMLSearch(std::vector<Eigen::Matrix3f>& Principal_frame_source, std::vector<Eigen::Vector3f> &eigen_values_source,
        IWeightingFunction<float> &WeightFunct, float scalefactor = 1.0f, int polynomialOrder = 1,
        int itr_max = 20, float threshold = 0.7f, bool with_filter = false, bool curv_filtering = false, bool t_projection =false, float Resolution = 0.0f ) :
        /* inputCloud(targetCloud),*/
        polynomialOrder_(polynomialOrder),
        scaleF(scalefactor),
        WeightFunc(WeightFunct),
        mls_itr_max(itr_max),
        Principal_frame_source(Principal_frame_source),
        eigen_values_source(eigen_values_source),
        m_resolution(Resolution),
        with_filtering(with_filter),
        normal_filter_threshold(threshold),
        _curvatureFiltering(curv_filtering),
        triangle_projection(t_projection)
       
    {};
    ~cMLSearch();
    void PrepareSample(CloudWithoutType &targetCloud) override;
    void findPoint(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint);
    void findApproximatePointforMLSSmoothing(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint);
    int findIndices(PointRGBNormalType &querypoint);
    std::vector<float> ComputeWeightFromLocalNeighborhood(PointNormalType &querySourcePoint, CloudPtr &targetCloud,
        std::vector<int> &nn_indices, IWeightingFunction<float> &weightingFunction, float kernelSize);
    Eigen::Vector3f ComputeWeightedCovariancematrix(CloudPtr &targetCloud, std::vector<float>& Weights,
        std::vector<int> nn_indices, Eigen::Vector3f &meanpoint, std::vector<Eigen::Vector3f> &Eigen_Vectors,
        Eigen::MatrixXf &cov_matrix);
    Eigen::Vector3f ComputeLocalReferenceDomainOrigin(PointNormalType &querySourcePoint,
        std::vector<Eigen::Vector3f> &Eigen_Vectors, Eigen::Vector3f &meanpoint);
    std::vector<Eigen::Vector3f> transformNeighborhoodtoLocalFrame(PointNormalType &querySourcePoint,
        CloudPtr &targetCloud,
        std::vector<int>&nn_indices, Eigen::Vector3f &meanpoint, std::vector<Eigen::Vector3f> &Eigen_Vectors,
        Eigen::Vector3f &localOrigin);
    std::vector<Eigen::Vector3f> ReProjectneighborstoWorldFrame(std::vector<Eigen::Vector3f> &transformedlocalNeighbor,
        Eigen::Vector3f &localOrigin,
        std::vector<Eigen::Vector3f> &Eigen_Vectors);
    Eigen::VectorXf poly_fit(int polynomialOrder_, std::vector<Eigen::Vector3f> &transformedlocalNeighbor,
        std::vector<float>&Weights);
    Eigen::VectorXf ComputeCorrespondenceUsingPolyfit(Eigen::VectorXf &coefficients, Eigen::Vector3f &localOrigin,
        std::vector<Eigen::Vector3f> &Eigen_Vectors);
    CloudWithoutType &getTargetCloudForMLSSurface();
    Eigen::Matrix3f GetPrincipalFrame();
    Eigen::Vector3f GetEigenValues();
    Eigen::Vector3f ComputeEigenValues(Eigen::MatrixXf covariance_matrix);
    void SetFeatureAngleForSourceCloud(std::vector<double>feature_angles_s);
    void getFeatureAngle(Eigen::VectorXf &querypointA, double &Fa_source);

    Eigen::Vector3f getEigenValuesForPoint(PointRGBNormalType &querypoint);
    Eigen::Matrix3f getPrincipalFrameForPoint(PointRGBNormalType &querypoint);
    void CreateIndicesVsPointMap(CloudWithoutType & PointCloud);
    typedef pcl::KdTreeFLANN<PointType>KdTreeType;
    typedef PointData<float> PointDataType;
    void SetIndicesVsPointMap(std::map<PointDataType, int>pf_source_map);
    void setPolygonMesh(pcl::PolygonMesh mesh);
    bool GetProjectedPointFromSurface(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint);

    // new
    typedef pcl::KdTreeFLANN<PointNormalType>KnTree;

    struct SNeighbor
    {
        //!list of triangles which are connected to a vertex
        std::vector< std::vector<pcl::Vertices>> m_acListTri;
        //!indices of neighboring triangles of a vertex in a mesh
        std::vector<int> m_anFaceIndicesList;
        //!indices of neighboring vertices of a vertex in a mesh
        std::vector<int> m_anVertexIndxList;
        //!normals of all neighboring triangles
        std::vector<Eigen::Vector3f> m_acUnitNormals;
        //! max area of neighboring triangles
        double dMaxArea;
        //! sum of areas of all neighboring triangles
        double dAreaSum;
        //!average of all normals of neighboring triangles	
        Eigen::Vector3f avgUnitNormal;

        //! initialize the members
        SNeighbor()
        {
            dMaxArea = -1;
            avgUnitNormal.setZero();
            dAreaSum = 0;
        }
    };
    typedef SNeighbor pointNeigbor;

    void GenerateFaceNeighborhood(pcl::PolygonMesh &mesh, std::vector<SNeighbor> &ptNeighbors);

protected:
    KdTreeType *kdTree = nullptr;
    int targetCloudSize = 0;
    CloudWithoutType inputCloud;
    float avgDistanceTargetCloud;
    int polynomialOrder_;
    float scaleF;
    IWeightingFunction<float> &WeightFunc;
    CloudPtr inputTargetPointSet;
    CloudWithNormalPtr inputTargetWithNormal;
    int mls_itr_max;
    Eigen::Matrix3f principalFrame;
    Eigen::Vector3f Eigen_Values;
    std::vector<Eigen::Matrix3f> Principal_frame_source;
    std::map<PointDataType, int>Principal_frame_Index_source_map;
    std::vector<Eigen::Vector3f> eigen_values_source;
    bool with_filtering;
    float normal_filter_threshold;
    float m_resolution;
    std::vector<double>Point_feature_angles_source;
    bool _curvatureFiltering;
    KnTree * kntree = nullptr;
    bool triangle_projection;
    pcl::PolygonMesh Mesh;
    std::vector<SNeighbor>pcNeigborhood;
};
