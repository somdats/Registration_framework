#pragma once
#include"ICorrespondenceEstimator.h"
#include"CirconDescriptor.h"
#include"ISearch.h"
#include"GridSearch.h"
#include"Datatypes.h"
#include"pct_io.h"
#include"Common.h"
#include<pcl/kdtree/kdtree_flann.h>
#include"NurbsSurface.h"
#define FULL_ANGLE 2 *M_PI
class REG3D_API CirconCorrespondence : public CorrespondenceEstimator::ICorrespondenceEstimator
{
public:
    CirconCorrespondence(CloudWithoutType sourceCloud, CloudWithoutType targetCloud, int div_row, int div_col, int height_division,
        int nr_resolution = 4, int col_search = 32, const ON_NurbsSurface &surface_fit = ON_NurbsSurface(), const ON_NurbsCurve &curve_fit = ON_NurbsCurve(),
        const ON_NurbsSurface &surface_fit_tgt = ON_NurbsSurface(), const ON_NurbsCurve &curve_fit_tgt = ON_NurbsCurve(),
        bool write_data = false, float rho = 1.0f, float lambda = 1.0f, bool poi = false) :
        input_source(sourceCloud),
        input_target(targetCloud),
        rho_(rho),
        lambda_(lambda),
        division_col(div_col),
        division_row(div_row),
        cid_source(sourceCloud, div_row, div_col, height_division, surface_fit, curve_fit),
        cid_target(targetCloud, div_row, div_col, height_division, surface_fit_tgt, curve_fit_tgt),
        basic_descriptor(poi),
        num_resolution(nr_resolution),
        nr_search_col(col_search),
        _write(write_data),
        original_src_cloud_with_normal(new pcl::PointCloud <PointNormalType>)
    {
        pcl::fromPCLPointCloud2(*sourceCloud, *original_src_cloud_with_normal);
    }
   
    struct Measure
    {
        float similarity_value;
        int cell_index; // cell_index corresponding to basic descriptor for which similarity value is maximum
        int rotation_index;
        std::vector<float>cell_values; // cell_values correspodning to descriptor with max. similarity value
        PointNormalType point_of_interest;
        size_t point_index;
        Eigen::Matrix4f WlTransform;
        std::map<int, size_t>img_pt_index_map;
        int primary_idx;
        float pix_value;
    };
    typedef PointData<float> PointDataType;
    void PrepareDataForCorrespondenceEstimation(CloudWithoutType &srcCloud, CloudWithoutType &tarCloud);
    search::ISearch* getSearchStrategy()const;
    Eigen::Matrix4f ComputeTransformation();
    void PrepareDescriptor(CirconImageDescriptor& cid, PointNormalType rotpoint);
    void PrepareTargetDescriptor(PointNormalType rotpoint);
    //TODO functions to select point of interest in source and target cloud
    float ComputeMeasureOfSimilarity(const std::vector<float>& src_image, const std::vector<float>& tar_image);
    CirconImageDescriptor TransformCirconDescriptor( int index);
    Measure CompareDescriptorWithSimilarityMeasure(const std::vector<float>& src_image, const std::vector<float>& tar_image);
    void CompareDescriptor(const std::vector<float>& src_image, const std::vector<float>& tar_image, int with_nurbs);
  static std::vector<float> ComputelaplacianOfNormals(CloudWithoutType &inputCloud);
 static  void WriteLaplacianImage(std::string fileName, CloudWithoutType inputCloud, std::vector <UVData<float>>pixelIndex);
 static std::map<float, int> ComputeMeanCurvatureFromPointCloud(CloudWithoutType inputCloud);
 static CloudWithNormalPtr WriteFilteredPoints(std::string FileName, CloudWithoutType inputCloud,  std::map<float, int>curvature_ratio, float threshold);
 void SetResolutionOfDescriptor( CirconImageDescriptor &dsc, int num_rows, int num_cols, int hei_div = 128);
 std::vector<std::pair<int, float>> ComputeFictitiousCorrespondence(std::map<int, size_t>img_point_map, CloudWithNormalPtr cloud,
     int corres_1_source_idx, PointNormalType corres_point);

 std::vector<std::pair<float, std::pair<int, int>>> ComputeCorrespondencePair(std::vector<std::pair<int, float>>src_poi,
     std::vector<std::pair<int, float>>tar_poi);
 Eigen::Matrix4f FindTransformationUsingFictitiousCorrespondence(std::vector<std::pair<float, std::pair<int, int>>> corres_pair,
     PointNormalType src_corres, PointNormalType tar_corres, Eigen::Matrix4f Pair_Transform);
 std::pair<float, float> ComputeStoppingCriteria(std::vector<std::pair<int, float>>src_poi, std::vector<std::pair<int, float>>tar_poi,
     PointNormalType src_corres, PointNormalType tar_corres, Eigen::Matrix4f Pair_Transform, Eigen::Matrix4f Corres_Transform);
 //void CreateIndicesVsPointMap(CloudWithoutType & Source, CloudWithoutType & Target);
 int ReadCorrespondenceIndex(std::string FileNameA, std::string fileNameB);
 Eigen::Matrix4f ComputeAlignmentMatrixinLocalFrame(Eigen::Matrix4f T1, Eigen::Vector3f Trans);
 void ComputeSimilarityBetweenTwoCloud(std::string dirname, std::string  OutputFile);
 float ComputeRotationIndexFromShift(std::vector<float>secondary_descriptor, std::vector<float>target_point_descriptor, 
     std::vector<float>&max_shift_descriptor, int &rotation_idx, std::map<int, size_t>&image_point_index_map_current, int& secondary_dsc_posn);
 void EvaluateSimilarityMeasureParameter(int src_idx, int tar_idx, float delta_step,std::string dirname, std::string  OutputFile);
 void SetParameterForSimilarityMeasure(float row, float lambda);
 std::pair<PointNormalType, PointNormalType> ComputeFictituousCorrespondencePair(PointNormalType qSrcPt, pcl::KdTreeFLANN<PointNormalType>ktree_src, 
     pcl::KdTreeFLANN<PointNormalType>ktree_tar, float avgDist, Eigen::Matrix4f curr_transform, std::pair<int,int>&index_pair);
 std::pair<float, float> EstimateStopParameter(std::pair<PointNormalType, PointNormalType>corres_pair, std::pair<int, int>index_pair, 
     Eigen::Matrix4f Pair_Transform, Eigen::Matrix4f Corres_Transform);
 static void ComputePointOfInterestbasedOnUniformSampling(CloudWithoutType inputCloud,  double radius, std::string OutputFileName);
 static double ComputeRadiusForSampling(CloudWithoutType CloudA, CloudWithoutType CloudB, double sf);
 float ComputeMaximumRotationShiftParameter(const std::vector<float>& secondary_descriptor, const std::vector<float>& target_point_descriptor,
std::vector<float>&max_shift_descriptor, std::map<int, size_t>&image_point_index_map_current, std::vector<_dV> &descriptor_content,
int &rotation_idx, int& secondary_dsc_posn);
 CirconImageDescriptor TransformCirconDescriptor(const Eigen::VectorXd &pt);

 
   



protected:
    CloudWithoutType input_source;
    CloudWithoutType input_target;
    CloudWithoutType srcCloudCorrespondence_;
    CloudWithoutType targetCloudCorrespondence_;
    bool _write;
    float rho_;
    float lambda_;
    CirconImageDescriptor cid_source;
    CirconImageDescriptor cid_target;
    cGridSearch *newsearch = nullptr;
    std::vector<float>source_descriptor_image;
    std::vector<float>target_descriptor_image;
    bool basic_descriptor;
    int division_col;
    int division_row;
    Measure max_measure;
    float Epsilon; // stop criterion for correspondence search
    std::map<float, size_t>pt_validity_ratio;
    std::vector<size_t>non_valid_points;
    int num_resolution = 4;
    int source_basic_index;
    int target_basic_index;
    int max_column_nr;
    std::vector<int>original_source_index;
    std::vector<int>original_target_index;
    double diagonal_length;
    int nr_search_col;
    std::vector<int>non_valid_index;
    CloudWithNormalPtr original_src_cloud_with_normal;
    float maximum_radius;
   /* std::map<PointDataType, int>source_correspondence_map;
    std::map<PointDataType, int>target_correspondence_map;*/
    std::vector<std::vector<float>> ResScaleTargetDescriptor(const std::vector<float>& Descriptor_Current);
    std::vector<float> RotateImageByIndexNumber(const Eigen::MatrixXf & vector_matrix, int rot_idx);
    float EvaluateSimilarityByRotationShift(const std::vector<std::vector<float>>& src_descriptors, const std::vector<std::vector<float>>& target_descriptors,
        int& rotation_idx, std::vector<float>& max_descriptor, int min_res = 8);
};
