#pragma once
#include"IFeatureEstimator.h"
#include"Datatypes.h"
#include"Config.h"
#include"NurbsSurface.h"
#include<map>
#include <chrono>
#include"UniformGrid.h"

struct descriptor_Value
{
    int row_idx;
    int col_idx;
    int pix_idx;
    int pt_idx;
    float val_at_pixel;
    Eigen::Vector2d st;
    PointNormalType pt;
    descriptor_Value(): row_idx(-1), col_idx(-1), pix_idx(-1), pt_idx(-1), val_at_pixel(-INFINITY), st(-1,-1) {};
    descriptor_Value(int r, int c, int pix, int pt, int val, Eigen::Vector2d st_, PointNormalType pt_) :
        row_idx(r), col_idx(c), pix_idx(pix), pt_idx(pt), val_at_pixel(val), st(st_),
        pt(pt_)
    {}
  
    descriptor_Value(const descriptor_Value& temp)
    {
        row_idx = temp.row_idx;
        col_idx = temp.col_idx;
        pix_idx = temp.pix_idx;
        pt_idx = temp.pt_idx;
        val_at_pixel = temp.val_at_pixel;
        st = temp.st;
        pt = temp.pt;
    
    }
    descriptor_Value& operator =(const descriptor_Value &temp)
    {
        if (this == &temp)
            return *this;
        row_idx = temp.row_idx;
        col_idx = temp.col_idx;
        pix_idx = temp.pix_idx;
        pt_idx = temp.pt_idx;
        val_at_pixel = temp.val_at_pixel;
        st = temp.st;
        pt = temp.pt;
        return *this;
    }
  
    bool operator<(const descriptor_Value& a)
    {
        return col_idx < a.col_idx;
    }

};
typedef struct descriptor_Value _dV;

class CirconImage
{
protected:
    std::vector<float>cell;
    std::vector<std::vector<float>>image;

    int columns;
    int rows;
    

public:
    CirconImage() {}
    CirconImage(int columns, int rows);
    ~CirconImage();

    void addCellAt(int row, int col,int col_div, float entry);
    float getCell(int row, int column, int col_div);
    void clearMatrix();
    std::vector<std::vector<float>> getImageData();
    void SetImageData(std::vector<float>new_img_data);
    void WriteImage(std::string &pszFileName, std::vector<std::vector<float>>ImageData);
    void SetImage(int col, int row);
    unsigned int WriteBitMap(std::string &pszFileName, unsigned char *pImgData);  // TODO change the location to neutral accessible source file
};


class REG3D_API CirconImageDescriptor/*: public FeatureEstimators::IFeature*/
{
public:
    CirconImageDescriptor(){}
   // ~CirconImageDescriptor();
    CirconImageDescriptor(CloudWithoutType &inputCloud, int division_row, int division_col, int height_division, std::vector<Eigen::Vector2d> st_param = {Eigen::Vector2d(-1.0,-1.0)}, const ON_NurbsSurface &surface_fit = ON_NurbsSurface(),
        const ON_NurbsCurve &curve_fit = ON_NurbsCurve(), int max_search = 16) :
        inputCloud(inputCloud),
        num_division_row(division_row), 
        num_division_col(division_col),
        num_height_division(height_division),
        Image2D(division_col, division_row),
        transformed_inputCloud(new pcl::PCLPointCloud2),
        basic_cell_index(-1),
        nb_surface(surface_fit),
        nb_curve(curve_fit),
        no_col_search(max_search),
        st_params(st_param),
        kdTree(new pcl::KdTreeFLANN<PointType>),
        Cloud2D(new pcl::PointCloud<PointType>),
        original_cloud_with_normal(new pcl::PointCloud <PointNormalType>)
    {
        pcl::fromPCLPointCloud2(*inputCloud, *original_cloud_with_normal);
    }
   
    CirconImageDescriptor & operator=(const CirconImageDescriptor &cid); // asignment
    CirconImageDescriptor( const CirconImageDescriptor &cid);  // copy constructor
    void ComputeFeature(const CUniformGrid2D &cGrid2D, const cParameterGrid &_pGrid);
    void ComputeFeature(int i);
    void SetAngularResolution(float angle_resolution, int num_division);
    void SetRadialResolution(float distance, int num_division);
    void HeightResolution(float height_resolution, int num_division);
    CloudWithoutType TransformPointToLocalFrame();
    PointNormalType ComputeBasicPointOfInterest();
    void ConstructLocalFrameOfReference();
    float ComputeMaximumRadius(const CloudWithoutType& input_Cloud);
    float ComputeheightFromPointCloud(const CloudWithNormalPtr& input_Cloud);
    void SetImageDescriptorResolution(float full_angle, float max_radius, float height_max);
    void WriteDescriptorAsImage(std::string FileName);
    void ReconstructPointCloud();
    void WritePointCloud(std::string fileName);
    void SetInputAsTransformedCloud(CloudWithoutType InputCloud);
    std::vector<std::vector<float>> GetDescriptorImage();
    std::vector<float>TransformImageData(std::vector<float>prev_data, int query_index);
    void UpdateLocalFrameafterTransformation(PointNormalType new_origin, std::vector<Eigen::Vector3f> new_local_frame);
    PointNormalType GetRotationAXisPoint();
    float GetRotationAngle();
    int GetRotationIndex();
    int GetPointIndexFromCloud(int img_idx);
    void UpdateImageDataAfterTransformation(int index);
    void SetRotationAxisPoint(PointNormalType rotpoint);
    float GetRadialResolution();
    float GetColumnDivision();
    float GetRowDivision();
    float GetHeightDivision();
    Eigen::Matrix4f GetTransformation();
    float GetAngularResolution();
    void SetDivision(int num_rows, int num_cols, int num_hei);
    float GetRejectionRatioOfPoints();
    void  UpdateImageDimension(int col, int row);
    std::map<int, size_t> GetImagePointMap();
    void SetBasicPointIndex(int Idx);
    int GetBasicPointIndex();
    CloudWithNormalPtr GetoriginalCloud();
    int GetDescriptorCellIndexForInterestPoint();
    float GetHeightResolution();
    void SetRotationIndex(int index);
    float GetRadiusFromCloud();  // useful for computing radial resolution
    void SetMaximumRadius(float rad);
    std::vector<std::vector<_dV>> GetDescriptorContent();
    void CreateSecondaryDescriptor(const PointNormalType /*Eigen::VectorXd*/ &pt, const cParameterGrid & pGrid);
    std::unique_ptr<ON_NurbsSurface> GetNurbsSurface();
    std::unique_ptr<ON_NurbsCurve> GetNurbsCurve();
    void SetNurbsSurfaceAndCurve(const ON_NurbsSurface &nbs);
   // nb_surface(inputCloud);
    static Eigen::Matrix4f ConstructLocalCoordinateAxes(CirconImageDescriptor &cid, PointNormalType &axis_point);
    void SetLocalFrame(const Eigen::Matrix4f &l_frame);
    void UpdateDeescriptor(const PointNormalType /*Eigen::VectorXd*/ &pt);
    void SetMaxSearchColumn(int max_value);
    void SetBoundingBoxInformation(const surface::CloudBoundingBox &box);
    Eigen::Vector2d TransformAndScaleParametricCoordinate(const Eigen::Vector2d & vec2d, const double &w, const double &h);
    void SetParametersforDescriptor(std::vector<Eigen::Vector2d> st_parameter);
    void Set2DCloud(const CloudPtr &cloud);
    typedef pcl::KdTreeFLANN<PointType>KdTreeType;
    void SetMaximumAverageDistance(const float &dist);
    void Set2DUniformGrid(CloudWithNormalPtr &in_cloud);
    void SetFlagForHighResolution(const bool &flag);
    void ResetFlagForHighResolution();
    void SetUpResolutionCount(int res);
    float GetAverageDist();
    std::vector<Eigen::Vector2d> GetInitialParameters();
    void setParametergrid(const cParameterGrid &pGrid);
   cParameterGrid GetParameterGrid();
   std::unique_ptr<cParameterGrid> duplicate(std::unique_ptr<cParameterGrid> const& ptr);
  
protected:
    CloudWithoutType inputCloud;
    std::vector<std::pair<UVData<float>, float>>circon_data;  // stores values at each image indices
    std::map<int, size_t>index_index_map;  // stores image index corresponding to actual point index 
    float angle_resolution;
    float rad_resolution;
    float height_resolution;
    std::vector<Eigen::Vector3f>Local_Coordinate_Frame;
    int num_division_row; // in the power of 2^x
    int num_division_col;
    int num_height_division;
    PointNormalType RotationAxisPoint;
    CirconImage Image2D;
    float max_value_image;
    float min_value_image;
    std::vector<Eigen::Vector3f>reconstructed_points;
    CloudWithoutType transformed_inputCloud;   // points transformed to local_frame at the beginning
    Eigen::Matrix3f World_to_local;  // basic point of interest rotation frame information
    int basic_point_index;
    CloudWithNormalPtr original_cloud_with_normal;
    CloudPtr Cloud2D;
   
  
    Eigen::Matrix3f rot_matrix;  // if the current descriptor is rotated around different point: stores the rotation matrix
    int rotation_index;  // indicates no. of row shift from the original descriptor after the transformtion
    Eigen::Matrix4f WorldLocalTransformation; // stores the transformation associated with expressing world coordinate to point of interest(local frame)
    float sigma_threshold;
    int basic_cell_index;
    float max_radius;
    std::vector<std::vector<_dV>>descriptor_content;
    ON_NurbsSurface nb_surface;
    ON_NurbsCurve   nb_curve;
    int no_col_search;
    surface::CloudBoundingBox bbs;
    std::vector<std::map<float, int>>vector_of_maps;
    
    std::vector<Eigen::Vector2d>st_params;
    KdTreeType kdTree;
   // CUniformGrid2D cGrid2D;
    float avgpoint_dist;
    bool high_res_flag = false;
    int up_resolution_count = 16;
   // cParameterGrid _pGrid;
 
};

