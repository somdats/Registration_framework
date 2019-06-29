#pragma once
#include"IFeatureEstimator.h"
#include"Datatypes.h"
#include"Config.h"
#include"NurbsSurface.h"
#include<map>
#include <chrono>

struct descriptor_Value
{
    int row_idx;
    int col_idx;
    int pix_idx;
    int pt_idx;
    float val_at_pixel;
    Eigen::Vector2d st;
    Eigen::VectorXd pt;
    descriptor_Value(): row_idx(-1), col_idx(-1), pix_idx(-1), pt_idx(-1), val_at_pixel(-INFINITY), st(-1,-1) {};
    descriptor_Value(int r, int c, int pix, int pt, int val, Eigen::Vector2d st_, Eigen::VectorXd pt_) :
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

    int columns;
    int rows;
    

public:
    CirconImage() {}
    CirconImage(int columns, int rows);
    ~CirconImage();

    void addCellAt(int row, int col,int col_div, float entry);
    float getCell(int row, int column, int col_div);
    void clearMatrix();
    std::vector<float> getImageData();
    void SetImageData(std::vector<float>new_img_data);
    void WriteImage(std::string &pszFileName, std::vector<float>ImageData);
    void SetImage(int col, int row);
    unsigned int WriteBitMap(std::string &pszFileName, unsigned char *pImgData);  // TODO change the location to neutral accessible source file
};


class REG3D_API CirconImageDescriptor: public FeatureEstimators::IFeature
{
public:
    CirconImageDescriptor(){}
    CirconImageDescriptor(CloudWithoutType &inputCloud, int division_row, int division_col, int height_division, const ON_NurbsSurface &surface_fit = ON_NurbsSurface(),
        const ON_NurbsCurve &curve_fit = ON_NurbsCurve()) :
        inputCloud(inputCloud),
        num_division_row(division_row), 
        num_division_col(division_col),
        num_height_division(height_division),
        Image2D(division_col, division_row),
        transformed_inputCloud(new pcl::PCLPointCloud2),
        basic_cell_index(-1),
        nb_surface(surface_fit),
        nb_curve(curve_fit),
        original_cloud_with_normal(new pcl::PointCloud <PointNormalType>)
    {
        pcl::fromPCLPointCloud2(*inputCloud, *original_cloud_with_normal);
    }
   
    CirconImageDescriptor & operator=(const CirconImageDescriptor &cid); // asignment
    CirconImageDescriptor( const CirconImageDescriptor &cid);  // copy constructor
    void ComputeFeature();
    void ComputeFeature(int i);
    void SetAngularResolution(float angle_resolution, int num_division);
    void SetRadialResolution(float distance, int num_division);
    void HeightResolution(float height_resolution, int num_division);
    CloudWithoutType TransformPointToLocalFrame();
    PointNormalType ComputeBasicPointOfInterest();
    void ConstructLocalFrameOfReference();
    float ComputeMaximumRadius(const CloudWithoutType& input_Cloud);
    float ComputeheightFromPointCloud(const CloudWithoutType& input_Cloud);
    void SetImageDescriptorResolution(float full_angle, float max_radius, float height_max);
    void WriteDescriptorAsImage(std::string FileName);
    void ReconstructPointCloud();
    void WritePointCloud(std::string fileName);
    void SetInputAsTransformedCloud(CloudWithoutType InputCloud);
    std::vector<float> GetDescriptorImage();
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
    std::vector<_dV> GetDescriptorContent();
    void CreateSecondaryDescriptor(const Eigen::VectorXd &pt);
    std::unique_ptr<ON_NurbsSurface> GetNurbsSurface();
    std::unique_ptr<ON_NurbsCurve> GetNurbsCurve();
    void SetNurbsSurfaceAndCurve(const ON_NurbsSurface &nbs, const ON_NurbsCurve &ncs);
   // nb_surface(inputCloud);
    static Eigen::Matrix4f ConstructLocalCoordinateAxes(CirconImageDescriptor &cid, PointNormalType &axis_point);
    void SetLocalFrame(const Eigen::Matrix4f &l_frame);
  
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
  
    Eigen::Matrix3f rot_matrix;  // if the current descriptor is rotated around different point: stores the rotation matrix
    int rotation_index;  // indicates no. of row shift from the original descriptor after the transformtion
    Eigen::Matrix4f WorldLocalTransformation; // stores the transformation associated with expressing world coordinate to point of interest(local frame)
    float sigma_threshold;
    int basic_cell_index;
    float max_radius;
    std::vector<_dV>descriptor_content;
    ON_NurbsSurface nb_surface;
    ON_NurbsCurve   nb_curve;
};

