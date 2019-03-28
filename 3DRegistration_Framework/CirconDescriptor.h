#pragma once
#include"IFeatureEstimator.h"
#include"Datatypes.h"
#include"Config.h"
#include<map>

class CirconImage
{
protected:
    std::vector<float>cell;

    int columns;
    int rows;
    unsigned int WriteBitMap(std::string &pszFileName, unsigned char *pImgData);  // TODO change the location to neutral accessible source file

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
};


class REG3D_API CirconImageDescriptor: public FeatureEstimators::IFeature
{
public:
    CirconImageDescriptor(){}
    CirconImageDescriptor(CloudWithoutType inputCloud, int division_row, int division_col, int height_division) :
        inputCloud(inputCloud),
        num_division_row(division_row), 
        num_division_col(division_col),
        num_height_division(height_division),
        Image2D(division_col, division_row),
        transformed_inputCloud(new pcl::PCLPointCloud2),
        basic_cell_index(-1),
        original_cloud_with_normal(new pcl::PointCloud <PointNormalType>)
    {
        pcl::fromPCLPointCloud2(*inputCloud, *original_cloud_with_normal);
    }
    CirconImageDescriptor & operator=(const CirconImageDescriptor &cid); // asignment
    CirconImageDescriptor( const CirconImageDescriptor &cid);  // copy constructor
    void ComputeFeature();
    void SetAngularResolution(float angle_resolution, int num_division);
    void SetRadialResolution(float distance, int num_division);
    void HeightResolution(float height_resolution, int num_division);
    CloudWithoutType TransformPointToLocalFrame();
    PointNormalType ComputeBasicPointOfInterest();
    void ConstructLocalFrameOfReference();
    float ComputeMaximumRadius(CloudWithoutType inputCloud);
    float ComputeheightFromPointCloud(CloudWithoutType inputCloud);
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
    void SetBasicPointIndex(size_t Idx);
    size_t GetBasicPointIndex();
    CloudWithNormalPtr GetoriginalCloud();
    int GetDescriptorCellIndexForInterestPoint();
   
    
        
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
    size_t basic_point_index;
    CloudWithNormalPtr original_cloud_with_normal;
    Eigen::Matrix3f rot_matrix;  // if the current descriptor is rotated around different point: stores the rotation matrix
    int rotation_index;  // indicates no. of row shift from the original descriptor after the transformtion
    Eigen::Matrix4f WorldLocalTransformation; // stores the transformation associated with expressing world coordinate to point of interest(local frame)
    float sigma_threshold;
    int basic_cell_index;
};

