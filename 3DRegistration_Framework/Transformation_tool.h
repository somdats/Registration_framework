#pragma once
#include"Datatypes.h"


#include"Config.h"

namespace tool
{
    typedef struct CameraParameter
    {
        float focallength;
        float PrincipalOffsetX;
        float PrincipalOffsetY;
        int ImageWidth;
        int ImageHeight;
    }CameraParameter;
    REG3D_API CloudWithoutType TransFormationOfCloud(CloudWithoutType inputCloud, Eigen::Matrix4f transformationMatrix);
    REG3D_API void writeTransformationMatrix(const std::string &matrixFileName, std::vector<Eigen::Matrix4f> transformationMatrix);
    REG3D_API void writeMatrix(const std::string &matrixFileName, std::vector<Eigen::Matrix4f> transformationMatrix);
    REG3D_API std::vector<Eigen::Affine3f> ReadTransformationMatrix(const std::string &matrixFileName);
    REG3D_API Eigen::Matrix4Xf GenerateCameraMatrix(CloudWithoutType &Coordinates3D,std::vector<UVData<float>> &pixelCoordinates, CameraParameter &CamParam);
    REG3D_API  Eigen::Matrix3Xf createPointMatrixFromCloud(CloudWithoutType &cloudInput);
    REG3D_API Eigen::Matrix3Xf createNormalMatrixFromCloud(CloudWithoutType &cloudInput);
    REG3D_API int writeTransformationMatrixinBlock(char* pathAndName, std::vector<Eigen::Matrix4f> transformationMatrix);
    REG3D_API Eigen::Affine3f ComputeRelativeTransformation(Eigen::Affine3f target, Eigen::Affine3f source, Eigen::Affine3f rotate = Eigen::Affine3f::Identity());
    REG3D_API Eigen::Matrix4f ReadTransformationMatrixfromBlock(const std::string &matrixFileName);
    REG3D_API std::vector<Eigen::Affine3f> ReadTransformationMatrixColumnWise(const std::string &matrixFileName);
    REG3D_API Eigen::Matrix4f LoadProjectionmatrix(const std::string &matrixFileName);
    REG3D_API CloudWithoutType CopyPointWithoutNormal(CloudWithoutType &inputCloud, CloudWithoutType& ModifiedCloud);
    REG3D_API CloudWithoutType CopytNormalWithoutPoint(CloudWithoutType &inputCloud, CloudWithoutType& ModifiedCloud);

    REG3D_API float ComputeOrientedBoundingBoxOfCloud(CloudWithNormalPtr cloud, Eigen::Vector3f &min_pt, Eigen::Vector3f &max_pt);
    REG3D_API std::pair<double, double> GetTransformationError(Eigen::Matrix4f &GT, Eigen::Matrix4f &CT);
    REG3D_API std::vector<Eigen::Matrix4f> ReadGroundTruthTransFormationFromCSVFile( const std::string &FileName);
    REG3D_API CloudWithoutType ApplyTransformationToPointCloud(CloudWithoutType& inputCloud, double RotAngleinDeg,  Eigen::Vector3f rotAxis = Eigen::Vector3f::UnitZ(),
        Eigen::Vector3f translation = Eigen::Vector3f::UnitZ());
    REG3D_API Eigen::Matrix4f CreateMatrixFromFixedAngleAndTranslation(const double RotAngleinDeg, Eigen::Vector3f rotAxis = Eigen::Vector3f::UnitZ(),
        Eigen::Vector3f translation = Eigen::Vector3f::UnitZ());
    REG3D_API Eigen::MatrixXf RowShiftMatrix(const Eigen::MatrixXf & in, int down);
    REG3D_API  Eigen::MatrixXf CreateMatrixFromStlVector( std::vector<float> Data, const int row, const int col);
    REG3D_API  std::vector<float>CreateStlVectorFromMatirx( Eigen::MatrixXf in);
    REG3D_API std::vector<float> ReScaleImageBilinear(const std::vector<float> & ImgData, int row_prev, int col_prev, int row_new, int col_new);
    REG3D_API std::vector<float> testBilinearInterpolation(std::vector<float>ImgData, int row_prev, int col_prev, int row_new, int col_new);
}