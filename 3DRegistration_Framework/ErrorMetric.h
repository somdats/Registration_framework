#pragma once
#include"Datatypes.h"
#include"Common.h"
#include"Config.h"
#include "ply_file_io.h"
namespace metric
{
    REG3D_API float ComputeRMSE(Eigen::MatrixXf cloudA, Eigen::MatrixXf cloudB, double rmse_unit);
    REG3D_API float ComputeRMSE(CloudWithoutType cloudA, CloudWithoutType cloudB, double rmse_unit  = 1.0);
    REG3D_API double EstimatePointAvgDistance(const CloudWithoutType &pCloud);
    REG3D_API void ComputeAlphaRecallForMethod(std::string &inputRMSEFiles, std:: string &outPutFileName, std::string &OutputFolderName);
    REG3D_API void ComputeAlphaRecallGraph(std::string &inputRMSEFiles, double spacing, std::vector<double>&recall_value,
        std::vector<double>&comparator);
    REG3D_API Eigen::Vector3f ComputeAverageNormalFromPointCloud(const CloudWithoutType &pCloud);
    REG3D_API double CalcRMSE(std::vector<Eigen::Vector3f> & points1, std::vector<Eigen::Vector3f> &points2);
    REG3D_API double Orientate(std::vector<Eigen::Vector3f> & points1, std::vector<Eigen::Vector3f> &points2, Eigen::Matrix4f & transformation);
    REG3D_API void WriteRMSE(std::string &fileName, std::vector<double>RmseValues);
    REG3D_API std::vector<double> ReadRMSE(std::string FullFileName);
    REG3D_API void ReadTimeProcessingFile(const std::string &fileName, std:: vector<double>&SamplingTime, std::vector<double> &FrameComputationTime,
        std:: vector<double> &RegistrationTime, bool _baseline = false);
    REG3D_API void WriteTimingsOfMethodInASingleFile( std:: string &Path, std::string &FolderList, std::string &OutputTimingFile, std::string &AlgorithmName, bool variable = false);
    REG3D_API std::vector<double>ComputeFeatureAngleForACloud(CloudWithoutType &InputCloud, int NumNNQuery =  10);
    REG3D_API bool FilterPointsOnCurvatureValues(double features_source, double features_target, float eig_value_source, float eig_value_target);
  //  REG3D_API double ComputeFeatureAngleForAPoint( Eigen::VectorXf QueryPoint, CloudWithNormalPtr &InputCloud, pcl::KdTreeFLANN<PointNormalType>*ktree, int NumNNQuery = 10);
   
}