#include "pch.h"
#include"ErrorMetric.h"
//#include<pcl/kdtree/kdtree_flann.h>
#include"Transformation_tool.h"
#include"pct_io.h"
#include<fstream>
#include <Eigen/SVD>
namespace
{
    void SortData(std::vector<double> &RMSEs)
    {
        typedef double dbl;
        std::sort(RMSEs.begin(), RMSEs.end(),
            [&](const dbl& v1, const dbl& v2)
        {
            return v1 < v2;
        });
    }
}


 float metric::ComputeRMSE(Eigen::MatrixXf cloudA, Eigen::MatrixXf cloudB, double rmse_unit)
{
     int nCount = 0;
     float dError = 0.0f;
     if (cloudA.cols() != cloudB.cols())
     {
         return -1.0;
     }
     for (int i = 0; i < cloudA.cols(); i++)
     {
         dError += (cloudA.col(i) - cloudB.col(i)).squaredNorm();
         nCount++;
     }
     dError = sqrt(dError) / (float)nCount;
     dError = dError / rmse_unit;
     return dError;
}
 float metric::ComputeRMSE(CloudWithoutType cloudA, CloudWithoutType cloudB, double rmse_unit)
 {
     int nCount = 0;
     float dError = 0.0f;
     CloudPtr pTarget(new pcl::PointCloud <PointType>);
     CloudPtr pSource(new pcl::PointCloud <PointType>);
     pcl::fromPCLPointCloud2(*cloudA, *pSource);
     pcl::fromPCLPointCloud2(*cloudB, *pTarget);
     if (pSource->points.size() != pTarget->points.size())
     {
         return -1.0;
     }
     for (int i = 0; i < pSource->points.size(); i++)
     {
         dError += (pSource->points[i].getVector3fMap() - pTarget->points[i].getVector3fMap()).squaredNorm();
         nCount++;
     }
     Eigen::Vector3f min_pt, max_pt;
     CloudWithNormalPtr GroundTruthcloud(new pcl::PointCloud<PointNormalType>);
     pcl::fromPCLPointCloud2(*cloudA, *GroundTruthcloud);
     
     
     if (rmse_unit == 1.0)
     {
         dError = sqrt(dError) / (float)nCount;
         float diaglength = EstimatePointAvgDistance(cloudB);// tool::ComputeOrientedBoundingBoxOfCloud(GroundTruthcloud, min_pt, max_pt);
         dError = dError / diaglength;
     }
     else
     {
         dError = (dError) / (float)nCount;
         dError = dError / (static_cast<float>(rmse_unit) * static_cast<float>(rmse_unit));
         dError = sqrt(dError);
     }

     return dError;
 }

 // Function to compute average point distance
 double metric::EstimatePointAvgDistance(const CloudWithoutType &pCloud)
 {
     CloudPtr pTarget(new pcl::PointCloud <PointType>);
     pcl::fromPCLPointCloud2(*pCloud, *pTarget);
     double avg = 0.0;
     int K = 2;
    std:: vector<int> SearchResults(K);
    std:: vector<float> SearchDistances(K);
     pcl::KdTreeFLANN<PointType> KdTree;
     KdTree.setInputCloud(pTarget);

     for (int i = 0; i < pTarget->size(); i++)
     {
         KdTree.nearestKSearch(*pTarget, i, K, SearchResults, SearchDistances);
         avg += SearchDistances[1];
     }

     avg = sqrt(avg / (double)pTarget->size());

     return avg;
 }

 void metric:: ComputeAlphaRecallForMethod(std::string &inputRMSEFiles, std::string &outPutFileName, std::string &OutputFolderName)
 {
     std::string full_file_Name_Input = OutputFolderName + inputRMSEFiles;
     std::string full_file_Name_comparator = OutputFolderName + "comparator.txt";
     int noOfFiles = pct::readLines(full_file_Name_Input);
     FILE *pFile;
     pFile = fopen(full_file_Name_Input.c_str(), "rb");
     if (NULL == pFile)
     {
         std::cout << "Failed to read data file " << std::endl;
         exit(-1);
     }

     // read all RMSE values for a particular method and noise level
     char szParam1[50];
     std::vector<double>rmseValue;
     rmseValue.reserve(noOfFiles);
     for (int i = 0; i < noOfFiles; i++)
     {
         fscanf(pFile, "%s\n", szParam1);
         double value = static_cast<double>(atof(szParam1));
         rmseValue.push_back(value);
     }
     fclose(pFile);
     // Build a table : initializer table
     std::vector<double>comparator;
     comparator.reserve(1000);
     std::fstream outfile(full_file_Name_comparator, std::fstream::out);
     for (int i = 0; i < 1000; i++)
     {
         double val = static_cast<double>(i) *  std::pow<double>(10, -3) / static_cast<double>(1000 );
         comparator.push_back(val);
         outfile << val << std::endl;
     }

     // collect the alpha recall value using the rmse and the comparator
     std::vector<double>recallValue;
     recallValue.reserve(1000);
     for (int i = 0; i < comparator.size(); i++)
     {
         int counterLevel = 0;
         for (int j = 0; j < rmseValue.size(); j++)
         {
             if(rmseValue[j] <= comparator[i])
             {
                 counterLevel = counterLevel + 1;
             }
         }
         recallValue.push_back(static_cast<double>(counterLevel) / rmseValue.size());
     }

     // write o/p alpharecall value
     std::string full_file_Name_Output = OutputFolderName + outPutFileName;
     std::fstream txtfile(full_file_Name_Output, std::fstream::out);

     for (int i = 0; i < recallValue.size(); i++)
     {
         txtfile << recallValue.at(i) << std::endl;
     }
    
 }
 void metric::ComputeAlphaRecallGraph(std::string &inputRMSEFiles, double spacing, std::vector<double>&recall_value,
     std::vector<double>&comparator)
 {
     comparator.clear();
     comparator.reserve(1000);
     int num_Lines = pct::readLines(inputRMSEFiles);
     FILE *pFile;

     pFile = fopen(inputRMSEFiles.c_str(), "rb");

     if (NULL == pFile)
     {
         std::cout << "Failed to read data file " << std::endl;
         exit(-1);
     }

     // read all RMSE values for a particular type
     char szParam1[50];
     std::vector<double>rmseValue;
     rmseValue.reserve(num_Lines);
     for (int i = 0; i < num_Lines; i++)
     {
         fscanf(pFile, "%s\n", szParam1);
         double value = static_cast<double>(atof(szParam1));
         rmseValue.push_back(value);
     }
     fclose(pFile);

     for (int i = 0; i < 1000; i++)
     {
         double val = static_cast<double>(i) *  spacing / static_cast<double>(1000);
         comparator.push_back(val);

     }

     // collect the alpha recall value using the error value and the comparator
    
     recall_value.reserve(1000);
     for (int i = 0; i < comparator.size(); i++)
     {
         int counterLevel = 0;
         for (int j = 0; j < rmseValue.size(); j++)
         {
             if (rmseValue[j] <= comparator[i])
             {
                 counterLevel = counterLevel + 1;
             }
         }
         recall_value.push_back(static_cast<double>(counterLevel) / rmseValue.size());
     }
 }
 Eigen::Vector3f metric::ComputeAverageNormalFromPointCloud(const CloudWithoutType &pCloud)
 {
     Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
     std::string normal = "normal_x";
     const std::vector<pcl::PCLPointField>cloud_fields = pCloud->fields;
     auto itr = find_if(begin(cloud_fields), end(cloud_fields), [=](pcl::PCLPointField const& f) {
         return (f.name == normal);
     });
     bool if_found = (itr != end(cloud_fields));
     if (!if_found)
     {
         error_log("No normals found for the input cloud: the process aborted");
         abort();
     }
     else
     {
      
         CloudWithNormalPtr pNormalCloud(new pcl::PointCloud <PointNormalType>);
         pcl::fromPCLPointCloud2(*pCloud, *pNormalCloud);
         size_t nNumPoints = pNormalCloud->points.size();
         for (int i = 0; i < nNumPoints; i++)
         {
             avg_normal += pNormalCloud->points[i].getNormalVector3fMap();
         }
         avg_normal /= nNumPoints;
     }
     return avg_normal;
 }
 double metric::CalcRMSE(std::vector<Eigen::Vector3f> & points1, std::vector<Eigen::Vector3f> &points2)
 {
     if (points1.size() != points2.size())
     {
         getchar();
         exit(-1);
     }
     double rmse = 0;
     for (int po = 0; po < points1.size(); po++)
     {
         rmse += (points1[po] - points2[po]).norm();
     }
     rmse /= (double)points1.size();
     return rmse;
 }
 double metric:: Orientate(std::vector<Eigen::Vector3f> & points1, std::vector<Eigen::Vector3f> &points2, Eigen::Matrix4f & transformation)
 {
     if (points1.size() < 3)
         return -1.0;

     transformation.setIdentity();
     std::vector<Eigen::Vector3f> pts1Out(points1);
     std::vector<Eigen::Vector3f> pts2Out(points2);
     Eigen::Vector3f baryCenter1 = Eigen::Vector3f::Zero();
     Eigen::Vector3f baryCenter2 = Eigen::Vector3f::Zero();
  
     for each(Eigen::Vector3f vec1 in points1)
     {
         baryCenter1 += vec1;
     }
     baryCenter1 /= (float)points1.size();

     for each(Eigen::Vector3f vec1 in points2)
     {
         baryCenter2 += vec1;
     }
     baryCenter2 /= (float)points2.size();
     Eigen::Vector3f InbaryCenter1 = Eigen::Vector3f::Zero();
     Eigen::Vector3f InbaryCenter2 = Eigen::Vector3f::Zero();
     InbaryCenter1 = -baryCenter1;
     InbaryCenter1 = -baryCenter2;
      // translate each points of the vector by respective inbarycenter
#pragma omp parallel for
     for (int po = 0; po < points1.size(); po++)
     {
         points1[po] += InbaryCenter1;
     }
#pragma omp parallel for
     for (int po = 0; po < points1.size(); po++)
     {
         points2[po] += InbaryCenter2;
     }
     float scale = 1.0;
     // if scale  !=1 is used
     double sum1 = 0.0;
     double sum2 = 0.0;
#pragma omp parallel for
     for (int i = 0; i < points1.size(); i++)
     {
         sum1 += points1[i].norm();
         sum2 += points2[i].norm();
     }
     if (sum2 != 0.0)
     {
         scale = sum1 / sum2;
     }
      // calculate covariance matrix
     Eigen::MatrixXf S(points1.size(), 3), D(points2.size(), 3);
     
     for (int i = 0; i < points1.size(); i++)
     {
         S.row(i) = points1[i] - baryCenter1;
         D.row(i) = points2[i] - baryCenter2;
     }
     Eigen::MatrixXf Dt = D.transpose();
     Eigen::Matrix3f H = Dt*S;
     Eigen::Matrix3f W, U, V;
     Eigen::JacobiSVD<Eigen::MatrixXf>svd;
     Eigen::MatrixXf H_(3, 3);
     H_ = H;
     svd.compute(H_, Eigen::ComputeFullU | Eigen::ComputeFullV);
     if (!svd.computeU() || !svd.computeV())
     {
         std::cerr << "decomposition error" << endl;
          
     }
     Eigen::Matrix3f Vt = svd.matrixU().transpose();
     Eigen::Matrix3f R = svd.matrixV()*Vt;
     Eigen::Vector3f t = baryCenter2 - R*baryCenter1;
     Eigen::Affine3f rotation(R);
     if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0)
     {
         Eigen::Vector3f S = Eigen::Vector3f::Ones(); S(2) = -1.0;
         rotation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
     }
     else
     {
         rotation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
     }
     // Translate to barycenter
     Eigen::Affine3f mat(transformation);
     mat.translate(baryCenter1);

     // scale the rotation part
     R = scale * R;
     mat.rotate(R);
     // finally translate by inbarycenter2
     mat.translate(InbaryCenter2);
     std::vector<Eigen::Vector3f> tempointset2(points2), tempointset1(points1);
#pragma omp parallel for 
     for (int i = 0; i < points2.size(); i++)
     {
         points1[i] += baryCenter1;
         points2[i] += baryCenter2;
         tempointset2[i] = mat * points2[i];
     }
     double rmse =  metric::CalcRMSE(points1, tempointset2);
     transformation = mat.matrix();
     return rmse;
 }
 void metric::WriteRMSE(std::string &fileName, std::vector<double>RmseValues)
 {
     iospace::CreateFullPath(iospace:: GetPathDir(fileName)); // currently added to check the existence of new directory
     FILE *f_rmse = fopen(fileName.c_str(), "ab");

     if (NULL != f_rmse)
     {
         for each(double value in RmseValues)
         {
             fprintf(f_rmse, "%.9f\r\n", value);
         }
         fclose(f_rmse);
         f_rmse = NULL;
     }
     else
     {
         error_log("Unable to read rmse values from file\n");
         exit(-1);
     }
 }

 std::vector<double> metric::ReadRMSE(std::string FullFileName)
 {
     std::string line;
     std::ifstream myfile(FullFileName.c_str());
     std::vector<double>RMSEs;
    // RMSEs.reserve(10000);
     unsigned numberOfLines = 0;
     if (myfile.is_open())
     {
         while (std::getline(myfile, line))
         {
             double rmse = atof(line.c_str());
             if (rmse != -1.0)  // reads only value which is lower than threshold for evaluation puropose
             {
                 RMSEs.push_back(rmse);
                 numberOfLines++;
             }
         }

     }
     myfile.close();
     return RMSEs;
 }
 void metric ::ReadTimeProcessingFile(const std::string &fileName, std::vector<double>&SamplingTime, std::vector<double> &FrameComputationTime,
     std::vector<double>& RegistrationTime,  bool _baseline)
 {

     std::string line;
     if (_baseline == false)
     {
         std::ifstream myfile(fileName.c_str());
         if (myfile.is_open())
         {
             while (std::getline(myfile, line))
             {
                 if (line.c_str()[0] == 'n')
                 {
                     int pos = line.find(":");
                     if (pos != std::string::npos)
                     {
                         std::string value = line.substr(pos + 1, line.length() - 1);
                         SamplingTime.push_back(atof(value.c_str()));
                     }
                 }
                 else if (line.c_str()[0] == 'p')
                 {
                     int pos = line.find(":");
                     if (pos != std::string::npos)
                     {
                         std::string value = line.substr(pos + 1, line.length() - 1);
                         FrameComputationTime.push_back(atof(value.c_str()));
                     }
                 }
                 else  if (line.c_str()[0] == 'i')
                 {
                     int pos = line.find(":");
                     if (pos != std::string::npos)
                     {
                         std::string value = line.substr(pos + 1, line.length() - 1);
                         RegistrationTime.push_back(atof(value.c_str()));
                     }
                 }
                 else
                     continue;
             }
         }
         myfile.close();
         return;
     }
     else
     {
         unsigned noOfLines = pct::readLines(fileName);
         int fileRead = 0;
         FILE *pFile;
         pFile = fopen(fileName.c_str(), "rb");
         if (NULL == pFile)
         {
             std::cout << "Failed to read data file " << std::endl;
             throw std::runtime_error(" unable to open file\n");
         }
         char szParam1[50], szParam2[50], szParam3[50];
         fscanf(pFile, "%s %s %s", szParam1, szParam2, szParam3);
         for (int i = 1; i < noOfLines; i++)
         {
             fscanf(pFile, "%s %s %s\n", szParam1, szParam2, szParam3);
             double n_time = atof(szParam1);
             double s_time = atof(szParam2);
             double r_time = atof(szParam3);
             SamplingTime.push_back(n_time);
             FrameComputationTime.push_back(s_time);
             RegistrationTime.push_back(r_time);
         }
         fclose(pFile);
         return;
     }

  }
 void metric::WriteTimingsOfMethodInASingleFile(std::string &Path, std::string &FolderList, std::string &OutputTimingFile,
     std::string &AlgorithmName, bool variable)
 {
     std::string PathAndFile = Path + "/" + FolderList;
     std::vector<std::string> folders;
     std::string line;
     std::ifstream myfile(PathAndFile.c_str());
     folders.reserve(100);
     unsigned numberOfLines = 0;
    // static bool deleteFile = false;
     if (myfile.is_open())
     {
         while (std::getline(myfile, line))
         {
             folders.push_back(line);
             numberOfLines++;
         }
         std::cout << "Number of DataFolder: " << numberOfLines << std::endl;
     }
     myfile.close();
     if (iospace::ExistsFile(OutputTimingFile) /*&& deleteFile == false*/)
     {
         remove(OutputTimingFile.c_str());
        // deleteFile = true;
     }
     std::vector<double> regis_time, sampling_time, smoothing_time, regis_time_all;
     for (int itrFolder = 0; itrFolder < folders.size(); itrFolder++)
     {
         std::cout << "Processing Folder:" << folders[itrFolder] << std::endl;
         std::string FileName = Path + "/" + folders[itrFolder] + "_processing_time_Baseline_" + AlgorithmName + ".txt";
         ReadTimeProcessingFile(FileName, sampling_time, smoothing_time, regis_time, variable);
         std::string rmse_file  = Path + "/" + folders[itrFolder] + "_rmse_Baseline_" + AlgorithmName + ".txt";
         std::vector<double>rmse_values = ReadRMSE(rmse_file);
         std::vector<double>refined_regis_time;
         for (int itr = 0; itr < rmse_values.size(); itr++)
         {
             if (rmse_values[itr] < 5.0)
             {
                 refined_regis_time.push_back(regis_time[itr]);
             }
         }
         std::move(refined_regis_time.begin(), refined_regis_time.end(), std::back_inserter(regis_time_all));
         sampling_time.clear();
         smoothing_time.clear();
         regis_time.clear();

     }
     SortData(regis_time_all);
     std::string outputfilename = Path + "/" + AlgorithmName + "_" + OutputTimingFile + ".txt";
     FILE *f_rmse = fopen(outputfilename.c_str(), "ab");

     if (NULL != f_rmse)
     {
         for each(double value in regis_time_all)
         {
             fprintf(f_rmse, "%.5f\n", value);
         }
         fclose(f_rmse);
         f_rmse = NULL;
     }
     else
     {
         throw std::runtime_error(" Unable to accumulate timings\n");
     }
 }
 std::vector<double>metric::ComputeFeatureAngleForACloud(CloudWithoutType &InputCloud, int NumNNQuery)
 {
     CloudWithNormalPtr TempCloud(new pcl::PointCloud<PointNormalType>);
     pcl::fromPCLPointCloud2(*InputCloud, *TempCloud);
     pcl::KdTreeFLANN<PointNormalType> *tree = nullptr;
     tree = new(pcl::KdTreeFLANN<PointNormalType>);
     tree->setInputCloud(TempCloud);
     std::vector<int> pointIdx;
     std::vector<float> pointSquaredDistance;
     std::vector<double>features;
     size_t Size = TempCloud->points.size();
     features.resize(Size);
     for (size_t index = 0; index < Size; index++)
     {
         double norm_angle = 0.0;
         tree->nearestKSearch(TempCloud->points[index], NumNNQuery, pointIdx, pointSquaredDistance);
         for (size_t idx = 1; idx < pointIdx.size(); idx++)
         {
             Eigen::Vector3f po = TempCloud->points[index].getNormalVector3fMap();
             Eigen::Vector3f pn = TempCloud->points[pointIdx[idx]].getNormalVector3fMap();
             double angle = acos(po.dot(pn));
             norm_angle += angle;   // angle in radian
         }
         features[index] = norm_angle/Size;
     }
     return features;
 }

// double metric::ComputeFeatureAngleForAPoint(Eigen::VectorXf QueryPoint, CloudWithNormalPtr &InputCloud, pcl::KdTreeFLANN<PointNormalType> *ktree, int NumNNQuery)
// {
//     /*CloudWithNormalPtr TempCloud(new pcl::PointCloud<PointNormalType>);
//     pcl::fromPCLPointCloud2(*InputCloud, *TempCloud);*/
//   /*  pcl::KdTreeFLANN<PointNormalType> *tree = nullptr;
//     tree = new(pcl::KdTreeFLANN<PointNormalType>);
//     tree->setInputCloud(TempCloud);*/
//     std::vector<int> pointIdx;
//     std::vector<float> pointSquaredDistance;
//     double features;
//     size_t Size = InputCloud->points.size();
//
//     PointNormalType qPoint;
//     qPoint.getVector3fMap() = QueryPoint.head<3>();
//     qPoint.getNormalVector3fMap() = QueryPoint.tail<3>();
//     double norm_angle = 0.0;
//     ktree->nearestKSearch(qPoint, NumNNQuery, pointIdx, pointSquaredDistance);
//     if (pointIdx.size() > 1)
//     {
//#pragma omp parallel for
//         for (int idx = 1; idx < pointIdx.size(); idx++)
//         {
//             Eigen::Vector3f po = qPoint.getNormalVector3fMap();
//             Eigen::Vector3f pn = InputCloud->points[pointIdx[idx]].getNormalVector3fMap();
//             double angle = acos(po.dot(pn));
//             norm_angle += angle;   // angle in radian
//         }
//         features = norm_angle / Size;
//     }
//     else
//     {
//         features = 0.0;
//     }
//
//     return features;
// }
 bool metric::FilterPointsOnCurvatureValues(double features_source, double features_target, float eig_value_source, float eig_value_target)
 {
     bool fitering_criteria = false;
     double fad = (features_source - features_target);
     double evd = eig_value_source - eig_value_target;
     if ((abs(fad) <= 1e-4) == true && (abs(evd) <= 0.1) == true)
     {
         fitering_criteria = true;
         return fitering_criteria;
     }
     return fitering_criteria;
 }