#include"pch.h"
//#include<pcl/common/common.h>
#include"Transformation_tool.h"
#include"pct_io.h"
#include <fstream>

// transform a point cloud with transformationMatrix
 // i/p: inputCloud
 // i/p : Eigen::Matrix4f transformationMatrix
//o/p : outputcloud(pcl::pointCloud2)
namespace
{
    float bilinear(
        const float &tx,
        const float  &ty,
        const float &c00,
        const float &c10,
        const float &c01,
        const float &c11)
    {
#if 1 
        float  a = c00 * (1 - tx) + c10 * tx;
        float  b = c01 * (1 - tx) + c11 * tx;
        return a * ((1) - ty) + b * ty;
#else 
        return (1 - tx) * (1 - ty) * c00 +
            tx * (1 - ty) * c10 +
            (1 - tx) * ty * c01 +
            tx * ty * c11;
#endif 
    }
    float ComputeNewPixelValue(float A, float B, float C, float D, float x_diff, float y_diff)
    {
        float gray;
        if (A != -INFINITY && B != -INFINITY && C != -INFINITY && D != -INFINITY)
        {
            gray = A*(1 - x_diff)*(1 - y_diff) + B*(x_diff)*(1 - y_diff) +
                C*(y_diff)*(1 - x_diff) + D*(x_diff*y_diff);
            return gray;
        }
        else  if ((A == -INFINITY) && (B == -INFINITY) && (C == -INFINITY) && (D != -INFINITY))
        {
            gray =  D*(x_diff*y_diff);
            return gray;
        }
        else  if ((A == -INFINITY) && (B == -INFINITY) && (C != -INFINITY) && (D == -INFINITY))
        {
            gray = C*(y_diff)*(1 - x_diff);
            return gray;
        }
        else  if ((A == -INFINITY) && (B == -INFINITY) && (C != -INFINITY) && (D != -INFINITY))
        {
            gray = C*(y_diff)*(1 - x_diff) + D*(x_diff*y_diff);
            return gray;
        }
        else  if ((A == -INFINITY) && (B != -INFINITY) && (C == -INFINITY) && (D == -INFINITY))
        {
            gray = B*(x_diff)*(1 - y_diff);

            return gray;
        }
        else  if ((A == -INFINITY) && (B != -INFINITY) && (C == -INFINITY) && (D != -INFINITY))
        {
            gray =  B*(x_diff)*(1 - y_diff) + D*(x_diff*y_diff);
            return gray;
        }
        else  if ((A == -INFINITY) && (B != -INFINITY) && (C != -INFINITY) && (D == -INFINITY))
        {
            gray = B*(x_diff)*(1 - y_diff) +  C*(y_diff);
            return gray;
        }
        else  if ((A == -INFINITY) && (B != -INFINITY) && (C != -INFINITY) && (D != -INFINITY))
        {
            gray =  B*(x_diff)*(1 - y_diff) + C*(y_diff)*(1 - x_diff) + D*(x_diff*y_diff);
            return gray;
        }
        else  if ((A != -INFINITY) && (B == -INFINITY) && (C == -INFINITY) && (D != -INFINITY))
        {
            gray = A*(1 - x_diff)*(1 - y_diff);
            return gray;
        }
        else  if ((A != -INFINITY) && (B == -INFINITY) && (C == -INFINITY) && (D != -INFINITY))
        {
            gray = A*(1 - x_diff)*(1 - y_diff) + D*(x_diff*y_diff);
            return gray;
        }
        else  if ((A != -INFINITY) && (B == -INFINITY) && (C != -INFINITY) && (D == -INFINITY))
        {
            gray = A*(1 - x_diff)*(1 - y_diff) + C*(y_diff)*(1 - x_diff);
            return gray;
        }
        else  if ((A != -INFINITY) && (B == -INFINITY) && (C != -INFINITY) && (D != -INFINITY))
        {
            gray = A*(1 - x_diff)*(1 - y_diff) + C*(y_diff)*(1 - x_diff) + D*(x_diff*y_diff);
            return gray;
        }
        else  if ((A != -INFINITY) && (B != -INFINITY) && (C == -INFINITY) && (D == -INFINITY))
        {
            gray = A*(1 - x_diff)*(1 - y_diff) + B*(x_diff)*(1 - y_diff);
            return gray;
        }
      
        else  if ((A != -INFINITY) && (B != -INFINITY) && (C == -INFINITY) && (D != -INFINITY))
        {
            gray = A*(1 - x_diff)*(1 - y_diff) + B*(x_diff)*(1 - y_diff) + D*(x_diff*y_diff);
            return gray;
        }
        else  if ((A != -INFINITY) && (B != -INFINITY) && (C != -INFINITY) && (D == -INFINITY))
        {
            gray = A*(1 - x_diff)*(1 - y_diff) + B*(x_diff)*(1 - y_diff) +
                C*(y_diff)*(1 - x_diff);
            return gray;
        }
        else  if ((A == -INFINITY) && (B == -INFINITY) && (C == -INFINITY) && (D == -INFINITY))
        {
            gray = -INFINITY;
            return gray;
        }

    }
}
CloudWithoutType tool ::TransFormationOfCloud(CloudWithoutType inputCloud, Eigen::Matrix4f transformationMatrix)
{
    CloudWithoutType transformedCloud(new pcl::PCLPointCloud2);
    std::vector<pcl::PCLPointField> fields = inputCloud->fields;
    std::string normal = "normal_x";
    auto it = find_if(begin(fields), end(fields), [=](pcl::PCLPointField const& f) {
        return (f.name == normal);
    });
    bool found = (it != end(fields));
    if (found)
    {
        CloudWithNormalPtr cloudWithNormal(new pcl::PointCloud <PointNormalType>);
        CloudWithNormalPtr transformedNormalCloud(new pcl::PointCloud <PointNormalType>);
        pcl::fromPCLPointCloud2(*inputCloud, *cloudWithNormal);
        pcl::transformPointCloudWithNormals(*cloudWithNormal, *transformedNormalCloud, transformationMatrix);
        pcl::toPCLPointCloud2(*transformedNormalCloud, *transformedCloud);
    }
    else
    {
        CloudPtr cloud(new pcl::PointCloud <PointType>);
        CloudPtr outputCloud(new pcl::PointCloud <PointType>);
        pcl::fromPCLPointCloud2(*inputCloud, *cloud);
        pcl::transformPointCloud(*cloud, *outputCloud, transformationMatrix);
        pcl::toPCLPointCloud2(*outputCloud, *transformedCloud);
    }
    return transformedCloud;
}

// write transformationmatrices in a file
// i/p : array of Eigen::Matrix4f transformationMatrix
//o/p : text file of transformationMatrix
 void tool::writeTransformationMatrix(const std::string &matrixFileName, std::vector<Eigen::Matrix4f> transformationMatrix)
 {
     FILE *nFile = fopen(matrixFileName.c_str(), "ab");  //wb
     if (NULL == nFile)
     {
         exit(1);
     }
     int size = transformationMatrix.size();
     for each (Eigen::Matrix4f matrix in transformationMatrix)
     {
        // fprintf(nFile, " %f %f %f %f %f %f %f %f %f %f %f %f \r\n", 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
         fprintf(nFile, " %f %f %f %f %f %f %f %f %f %f %f %f \r\n", matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(1, 0),
             matrix(1, 1), matrix(1, 2), matrix(2, 0),
             matrix(2, 1), matrix(2, 2), matrix(0, 3), matrix(1, 3), matrix(2, 3));
     }
     fclose(nFile);
 }
 void tool::writeMatrix(const std::string &matrixFileName, std::vector<Eigen::Matrix4f> transformationMatrix)
 {
     FILE *nFile = fopen(matrixFileName.c_str(), "wb");  //wb
     if (NULL == nFile)
     {
         exit(1);
     }
     int size = transformationMatrix.size();
     for each (Eigen::Matrix4f matrix in transformationMatrix)
     {
         // fprintf(nFile, " %f %f %f %f %f %f %f %f %f %f %f %f \r\n", 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
         fprintf(nFile, " %f %f %f %f %f %f %f %f %f %f %f %f \r\n", matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(1, 0),
             matrix(1, 1), matrix(1, 2), matrix(2, 0),
             matrix(2, 1), matrix(2, 2), matrix(0, 3), matrix(1, 3), matrix(2, 3));
     }
     fclose(nFile);
     nFile = NULL;
 }
 // read transformationmatrices from a file
 // i/p : input text file
 //o/p :  transformationMatrix in Affine3f form
 std::vector<Eigen::Affine3f>  tool::ReadTransformationMatrix(const std::string &matrixFileName)
 {
     std::vector<Eigen::Affine3f> inputTransformationMatrices;
     inputTransformationMatrices.clear();
     // open file for reading
     FILE *pFile = fopen(matrixFileName.c_str(), "rb");
     if (NULL == pFile)
     {
         std::cout << "Failed to read data file " <<  std:: endl;
         exit(0);
     }
     else
     {
         unsigned numLines = pct::readLines(matrixFileName);
         inputTransformationMatrices.resize(numLines);
         Eigen::Matrix4f tempmatrix = Eigen::Matrix4f::Identity(4, 4);
         char szParam1[50], szParam2[50], szParam3[50], szParam4[50], szParam5[50], szParam6[50],
             szParam7[50], szParam8[50], szParam9[50], szParam10[50], szParam11[50], szParam12[50];
         for (int i = 0; i < numLines; i++)
         {
             fscanf(pFile, " %s %s %s %s %s %s %s %s %s %s %s %s \r\n", szParam1, szParam2, szParam3,
                 szParam4, szParam5, szParam6, szParam7, szParam8, szParam9, szParam10, szParam11, szParam12);
             Eigen::Vector4f r1(atof(szParam1), atof(szParam2), atof(szParam3), atof(szParam10));
             Eigen::Vector4f r2(atof(szParam4), atof(szParam5), atof(szParam6), atof(szParam11));
             Eigen::Vector4f r3(atof(szParam7), atof(szParam8), atof(szParam9), atof(szParam12));
             Eigen::Vector4f r4(0.0f, 0.0f, 0.0f, 1.0f);
             tempmatrix.row(0) = r1;
             tempmatrix.row(1) = r2;
             tempmatrix.row(2) = r3;
             tempmatrix.row(3) = r4;
             inputTransformationMatrices[i] = tempmatrix;
         }

     }
     fclose(pFile);
     return inputTransformationMatrices;
 }
 // read transformationmatrices from a file
 // i/p : input text file
 //o/p :  transformationMatrix in Affine3f form (columnwise read)
 std::vector<Eigen::Affine3f>tool::ReadTransformationMatrixColumnWise(const std::string &matrixFileName)
 {
     std::vector<Eigen::Affine3f> inputTransformationMatrices;
     inputTransformationMatrices.clear();
     // open file for reading
     FILE *pFile = fopen(matrixFileName.c_str(), "rb");
     if (NULL == pFile)
     {
         std::cout << "Failed to read data file " << std::endl;
         exit(0);
     }

     else
     {
         unsigned numLines = pct::readLines(matrixFileName);
         inputTransformationMatrices.resize(numLines);
         Eigen::Matrix4f tempmatrix = Eigen::Matrix4f::Identity(4, 4);
         char szParam1[50], szParam2[50], szParam3[50], szParam4[50], szParam5[50], szParam6[50],
             szParam7[50], szParam8[50], szParam9[50], szParam10[50], szParam11[50], szParam12[50];
         for (int i = 0; i < numLines; i++)
         {
             fscanf(pFile, " %s %s %s %s %s %s %s %s %s %s %s %s \r\n", szParam1, szParam2, szParam3,
                 szParam4, szParam5, szParam6, szParam7, szParam8, szParam9, szParam10, szParam11, szParam12);
             Eigen::Vector4f r1(atof(szParam1), atof(szParam2), atof(szParam3), 0.0f);
             Eigen::Vector4f r2(atof(szParam4), atof(szParam5), atof(szParam6), 0.0f);
             Eigen::Vector4f r3(atof(szParam7), atof(szParam8), atof(szParam9), 0.0f);
             Eigen::Vector4f r4(atof(szParam10), atof(szParam11), atof(szParam12), 1.0f);
             tempmatrix.col(0) = r1;
             tempmatrix.col(1) = r2;
             tempmatrix.col(2) = r3;
             tempmatrix.col(3) = r4;
             inputTransformationMatrices[i] = tempmatrix;
         }

     }
     fclose(pFile);
     return inputTransformationMatrices;
 }

 // read transformationmatrices from a file
 // i/p : input text file
 //o/p :  transformationMatrix in Matrix4d form (columnMajor)
 Eigen::Matrix4f tool::LoadProjectionmatrix(const std::string &matrixFileName)
 {
     FILE *pFile = fopen(matrixFileName.c_str(), "rb");
     char szParam1[50], szParam2[50], szParam3[50], szParam4[50], szParam5[50], szParam6[50],
         szParam7[50], szParam8[50], szParam9[50], szParam10[50], szParam11[50], szParam12[50], szParam13[20], szParam14[20],
         szParam15[20], szParam16[20];
     Eigen::Matrix4f temp;
     if (NULL == pFile)
     {
        std::cout << "Failed to read data file " << std::endl;
         exit;
     }
     fscanf(pFile, " %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s \r\n", szParam1, szParam2, szParam3,
         szParam4, szParam5, szParam6, szParam7, szParam8, szParam9, szParam10, szParam11, szParam12, szParam13, szParam14,
         szParam15, szParam16);
     Eigen::Vector4f r1(atof(szParam1), atof(szParam2), atof(szParam3), atof(szParam4));
     Eigen::Vector4f r2(atof(szParam5), atof(szParam6), atof(szParam7), atof(szParam8));
     Eigen::Vector4f r3(atof(szParam9), atof(szParam10), atof(szParam11), atof(szParam12));
     Eigen::Vector4f r4(atof(szParam13), atof(szParam14), atof(szParam15), atof(szParam16));
     temp.col(0) = r1;
     temp.col(1) = r2;
     temp.col(2) = r3;
     temp.col(3) = r4;
     fclose(pFile);
     return temp;
 }

 // Computes Cameramatrix(Intrinsic) from parameters
 // i/p : 3D Coordinates, pixelIndices, set of camera-parameter
 //o/p :  transformationMatrix in Matrix4f
 Eigen::Matrix4Xf tool:: GenerateCameraMatrix(CloudWithoutType &Coordinates3D, std::vector<UVData<float>> &pixelCoordinates, CameraParameter &CamParam)
 {
   
     float sensorwidth = 0, sensorheight = 0, sw = 0, sh = 0;
     Eigen::Vector4f centroid;
     std::vector<float>sWs;
     std::vector<float>sHs;
     Eigen::Matrix4Xf cameraMatrix = Eigen::Matrix4Xf::Zero(4, 4);
     sWs.reserve(pixelCoordinates.size());
     sHs.reserve(pixelCoordinates.size());
     CloudPtr cloudXYZ(new pcl::PointCloud<PointType>);
     pcl::fromPCLPointCloud2(*Coordinates3D, *cloudXYZ);
     if (pixelCoordinates.size() > 0 && cloudXYZ->points.size() > 0)
     {
         for (int i = 0; i < pixelCoordinates.size(); i++)
         {
             sw = (CamParam.ImageWidth / pixelCoordinates[i].u) *(CamParam.focallength *(cloudXYZ->points[i].x / cloudXYZ->points[i].z)
                 + CamParam.PrincipalOffsetX);
             sh = (CamParam.ImageHeight / pixelCoordinates[i].v) *(CamParam.focallength *(cloudXYZ->points[i].y / cloudXYZ->points[i].z)
                 + CamParam.PrincipalOffsetY);
             sensorwidth += sw;
             sensorheight += sh;
             sWs.push_back(sw);
             sHs.push_back(sh);
         }
         sensorwidth /= cloudXYZ->points.size();
         sensorheight /= cloudXYZ->points.size();
         float widthAspect = CamParam.ImageWidth / sensorwidth;
         float heightAspect = CamParam .ImageHeight/ sensorheight;
         float fpixX = widthAspect * CamParam.focallength;
         float fpixY = heightAspect * CamParam.focallength;
         float pOffsetX = CamParam.PrincipalOffsetX * widthAspect;
         float pOffsetY = CamParam.PrincipalOffsetY * heightAspect;
         cameraMatrix(0, 0) = fpixX;
         cameraMatrix(1, 1) = fpixY;
         cameraMatrix(0, 2) = pOffsetX;
         cameraMatrix(1, 2) = pOffsetY;
     }
     else

     {
         std::cout << "Input Cloud Data empty" << std::endl;
         abort();
     }
     return cameraMatrix;

 }

 // Create a matrix of Points from pcl::cloud
 //i/p pcl::pointcloud2
 //o/p Matrix of points

 Eigen::Matrix3Xf tool::createPointMatrixFromCloud(CloudWithoutType &cloudInput)
 {
     CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
     pcl::fromPCLPointCloud2(*cloudInput, *pTarget);
     Eigen::Matrix3Xf tempMatrix = Eigen::Matrix3Xf::Zero(3, pTarget->points.size());
     for (int i = 0; i < pTarget->points.size(); i++)
     {
         tempMatrix.col(i) = pTarget->points[i].getVector3fMap();
      
     }
     return tempMatrix;
 }

 // Create a matrix of Normals(if available) from pcl::cloud
 //i/p pcl::pointcloud2
 //o/p Matrix of Normals
 Eigen::Matrix3Xf tool::createNormalMatrixFromCloud(CloudWithoutType &cloudInput)
 {
     CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
     pcl::fromPCLPointCloud2(*cloudInput, *pTarget);
     Eigen::Matrix3Xf tempMatrix = Eigen::Matrix3Xf::Zero(3, pTarget->points.size());
     for (int i = 0; i < pTarget->points.size(); i++)
     {
         tempMatrix.col(i) = pTarget->points[i].getNormalVector3fMap();
     }
     return tempMatrix;
 }
  // write matrix in block format
 //i/p array of matrix
 //o/p  text file
                                 
  int tool::writeTransformationMatrixinBlock(char* pathAndName, std::vector<Eigen::Matrix4f> transformationMatrix)
 {
      int state = 0;
     std::ofstream file(pathAndName, std::ios::out|std::ios::trunc);
   
     if (file)  
     {
         // instructions
         for (int i = 0; i < transformationMatrix.size(); i++)
         {
             file << "camera_pose_" << i << "\n" << transformationMatrix[i] << "\n";

         }
         file.close();  
         state = 1;
     }
     return state;
     
    
 }
  // Compares relativeTransformations
 // i/p : sourceTransformation, targetTransformation
  // o/p TransformationMatrix: relativetransformation
  Eigen::Affine3f tool::ComputeRelativeTransformation(Eigen::Affine3f target, Eigen::Affine3f source, 
      Eigen::Affine3f rotate)
  {
      Eigen::Vector3f tt = (target * source.inverse()).translation();
      target.linear() = rotate * target.linear();
      source.linear() = rotate * source.linear();
      tt = rotate * tt;
      Eigen::Affine3f relTrans  = Eigen::Affine3f::Identity();
      relTrans.linear() = target.linear() * (source.linear()).inverse();
      relTrans.translation() = tt;
      return relTrans;
  }

  // read matrix in block format
  //i/p text file
  //o/p  matrix4f 
  Eigen::Matrix4f tool::ReadTransformationMatrixfromBlock(const std::string &matrixFileName)
  {
      Eigen::Matrix4f  TransformationMatrix = Eigen::Matrix4f::Identity();
      // open file for reading
      FILE *pFile = fopen(matrixFileName.c_str(), "rb");
      if (NULL == pFile)
      {
          std::cout << "Failed to read data file " << std::endl;
          exit(0);
      }
      else
      {
          unsigned numLines = pct::readLines(matrixFileName);
          char szParam1[50], szParam2[50], szParam3[50], szParam4[50];
          fscanf(pFile, "%s %s %s", szParam1, szParam2, szParam3);
          for (int i = 1; i < numLines; i++)
          {
              fscanf(pFile, " %s %s %s %s\r\n", szParam1, szParam2, szParam3,
                  szParam4);
              Eigen::Vector4f r1(atof(szParam1), atof(szParam2), atof(szParam3), atof(szParam4));
              TransformationMatrix.row(i - 1) = r1;
          }
      }
      fclose(pFile);
      return TransformationMatrix;
  }
  // Computes the orientedBoundingBox from a cloud
  // i/p: pointcloud
  // o/p minimum point3d, maximumpoint3d,  length (diagonal) of BoundingBox
  float tool::ComputeOrientedBoundingBoxOfCloud(const CloudWithNormalPtr &cloud, Eigen::Vector3f &min_pt, Eigen::Vector3f &max_pt)
  {
      // compute principal direction
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud, centroid);
      Eigen::Matrix3f covariance;
      computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
      Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
      eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));
     // std::cout << eigDx << std::endl;
      // move the points to the that reference frame
      Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
      p2w.block<3, 3>(0, 0) = eigDx.transpose();
      p2w.block<3, 1>(0, 3) = -1.f * (p2w.block<3, 3>(0, 0) * centroid.head<3>());
      CloudWithNormalPtr cPoints(new pcl::PointCloud <PointNormalType>);;
      pcl::copyPointCloud(*cloud, *cPoints);
     // cPoints = cloud;
      pcl::transformPointCloudWithNormals(*cloud, *cPoints, p2w);

      Eigen::Vector4f minpt, maxpt;
      pcl::getMinMax3D(*cPoints, minpt, maxpt);
      min_pt = minpt.head<3>();
      max_pt = maxpt.head<3>();

      float length = (max_pt - min_pt).norm();
      return length;
  }
  CloudWithoutType tool::CopyPointWithoutNormal(CloudWithoutType &InputCloud, CloudWithoutType& ModifiedCloud)
  {
      std::vector<pcl::PCLPointField> fields = InputCloud->fields;
      
      auto it = find_if(begin(fields), end(fields), [=](pcl::PCLPointField const& f) {
          return (f.name == "normal_x");
      });
      bool found = (it != end(fields));
      if (found)
      {
          CloudPtr points_xyz(new pcl::PointCloud<PointType>);
          NormalPtr normals_xyz(new pcl::PointCloud<NormalType>);
          pcl::fromPCLPointCloud2(*InputCloud, *normals_xyz);
          pcl::fromPCLPointCloud2(*ModifiedCloud, *points_xyz);
          CloudWithNormalPtr  resultant_cloud(new pcl::PointCloud<PointNormalType>);
          pcl::concatenateFields(*points_xyz, *normals_xyz, *resultant_cloud);
          CloudWithoutType new_cloud( new pcl::PCLPointCloud2);
          pcl::toPCLPointCloud2(*resultant_cloud, *new_cloud);
          return new_cloud;
      }
      else
      {
          exit(0);
      }
  }

  CloudWithoutType tool::CopytNormalWithoutPoint(CloudWithoutType &InputCloud, CloudWithoutType& ModifiedCloud)
  {
      std::vector<pcl::PCLPointField> fields = InputCloud->fields;

      auto it = find_if(begin(fields), end(fields), [=](pcl::PCLPointField const& f) {
          return (f.name == "normal_x");
      });
      bool found = (it != end(fields));
      if (found)
      {
          CloudPtr points_xyz(new pcl::PointCloud<PointType>);
          NormalPtr normals_xyz(new pcl::PointCloud<NormalType>);
          pcl::fromPCLPointCloud2(*ModifiedCloud, *normals_xyz);
          pcl::fromPCLPointCloud2(*InputCloud, *points_xyz);
          CloudWithNormalPtr  resultant_cloud(new pcl::PointCloud<PointNormalType>);
          pcl::concatenateFields(*points_xyz, *normals_xyz, *resultant_cloud);
          CloudWithoutType new_cloud(new pcl::PCLPointCloud2);
          pcl::toPCLPointCloud2(*resultant_cloud, *new_cloud);
        
          return new_cloud;
      }
      else
      {
          exit(0);
      }
  }

  std::pair<double, double> tool::GetTransformationError(Eigen::Matrix4f &GT, Eigen::Matrix4f &CT)
  {
      Eigen::Matrix4f deltaTransform =  CT * GT.inverse();
      double TransError = static_cast<double>(deltaTransform.col(3).head<3>().norm());
      Eigen::Affine3f delta_affine;
      delta_affine.matrix() = deltaTransform;
      double deltaR = (delta_affine.rotation().trace() - 1.0f) / 2.0f;
      deltaR = std::min(deltaR, 1.0);
      deltaR = std::max(deltaR, -1.0);
      double RotationError  = static_cast<double>(acos(deltaR));
      std::pair<double, double>error_pair;
      error_pair = std::make_pair(RotationError, TransError);
      return error_pair;
  }
  std::vector<Eigen::Matrix4f> tool::ReadGroundTruthTransFormationFromCSVFile(const std::string &FileName)
  {
      int k = 1;
      const std::string s = ",";
      //open file
     std::ifstream in(FileName.c_str());
      if (!in.is_open())
      {
          //print_error("Error loading file\n");
          exit(1);
      }
      std::vector<Eigen::Matrix4f> transform_matrix;
      transform_matrix.reserve(10000);
      int n_lines = 0;

      std::string line;
      std::vector<std::string > vec;
      while (in.eof() != 1)
      {
          //Always get the line -> so while advances
          getline(in, line);
          // 		cout << line << endl;

          //skip a number k of rows
          n_lines++;
          if (n_lines <= k)
          {
              continue;
          }

          //void lines could be accientaly present, in case continue
          if (line.size() == 0)
          {
              continue;
          }


          //Tokenize the line
          typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
          boost::char_separator<char> sep(s.c_str());
          tokenizer tokens(line, sep);

          //Assign tokens to a string vector
          vec.clear();
          vec.assign(tokens.begin(), tokens.end());
          Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
          temp.row(0) = Eigen::Vector4f(atof(vec[2].c_str()), atof(vec[3].c_str()), atof(vec[4].c_str()), atof(vec[14].c_str()));
          temp.row(1) = Eigen::Vector4f(atof(vec[6].c_str()), atof(vec[7].c_str()), atof(vec[8].c_str()), atof(vec[15].c_str()));
          temp.row(2) = Eigen::Vector4f(atof(vec[10].c_str()), atof(vec[11].c_str()), atof(vec[12].c_str()), atof(vec[16].c_str()));
          temp.col(3) = Eigen::Vector4f(atof(vec[5].c_str()), atof(vec[9].c_str()), atof(vec[13].c_str()), atof(vec[17].c_str()));
         /* temp.col(0) = Eigen::Vector4f(atof(vec[2].c_str()), atof(vec[6].c_str()), atof(vec[10].c_str()), atof(vec[14].c_str()));
          temp.col(1) = Eigen::Vector4f(atof(vec[3].c_str()), atof(vec[7].c_str()), atof(vec[11].c_str()), atof(vec[15].c_str()));
          temp.col(2) = Eigen::Vector4f(atof(vec[4].c_str()), atof(vec[8].c_str()), atof(vec[12].c_str()), atof(vec[16].c_str()));
          temp.col(3) = Eigen::Vector4f(atof(vec[5].c_str()), atof(vec[9].c_str()), atof(vec[13].c_str()), atof(vec[17].c_str()));*/
          transform_matrix.push_back(temp);

      }
      in.close();
      return transform_matrix;
  }

  CloudWithoutType tool::ApplyTransformationToPointCloud(CloudWithoutType &inputCloud, double RotAngleinDeg,
       Eigen::Vector3f rotAxis , Eigen::Vector3f translation )
  {
      CloudWithoutType outputCloud(new pcl::PCLPointCloud2);
      
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();

      //set the rotation around an axis
      float theta = RotAngleinDeg  * M_PI / 180.0f; // The angle of rotation in radians
      transform.rotate(Eigen::AngleAxisf(theta, rotAxis));

      //set up translation
      transform.translation() << translation(0),translation(1),translation(2);

      std::string normal = "normal_x";
      auto it = find_if(begin(inputCloud->fields), end(inputCloud->fields), [=](pcl::PCLPointField const& f) {
          return (f.name == normal);
      });
      bool found = (it != end(inputCloud->fields));
      if (found)
      {
          CloudWithNormalPtr cloudWithNormal(new pcl::PointCloud <PointNormalType>);
          CloudWithNormalPtr transformedNormalCloud(new pcl::PointCloud <PointNormalType>);
          pcl::fromPCLPointCloud2(*inputCloud, *cloudWithNormal);
          pcl::transformPointCloudWithNormals(*cloudWithNormal, *transformedNormalCloud, transform);
          pcl::toPCLPointCloud2(*transformedNormalCloud, *outputCloud);
      }
      else
      {
          CloudPtr cloud(new pcl::PointCloud <PointType>);
          CloudPtr outputPtCloud(new pcl::PointCloud <PointType>);
          pcl::fromPCLPointCloud2(*inputCloud, *cloud);
          pcl::transformPointCloud(*cloud, *outputPtCloud, transform);
          pcl::toPCLPointCloud2(*outputPtCloud, *outputCloud);
      }

      return outputCloud;

  }

  Eigen::Matrix4f tool::CreateMatrixFromFixedAngleAndTranslation(const double RotAngleinDeg, Eigen::Vector3f rotAxis,
      Eigen::Vector3f translation)
  {
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      float theta = RotAngleinDeg * M_PI / 180.0f; // The angle of rotation in radians
      transform.rotate(Eigen::AngleAxisf(theta, rotAxis));
      transform.translation() = translation;
      Eigen::Matrix4f groundTruthTransform = transform.matrix();
      return groundTruthTransform;
  }
  Eigen::MatrixXf tool::RowShiftMatrix(const Eigen::MatrixXf & in, int down)
  {
      if (!down)
          return in;
      Eigen::MatrixXf out(in.rows(), in.cols());
      if (down > 0) down = down % in.rows();
      else down = in.rows() - (-down % in.rows());
      // We avoid the implementation-defined sign of modulus with negative arg. 
      int rest = in.rows() - down;
      out.topRows(down) = in.bottomRows(down);
      out.bottomRows(rest) = in.topRows(rest);
      return out;
  }

  Eigen::MatrixXf tool::CreateMatrixFromStlVector(const std::vector<std::vector<float>> &Data,const int row, const int col)
  {
      Eigen::MatrixXf eigMat2(row, col);
      for (int i = 0; i < row; i++)
         eigMat2.row(i) = Eigen::VectorXf::Map(&Data[i][0], Data[i].size());
      //Eigen::MatrixXf eigMat1 = Eigen::MatrixXf::Zero(row, col);// (Data.data())
      //int row_step = 0;
      //std::cout <<"before:" << eigMat2 << std::endl;
      //for (int itr = 0; itr < row; itr++)
      //{
      //    std::vector<float>::const_iterator first = Data.begin() + row_step;
      //    std::vector<float>::const_iterator last = Data.begin() + row + row_step;
      //   std::vector<float> v2(first, last);
      //   eigMat1.row(itr) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(v2.data(), v2.size());
      //    row_step = row_step + row;
      //   
      //}
     
      //std::cout << eigMat1 << std::endl;
     // eigMat2.transposeInPlace();
    /*  std::cout << eigMat2 << std::endl;
      std::vector<float>temp_data;
      temp_data.insert(temp_data.end(), std::make_move_iterator(eigMat2.data()), std::make_move_iterator(eigMat2.data() + eigMat2.size()));*/
      return eigMat2;
  }

  std::vector<float> tool::CreateStlVectorFromMatirx( Eigen::MatrixXf in)
  {
      in.transposeInPlace();
      std::vector<float>temp_data;
      temp_data.insert(temp_data.end(), std::make_move_iterator(in.data()), std::make_move_iterator(in.data() + in.size()));
      return temp_data;
  }

  void  tool::Create2DStlVectorFromMatrix(const Eigen::MatrixXf &in, std::vector<std::vector<float>> &out_vector, int row, int col)
  {
    //  Eigen::MatrixXf temp_matrix= in;
     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>temp_matrix = in;
     // temp_matrix =  in.transpose();
      std::vector<std::vector<float>>temp_data_storage_vector(row, std::vector<float>(col));
     // temp_data_storage_vector.resize(row);
    
    /*  typedef Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrixXf;
      Eigen::Map<RowMatrixXf>(&temp_data_storage_vector[0][0], row, col) = in;*/
      for (int i = 0; i<temp_matrix.rows(); ++i)
      {
          const float* begin = &temp_matrix.row(i).data()[0];
          temp_data_storage_vector[i] = (std::vector<float>(begin, begin + temp_matrix.cols()));
      }
    /*  for (int i = 0; i < row; i++)
      {
          temp_data_storage_vector[i].insert(temp_data_storage_vector[i].end(), std::make_move_iterator(temp_matrix.row(i).data()), 
              std::make_move_iterator(temp_matrix.row(i).data() + col));
      }*/
      out_vector = std::move(temp_data_storage_vector);
  }

  std::vector<std::vector<float>> tool::ReScaleImageBilinear(const std::vector<std::vector<float>> &ImgData, int row_prev, int col_prev, int row_new, int col_new)
  {
      int new_size = row_new * col_new;
      std::vector<std::vector<float>> temp(row_new, std::vector<float>(col_new,-INFINITY));
      float A, B, C, D, gray;
      float y_ratio = ((float)(col_prev)) / col_new;
      float x_ratio = ((float)(row_prev)) / row_new;
      float x_diff, y_diff, ya, yb;
      int offset = 0;
      int  index, x, y;
      std::vector<std::vector<float>> ImageValues(ImgData);
      for (int i = 0; i < row_new; i++)
      {
          for (int j = 0; j < col_new; j++)
          {
            
             
              y = (int)(y_ratio * j);
              x = (int)(x_ratio * i);
              float accumulate = 0;
              int count = 0;
              for (int x_itr = x; x_itr < x +(int)( x_ratio); x_itr++) // x_ratio
              {
                  for (int y_itr = y; y_itr < y + (int)(y_ratio); y_itr++) //y_ratio
                  {
                      float val = ImageValues[x_itr][y_itr];
                      if (val != -INFINITY)
                      {
                          accumulate += val;
                          count++;
                      }

                    
                  }

              }
              if (count > 0)
              {
                  // some confusion with concept of offset into 2d index as offset was temp[offset]
                  int ir = offset / col_new;
                  int ic = offset % col_new;
                  temp[i][j] = std::round(accumulate / float(count));
              }
         
              offset++;
          }
      }
      return temp;
  }
  std::vector<float>tool::testBilinearInterpolation(std::vector<float>ImgData, int row_prev, int col_prev, int row_new, int col_new)
  {
      // testing bilinear interpolation
      int new_size = row_new * col_new;
      std::vector<float> temp(new_size, -INFINITY);

      // now compute our final image using bilinear interpolation
      int offset = 0;
      for (int j = 0; j < row_new; ++j)
      {
          for (int i = 0; i < col_new; ++i)
          {
              // convert i,j to grid coordinates
              float gx = i / float(col_new) * col_prev;
              float gy = j / float(row_new) * row_prev;
              int gxi = int(gx);
              int gyi = int(gy);
              const float c00 = ImgData[gyi * (col_new ) + gxi];
              const float c10 = ImgData[gyi * (col_new) + (gxi + 1)];
              const float c01 = ImgData[(gyi + 1) * (col_new) + gxi];
              const float c11 = ImgData[(gyi + 1) * (col_new ) + (gxi + 1)];
              std::cout << gyi * (col_new) + gxi << std::endl;
              std::cout << gyi * (col_new) + (gxi + 1) << std::endl;
              std::cout << (gyi + 1) * (col_new) + gxi << std::endl;
              std::cout << (gyi + 1) * (col_new) + (gxi + 1) << std::endl;
              if (c00 != -INFINITY && c10 != -INFINITY && c01 != -INFINITY && c11 != -INFINITY)
              {
                  temp[offset] = bilinear(gx - gxi, gy - gyi, c00, c10, c01, c11);
              }
              offset++;
          }
      }
      return  temp;

  }