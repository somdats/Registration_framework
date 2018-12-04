#include"pch.h"
#include"TransformationEstimationSparseICP.h"
#include"ErrorMetric.h"
#include"ply_file_io.h"

using namespace Registration;

cTransformEstimationSparseICP::cTransformEstimationSparseICP(int errorMetricType_)
{
    m_TransformationMatrix.setIdentity();
    target_has_normals_ = false;
    errorMetricType = errorMetricType_;
}
cTransformEstimationSparseICP::cTransformEstimationSparseICP(cTransformEstimationSparseICP & transformEstimation)
{
    srcCloud_ = transformEstimation.srcCloud_;
    targetCloud_ = transformEstimation.targetCloud_;
    targetNormalMatrix_ = transformEstimation.targetNormalMatrix_;
    targetPointMatrix_ = transformEstimation.targetPointMatrix_;
    target_has_normals_ = transformEstimation.target_has_normals_;
    m_TransformationMatrix = transformEstimation.m_TransformationMatrix;
}
cTransformEstimationSparseICP ::~cTransformEstimationSparseICP()
{
    if (NULL != kdt)
    {
        delete kdt;
        kdt = NULL;
    }
    if (NULL != cdp)
    {
        delete cdp;
        cdp = NULL;
    }
    if (NULL != cpp)
    {
        delete cpp;
        cpp = NULL;
    }
    pixelCoordinates_.clear();
}

 void cTransformEstimationSparseICP :: PrepareSample(CloudWithoutType &cloudSrc, CloudWithoutType &cloudTarget)
{
    const std::vector<pcl::PCLPointField> fields = cloudTarget->fields;
     std::string normal = "normal_x";
     // Target- Normals
     auto it = find_if(begin(fields), end(fields), [=](pcl::PCLPointField const& f) {
         return (f.name == normal);
     });
     bool found = (it != end(fields));
     if (found)
     {
         target_has_normals_ = true;
         targetNormalMatrix_ = tool::createNormalMatrixFromCloud(cloudTarget);
     }

     // source-Normals
     const std::vector<pcl::PCLPointField>cloud_fields = cloudSrc->fields;
     auto itr = find_if(begin(cloud_fields), end(cloud_fields), [=](pcl::PCLPointField const& f) {
         return (f.name == normal);
     });
     bool if_found = (itr != end(cloud_fields));
     if (if_found)
     {
         sourceNormalMatrix_ = tool::createNormalMatrixFromCloud(cloudSrc);
     }
     srcCloud_ = cloudSrc;
     targetCloud_ = cloudTarget;
     srcPointMatrix_ = tool::createPointMatrixFromCloud(cloudSrc);
     targetPointMatrix_ = tool:: createPointMatrixFromCloud(cloudTarget);
     pixelCoordinates_.reserve(targetPointMatrix_.cols());
}
 void cTransformEstimationSparseICP::align()
{
  //TODO
}

 Eigen::Affine3f cTransformEstimationSparseICP::getFinalTransformation()const
 {
      return m_TransformationMatrix;
    
 }

 void cTransformEstimationSparseICP:: setSourceCloud(Eigen::Matrix3Xf src)
 {
     srcPointMatrix_ = src;
 }

 void cTransformEstimationSparseICP::SettargetCloud(Eigen::Matrix3Xf tgt, Eigen::Matrix3Xf tgtNml)
 {
     targetPointMatrix_ = tgt;
     targetNormalMatrix_ = tgtNml;
 }
 CorrespondenceEstimator::ICorrespondenceEstimator* cTransformEstimationSparseICP::selectCorrespondenceDeterminationType(Function f)
 {
 
   /*  switch (f)
     {
     case CORRES_KDTREE:
         kdt = new cCorrespondenceKdtree;
         return kdt;
         break;
     case CORRES_DP:
         cdp = new cCorrespondenceDepth;
         return cdp;
         break;
     case CORRES_PP:
         cpp = new cCorrespondencePlaneProjection;
         return cpp;
         break;
     default:
         throw std::runtime_error("select correspondence estimator type");
         break;
     }*/
     return nullptr;
 }

 void cTransformEstimationSparseICP::setSparseICPParameter( Eigen::Affine3f intialGuessTransform,
     float norm , int icpIteration , int searchStrategy, Eigen::Matrix4f gt, float stopCriteria, bool fallback, double rmse_unit, std::pair<double, double>error)
 {
     SICP::Parameters param;
     param.groundTruthTransform = gt;
     param.initialTransform = &intialGuessTransform;
     param.p = norm;
     param.max_icp = icpIteration;
     param.max_outer = 10;
     param.max_inner = 1;
     param.searchtype = searchStrategy;
     param.stop = stopCriteria;
     param.error_pair = error;
     param.fall_Back = fallback;
     param.diagonalLength = rmse_unit;
     params_ = param;
     
 }

 std::vector<Eigen::Matrix4f> cTransformEstimationSparseICP::GetTransformationForEachICPIteration()
 {
     return SICP::iteration_transform;
 }

 std::vector<std::pair<double, double>>cTransformEstimationSparseICP::GetTimingsForEachICPIteration()
 {
     return SICP::distribution_time;
 }
 std::vector<double> cTransformEstimationSparseICP::GetTotalTimeForEachRegistration()
 {
     return SICP::iterative_step_execution_time;
 }

 std::vector<double> cTransformEstimationSparseICP::GenerateRMSEListforEachViewPair(std::string &fileName, CloudWithoutType sourceCloud, CloudWithoutType targetCloud)
 {
     std::vector<Eigen::Matrix4f>iteration_transform  =  GetTransformationForEachICPIteration();
     std::vector<std::pair<double, double>>icp_component_time = GetTimingsForEachICPIteration();
     if (iteration_transform.size() != icp_component_time.size())
     {
         throw std::runtime_error("Number of iteratons and icp component time is different\n");
     }
     std::vector<double>rmse;
     rmse.resize(iteration_transform.size(), 0.0);
     for (int ii = 0; ii < iteration_transform.size(); ii++)
     {
         CloudWithoutType tranformed_groundTruth_source = tool::TransFormationOfCloud(sourceCloud, iteration_transform[ii]);
          rmse[ii] = static_cast<double>(metric::ComputeRMSE(tranformed_groundTruth_source, targetCloud, 1.0));
     }
     if (iospace::ExistsFile(fileName) /*&& deleteFile == false*/)
     {
         remove(fileName.c_str());
         // deleteFile = true;
     }
     metric::WriteRMSE(fileName, rmse);
     return rmse;
 }

 void cTransformEstimationSparseICP::WriteRMSEAndTime(std::string &fileName, std::vector<double>rmse, std::vector<std::pair<double, double>> error_values)
 {
     if (iospace::ExistsFile(fileName) /*&& deleteFile == false*/)
     {
         remove(fileName.c_str());
         // deleteFile = true;
     }
     std::vector<double>regis_time = GetTotalTimeForEachRegistration();
     std::vector<std::pair<double, double>>distribution_time = GetTimingsForEachICPIteration();
     FILE	*g_pLogFile = fopen(fileName.c_str(), "wb");
     if (NULL == g_pLogFile)
     {
         abort();
     }
     
     fprintf(g_pLogFile, "RMSE\tCorresp_time\tOptim_time\tIteration_time\terror_value\n");
     
     for (int i = 0; i < regis_time.size(); i++)
     {
         if (i == 0)
             fprintf(g_pLogFile, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", rmse[i], distribution_time[i].first, distribution_time[i].second, regis_time[i],
                 error_values[i].first);
         else
         {
            
             double time = 0.0;
             for ( int j = i; j >= 0; j--)
             {

                 time += regis_time[j];
             }
             fprintf(g_pLogFile, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", rmse[i], distribution_time[i].first, distribution_time[i].second, time,
                 error_values[i].first);
         }
     }
     if (NULL != g_pLogFile)
     {
         fclose(g_pLogFile);
         g_pLogFile = NULL;
     }
 }

 void cTransformEstimationSparseICP::SetFileNameSicp(std::string &fileName)
 {
     SICP::rmse_filename = fileName;
 }
 void cTransformEstimationSparseICP::GetIcpTimings(double &CorrsTime, double &OptimTime)

 {
     CorrsTime = SICP::GetCorrespondenceComputationTime();
     OptimTime = SICP::GetOptimizationTime();
 }
 Eigen::Affine3f cTransformEstimationSparseICP::ExecuteSparseICPRegistration(CloudWithoutType &cloudSrc, CloudWithoutType &cloudTarget, SICP::Parameters param,
     CorrespondenceEstimator::ICorrespondenceEstimator *Estimators, std::vector<std::pair<double, double>>&error_data)
 {
     Eigen::Affine3f finalTransform = Eigen::Affine3f::Identity();
 
     Estimators->PrepareDataForCorrespondenceEstimation(cloudSrc, cloudTarget);
     auto searchStrategy = Estimators->getSearchStrategy();
     // cdp->SetTargetPixelCoordinate(pixelCoordinates_);
     if (errorMetricType == 0)
         finalTransform = SICP::point_to_plane(srcPointMatrix_, targetPointMatrix_, targetNormalMatrix_, searchStrategy, error_data, sourceNormalMatrix_, param );
     else
     {
         finalTransform = SICP::point_to_point(srcPointMatrix_, targetPointMatrix_, searchStrategy, error_data, param);
     }
     return finalTransform;
 }
 std::vector<std::pair<double, double>> cTransformEstimationSparseICP::estimateRigidRegistration(CorrespondenceEstimator::ICorrespondenceEstimator *Estimators,
     Eigen::Matrix4f &Transformation_matrix)
 {
     SICP::Parameters param_data = getsparseICPParameter();
     std::vector<std::pair<double, double>>error_data;
     Eigen ::Affine3f  trans_matrix =  ExecuteSparseICPRegistration(srcCloud_, targetCloud_, param_data,Estimators, error_data);
     Transformation_matrix = trans_matrix.matrix();
     return error_data;
 }

 SICP::Parameters cTransformEstimationSparseICP::getsparseICPParameter()
 {
     return params_;
 }

 void cTransformEstimationSparseICP :: SetUVDataForRegistration(std::vector<UVData<float>>pixelCoordinates)
 {
     pixelCoordinates_ = pixelCoordinates;
 }
