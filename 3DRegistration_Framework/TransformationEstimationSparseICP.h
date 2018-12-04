#pragma once
#include"IRegistattion.h"
#include"Datatypes.h"
#include"ISearch.h"
#include"KdTreeSearch.h"
#include"ICorrespondenceEstimator.h"
#include"CorrespondenceEstimationKdtree.h"
#include"CorrespondenceEstimationDepthMap.h"
#include"CorrespondenceEstimationPlaneProjection.h"
#include"sparse_icp.h"
#include"Transformation_tool.h"

    class REG3D_API cTransformEstimationSparseICP : public Registration::iRegsitration
    {
    public:
        cTransformEstimationSparseICP(int errorMetricType_ = 0);
        cTransformEstimationSparseICP(cTransformEstimationSparseICP &transformEstimation);
        ~cTransformEstimationSparseICP();
       enum Function
       {
           CORRES_KDTREE,
           CORRES_DP,
           CORRES_PP
       };
       void PrepareSample(CloudWithoutType &cloudSrc, CloudWithoutType &cloudTarget);
       std::vector<std::pair<double, double>> estimateRigidRegistration(CorrespondenceEstimator::ICorrespondenceEstimator *Estimators, Eigen::Matrix4f &Transformation_matrix);
       void align();
       Eigen::Affine3f getFinalTransformation()const;
       CorrespondenceEstimator::ICorrespondenceEstimator* selectCorrespondenceDeterminationType(Function f);  // create respective object depending on type
       Eigen::Affine3f ExecuteSparseICPRegistration(CloudWithoutType &cloudSrc, CloudWithoutType &cloudTarget, SICP::Parameters param, 
           CorrespondenceEstimator::ICorrespondenceEstimator *Estimators, std::vector<std::pair<double, double>>&error_data);
       void setSparseICPParameter(Eigen::Affine3f intialGuessTransform, float norm = 0.1f, int icpIteration = 50, 
           int searchStrategy = 0, Eigen::Matrix4f gt = Eigen::Matrix4f::Identity(), float stopCriteria = 1e-5f, bool fallback = true, double rmse_unit = 1.0f,std::pair<double, double>error = std::make_pair(1e-5f, 1e-5f));
       void setSourceCloud(Eigen::Matrix3Xf src);
       void SettargetCloud(Eigen::Matrix3Xf tgt, Eigen::Matrix3Xf tgtNml);
       SICP::Parameters getsparseICPParameter();
       void SetUVDataForRegistration(std::vector<UVData<float>>pixelCoordinates);
       void GetIcpTimings(double &CorrsTime, double &OptimTime);
       void SetFileNameSicp(std::string &fileName);
       std::vector<Eigen::Matrix4f> GetTransformationForEachICPIteration();
       std::vector<std::pair<double, double>> GetTimingsForEachICPIteration();
       std::vector<double>GetTotalTimeForEachRegistration();
       std::vector<double> GenerateRMSEListforEachViewPair(std::string &fileName, CloudWithoutType sourceCloud, CloudWithoutType targetCloud);
       void WriteRMSEAndTime(std::string &fileName, std::vector<double>rmse, std::vector<std::pair<double, double>> error_values);
     


    protected:
        SICP::Parameters params_;
        CloudWithoutType srcCloud_;
        CloudWithoutType targetCloud_;
        Eigen::Affine3f m_TransformationMatrix;
        Eigen::Matrix3Xf srcPointMatrix_;
        Eigen::Matrix3Xf targetPointMatrix_;
        Eigen::Matrix3Xf targetNormalMatrix_;
        Eigen::Matrix3Xf sourceNormalMatrix_;
        bool target_has_normals_;
       cCorrespondenceDepth *cdp = nullptr;
       cCorrespondencePlaneProjection *cpp = nullptr;
       cCorrespondenceKdtree *kdt = nullptr;
       std::vector<UVData<float>>pixelCoordinates_;
       int errorMetricType;
    };

