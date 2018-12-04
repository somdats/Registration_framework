#pragma once

#include<string>
#include<vector>
#include"Config.h"
#include"pct_io.h"
#include"Transformation_tool.h"
#include"ply_file_io.h"



/*  class serves as an Interface to evaluate Protocols for ETH ASL Dataset*/
class REG3D_API Evaluations
{
public:
    
     void ParseProtocolFile(std::string & fileName);
     void ParseValidationFile(std::string & fileName);
     void WriteEvaluationParameter(std::string &FileName);
     double ComputeStandardDeviation(std::vector<std::pair<double, double>>&Data);
     void  ComputeCumulativeRecallDataSorted(std::vector<double> &Value_type1, std::vector<double>&Value_type2);
     std::vector<double> ComputeCumulativeRecallGraph(std::vector<double> Value_type1, double spacing, std::vector<double>&comparator);
     std::vector<std::pair<double, double>>ReadEvaluationFile(std::string &FileName, std::string pose_type, float overlap = 0.0f);
     void ReadMethodComparisonData(std::string & fileName, std::vector<Eigen::Matrix4f> & method_results, std::vector<double> &method_execution_time);
     std::vector<std::pair< std::string, std::string>> GetComparisonDataName();
     std::vector<Eigen::Matrix4f> GetGroundTruthData();
     std::vector<Eigen::Matrix4f> GetPerturbationData();
     std::vector<std::string> GetPoseType();
     void setComputationTimeforMethod(std::vector<double> &time);
     void setEvaluationParameter(std::vector<std::pair<double, double>> &evaluationParameter_);
     void GeneratePlotDataForMethod(std::vector<std::string>&fileNameLocation, std::string &PoseType, std::vector<std::vector<double>>&rotation_data,
         std::vector<std::vector<double>>&translation_data, std::vector<std::vector<double>>&timing_data, float overlap = 0.0f);
     void writeFinalRegistrationMatricesForMethod(std::string &FileNameLocation);
     void CollectFinalRegistrationMatrices(std::vector<Eigen::Matrix4f> &registration_matrices);
     std::pair<double, double> ComputeStandardDeviationFromTransformationParamter(std::vector<std::pair<double, double>>&Data);
     void Reset(); // clears member variable 

protected:
    std::vector<Eigen::Matrix4f>groundTruth_matrices;
    std::vector<Eigen::Matrix4f>perturbation_matrices;
    std::vector<Eigen::Matrix4f>Final_registration_matrices;
    std::vector<std::pair< std::string, std::string>>comp_files;
    std::vector<std::string>Pose_Type;
    std::vector<std::pair<double, double>>evaluationParameter;
    std::vector<double>overlap_ratio;
    std::vector<double>times;
 
};

class REG3D_API Evaluation_Ext
{
    public:
        std::vector<std::pair<std::string, Eigen::Matrix4f>> ReadTransformationmatrixFromFile(const std::string &matrixFileName);
        std::vector<std::pair<std::string, std::string>> ReadViewPairForRegistration(const std::string &ViewList);
        CloudWithoutType TransformView(CloudWithoutType &inputCloud, Eigen::Matrix4f & Gt_Target, Eigen::Matrix4f &R_Transformation);
        double CalculateRMSEForEvaluationDataSet(std::vector<std::pair<std::string, Eigen::Matrix4f>> & view_vs_transform,
            Eigen::Matrix4f &output_transform, std::string source_view, std::string target_view, CloudWithoutType &source_viewCloud, double rmse_unit);
        double CalculateRMSEForEvaluationDataSet(std::vector<std::pair<std::string, Eigen::Matrix4f>> & view_vs_transform,
            Eigen::Matrix4f &output_transform, std::string fake_view, CloudWithoutType &source_viewCloud, double rmse_unit);
        void WriteViewpair(std::vector<std::pair<std::string, std::string>> & view_pair, std::string &FileName);
        void CompareRMSEWithThreshold(const std::string fileName, double Threshold, int &trueCount, int &falseCount);
        double ComputeMeshResolutionForAllViews(const std::string &dirName, const std::string &fileExt, const std::string &absMeshResFile = "");
       double ComputeMeshResolution(pcl::PolygonMesh &mesh);
       std::vector<std::pair<std::string, std::string>> GenerateViewPairForEvaluation(const std::string &dirName, const std::string &fileExt);
      

        struct comp
        {
            comp(std::string const& s) : _s(s) { }

            bool operator () (std::pair<std::string, Eigen::Matrix4f> const& p)
            {
                return (p.first == _s);
            }

            std::string _s;
        };
        typedef comp comp_;

        struct SPointNeighbor
        {
            //!list of triangles which are connected to a vertex
            std::vector< std::vector<pcl::Vertices>> m_acListTri;
            //!indices of neighboring triangles of a vertex in a mesh
            std::vector<int> m_anFaceIndicesList;
            //!indices of neighboring vertices of a vertex in a mesh
            std::vector<int> m_anVertexIndxList;
            //!normals of all neighboring triangles
            std::vector<Eigen::Vector3f> m_acUnitNormals;
            //! max area of neighboring triangles
            double dMaxArea;
            //! sum of areas of all neighboring triangles
            double dAreaSum;
            //!average of all normals of neighboring triangles	
            Eigen::Vector3f avgUnitNormal;

            //! initialize the members
            SPointNeighbor()
            {
                dMaxArea = -1;
                avgUnitNormal.setZero();
                dAreaSum = 0;
            }
        };
        typedef SPointNeighbor pointNeigbor_;
        void GenerateFaceNeighborhoodFromMesh(pcl::PolygonMesh &mesh, pointNeigbor_  *ptNeighbors);
        void ComputePointNormal(pcl::PolygonMesh &mesh, std::vector<Eigen::Vector3f> &Normals, pointNeigbor_  *ptNeighbors);
        void GenerateFaceNeighborList(pcl::PolygonMesh &mesh, std::vector<std::vector<int>> &Facelist, pointNeigbor_  *ptNeighbors);
        void ComputePointNormal(const std::string &dirName, const std::string &fileExt,  const std::string tarDir);
        void PrintResults(const std::string outputFileName, const double Standard_dev, const double MeshResolution, const int totalViews, int trueCount);
        std::vector<std::pair<std::string, std::string>> GetFakeNamePairs();
        void ComputePointNormal(pcl::PolygonMesh &Mesh);
       


protected:
    std::vector<std::string>Views;
    std::vector<Eigen::Matrix4f>TransformationMatrices;
    std::vector<std::pair<std::string, std::string>> fake_pairs;  // customized for evaluation of room dataset:http://www.vision.deis.unibo.it/research/78-cvlab/108-pairwiseregistrationbenchmark

};
