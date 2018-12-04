
#pragma once
#include<vector>
#include"Datatypes.h"
using namespace Eigen;

class Normal
{
protected:
    float m_dX;
    float m_dY;
    float m_dZ;
    float Curvature;
    std::vector<int> Index;
    std::vector<float>Dist;
    std::vector<Eigen::Vector4f> Data;




public:
    Normal();
    //~NormalEstimation();
    void computeRoots2(Eigen::Matrix3f::Scalar &a, Eigen::Matrix3f::Scalar &b, Eigen::Vector3f &c);
    void computeRoots(Eigen::Matrix3f &m, Eigen::Vector3f &roots);
    void eigen33(Eigen::Matrix3f &mat, Eigen::Vector3f::Scalar &eigenvalue, Eigen::Vector3f &eigenvector);
    void centroid(std::vector<int> data, pcl::PointCloud<pcl::PointXYZ >&cld, int count, Eigen::Vector3f &Vector);
    unsigned int computecovmatrix(Eigen::Vector3f &Cvec, std::vector<int>&Arr, CloudPtr &ptcld, Eigen::Matrix3f &covariance_matrix);
    void solve_plane(Eigen::Matrix3f &covariance_matrix, float &nx, float &ny, float &nz, float &curvature);

};

