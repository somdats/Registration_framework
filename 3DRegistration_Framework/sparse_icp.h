#pragma once
///////////////////////////////////////////////////////////////////////////////
///   "Sparse Iterative Closest Point"
///   by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
///   Copyright (C) 2013  LGG, EPFL
///////////////////////////////////////////////////////////////////////////////
///   1) This file contains different implementations of the ICP algorithm.
///   2) This code requires EIGEN and NANOFLANN.
///   3) If OPENMP is activated some part of the code will be parallelized.
///   4) This code is for now designed for 3D registration
///   5) Two main input types are Eigen::Matrix3Xd or Eigen::Map<Eigen::Matrix3Xd>
///////////////////////////////////////////////////////////////////////////////
///   namespace nanoflann: NANOFLANN KD-tree adaptor for EIGEN
///   namespace RigidMotionEstimator: functions to compute the rigid motion
///   namespace SICP: sparse ICP implementation
///   namespace ICP: reweighted ICP implementation
///////////////////////////////////////////////////////////////////////////////
//#ifndef sparse_icp.h
//#define sparse_icp.h
#include "nanoflann.hpp"
#include <Eigen/Dense>
#include<stdio.h>
#include<math.h>
#include"ISearch.h"
#include"ICorrespondenceEstimator.h"
#include"PointData.h"
#include"Transformation_tool.h"
#include<chrono>
#include"Common.h"
#include"ply_file_io.h"
#include"Transformation_tool.h"
#include<limits>
#include"CpuTimeProfiler.h"
#include"KdTreeSearch.h"
#define epsilon_error  std::numeric_limits<float>::epsilon()



///////////////////////////////////////////////////////////////////////////////
namespace nanoflann {
    /// KD-tree adaptor for working with data directly stored in an Eigen Matrix, without duplicating the data storage.
    /// This code is adapted from the KDTreeEigenMatrixAdaptor class of nanoflann.hpp
    template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
    struct KDTreeAdaptor 
    {
        typedef KDTreeAdaptor<MatrixType, DIM, Distance> self_t;
        typedef typename MatrixType::Scalar              num_t;
        typedef typename Distance::template traits<num_t, self_t>::distance_t metric_t;
        typedef KDTreeSingleIndexAdaptor< metric_t, self_t, DIM, IndexType>  index_t;
        index_t* index;
        KDTreeAdaptor(const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat) 
        {
            const size_t dims = mat.rows();
            index = new index_t(dims, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size, dims));
            index->buildIndex();
        }
        ~KDTreeAdaptor() { delete index; }
        const MatrixType &m_data_matrix;
        /// Query for the num_closest closest points to a given point (entered as query_point[0:dim-1]).
        inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices, num_t *out_distances_sq) const {
            nanoflann::KNNResultSet<typename MatrixType::Scalar, IndexType> resultSet(num_closest);
            resultSet.init(out_indices, out_distances_sq);
            index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
        }
        /// Query for the closest points to a given point (entered as query_point[0:dim-1]).
        inline IndexType closest(const num_t *query_point) const {
            IndexType out_indices;
            num_t out_distances_sq;
            query(query_point, 1, &out_indices, &out_distances_sq);
            return out_indices;
        }
        const self_t & derived() const { return *this; }
        self_t & derived() { return *this; }
        inline size_t kdtree_get_point_count() const { return m_data_matrix.cols(); }
        /// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
        inline num_t kdtree_distance(const num_t *p1, const size_t idx_p2, size_t size) const {
            num_t s = 0;
            for (size_t i = 0; i<size; i++) {
                const num_t d = p1[i] - m_data_matrix.coeff(i, idx_p2);
                s += d*d;
            }
            return s;
        }
        /// Returns the dim'th component of the idx'th point in the class:
        inline num_t kdtree_get_pt(const size_t idx, int dim) const {
            return m_data_matrix.coeff(dim, idx);
        }
        /// Optional bounding-box computation: return false to default to a standard bbox computation loop.
        template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
    };
}
///////////////////////////////////////////////////////////////////////////////
/// Compute the rigid motion for point-to-point and point-to-plane distances
namespace RigidMotionEstimator 
{
        template <typename Derived1, typename Derived2, typename Derived3>
        Eigen::Affine3f point_to_point(const Eigen::MatrixBase<Derived1>& X,
            const Eigen::MatrixBase<Derived2>& Y,
            const Eigen::MatrixBase<Derived3>& w,
            const Eigen::Affine3f& initialTransform)
        {
            /// Normalize weight vector
            Eigen::VectorXd w_normalized = w.cast<double>() / (double)w.sum();
            /// De-mean
            Eigen::Vector3d X_mean, Y_mean;
            for (int i = 0; i<3; ++i)
            {
                X_mean(i) = (X.row(i).cast<double>().array()*w_normalized.transpose().array()).sum();
                Y_mean(i) = (Y.row(i).cast<double>().array()*w_normalized.transpose().array()).sum();
            }
            X_mean = initialTransform.cast<double>() * X_mean;

         /*   std::cout << "X mean: " << X_mean.transpose() << std::endl;
            std::cout << "Y mean: " << Y_mean.transpose() << std::endl;*/

            /// Compute transformation
            Eigen::Affine3d transformation = initialTransform.cast<double>();
            transformation.pretranslate(-X_mean.cast<double>());
            Eigen::Matrix3d sigma; sigma.setConstant(0.0);
            double error = 0;
            for (int i = 0; i < X.cols(); ++i)
            {
                sigma += (initialTransform.cast<double>() * X.col(i).cast<double>() - X_mean) * (double)w_normalized.coeff(i) * (Y.col(i).cast<double>() - Y_mean).transpose();
                error += (initialTransform * X.col(i) - Y.col(i)).squaredNorm() * w_normalized.coeff(i);
            }
            /*std::cout << "Initial error: " << sqrt(error) << std::endl;*/
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
           /* std::cout << svd.singularValues() << std::endl;*/

            Eigen::Affine3d rotation;
            rotation.setIdentity();
            if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
                Eigen::Vector3d S = Eigen::Vector3d::Ones(); S(2) = -1.0;
                rotation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
            }
            else {
                rotation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
            }
            /*sigma.col(0).normalize();
            sigma.col(1).normalize();
            sigma.col(2).normalize();
            rotation.linear().noalias() = sigma;*/
            transformation = rotation * transformation;
            //transformation.pretranslate(Y_mean - rotation.linear()*X_mean);
            transformation.pretranslate(Y_mean);

            error = 0;
            for (int i = 0; i < X.cols(); ++i)
            {
                error += (transformation.cast<float>() * X.col(i) - Y.col(i)).squaredNorm() * w_normalized.coeff(i);
            }
            /*std::cout << "Final error: " << sqrt(error) << std::endl;*/
            std::wstringstream ss;
           /* ss << "Final error: " << error << std::endl;
            OutputDebugString((ss.str().c_str()));*/

            /// Return transformation
            return transformation.cast<float>();
    }

        /// @param Source (one 3D point per column)
        /// @param Target (one 3D point per column)
        template <typename Derived1, typename Derived2>
        inline Eigen::Affine3f point_to_point(const Eigen::MatrixBase<Derived1>& X,
            const Eigen::MatrixBase<Derived2>& Y,
            const Eigen::Affine3f& initialTransform)
        {
            return point_to_point(X, Y, Eigen::VectorXf::Ones(X.cols()), initialTransform);
        }
        /// @param Source (one 3D point per column)
        /// @param Target (one 3D point per column)
        /// @param Target normals (one 3D normal per column)
        /// @param Confidence weights
        /// @param Right hand side
        template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
        Eigen::Affine3f point_to_plane(const Eigen::MatrixBase<Derived1>& X,
            const Eigen::MatrixBase<Derived2>& Y,
            const Eigen::MatrixBase<Derived3>& N,
            const Eigen::MatrixBase<Derived4>& w,
            const Eigen::MatrixBase<Derived5>& u,
            const Eigen::Affine3f& initialTransform, float &dist_plane) {
            typedef Eigen::Matrix<double, 6, 6> Matrix66;
            typedef Eigen::Matrix<double, 6, 1> Vector6;
            typedef Eigen::Block<Matrix66, 3, 3> Block33;
            /// Normalize weight vector
            Eigen::VectorXd w_normalized = w.cast<double>() / (double)w.sum();
            /// De-mean
            Eigen::Vector3d X_mean;
            for (int i = 0; i<3; ++i)
                X_mean(i) = (X.row(i).cast<double>().array()*w_normalized.transpose().array()).sum();
            X_mean = initialTransform.cast<double>() * X_mean;

            Eigen::Affine3d transformation = initialTransform.cast<double>();
            transformation.pretranslate(-X_mean);
            dist_plane = 0.0f;
            /// Prepare LHS and RHS
            Matrix66 LHS = Matrix66::Zero();
            Vector6 RHS = Vector6::Zero();
            Block33 TL = LHS.topLeftCorner<3, 3>();
            Block33 TR = LHS.topRightCorner<3, 3>();
            Block33 BR = LHS.bottomRightCorner<3, 3>();
            Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3, X.cols());
#pragma omp parallel
            {
#pragma omp for
                for (int i = 0; i<X.cols(); i++) 
                {
                    C.col(i) = ((initialTransform * X.col(i)).cast<double>() - X_mean).cross(N.col(i).cast<double>());
                }
#pragma omp sections nowait
                {
#pragma omp section
                    for (int i = 0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
#pragma omp section
                    for (int i = 0; i<X.cols(); i++) TR += (C.col(i) * N.col(i).cast<double>().transpose())*w(i);
#pragma omp section
                    for (int i = 0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(N.col(i).cast<double>(), w(i));
#pragma omp section
                    for (int i = 0; i<C.cols(); i++) {
                       float  dist_to_plane = -(((initialTransform * X.col(i)).cast<double>() - Y.col(i).cast<double>()).dot(N.col(i).cast<double>()) - u(i)) * w(i);
                        RHS.head<3>() += C.col(i)*dist_to_plane;
                        RHS.tail<3>() += N.col(i).cast<double>() * dist_to_plane;
                        dist_plane += abs(dist_to_plane);
                    }
                }
            }
            LHS = LHS.selfadjointView<Eigen::Upper>();
            /// Compute transformation
            Eigen::LDLT<Matrix66> ldlt(LHS);
            RHS = ldlt.solve(RHS);
            transformation = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ()) * transformation;
            transformation.pretranslate(RHS.tail<3>());
            transformation.pretranslate(X_mean);
            /// Apply transformation
            //X = transformation*X;
            /// Re-apply mean
            //X.colwise() += X_mean;
            //Y.colwise() += X_mean;
            /// Return transformation
            return transformation.cast<float>();
        }
        /// @param Source (one 3D point per column)
        /// @param Target (one 3D point per column)
        /// @param Target normals (one 3D normal per column)
        /// @param Confidence weights
        template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
        inline Eigen::Affine3f point_to_plane(const Eigen::MatrixBase<Derived1>& X,
            const Eigen::MatrixBase<Derived2>& Yp,
            const Eigen::MatrixBase<Derived3>& Yn,
            const Eigen::MatrixBase<Derived4>& w,
            const Eigen::Affine3f& initialTransform)
        {
            return point_to_plane(X, Yp, Yn, w, Eigen::VectorXf::Zero(X.cols()), initialTransform);
        }
}
///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using ADMM/ALM/Penalty method
namespace SICP
{
    double corres_computaion_time;
    double optimization_time;
    double GetCorrespondenceComputationTime() { return corres_computaion_time; }
    double GetOptimizationTime() { return optimization_time; }
    std::string rmse_filename = "";
    int icp_convergence_iteration = 0;
    std::vector<Eigen::Matrix4f>iteration_transform;
    std::vector <std::pair<double, double>>distribution_time;
    std::vector<double>iterative_step_execution_time;


    struct Parameters
    {
        bool use_penalty = false; /// if use_penalty then penalty method else ADMM or ALM (see max_inner)
        float p = 1.0f;           /// p norm
        float mu = 10.0f;         /// penalty weight
        float alpha = 1.2f;       /// penalty increase factor
        float max_mu = 1e5f;      /// max penalty
        int max_icp = 100;        /// max ICP iteration
        int max_outer = 10;      /// max outer iteration
        int max_inner = 1;        /// max inner iteration. If max_inner=1 then ADMM else ALM
        float stop = 1e-5f;       /// stopping criteria
        const Eigen::Affine3f* initialTransform = nullptr;
        bool print_icpn = false;  /// (debug) print ICP iteration 
        int searchtype = 0;
        std::pair<double, double>error_pair = std::make_pair(1e-5f, 1e-5f);
        Eigen::Matrix4f groundTruthTransform = Eigen::Matrix4f::Identity();
        bool fall_Back = false;
        bool with_filter = false;
        bool curvature_filter = false;
        double diagonalLength = 1.0f;
    };
    /// Shrinkage operator (Automatic loop unrolling using template)
    template<unsigned int I>
    inline double shrinkage(double mu, double n, double p, double s)
    {
        return shrinkage<I - 1>(mu, n, p, 1.0 - (p / mu)*std::pow(n, p - 2.0)*std::pow(s, p - 1.0));
    }
    template<>
    inline double shrinkage<0>(double, double, double, double s) { return s; }
    /// 3D Shrinkage for point-to-point
    template<unsigned int I>
    inline void shrink(Eigen::Matrix3Xf& Q, double mu, double p)
    {
        double Ba = std::pow((2.0 / mu)*(1.0 - p), 1.0 / (2.0 - p));
        double ha = Ba + (p / mu)*std::pow(Ba, p - 1.0);
#pragma omp parallel for
        for (int i = 0; i < Q.cols(); ++i)
        {
            double n = Q.col(i).norm();
            double w = 0.0;
            if (n > ha) w = shrinkage<I>(mu, n, p, (Ba / n + 1.0) / 2.0);
            Q.col(i) *= w;
        }
    }
    /// 1D Shrinkage for point-to-plane
    template<unsigned int I>
    inline void shrink(Eigen::VectorXf& y, double mu, double p)
    {
        double Ba = std::pow((2.0 / mu)*(1.0 - p), 1.0 / (2.0 - p));
        double ha = Ba + (p / mu)*std::pow(Ba, p - 1.0);
#pragma omp parallel for
        for (int i = 0; i < y.rows(); ++i)
        {
            double n = std::abs(y(i));
            double s =  0.0;
            if (n > ha) s = shrinkage<I>(mu, n, p, (Ba / n + 1.0) / 2.0);
            y(i) *= s;
        }
    }
    float EnergyFunctionSummation(Eigen::VectorXf X, Eigen::VectorXf Y, Eigen::VectorXf Z, float mu)
    {
        float SumEnergy = 0.0f;
        for (int i = 0; i < X.size(); i++)
        {
            std::vector<float> z_values(X.data(), X.data() + X.rows() * X.cols());
            float aa =  X(i);
            float bb = Y(i) * Z(i);
            float cc = (mu / 2.0)* std::pow(Z(i), 2);
            SumEnergy = SumEnergy +  std::abs(Z(i)); // (aa + bb + cc);
        }
        return SumEnergy;
     }
    float ComputeOverallAlgorithmEnergy(float corres_sum, Eigen::VectorXf X,  Eigen::VectorXf Y, Eigen::VectorXf Z, float mu)
    {
        float SumEnergy = 0.0f;
        for (int i = 0; i < X.size(); i++)
        {
            std::vector<float> z_values(X.data(), X.data() + X.rows() * X.cols());
            float aa = X(i);
            float bb = Y(i) * Z(i);
            float cc = (mu / 2.0)* std::pow(Z(i), 2);
            SumEnergy = SumEnergy + (aa + bb + cc);
        }
       // SumEnergy += corres_sum;
        return SumEnergy;
    }
    /// Sparse ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2>
    inline Eigen::Affine3f point_to_point(Eigen::MatrixBase<Derived1>& X,
        Eigen::MatrixBase<Derived2>& Y, search::ISearch *searchStrategy, std::vector<std::pair<double, double>>&error_data,
        Parameters par = Parameters())
    {

        iteration_transform.clear();
        iteration_transform.shrink_to_fit();
        distribution_time.clear();
        distribution_time.shrink_to_fit();
        iterative_step_execution_time.clear();
        iterative_step_execution_time.shrink_to_fit();

        iteration_transform.reserve(par.max_icp);
        distribution_time.reserve(par.max_icp);
        iterative_step_execution_time.reserve(par.max_icp);

        Eigen::Affine3f mat = Eigen::Affine3f::Identity();
        /// Build kd-tree
        //  nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);

        /// Buffers

        Eigen::Affine3f transform = *par.initialTransform; // mat
        Eigen::Matrix3Xf Xo1 = transform * X;
        Eigen::Matrix3Xf Xo2 = transform * X;
        Eigen::VectorXf w = Eigen::VectorXf::Ones(X.cols());
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);

        // initialize strorage
        std::vector<Eigen::Vector3f>sX, tX, tN;
        sX.resize(X.cols());
        tX.resize(X.cols());

        Eigen::Vector3f dummyVertex(0.0f, 0.0f, 0.0f), dummyNormal(0.0f, 0.0f, 0.0f);

        char Timingfilename[150];
        sprintf(Timingfilename, "C:/WorkSpace/Regis_3D/Regis_3D/Timings_mls.txt");
        FILE *pFile;
        pFile = fopen(Timingfilename, "wb");
        auto starticp = std::chrono::high_resolution_clock::now();

        // initialize error_values(rot,tans)
        error_data.clear();
        error_data.reserve(par.max_icp);
        corres_computaion_time = 0.0;
        optimization_time = 0.0;

        auto startItr = std::chrono::high_resolution_clock::now();
        /// ICP
        int icp;
        for (icp = 0; icp < par.max_icp; ++icp)
        {
            std::cout << "current icp iteration:" << icp << std::endl;
            auto starticp = std::chrono::high_resolution_clock::now();
            float distance = 0.0f;
            /*if (par.print_icpn)
            std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;*/
            auto startcorres = std::chrono::high_resolution_clock::now();
            /// Find closest point
#pragma omp parallel for
            for (int i = 0; i < X.cols(); ++i)
            {
                // search for closest point using sparseICP Kdtree
                //searchStrategy->findPoint()
                Eigen::Vector3f px = transform * X.col(i);
                if (par.searchtype == 0)
                {
                    // unsigned id = searchStrategy->findIndices(POINT_EIGEN_CAST(px, N.col(i)));
                    unsigned id = kdtree.closest(px.data());
                    if (id < Y.cols())
                    {
                        /* Qp.col(i) = Y.col(id);
                        Qn.col(i) = N.col(id);*/
                        // w(i) = 0;
                        sX[i] = X.col(i);
                        tX[i] = Y.col(id);

                    }
                    else
                    {
                        sX[i] = dummyVertex;
                        tX[i] = dummyVertex;

                    }
                }
                // excutes when the correspondence searchtype is WDP, WPP or MLS
                else
                {
                    PointRGBNormalType Corresp;
                    Eigen::Vector3f tempNormal(0.0f, 0.0f, 1.0f);
                    PointRGBNormalType sourcePt = POINT_EIGEN_CAST(X.col(i), tempNormal);  // source normals not reqd
                    auto startItr = std::chrono::high_resolution_clock::now();
                    searchStrategy->findPoint(sourcePt, transform, Corresp);     //TODO Validity check for correspondence
                    auto finishItr = std::chrono::high_resolution_clock::now();
                    double executeTime = std::chrono::duration_cast<
                        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
                    executeTime = executeTime / double(1000);
                    // MY_LOG << "Time:" << executeTime << "secs" << std::endl;
                    if (isPointFinite(Corresp) == true)
                    {
                        /* Qp.col(i) = EIGEN_POINT_CAST(Corresp);
                        Qn.col(i) = EIGEN_NORMAL_CAST(Corresp);*/

                        sX[i] = X.col(i);
                        tX[i] = Corresp.getVector3fMap();


                    }
                    else
                    {
                        sX[i] = dummyVertex;
                        tX[i] = dummyVertex;

                        continue;
                    }

                }
            }
            std::vector<Eigen::Vector3f>isX, itX, itN;
            isX.reserve(sX.size());
            itX.reserve(sX.size());

            for (int i = 0; i < sX.size(); i++)
            {
                if (sX[i] != dummyVertex)
                {
                    isX.push_back(sX[i]);
                    itX.push_back(tX[i]);

                }
            }

            // collect the refined source, target correspondences
            Eigen::Matrix3Xf refinedX = Eigen::Matrix3Xf::Zero(3, isX.size());
            Eigen::Matrix3Xf Qp = Eigen::Matrix3Xf::Zero(3, isX.size());
            Eigen::Matrix3Xf Qn = Eigen::Matrix3Xf::Zero(3, isX.size());
            Eigen::Matrix3Xf Z = Eigen::Matrix3Xf::Zero(3, isX.size());
            Eigen::Matrix3Xf C = Eigen::Matrix3Xf::Zero(3, isX.size());
            Eigen::VectorXf w = Eigen::VectorXf::Ones(isX.size());
#pragma omp parallel for 
            for (int i = 0; i < isX.size(); i++)
            {
                refinedX.col(i) = isX[i];
                Qp.col(i) = itX[i];

            }
            auto finishcorres = std::chrono::high_resolution_clock::now();
            double executecorres = std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(finishcorres - startcorres).count();
            executecorres = executecorres / double(1000);
            corres_computaion_time += executecorres;
            CpuTimeProfiler  cpuTimeO;
            auto startopto = std::chrono::high_resolution_clock::now();
            /// Computer rotation and translation
            double mu = par.mu;
            for (int outer = 0; outer < par.max_outer; ++outer)
            {
                double dual = 0.0;
                for (int inner = 0; inner < par.max_inner; ++inner)
                {
                    Eigen::Matrix3Xf transformedX = transform * refinedX;
                    //Z update (shrinkage)
                    Z = transformedX - Qp + C / mu;
                    shrink<3>(Z, mu, par.p);
                    // Rotation and translation update
                    Eigen::Matrix3Xf U = Qp + Z - C / mu;
                    transform = RigidMotionEstimator::point_to_point(refinedX, U, w, transform);
                    // Stopping criteria
                    dual = (transform * X - Xo1).colwise().norm().maxCoeff();
                    Xo1 = transform * X;
                    if (dual < par.stop) break;
                }
                // C update (lagrange multipliers)
                Eigen::Matrix3Xf P = transform * refinedX - Qp - Z;
                if (!par.use_penalty) C.noalias() += mu*P;
                // mu update (penalty)
                if (mu < par.max_mu) mu *= par.alpha;
                // Stopping criteria
                double primal = P.colwise().norm().maxCoeff();
                //std::cout << primal << std::endl;
                if (primal < par.stop && dual < par.stop) break;
            }
            double opt_time = (cpuTimeO.GetElapsedMilli()) / 1000.0;

            auto endopto = std::chrono::high_resolution_clock::now();
            double executeopto = std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(endopto - startopto).count();
            executeopto = executeopto / double(1000);
            optimization_time += executeopto;
            // Stopping criteria
            double stop = (transform * X - Xo2).colwise().norm().maxCoeff();
            // std::cout << stop << std::endl;
            Xo2 = transform * X;
            auto finishItr = std::chrono::high_resolution_clock::now();
            double executetest = std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
            executetest = executetest / double(1000);
            std::pair<double, double>error_pair = std::make_pair(stop, executetest);
            error_data.push_back(error_pair);  // temporary usage
            auto finishicp = std::chrono::high_resolution_clock::now();
            double executeicp = std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(finishicp - starticp).count();
            executeicp = executeicp / double(1000);
            iterative_step_execution_time.push_back(executeicp);
            iteration_transform.push_back(transform.matrix()); // collect transformation after every iteration 
            distribution_time.push_back(std::pair<double, double>(executecorres, executeopto));

            if (stop < par.stop) break;
        }
        auto finishicp = std::chrono::high_resolution_clock::now();
        double executeTime = std::chrono::duration_cast<
            std::chrono::duration<double, std::milli>>(finishicp - starticp).count();
        executeTime = executeTime / double(1000);
        std::cout << " icp Time:" << executeTime << "secs" << std::endl;
        return transform;

    }

    ////////////////////////***********sparse ICP with point to plane************///////////////////////////////
    /// Sparse ICP with point to plane
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2, typename Derived3>
    Eigen::Affine3f point_to_plane(Eigen::MatrixBase<Derived1>& X, Eigen::MatrixBase<Derived2>& Y, Eigen::MatrixBase<Derived3>& N,
        search::ISearch *searchStrategy, std::vector<std::pair<double, double>>&error_data, Eigen::MatrixBase<Derived3>&Ns,
        Parameters par = Parameters())
    {
        // std:: string  iterationTransform = "C:/WorkSpace/Regis_3D/Regis_3D/RecentEvaluationResult/ itr_transform_mlsf.txt";

        iteration_transform.clear();
        iteration_transform.shrink_to_fit();
        distribution_time.clear();
        distribution_time.shrink_to_fit();
        iterative_step_execution_time.clear();
        iterative_step_execution_time.shrink_to_fit();

        iteration_transform.reserve(par.max_icp);
        distribution_time.reserve(par.max_icp);
        iterative_step_execution_time.reserve(par.max_icp);

        Eigen::Affine3f mat = Eigen::Affine3f::Identity();
        /// Build kd-tree
      //  nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);

        /// Buffers
        Eigen::Matrix4f transform_inner = Eigen::Matrix4f::Identity();
        Eigen::Affine3f transform = *par.initialTransform; //*par.initialTransform; // mat
        Eigen::Matrix3Xf Xo1 = transform * X;
        Eigen::Matrix3Xf Xo2 = transform * X;
        Eigen::VectorXf w = Eigen::VectorXf::Ones(X.cols());
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);

        // initialize strorage
        std::vector<Eigen::Vector3f>sX, tX, tN, sN;
        sX.resize(X.cols());
        tX.resize(X.cols());
        tN.resize(X.cols());
        sN.resize(X.cols());
        Eigen::Vector3f dummyVertex(0.0f, 0.0f, 0.0f), dummyNormal(0.0f, 0.0f, 0.0f);

        char Timingfilename[150], corresp_source[200], corresp_tar[200];
        /*sprintf(Timingfilename, "C:/WorkSpace/Regis_3D/Regis_3D/stop_criteria.txt");
        FILE *pFile;
        pFile = fopen(Timingfilename, "wb");*/
        auto startItr = std::chrono::high_resolution_clock::now();

        // initialize error_values(rot,tans)
        error_data.clear();
        error_data.reserve(par.max_icp);
        corres_computaion_time = 0.0;
        optimization_time = 0.0;
        /// ICP
        for (int icp = 0; icp < par.max_icp; ++icp)
        {
            CpuTimeProfiler cpuTimeR;
            auto starticp = std::chrono::high_resolution_clock::now();
            std::cout << "current icp iteration:" << icp << std::endl;
            float distance = 0.0f;
            /*if (par.print_icpn)
                std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;*/

            int false_mls_count = 0;
            int true_correspondence = 0;
            CpuTimeProfiler cpuTimeC;
            auto startcorres = std::chrono::high_resolution_clock::now();
            /// Find closest point
//#pragma omp parallel for 
            for (int i = 0; i < X.cols(); ++i)
            {
                // std::cout << "i:" << i << std::endl;
                 // search for closest point using sparseICP Kdtree
                 //searchStrategy->findPoint()
                Eigen::Vector3f px = transform * X.col(i);

                if (par.searchtype == 0)
                {
                    // unsigned id = searchStrategy->findIndices(POINT_EIGEN_CAST(px, N.col(i)));
                    unsigned id = kdtree.closest(px.data());
                    if (id < Y.cols())
                    {
                        /* Qp.col(i) = Y.col(id);
                        Qn.col(i) = N.col(id);*/
                        // w(i) = 0;
                        if (par.with_filter == true)
                        {
                            Eigen::Vector3f Ns_t = transform.linear() * Ns.col(i);
                            if (Ns_t.dot(N.col(id)) >= 0.7f)
                            {
                                if (par.curvature_filter == true)
                                {
                                    cKDTreeSearch *ptr = dynamic_cast<cKDTreeSearch*>(searchStrategy);
                                    Eigen::Vector3f eig_val_source;
                                    Eigen::Vector3f eig_val_tar;
                                    Eigen::Vector3f ptA = X.col(i);
                                    Eigen::Vector3f ptB = Y.col(id);
                                    Eigen::VectorXf pptA = Eigen::VectorXf::Zero(6);
                                    Eigen::VectorXf pptB = Eigen::VectorXf::Zero(6);
                                    pptA.head<3>() = ptA;
                                    pptA.tail<3>() = Ns.col(i);
                                    pptB.head<3>() = ptB;
                                    pptB.tail<3>() = N.col(id);
                                    ptr->getEigenValuesForPoint(pptA, id, eig_val_source, eig_val_tar);
                                    float comp1 = eig_val_source(1) / eig_val_tar(1);
                                    float comp2 = eig_val_source(2) / eig_val_tar(2);
                                    if ((0.5f <= comp1 && comp1 <= 1.5f) && (0.5f <= comp2 && comp2 <= 1.5f))
                                    {
                                        sX[i] = X.col(i);
                                        sN[i] = Ns.col(i);
                                        tX[i] = Y.col(id);
                                        tN[i] = N.col(id);
                                    }
                                    else
                                    {
                                        sX[i] = dummyVertex;
                                        tX[i] = dummyVertex;
                                        tN[i] = dummyNormal;
                                        sN[i] = dummyNormal;
                                    }
                                }
                                else
                                {
                                    sX[i] = X.col(i);
                                    sN[i] = Ns.col(i);
                                    tX[i] = Y.col(id);
                                    tN[i] = N.col(id);
                                }
                            }
                            else
                            {
                                sX[i] = dummyVertex;
                                tX[i] = dummyVertex;
                                tN[i] = dummyNormal;
                                sN[i] = dummyNormal;
                            }
                        }
                        else
                        {
                            sX[i] = X.col(i);
                            sN[i] = Ns.col(i);
                            tX[i] = Y.col(id);
                            tN[i] = N.col(id);
                        }
                    }
                    else
                    {
                        sX[i] = dummyVertex;
                        tX[i] = dummyVertex;
                        tN[i] = dummyNormal;
                        sN[i] = dummyNormal;
                    }
                }
                // excutes when the correspondence searchtype is WDP, WPP or MLS
                else
                {
                    PointRGBNormalType Corresp;
                    // Eigen::Vector3f tempNormal = transform * Ns.col(i); // (0.0f, 0.0f, 1.0f);
                    PointRGBNormalType sourcePt = POINT_EIGEN_CAST(X.col(i), Ns.col(i));  // source normals not reqd

                    searchStrategy->findPoint(sourcePt, transform, Corresp);     //TODO Validity check for correspondence
                  /*  auto finishItr = std::chrono::high_resolution_clock::now();
                    double executeTime = std::chrono::duration_cast<
                        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
                    executeTime = executeTime / double(1000);*/
                    // MY_LOG << "Time:" << executeTime << "secs" << std::endl;
                    if (isPointFinite(Corresp) == true)
                    {
                        /* Qp.col(i) = EIGEN_POINT_CAST(Corresp);
                         Qn.col(i) = EIGEN_NORMAL_CAST(Corresp);*/

                        sX[i] = X.col(i);
                        tX[i] = Corresp.getVector3fMap();
                        tN[i] = Corresp.getNormalVector3fMap();
                        sN[i] = Ns.col(i);
                        true_correspondence++;
                        /*if (Corresp.getVector3fMap() == dummyVertex)
                        {
                           std::cout << " true"<< i << std::endl;
                        }
                       */
                    }
                    else
                    {
                        sX[i] = dummyVertex;
                        tX[i] = dummyVertex;
                        tN[i] = dummyNormal;
                        sN[i] = dummyNormal;
                        false_mls_count++;
                        continue;
                    }

                }
            }
            if (true == par.fall_Back)
            {
                if (false_mls_count > 0.99 * X.cols() || false_mls_count >= X.cols() - 50)  //0.99 for 1000 sample && -20
                {
                    //#ifdef LOGDATA
                    //                error_log("Inside Kdtree correspondence when iteration = %d and false count = %d\n",icp, false_mls_count);
                    //#endif
//#pragma omp parallel for 
                    for (int i = 0; i < X.cols(); ++i)
                    {
                        // std::cout << "i:" << i << std::endl;
                        // search for closest point using sparseICP Kdtree
                        //searchStrategy->findPoint()
                        Eigen::Vector3f px = transform * X.col(i);
                        // unsigned id = searchStrategy->findIndices(POINT_EIGEN_CAST(px, N.col(i)));
                        unsigned id = kdtree.closest(px.data());
                        if (id < Y.cols())
                        {
                            if (par.with_filter == true)
                            {
                                Eigen::Vector3f Ns_t = transform.linear() * Ns.col(i);
                                if (Ns_t.dot(N.col(id)) >= 0.7f)
                                {
                                    sX[i] = X.col(i);
                                    sN[i] = Ns.col(i);
                                    tX[i] = Y.col(id);
                                    tN[i] = N.col(id);
                                }
                                else
                                {
                                    sX[i] = dummyVertex;
                                    tX[i] = dummyVertex;
                                    tN[i] = dummyNormal;
                                    sN[i] = dummyNormal;
                                }
                            }
                            else
                            {
                                sX[i] = X.col(i);
                                sN[i] = Ns.col(i);
                                tX[i] = Y.col(id);
                                tN[i] = N.col(id);
                            }
                        }
                        else
                        {
                            sX[i] = dummyVertex;
                            tX[i] = dummyVertex;
                            tN[i] = dummyNormal;
                            sN[i] = dummyNormal;
                        }

                    }

                }
            }
            std::vector<Eigen::Vector3f>isX, itX, itN, isN;
            isX.reserve(sX.size());
            itX.reserve(sX.size());
            itN.reserve(sX.size());
            isN.reserve(sX.size());
            for (int i = 0; i < sX.size(); i++)
            {
                if (sX[i](0) != dummyVertex(0) && sX[i](1) != dummyVertex(1) && sX[i](2) != dummyVertex(2))
                {
                    if (tX[i](0) != dummyVertex(0) && tX[i](1) != dummyVertex(1) && tX[i](2) != dummyVertex(2))
                    {
                        isX.push_back(sX[i]);
                        itX.push_back(tX[i]);
                        itN.push_back(tN[i]);
                        isN.push_back(sN[i]);
                    }
                    /* else
                         continue;

                 }
                 else
                     continue;*/
                }
            }

            if (itX.size() == 0)
            {
                return transform;
            }
            // collect the refined source, target correspondences
            Eigen::Matrix3Xf refinedX = Eigen::Matrix3Xf::Zero(3, isX.size());
            Eigen::Matrix3Xf Qp = Eigen::Matrix3Xf::Zero(3, isX.size());
            Eigen::Matrix3Xf Qn = Eigen::Matrix3Xf::Zero(3, isX.size());
            Eigen::VectorXf Z = Eigen::VectorXf::Zero(isX.size());
            Eigen::VectorXf C = Eigen::VectorXf::Zero(isX.size());
            Eigen::VectorXf w = Eigen::VectorXf::Ones(isX.size());
           
            //#pragma omp parallel for
            for (int i = 0; i < isX.size(); i++)
            {
                refinedX.col(i) = isX[i];
                Qp.col(i) = itX[i];
                Qn.col(i) = itN[i];
            }
            double cor_time = cpuTimeC.GetElapsedSecs();
            auto finishcorres = std::chrono::high_resolution_clock::now();
            double executecorres = std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(finishcorres - startcorres).count();
            executecorres = executecorres / double(1000);
            corres_computaion_time += executecorres;
            //writeCorrespondence
            //sprintf(corresp_source, "C:/WorkSpace/Regis_3D/Regis_3D/corresp_source_%d", icp);
            //sprintf(corresp_tar, "C:/WorkSpace/Regis_3D/Regis_3D/corresp_target_%d", icp);

            //iospace::WriteCorrespondenceAsPointCloud(corresp_tar, itX, itN);
            //std::cout << "target Correspondence:" << itX.size() << std::endl;

            //std::vector<Eigen::Vector3f>t_isX(isX.size()), t_isN(isX.size());
            ////#pragma omp parallel for 
            //for (int i = 0; i < isX.size(); i++)
            //{
            //    t_isX[i] = transform * isX[i];
            //    t_isN[i] = transform.linear() * isN[i];
            //}
            //iospace::WriteCorrespondenceAsPointCloud(corresp_source, t_isX, t_isN);
            //// fprintf(pFile, " %f\r\n", stop);

            //t_isX.clear(); t_isN.clear();

            CpuTimeProfiler  cpuTimeO;
            auto startopto = std::chrono::high_resolution_clock::now();
            /// Compute rotation and translation optimization loop
            float mu = par.mu;
            Eigen::Matrix4f transform_B = Eigen::Matrix4f::Identity();
            Eigen::VectorXf P = Eigen::VectorXf::Zero(isX.size());
            float sume = 0.0f;
       
            for (int outer = 0; outer <  par.max_outer; ++outer)
            {
                float dual = 0.0;
                Eigen::Matrix4f transform_A = Eigen::Matrix4f::Identity();
               
                for (int inner = 0; inner < par.max_inner; ++inner)
                {
                    /// Z update (shrinkage)
                    Eigen::Matrix3Xf transformedQn = Qn;  //transform.linear() *
                    Eigen::Matrix3Xf transformedX = transform * refinedX;
                    Z = (transformedQn.array()*(transformedX - Qp).array()).colwise().sum().transpose() + C.array() / mu; //
                    shrink<3>(Z, mu, par.p);
                    /// Rotation and translation update
                    Eigen::VectorXf U = Z - C / mu;
                    transform = RigidMotionEstimator::point_to_plane(refinedX, Qp, Qn, w, U, transform, distance);
                    /// Stopping criteria
                   // dual = (transform * refinedX - Qp).colwise().norm().maxCoeff();
                   dual = (transform * X - Xo1).colwise().norm().maxCoeff();
                   // dual = metric::ComputeRMSE(transform * X, Xo1, par.diagonalLength);
                    Xo1 = transform * X;
                    if (dual <= par.stop) break;
                    /*std::pair<double, double>error_pair = tool::GetTransformationError(transform_A, transform.matrix());
                    transform_A = transform.matrix();
                    if (error_pair.first < par.error_pair.first && error_pair.second < par.error_pair.second)
                        break;*/

                }
                /// C update (lagrange multipliers)
                P = ((Qn).array()*(transform * refinedX - Qp).array()).colwise().sum().transpose() - Z.array(); //transform.linear() *
                if (!par.use_penalty) C.noalias() += mu*P;
                /// mu update (penalty)
                if (mu < par.max_mu)
                    mu *= par.alpha;

                /// Stopping criteria
                float primal = P.array().abs().maxCoeff();
                if (primal <= par.stop && dual <= par.stop)
                    break;
           
            }
           
            double opt_time = (cpuTimeO.GetElapsedMilli()) / 1000.0;

            auto endopto = std::chrono::high_resolution_clock::now();
            double executeopto = std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(endopto - startopto).count();
            executeopto = executeopto / double(1000);
            optimization_time += executeopto;

            /// Stopping criteria
            float stop = (transform * X - Xo2).colwise().norm().maxCoeff();
           // float stop =  metric::ComputeRMSE(transform * X, Xo2, par.diagonalLength);
            //std::cout << "stopping criteria:" << stop << endl;
            Xo2 = transform * X;
            auto finishItr = std::chrono::high_resolution_clock::now();
            double executetest = std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
            executetest = executetest / double(1000);
            std::pair<double, double>error_pair = std::make_pair(stop, executetest);
            error_data.push_back(error_pair);  // temporary usage
            auto finishicp = std::chrono::high_resolution_clock::now();
            double executeicp = std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(finishicp - starticp).count();
            executeicp = executeicp / double(1000);
            iterative_step_execution_time.push_back(executeicp);
            iteration_transform.push_back(transform.matrix()); // collect transformation after every iteration 
            distribution_time.push_back(std::pair<double, double>(executecorres, executeopto));
            if (stop <= par.stop)
                break;
            //std::pair<double, double>error_pair = tool::GetTransformationError(transform_inner, transform.matrix());
            //transform_inner = transform.matrix();
            //error_data.push_back(error_pair);  // temporary usage
            //if (error_pair.first < par.error_pair.first && error_pair.second < par.error_pair.second)
            //    break;

           // error_data.push_back( tool::GetTransformationError(par.groundTruthTransform, transform.matrix()));

        }


        // std::cout << " icp Time:" << executeTime << "secs" << std::endl;
#ifdef LOGDATA
       // error_log("icp execution time:%f\n", executeTime);
#endif
       // fclose(pFile);
       // tool::writeTransformationMatrix(iterationTransform, iteration_transform);
        return transform;
    }

    //    template <class Derived1>
    //    Eigen::Affine3f pt_to_plane(Eigen::MatrixBase<Derived1>& X, CRegister3D_FPFH &regis, Parameters par = Parameters())
    //    {
    //
    //        Eigen::Affine3f mat = Eigen::Affine3f::Identity();
    //        /// Build kd-tree
    //       // nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
    //
    //        /// Buffers
    //     
    //        Eigen::Affine3f transform = mat; // mat *par.initialTransform
    //        Eigen::Matrix3Xf Xo1 = transform *  X;       // transform *
    //        Eigen::Matrix3Xf Xo2 = transform * X;         // transform *
    //      
    //        bool status;
    //        Eigen::Matrix3f rot, scale;
    //        transform.computeRotationScaling(&rot, &scale);
    //        Eigen::AngleAxisf newAxisAngle(rot);
    //        static float rotAngle = newAxisAngle.angle();
    //        Eigen::Matrix3Xf Xtransformed = transform * X;
    //        static Eigen::Affine3f initTransform = Eigen::Affine3f::Identity();
    //        initTransform.rotate(Eigen::AngleAxisf(M_PI / 9.0, Eigen::Vector3f::UnitZ()));
    //        initTransform.translation() << 0.0, 5.0, 0.0;
    //        static Eigen::Matrix3f initRot = Eigen::Matrix3f::Identity();
    //        static Eigen::Vector3f initTrans = transform.translation();
    //        /// ICP
    //        int max_icp = par.max_icp;
    //        static int viewItr = 0;
    //        char filename[50], iteration[50];
    //        FILE *pFile;
    //        FILE *nFile;
    //        sprintf(filename, "distancePlot%d.txt", viewItr);
    //        sprintf(iteration, "IterationNo%d.txt", viewItr);
    //        pFile = fopen(filename, "wb");
    //        nFile = fopen(iteration, "wb");
    //        std::vector<Eigen::Vector3f>sX, tX, tN;
    //        //#pragma omp parallel for
    //        for (int icp = 0; icp < max_icp; ++icp)
    //        {
    //            std::cout << " current icp:" << icp << endl;
    //            float distance = 0.0f;
    //
    //            /* if (par.print_icpn)
    //                 std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;*/
    //
    //                 /// Find correspondence point
    //            float sumWeight = 0;
    //            int count = 0;
    ////#pragma omp parallel for 
    //            for (int i = 0; i < X.cols(); ++i)
    //            {
    //                Eigen::Vector3f pData = transform * X.col(i);
    //                Eigen::Vector3f ptCorresps, normalCorresps;
    //                auto start = std::chrono::high_resolution_clock::now();
    //                status = regis.findCorrespondenceUsingGridCorrespondences(pData, transform, ptCorresps, normalCorresps);
    //                if (status == false)
    //                {
    //                  /*   refinedX.col(i) = X.col(i);
    //                    Qp.col(i).setZero();
    //                    Qn.col(i) = Eigen::Vector3f::UnitX();*/
    //                    // w(i) = 0;
    //                }
    //                else
    //                {
    //                    // refinedX.col(i) = X.col(i); // pData;
    //                    //Qp.col(i) = ptCorresps;
    //                    //Qn.col(i) = normalCorresps;
    //                    //   refinedX.conservativeResize(NoChange, refinedX.cols() + 1);
    //                    //   Qp.conservativeResize(NoChange, Qp.cols() + 1);
    //                    //   Qn.conservativeResize(NoChange, Qn.cols() + 1);
    //                    //   count++;
    //                       // w(i) = 1;
    //                   // sumWeight += w(i);
    //                    sX.push_back(X.col(i));
    //                    tX.push_back(ptCorresps);
    //                    tN.push_back(normalCorresps);
    //                }
    //                // std::cout << "i:" << i << endl;
    //            }
    //            Eigen::Matrix3Xf refinedX = Eigen::Matrix3Xf::Zero(3, sX.size());
    //            Eigen::Matrix3Xf Qp = Eigen::Matrix3Xf::Zero(3, sX.size());
    //            Eigen::Matrix3Xf Qn = Eigen::Matrix3Xf::Zero(3, sX.size());
    //            Eigen::VectorXf Z = Eigen::VectorXf::Zero(sX.size());
    //            Eigen::VectorXf C = Eigen::VectorXf::Zero(sX.size());
    //            //Eigen::Matrix3Xf Xo1 = transform *  X;       // transform *
    //            //Eigen::Matrix3Xf Xo2 = transform * X;         // transform *
    //            Eigen::VectorXf w = Eigen::VectorXf::Ones(sX.size());
    //            for (int i = 0; i < sX.size(); i++)
    //            {
    //                refinedX.col(i) = sX[i];
    //                Qp.col(i) = tX[i];
    //                Qn.col(i) = tN[i];
    //            }
    //            sX.clear(); tX.clear(); tN.clear();
    //            /*  refinedX.conservativeResize(NoChange, refinedX.cols() - 1);
    //              Qp.conservativeResize(NoChange, Qp.cols() - 1);
    //              Qn.conservativeResize(NoChange, Qn.cols() - 1);
    //              Z.conservativeResize(Qp.cols() , NoChange);
    //              C.conservativeResize(Qp.cols(), NoChange);
    //              w.conservativeResize(Qp.cols(), NoChange);*/
    //              //#pragma omp parallel for
    //
    //                         /// Compute rotation and translation
    //            float mu = par.mu;
    //            for (int outer = 0; outer < par.max_outer; ++outer)
    //            {
    //                float dual = 0.0;
    //                for (int inner = 0; inner < par.max_inner; ++inner)
    //                {
    //                    /// Z update (shrinkage)
    //                    Eigen::Matrix3Xf transformedQn = Qn;   
    //                    Eigen::Matrix3Xf transformedX = transform * refinedX; //transform * X;
    //                    Z = (transformedQn.array()*(transformedX - Qp).array()).colwise().sum().transpose() + C.array() / mu;
    //                    shrink<3>(Z, mu, par.p);
    //                    /// Rotation and translation update
    //                    Eigen::VectorXf U = Z - C / mu;
    //                    transform = RigidMotionEstimator::point_to_plane(refinedX, Qp, Qn, w, U, transform, distance);
    //                    /// Stopping criteria
    //                    dual = (transform * X - Xo1).colwise().norm().maxCoeff();
    //                    Xo1 = transform * X;
    //                    if (dual < par.stop) break;
    //                }
    //                /// C update (lagrange multipliers)
    //                Eigen::VectorXf P = ((transform.linear() * Qn).array()*(transform * refinedX - Qp).array()).colwise().sum().transpose() - Z.array();
    //                if (!par.use_penalty) C.noalias() += mu*P;
    //                /// mu update (penalty)
    //                if (mu < par.max_mu)
    //                    mu *= par.alpha;
    //                /// Stopping criteria
    //                float primal = P.array().abs().maxCoeff();
    //                if (primal < par.stop && dual < par.stop)
    //                    break;
    //            }
    //            Eigen::Affine3f tempMat;
    //            tempMat.matrix() = regis.getTransformationMatrix(1);
    //            Eigen::Matrix3f rot = (transform  * tempMat).linear();
    //            Eigen::Matrix3f rr = initTransform.linear();
    //            float diffangle = ComputeDifferenceofRotation(rr, rot);
    //             initRot = rot;
    //            Eigen::AngleAxisf newAxisAngle(rot);
    //            float angle = newAxisAngle.angle();
    //            // Eigen::Vector3f axis = newAxisAngle.axis();
    //            // float diffangle =  0.34 - angle;
    //            rotAngle = angle;
    //
    //            Eigen::Vector3f transNew = (transform  * tempMat).translation();
    //            Eigen::Vector3f vector = initTransform.translation();
    //            Eigen::Matrix3Xf Xtransformed = transform * refinedX;
    //            float Transdistance = computeDifferenceofTranslationVector(transNew, vector);
    //            initTrans = transNew;
    //            float Eqdistance = ComputeDistancebetweenClouds(Xtransformed, Qp);
    //            fprintf(pFile, " %f\r\n", abs(Transdistance));
    //            fprintf(nFile, " %d\r\n", icp);
    //            /// Stopping criteria
    //            float stop = (transform * X - Xo2).colwise().norm().maxCoeff();
    //            std::cout << "stop:" << stop << endl;
    //            Xo2 = transform * X;
    //            if (stop < par.stop)
    //                break;
    //
    //        }
    //        fclose(pFile);
    //        fclose(nFile);
    //        viewItr++;
    //
    //        /* FILE *nFile = fopen("plot.txt", "wb");
    //         if (NULL == nFile)
    //         {
    //             exit(1);
    //         }
    //         int size = rotationMap.size();
    //         for (int i = 0; i < size; i++)
    //         {
    //             int itr = rotationMap[itr].first;
    //             float j = rotationMap[itr].second;
    //             fprintf(nFile, " %d:%f\r\n", itr, j);
    //         }
    //         fclose(nFile);*/
    //        return transform;
    //    }

}




