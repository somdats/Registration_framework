#pragma once
#include"Datatypes.h"
#include"Config.h"

namespace surface
{
    // forward declaration as other class cNurbsCurve is referenced within class cNurbsSurface
    class cNurbsCurve;
    enum algorithm
    {
        ASDM_2D = 0,
        APDM_2D
    };
    typedef pcl::PolygonMesh mesh;
    typedef  pcl::on_nurbs::NurbsDataSurface data;
    typedef pcl::on_nurbs::NurbsDataCurve2d cv_data;
    typedef pcl::on_nurbs::FittingCurve2dAPDM::FitParameter fit_parameter_curve;
    typedef Eigen::Vector2d vec2d;
    struct REG3D_API Ray_Intersect
    {
        Ray_Intersect(const ON_NurbsSurface &nrb) :nurbs(nrb) {};
        int a;
        const ON_NurbsSurface &nurbs;
        Eigen::Vector3d xyz;
        double x0, x1, y0, y1, w,h;

    };

    class REG3D_API cNurbsSurface
    {
    public:
        cNurbsSurface(){}
        cNurbsSurface(const CloudWithoutType &in_cloud, unsigned order = 3, unsigned refinement = 5, unsigned iterations = 10, unsigned mesh_resolution = 256) :
            order_(order),
            refinement_(refinement),
            iterations_(iterations),
            mesh_resolution_(mesh_resolution),
            input_cloud(new pcl::PointCloud <PointNormalType>)
        {
            pcl::fromPCLPointCloud2(*in_cloud, *input_cloud);
        }
       
        void FitNurbsSurfaceToData();
        void ComputeMeshFromSurface(pcl::on_nurbs::FittingSurface& fit);
        void PointCloud2Vector3d();
        std::unique_ptr<CloudWithoutType> GetFittedPointCloud();
       
        static  std::unique_ptr<ON_NurbsSurface> TransformControlPointsOfNurbsSurface(const ON_NurbsSurface &nurb, const Eigen::Matrix4d &transform);
        
        static double RayIntersect(const std::vector<double > &x, std::vector<double > &grad, void* f_data);
        static double OptimizeParameter(Ray_Intersect &r_it, double stop_threshold,
            std::vector<Eigen::Vector2d> &optim_parameter);
        static bool EvaluateParameterForCoordinate(std::vector<Eigen::Vector2d> &optim_parameter, const ON_NurbsSurface& ns, 
            const ON_NurbsCurve& nc, Eigen::VectorXd &pt, Eigen::Vector2d &pt_param);
        static Eigen::VectorXd ComputeClosetPointOnNurbSurface(const Eigen::VectorXd &pt_interest, const ON_NurbsSurface &ns,
            const ON_NurbsCurve &nc, double stop_threshold);
        friend bool TrimInputSurfaceUsingCurveBoundary( vec2d & vec_2d, const ON_NurbsSurface& ns, const ON_NurbsCurve& nc);
        data GetNurbSurfaceData();
       



    private:
        CloudWithNormalPtr input_cloud;
        unsigned order_;
        unsigned refinement_;
        unsigned iterations_;
        unsigned mesh_resolution_;
        data data_;
        mesh mesh_;
        ON_NurbsSurface nurbs_;
        const double interior_smoothness_ = 0.2;
        const double interior_weight_ = 1.0;
        const double boundary_smoothness_ = 0.2;
        const double boundary_weight_ = 0.0;
        pcl::on_nurbs::FittingSurface::Parameter p;

    };

    class REG3D_API cNurbsCurve
    {
    public:
        cNurbsCurve(){}
        cNurbsCurve(surface::algorithm a, data &curve_data, int max_cp = 200, int cp_iteration = 3, double cp_accuracy_threshold = 5e-2, 
            double cp_accuracy_stop = 1e-3, int fit_iteration = 1, int curve_order = 3)
            :curve(InitializeCurveData(curve_data)),
            curve_params(IntializeCurvaParameter(max_cp, cp_iteration, cp_accuracy_threshold, cp_accuracy_stop, fit_iteration, curve_order)),
            fitting_algorithm(a)
        {

        }
       cv_data InitializeCurveData(data &n_data);
       fit_parameter_curve IntializeCurvaParameter(int max_cp, int cp_iteration, double cp_accuracy_threshold, double cp_accuracy_stop, 
           int fit_iteration, int curve_order);
       void FitCurveToData();
       static  std::unique_ptr<ON_NurbsCurve> TransformControlPointsOfNurbsCurve(const ON_NurbsCurve &curve, const Eigen::Matrix4d &transform);
       friend bool TrimInputSurfaceUsingCurveBoundary(vec2d & vec_2d, const ON_NurbsSurface& ns, const ON_NurbsCurve& nc);
    private:
        fit_parameter_curve   curve_params;
        cv_data curve;
        ON_NurbsCurve nurbs_curve;
        algorithm fitting_algorithm;
        int order;
    };
  
}
