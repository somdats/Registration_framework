#include"pch.h"
#include"NurbsSurface.h"
#include"Common.h"
#include"levmar.h"



using namespace surface;
using namespace nlopt;


void  cNurbsSurface::PointCloud2Vector3d()
{
    for (unsigned i = 0; i < input_cloud->size(); i++)
    {
        PointNormalType &p = input_cloud->at(i);
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
            data_.interior.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }
}

void  cNurbsSurface::FitNurbsSurfaceToData()
{
    // initialize
    printf("  surface fitting ...\n");
    nurbs_ = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order_, &data_);
    pcl::on_nurbs::FittingSurface fit(&data_, nurbs_);
   
    // surface refinement
    for (unsigned i = 0; i < refinement_; i++)
    {
        fit.refine(0);
        fit.refine(1);
        fit.assemble(p);
        fit.solve();
   
    }
 
    nurbs_ = fit.m_nurbs;
   
}

// call FitNurbsSurfaceToData() before getting the fitted pointcloud without trimming
std::unique_ptr<CloudWithoutType> cNurbsSurface::GetFittedPointCloud()
{
    CloudPtr mesh_cloud(new pcl::PointCloud<PointType>);
    std::vector<pcl::Vertices> mesh_vertices;
    // mesh computation
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(nurbs_, mesh_, mesh_resolution_);
    pcl::on_nurbs::Triangulation::convertSurface2Vertices(nurbs_, mesh_cloud, mesh_vertices, mesh_resolution_);
    CloudWithoutType op_cloud(new  pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*mesh_cloud, *op_cloud);
    return std::make_unique<CloudWithoutType>( op_cloud);
}
void cNurbsSurface::ComputeMeshFromSurface(pcl::on_nurbs::FittingSurface& fit)
{

}
data cNurbsSurface::GetNurbSurfaceData()
{
    return data_;
}
cv_data surface::cNurbsCurve::InitializeCurveData(data &n_data)
{
    cv_data data;
    data.interior = n_data.interior_param;
    data.interior_weight_function.push_back(true);
    return data;
}
fit_parameter_curve cNurbsCurve::IntializeCurvaParameter(int max_cp, int cp_iteration, double cp_accuracy_threshold, 
    double cp_accuracy_stop, int fit_iteration,int curve_order)
{
    fit_parameter_curve curve_params;
    curve_params.addCPsAccuracy = cp_accuracy_threshold;
    curve_params.addCPsIteration = cp_iteration;
    curve_params.maxCPs = max_cp;
    curve_params.accuracy = cp_accuracy_stop;
    curve_params.iterations = fit_iteration;
    order = curve_order;

    curve_params.param.closest_point_resolution = 0;
    curve_params.param.closest_point_weight = 1.0;
    curve_params.param.closest_point_sigma2 = 0.1;
    curve_params.param.interior_sigma2 = 0.00001;
    curve_params.param.smooth_concavity = 1.0;
    curve_params.param.smoothness = 1.0;
    return curve_params;
}

// fit curve to pointcloud
void cNurbsCurve::FitCurveToData()
{
    int algo_type = static_cast<int>(fitting_algorithm);  //
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve.interior);
    if (algo_type == 0)
    {
        // curve fitting
        pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve, curve_nurbs);
        //  curve_fit.setQuiet (false); // enable/disable debug output
        curve_fit.fitting(curve_params);
        nurbs_curve = curve_fit.m_nurbs;
    }
    else
    {
        pcl::on_nurbs::FittingCurve2dAPDM curve_fit(&curve, curve_nurbs);
        //  curve_fit.setQuiet (false); // enable/disable debug output
        curve_fit.fitting(curve_params);
        nurbs_curve = curve_fit.m_nurbs;  // may be return a reference/unique_ptr to nurbs_curve
    }
    
}
std::unique_ptr<ON_NurbsCurve>cNurbsCurve::TransformControlPointsOfNurbsCurve(const ON_NurbsCurve &curve, const Eigen::Matrix4d &transform)
{
    int max_count = curve.CVCount();
  /*  int max_idx_col = curve.CVCount(1);
    int nr_control_point = curve.CVCount();*/
    ON_NurbsCurve curve_new(curve);
    for (int i = 0; i < max_count; i++)
    {
       
            ON_4dPoint pt;
            curve.GetCV(i,pt);
            Eigen::Vector4d cp_transformed = transform * Eigen::Vector4d(pt.x, pt.y, pt.z, pt.w);
            // transformed_control_points.push_back(cp_transformed.head<3>());
            ON_4dPoint pt_new(cp_transformed[0], cp_transformed[1], cp_transformed[2], cp_transformed[3]);
            curve_new.SetCV(i, pt_new);

       
    }
    return std::make_unique<ON_NurbsCurve>(curve_new);
}

bool surface::TrimInputSurfaceUsingCurveBoundary(vec2d & vec_2d, const ON_NurbsSurface & ns, const ON_NurbsCurve& nc, 
    const Eigen::Vector3d &a0, const Eigen::Vector3d &a1, const double &rScale)
{
    bool is_inside = false;
    // copy knots
    if (ns.KnotCount(0) <= 1 || ns.KnotCount(1) <= 1 || nc.KnotCount() <= 1)
    {
        printf("[Triangulation::convertTrimmedSurface2PolygonMesh] Warning: ON knot vector empty.\n");
        return false;
    }

    
    double x0 = ns.Knot(0, 0);
    double x1 = ns.Knot(0, ns.KnotCount(0) - 1);
    double w = x1 - x0;
    double y0 = ns.Knot(1, 0);
    double y1 = ns.Knot(1, ns.KnotCount(1) - 1);
    double h = y1 - y0;

   // Eigen::Vector3d a0, a1;
    double err, param;
    Eigen::Vector2d pc, tc;
    std::vector<double> params(1, 0.0);
 /*  pcl::on_nurbs::NurbsTools::computeBoundingBox(nc, a0, a1);
    double rScale = 1.0 / pcl::on_nurbs::NurbsTools::computeRScale(a0, a1);
*/
    if (nc.Order() == 2)
        param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMappingO2(nc, vec_2d, err, pc, tc);
    else
    {
        param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint(nc, vec_2d);
        param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping(nc, vec_2d, param, err, pc, tc, rScale);
    }

    Eigen::Vector3d a(vec_2d(0) - pc(0), vec_2d(1) - pc(1), 0.0);
    Eigen::Vector3d b(tc(0), tc(1), 0.0);
    Eigen::Vector3d z = a.cross(b);
    is_inside = (z(2) >= 0.0);
    return is_inside;
}

void surface::ComputeBoundingBoxAndScaleUsingNurbsCurve(const ON_NurbsCurve& nc, Eigen::Vector3d &a0, Eigen::Vector3d &a1, double &rScale)
{
    pcl::on_nurbs::NurbsTools::computeBoundingBox(nc, a0, a1);
    rScale = 1.0 / pcl::on_nurbs::NurbsTools::computeRScale(a0, a1);
}
std::unique_ptr<ON_NurbsSurface> cNurbsSurface::TransformControlPointsOfNurbsSurface(const ON_NurbsSurface &nurb, const Eigen::Matrix4d &transform)
{
    int max_idx_row = nurb.CVCount(0);
    int max_idx_col = nurb.CVCount(1);
    int nr_control_point = nurb.CVCount();
   /* std::vector<Eigen::Vector3d> transformed_control_points;
    transformed_control_points.reserve(nr_control_point);*/
    ON_NurbsSurface nurb_new(nurb);
    for (int i = 0; i < max_idx_row; i++)
    {
        for (int j = 0; j < max_idx_col; j++)
        {

            ON_4dPoint pt;
            nurb.GetCV(i, j, pt);
            Eigen::Vector4d cp_transformed = transform * Eigen::Vector4d(pt.x, pt.y, pt.z, pt.w) ;
           // transformed_control_points.push_back(cp_transformed.head<3>());
            ON_4dPoint pt_new(cp_transformed[0], cp_transformed[1], cp_transformed[2], cp_transformed[3]);
            nurb_new.SetCV(i, j, pt_new);
        
        }
    }
    return std::make_unique<ON_NurbsSurface>(nurb_new);
   
}

double cNurbsSurface:: RayIntersect(const std::vector<double > &x, std::vector<double > &grad, void* f_data)
{
    const Ray_Intersect *raydata = static_cast<Ray_Intersect*>(f_data);
    Eigen::Vector3d vec3[3];
    raydata->nurbs.Evaluate(x[0], x[1], 1, 3, &vec3[0][0]);
   // std::cout << x[0] << " " << x[1] << "->" << " " << vec3[0][0] << " " << vec3[0][1] << " " << vec3[0][2] << std::endl;
    double error = (raydata->xyz.head<2>() - vec3[0].head<2>()).squaredNorm();
   // std::cout << "Error:" << a << std::endl;
  
    // gradient compute
    double delta = 1e-3;
    Eigen::Vector3d vec31, vec32, vec33, vec34;
    raydata->nurbs.Evaluate(x[0] + delta, x[1], 0, 3, &vec31(0));
    raydata->nurbs.Evaluate(x[0] - delta, x[1], 0, 3, &vec32(0));
    raydata->nurbs.Evaluate(x[0], x[1] + delta, 0, 3, &vec33(0));
    raydata->nurbs.Evaluate(x[0], x[1] - delta, 0, 3, &vec34(0));
 
   /* grad.clear();
    grad.reserve(2);
*/ 
    double f1_x = (raydata->xyz.head<2>() - vec31.head<2>()).squaredNorm();
    double f2_x = (raydata->xyz.head<2>() - vec32.head<2>()).squaredNorm();
  //  std::cout << f1_x << "," << f2_x << std::endl;
    double f1_y = (raydata->xyz.head<2>() - vec33.head<2>()).squaredNorm();
    double f2_y = (raydata->xyz.head<2>() - vec34.head<2>()).squaredNorm();

   // std::cout << " delta:" << delta << std::endl;
   // std::cout << f1_y << "," << f2_y << std::endl;

    double x_grad = (f1_x - f2_x) / (2.0 * delta);
    double y_grad = (f1_y - f2_y) / (2.0 * delta);

  // std::cout << " grad:" <<  x_grad << "," << y_grad << std::endl;
    if (!grad.empty())
    {
        grad[0] =  (x_grad);
        grad[1] =  (y_grad);
    }
  
    return error;
}

double cNurbsSurface::RayIntersect3D(const std::vector<double > &x, std::vector<double > &grad, void* f_data)
{
    const Ray_Intersect *raydata = static_cast<Ray_Intersect*>(f_data);
    Eigen::Vector3d vec3[3];
    raydata->nurbs.Evaluate(x[0], x[1], 1, 3, &vec3[0][0]);
    // std::cout << x[0] << " " << x[1] << "->" << " " << vec3[0][0] << " " << vec3[0][1] << " " << vec3[0][2] << std::endl;
    double a = (raydata->xyz - vec3[0]).squaredNorm();
    // std::cout << "Error:" << a << std::endl;
    return a;
}
 void cNurbsSurface::objfn_adapter3D(double *p, double *hx, int m, int n, void *adata)
{
     const Ray_Intersect *raydata = static_cast<Ray_Intersect*>(adata);
     Eigen::Vector3d vec3[3];
     raydata->nurbs.Evaluate(p[0], p[1], 1, 3, &vec3[0][0]);
     // std::cout << x[0] << " " << x[1] << "->" << " " << vec3[0][0] << " " << vec3[0][1] << " " << vec3[0][2] << std::endl;
     hx[0] = (raydata->xyz(0) - vec3[0][0]);
    hx[1] = (raydata->xyz(1) - vec3[0][1]);
    hx[2] = (raydata->xyz(2) - vec3[0][2]);
   
}
 void cNurbsSurface::objfn_adapter2D(double *p, double *hx, int m, int n, void *adata)
 {
     const Ray_Intersect *raydata = static_cast<Ray_Intersect*>(adata);
     Eigen::Vector3d vec3[3];
     raydata->nurbs.Evaluate(p[0], p[1], 1, 3, &vec3[0][0]);
     // std::cout << x[0] << " " << x[1] << "->" << " " << vec3[0][0] << " " << vec3[0][1] << " " << vec3[0][2] << std::endl;
     hx[0] = (raydata->xyz(0) - vec3[0][0]);
     hx[1] = (raydata->xyz(1) - vec3[0][1]);
  
 }
double cNurbsSurface::OptimizeParameter(Ray_Intersect &r_it,double stop_threshold, std::vector<Eigen::Vector2d> &optim_parameter)
{
    
    nlopt::opt optim(LN_NELDERMEAD,2);
  
    optim.set_stopval(stop_threshold);
    optim.set_min_objective(RayIntersect, &r_it);
  //  optim.set_maxtime(0.1);
   // optim.set_vector_storage(100);
    double error1 = INF,error2 = INF, error3 = INF, error4 = INF;
    double min_error = INF;
    optim_parameter.reserve(4);

   /* std::vector<double>p = { 0.5, 0.5 };
    optim.optimize(p, error);
    optim_parameter.push_back(Eigen::Vector2d(p[0], p[1]));
*/
    std::vector<double>q = { r_it.init_param(0), r_it.init_param(1) }; //r_it.x0 + r_it.w * 0.25, r_it.y0 + r_it.h * 0.25
    try
    {

        optim.optimize(q, error1);

        if (q[0] >= r_it.x0 && q[0] <= r_it.x1 && q[1] >= r_it.y0 && q[1] <= r_it.y1)
        {
            if (error1 < 1e-4)
                optim_parameter.push_back(Eigen::Vector2d(q[0], q[1]));
            else
            {
               
                std::cout << " error greater:" << error1 << "," << q[0] << "," << q[1] <<  std::endl;
                std::cout << "init_parameter:" << r_it.init_param << std::endl;
            }
            /* if (error1 < min_error)
                 min_error = error1;*/
        }
    }
    catch(std::runtime_error &)
    {
       // std::cout << e.what() << std::endl;
        std::cout << " error greater:" << error1 << "," << q[0] << "," << q[1] << std::endl;
        std::cout << "init_parameter:" << r_it.init_param << std::endl;
    }
    //std::vector<double>r = { r_it.x0 + r_it.w * 0.25, r_it.y0 + r_it.h *0.75 };
    //try
    //{
    //    optim.optimize(r, error2);
    //    if (r[0] >= r_it.x0 && r[0] <= r_it.x1 && r[1] >= r_it.y0 && r[1] <= r_it.y1)
    //    {
    //        if (error2 < 1e-4)
    //            optim_parameter.push_back(Eigen::Vector2d(r[0], r[1]));
    //        /* if (error2 < min_error)
    //             min_error = error2;*/
    //    }
    //}
    //catch(std::runtime_error &e)
    //{
    //  //  std::cout << e.what() << std::endl;
    //}
    //std::vector<double>s = { r_it.x0 + r_it.w * 0.75, r_it.y0 + r_it.h * 0.75 };
    //try
    //{
    //    optim.optimize(s, error3);
    //    if (s[0] >= r_it.x0 && s[0] <= r_it.x1 && s[1] >= r_it.y0 && s[1] <= r_it.y1)
    //    {
    //        if (error3 < 1e-4)
    //            optim_parameter.push_back(Eigen::Vector2d(s[0], s[1]));
    //        /* if (error3 < min_error)
    //             min_error = error3;*/
    //    }
    //}
    //catch(std::runtime_error &e)
    //{
    //   // std::cout << e.what() << std::endl;
    //}
    //std::vector<double>t = { r_it.x0 + r_it.w *0.75, r_it.y0 + r_it.h *0.25 };
    //try
    //{
    //optim.optimize(t, error4);
    //
    //    if (t[0] >= r_it.x0 && t[0] <= r_it.x1 && t[1] >= r_it.y0 && t[1] <= r_it.y1)
    //    {
    //        if (error4 < 1e-4)
    //            optim_parameter.push_back(Eigen::Vector2d(t[0], t[1]));
    //        /*if (error4 < min_error)
    //            min_error = error4;*/
    //    }
    //}
    //catch(std::runtime_error &e)
    //{
    //   // std::cout << e.what() << std::endl;
    //}
  
    return min_error;
}
double cNurbsSurface::ComputeOptimizedParameterSpaceValue(Ray_Intersect &r_it, double stop_threshold,
    Eigen::Vector2d &optim_parameter, bool use_center)
{
    // set up optimizer
 /*   nlopt::opt optim(LN_PRAXIS, 2);
    optim.set_stopval(stop_threshold);
    optim.set_min_objective(RayIntersect, &r_it);*/

    double error1 = INF, error2 = INF, error3 = INF, error4 = INF;
    double min_error = INF;
   /* optim_parameter.clear();
    optim_parameter.reserve(1);*/

    // determine initial (s,t) parameter 
    // quadrant( - ,-), ( +, -), (+, +), ( -, +)
    // origin (0.5, 0.5)
    double x_diff = r_it.init_param(0) - 0.5;
    double y_diff = r_it.init_param(1) - 0.5;
  
    Eigen::Matrix<bool, 2, 1>quadrant;

    quadrant.x() = x_diff > 0.0;
    quadrant.y() = y_diff > 0.0;
    bool horizontal = false;
    double options[LM_OPTS_SZ] = {
        std::sqrt(LM_INIT_MU),     /* Initial dampening factor */
        std::sqrt(LM_STOP_THRESH), /* epsilon_1 */
        std::sqrt(LM_STOP_THRESH), /* epsilon_2 */
        std::sqrt(LM_STOP_THRESH), /* epsilon_3 */
        -1e-4                       /* Finite differences delta (pos.: forward, neg.: central) */
    };
    double info[LM_INFO_SZ];
    for (int i = 0; i < 1 + 3* static_cast<int>(use_center); i++)
    {
       
       std::vector<double>q;
        if (use_center || i > 0)
            q = { r_it.x0 + r_it.w *(0.25 + 0.5 * double(quadrant(0))), r_it.y0 + r_it.h * (0.25 + 0.5 * double(quadrant(1))) };
        else if (i == 0)
        {
            q = { r_it.init_param(0), r_it.init_param(1) };
        }
       // Optimize(optim, q, error1);
   
      int num_iters4 = dlevmar_dif(objfn_adapter2D,q.data(), 0, 2, 2, 1048576, options, info, 0, 0, &r_it);
        if (q[0] >= r_it.x0 && q[0] <= r_it.x1 && q[1] >= r_it.y0 && q[1] <= r_it.y1  && info[1] < 1e-4)
        {

            optim_parameter= (Eigen::Vector2d(q[0], q[1]));
            break;

        }
        if (i == 0)
        {
            if (abs(x_diff) > abs(y_diff))
            {
                quadrant.y() = !quadrant.y();
            }
            else
            {
                quadrant.x() = !quadrant.x();
                horizontal = true;
            }
        }
        else if (i == 1)
        {
            quadrant.x() = !quadrant.x();
            quadrant.y() = !quadrant.y();
        }
        else if (i == 2)
        {
            if (horizontal)
            {
                quadrant.x() = !quadrant.x();
            }
            else
                quadrant.y() = !quadrant.y();
        }
    }
    if (optim_parameter[0] >= r_it.x0 && optim_parameter[0] <= r_it.x1 && optim_parameter[1] >= r_it.y0 && optim_parameter[1] <= r_it.y1
        && info[1] < 1e-4)
    {

        return  info[1];
     
    }
    else
    {
        return  error1;
    }
  
}
 void cNurbsSurface::Optimize(nlopt::opt &optim, std::vector<double> &optimized_param, double &error)
 {
     try
     {

         optim.optimize(optimized_param, error);

     }
     catch (std::runtime_error &e)
     {
         // std::cout << e.what() << std::endl;
         std::cout << " error greater:" << error << "," << optimized_param[0] << "," << optimized_param[1] << std::endl;
       
     }
 }
double cNurbsSurface::OptimizeParameter3D(Ray_Intersect &r_it, double stop_threshold, std::vector<Eigen::Vector2d> &optim_parameter)
{
  
    nlopt::opt optim(LN_NELDERMEAD, 2);
    optim.set_xtol_abs(1e-4);
   // optim.set_stopval(stop_threshold);
    optim.set_min_objective(RayIntersect3D, &r_it);
    double error1 = INF, error2 = INF, error3 = INF, error4 = INF;
    double min_error = INF;
    optim_parameter.reserve(4);
    Eigen::Vector2d min_parameter;
    /* std::vector<double>p = { 0.5, 0.5 };
    optim.optimize(p, error);
    optim_parameter.push_back(Eigen::Vector2d(p[0], p[1]));
    */

    double info1[LM_INFO_SZ], info2[LM_INFO_SZ], info3[LM_INFO_SZ], info4[LM_INFO_SZ];
    double options[LM_OPTS_SZ] = {
        std::sqrt(LM_INIT_MU),     /* Initial dampening factor */
        std::sqrt(LM_STOP_THRESH), /* epsilon_1 */
        std::sqrt(LM_STOP_THRESH), /* epsilon_2 */
        std::sqrt(LM_STOP_THRESH), /* epsilon_3 */
        -1e-4                       /* Finite differences delta (pos.: forward, neg.: central) */
    };
   


    std::vector<double>q = { r_it.x0 + r_it.w * 0.25, r_it.y0 + r_it.h * 0.25 };
   // optim.optimize(q, error1);
    int num_iters1 = dlevmar_dif(objfn_adapter3D, q.data(), 0, 2, 3, 1048576, options, info1, 0, 0, &r_it);
    if (q[0] >= r_it.x0 && q[0] <= r_it.x1 && q[1] >= r_it.y0 && q[1] <= r_it.y1)
    {
        if (info1[1] < 1e-4) //error1
            optim_parameter.push_back(Eigen::Vector2d(q[0], q[1]));
        if (info1[1] < min_error)
        {
            min_error = info1[1];
            min_parameter = Eigen::Vector2d(q[0], q[1]);

        }
    }

    std::vector<double>r = { r_it.x0 + r_it.w * 0.25, r_it.y0 + r_it.h *0.75 };
    //optim.optimize(r, error2);
    int num_iters2 = dlevmar_dif(objfn_adapter3D, r.data(), 0, 2, 3, 1048576, options, info2, 0, 0, &r_it);
    if (r[0] >= r_it.x0 && r[0] <= r_it.x1 && r[1] >= r_it.y0 && r[1] <= r_it.y1)
    {
        if (info2[1] < 1e-4)
            optim_parameter.push_back(Eigen::Vector2d(r[0], r[1]));
        if (info2[1] < min_error)
        {
            min_error = info2[1];
            min_parameter = Eigen::Vector2d(r[0], r[1]);
        }
    }

    std::vector<double>s = { r_it.x0 + r_it.w * 0.75, r_it.y0 + r_it.h * 0.75 };
   // optim.optimize(s, error3);
    int num_iters3 = dlevmar_dif(objfn_adapter3D, s.data(), 0, 2, 3, 1048576, options, info3, 0, 0, &r_it);
    if (s[0] >= r_it.x0 && s[0] <= r_it.x1 && s[1] >= r_it.y0 && s[1] <= r_it.y1)
    {
        if (info3[1] < 1e-4)
            optim_parameter.push_back(Eigen::Vector2d(s[0], s[1]));
        if (info3[1] < min_error)
        {
            min_error = info3[1];
            min_parameter = Eigen::Vector2d(s[0], s[1]);
        }
    }

    std::vector<double>t = { r_it.x0 + r_it.w *0.75, r_it.y0 + r_it.h *0.25 };
   // optim.optimize(t, error4);
    int num_iters4 = dlevmar_dif(objfn_adapter3D, t.data(), 0, 2, 3, 1048576, options, info4, 0, 0, &r_it);
    if (t[0] >= r_it.x0 && t[0] <= r_it.x1 && t[1] >= r_it.y0 && t[1] <= r_it.y1)
    {
        if (info4[1] < 1e-4)
            optim_parameter.push_back(Eigen::Vector2d(t[0], t[1]));
        if (info4[1] < min_error)
        {
            min_error = info4[1];
            min_parameter = Eigen::Vector2d(t[0], t[1]);
        }
    }

    optim_parameter[0] = min_parameter;
   // std::cout << "minimum_error for closest point:" << min_error << std::endl;
    return min_error;
}

bool cNurbsSurface::EvaluateParameterForCoordinate(std::vector<Eigen::Vector2d> &optim_parameter, const ON_NurbsSurface& ns, const ON_NurbsCurve& nc
, Eigen::VectorXd &pt, Eigen::Vector2d &pt_param)
{
    double z_coordinate_max = -INF;
    bool inside = false;
    pt = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d a0, a1;
    double scale;
    surface::ComputeBoundingBoxAndScaleUsingNurbsCurve(nc, a0, a1, scale);
    if (optim_parameter.size() <= 0)
        return inside;
    for (Eigen::Vector2d vec2d : optim_parameter)
    {
        inside = TrimInputSurfaceUsingCurveBoundary(vec2d, ns, nc,a0,a1,scale);
       if (inside)
       {
           Eigen::Vector3d vec3[3];
           ns.Evaluate(vec2d[0], vec2d[1], 1, 3, &vec3[0][0]);
           if (vec3[0][2] > z_coordinate_max)
           {
               z_coordinate_max = vec3[0][2];
               vec3[1].normalize();
               vec3[2].normalize();
               pt.head<3>() = vec3[0];
               pt.tail<3>() = (vec3[1].cross(vec3[2])).normalized();
               pt_param = vec2d;
           }

       }
    }
    return inside;
}

bool cNurbsSurface::EvaluateParameterForCoordinate3D(std::vector<Eigen::Vector2d> &optim_parameter, const ON_NurbsSurface& ns, const ON_NurbsCurve& nc
    , Eigen::VectorXd &pt, Eigen::Vector2d &pt_param, int i)
{
    double z_coordinate_max = -INF;
    bool inside = false;
    pt = Eigen::VectorXd::Zero(6);
    if (optim_parameter.size() <= 0)
        return inside;
    Eigen::Vector3d a0, a1;
    double scale;
    surface::ComputeBoundingBoxAndScaleUsingNurbsCurve(nc, a0, a1, scale);
    /*for (Eigen::Vector2d vec2d : optim_parameter)
    {*/
        inside = TrimInputSurfaceUsingCurveBoundary(optim_parameter[0], ns, nc,a0,a1,scale);
        if (inside)
        {
            Eigen::Vector3d vec3[3];
            ns.Evaluate(optim_parameter[0][0], optim_parameter[0][1], 1, 3, &vec3[0][0]);
           /* if (vec3[0][2] > z_coordinate_max)
            {*/
                z_coordinate_max = vec3[0][2];
                vec3[1].normalize();
                vec3[2].normalize();
                pt.head<3>() = vec3[0];
                pt.tail<3>() = (vec3[1].cross(vec3[2])).normalized();
                pt_param = optim_parameter[0];
          /*  }*/

        }
 /*   }*/
    return inside;
}

void cNurbsSurface::CreateVerticesFromNurbsSurface(const ON_NurbsSurface &nurb, const ON_NurbsCurve &Curve, CloudWithNormalPtr &cloud, std::vector<Eigen::Vector2d> &st_params, const unsigned  &segX, const unsigned &segY)
{
  double x0 = nurb.Knot(0, 0);
  double x1 = nurb.Knot(0, nurb.KnotCount(0) - 1);
  double w = x1 - x0;
  double y0 = nurb.Knot(1, 0);
  double y1 = nurb.Knot(1, nurb.KnotCount(1) - 1);
  double h = y1 - y0;
  double dx = w / double(segX);
  double dy = h / double(segY);
  Eigen::Vector2d st;
  Eigen::Vector3d a0, a1;
  pcl::on_nurbs::NurbsTools::computeBoundingBox(Curve, a0, a1);
  double rScale = 1.0 / pcl::on_nurbs::NurbsTools::computeRScale(a0, a1);

  for (unsigned j = 0; j < segY; j++)
  {
      for (unsigned i = 0; i < segX; i++)
      {
          st.x() = x0 + double(i) * dx;
          st.y() = y0 + double(j) * dy;
          Eigen::Vector3d vec3[3];
          if (st.x() >= x0 && st.x() <= x1 &&  st.y() >= y0 &&  st.y() <= y1)
          {
              nurb.Evaluate(st.x(), st.y(), 1, 3, &vec3[0][0]);
              vec3[1].normalize();
              vec3[2].normalize();
              double err, param;
              Eigen::Vector2d pc, tc;
              PointNormalType pt;
              pt.getVector3fMap() = vec3[0].cast<float>();
              pt.getNormalVector3fMap() = (vec3[1].cross(vec3[2])).normalized().cast<float>();
              Eigen::Vector2d vp(st.x(), st.y());

              if (Curve.Order() == 2)
                  param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMappingO2(Curve, vp, err, pc, tc);
              else
              {
                  param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint(Curve, vp);
                  param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping(Curve, vp, param, err, pc, tc, rScale);
              }

              Eigen::Vector3d a(vp(0) - pc(0), vp(1) - pc(1), 0.0);
              Eigen::Vector3d b(tc(0), tc(1), 0.0);
              Eigen::Vector3d z = a.cross(b);

              if (z(2) >= 0.0)
              {
                  cloud->push_back(pt);
                  st_params.push_back(st);
              }
          }
      }
  }
}

Eigen::VectorXd cNurbsSurface::ComputeClosetPointOnNurbSurface(const Eigen::VectorXd &pt_interest, const ON_NurbsSurface &ns,
    const ON_NurbsCurve &nc, double stop_threshold)
{
    surface::Ray_Intersect r_it(ns);
    r_it.x0 = ns.Knot(0, 0);
    r_it.x1 = ns.Knot(0, ns.KnotCount(0) - 1);
    r_it.w = r_it.x1 - r_it.x0;
    r_it.y0 = ns.Knot(1, 0);
    r_it.y1 = ns.Knot(1, ns.KnotCount(1) - 1);
    r_it.h = r_it.y1 - r_it.y0;

    r_it.xyz = pt_interest.head<3>();

    // compute the optimized parameter (s,t) 
    std::vector<Eigen::Vector2d>optim_paramter;
    double min_error = surface::cNurbsSurface::OptimizeParameter3D(r_it, stop_threshold, optim_paramter);

    Eigen::VectorXd pt;
    Eigen::Vector2d parameter;

    // closest point on the nurb surface
   surface::cNurbsSurface::EvaluateParameterForCoordinate3D(optim_paramter, ns, nc, pt, parameter);
   return pt;
}