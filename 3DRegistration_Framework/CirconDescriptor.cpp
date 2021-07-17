#include"pch.h"
#include"CirconDescriptor.h"
#include<math.h>
#include<algorithm>
#include"Transformation_tool.h"


#define eps 1e-6f
namespace
{
    bool SortOnZ(PointNormalType &A, PointNormalType &B)
    {
        return A.z > B.z;
    }

 
}

CirconImageDescriptor& CirconImageDescriptor:: operator=(const CirconImageDescriptor &cid)
{
    if (this == &cid)
        return *this;
    inputCloud = cid.inputCloud;
    num_division_row = cid.num_division_row;
    num_division_col = cid.num_division_col;
    num_height_division = cid.num_height_division;
    angle_resolution = cid.angle_resolution;
    rad_resolution = cid.rad_resolution;
    RotationAxisPoint = cid.RotationAxisPoint;
    rotation_index = cid.rotation_index;
    height_resolution = cid.height_resolution;
    Image2D = cid.Image2D;
    rot_matrix = cid.rot_matrix;
    Local_Coordinate_Frame = cid.Local_Coordinate_Frame;
    index_index_map = cid.index_index_map;
    basic_point_index = cid.basic_point_index;
    World_to_local = cid.World_to_local;
    WorldLocalTransformation = cid.WorldLocalTransformation;
    max_value_image = cid.max_value_image;
    min_value_image = cid.min_value_image;
    sigma_threshold = cid.sigma_threshold;
    basic_cell_index = cid.basic_cell_index;
    reconstructed_points = cid.reconstructed_points;
    max_radius = cid.max_radius;
    transformed_inputCloud.reset(new pcl::PCLPointCloud2);
    *transformed_inputCloud = *cid.transformed_inputCloud;
    original_cloud_with_normal.reset(new pcl::PointCloud <PointNormalType>);
    *original_cloud_with_normal = *cid.original_cloud_with_normal;
   /* Cloud2D.reset(new pcl::PointCloud<PointType>);
    *Cloud2D = *cid.Cloud2D;*/
    
   // _pGrid = cid._pGrid;
  
    descriptor_content = cid.descriptor_content;
    reconstructed_points = cid.reconstructed_points;
    nb_curve = cid.nb_curve;
    nb_surface = cid.nb_surface;
    no_col_search = cid.no_col_search;
    bbs = cid.bbs;
    vector_of_maps = cid.vector_of_maps;
    st_params = cid.st_params;
    avgpoint_dist = cid.avgpoint_dist;
    high_res_flag = cid.high_res_flag;
    up_resolution_count = cid.up_resolution_count;
    Cluster_labels = cid.Cluster_labels;
   // kdTree = cid.kdTree;
    return *this;
}
CirconImageDescriptor :: CirconImageDescriptor(const CirconImageDescriptor &cid)
{
    inputCloud = cid.inputCloud;
    num_division_row = cid.num_division_row;
    num_division_col = cid.num_division_col;
    num_height_division = cid.num_height_division;
    angle_resolution = cid.angle_resolution;
    rad_resolution = cid.rad_resolution;
    RotationAxisPoint = cid.RotationAxisPoint;
    rotation_index = cid.rotation_index;
    height_resolution = cid.height_resolution;
    Image2D = cid.Image2D;
    rot_matrix = cid.rot_matrix;
    Local_Coordinate_Frame = cid.Local_Coordinate_Frame;
    index_index_map = cid.index_index_map;
    basic_point_index = cid.basic_point_index;
    World_to_local = cid.World_to_local;
    WorldLocalTransformation = cid.WorldLocalTransformation;
    max_value_image = cid.max_value_image;
    min_value_image = cid.min_value_image;
    sigma_threshold = cid.sigma_threshold;
    basic_cell_index = cid.basic_cell_index;
    max_radius = cid.max_radius;
    transformed_inputCloud.reset(new pcl::PCLPointCloud2);
    *transformed_inputCloud = *cid.transformed_inputCloud;
    original_cloud_with_normal.reset(new pcl::PointCloud <PointNormalType>);
    *original_cloud_with_normal = *cid.original_cloud_with_normal;
    Cloud2D.reset(new pcl::PointCloud<PointType>);
    *Cloud2D = *cid.Cloud2D;
    reconstructed_points = cid.reconstructed_points;
    descriptor_content = cid.descriptor_content;
    nb_curve = cid.nb_curve;
    nb_surface = cid.nb_surface;
    no_col_search = cid.no_col_search;
    vector_of_maps = cid.vector_of_maps;
    st_params = cid.st_params;
    bbs = cid.bbs;
    avgpoint_dist = cid.avgpoint_dist;
    high_res_flag = cid.high_res_flag;
    up_resolution_count = cid.up_resolution_count;
    Cluster_labels = cid.Cluster_labels;
    
  //  _pGrid = cid._pGrid;
   // kdTree = cid.kdTree;

}


std::unique_ptr<cParameterGrid> CirconImageDescriptor::duplicate(std::unique_ptr<cParameterGrid> const& ptr)
{
    return std::make_unique<cParameterGrid>(*ptr);
}
//CirconImageDescriptor::~CirconImageDescriptor()
//{
//    if (NULL != kdTree)
//    {
//        delete kdTree;
//        kdTree = NULL;
//    }
//
//}
void CirconImageDescriptor::ComputeFeature(const CUniformGrid2D &cGrid2D, const cParameterGrid &_pGrid)
{

    if (false) //num_division_row >= 256 || high_res_flag
    {
        num_division_col = 32;
        num_division_row = 32;
        Image2D.SetImage(num_division_col, num_division_row);

    }
    int col_reduced;
    if (num_division_row < 32)
        col_reduced = 16; //32
    else if (num_division_row == 32)
        col_reduced = 32; //32
    else
        col_reduced = 64;
    /*else
        col_reduced = 64;*/
        /* CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
        pcl::fromPCLPointCloud2(*transformed_inputCloud, *pTarget);*/
        /* CloudWithNormalPtr pTarget_new(new pcl::PointCloud <PointNormalType>);
         pTarget_new = pTarget;*/

         // size_t total_num_points = pTarget->points.size();
    index_index_map.clear();
    // int i = 0;
    // Eigen::Vector3f test1(0.0, 0.0, 0.0);
    float max_radius;
    // _dV dv;
    _dV dv1;
    /* std::vector<_dV>temp;
     temp.resize(num_division_col, dv1);*/
     //  std::cout << "after col copy constructor" << std::endl;
    std::vector<std::vector<_dV>>temp_descriptor_content(num_division_row, std::vector<_dV>(num_division_col, dv1));
    // CirconImage temp_image2d(num_division_col, num_division_row);
    // temp_descriptor_content.resize(num_division_row, temp);
    // temp_descriptor_content.resize(num_division_row, std::vector<_dV>(num_division_col, dv));
   /*  descriptor_content.clear();
     descriptor_content.shrink_to_fit();

     descriptor_content.resize(num_division_row, std::vector<_dV>(num_division_col, dv));*/
    std::map<float, int>d_map;// = { {-INFINITY, -1} };
   /* vector_of_maps.clear();
    vector_of_maps.resize(num_division_row *num_division_col);*/
    std::vector<size_t>indices_list;
    bool use_z_max = false;

    Eigen::Matrix4d tfs = Eigen::Matrix4d::Identity();
    Eigen::Matrix4f temp_wl = WorldLocalTransformation.inverse();

    // tranform nurb surfaces to local frame
    std::vector<std::unique_ptr<ON_NurbsSurface>> nb_surface_tfs = TransformNurbSurfacesInDescriptorFrame(nb_surface,
        WorldLocalTransformation.cast<double>());
    //std::unique_ptr<ON_NurbsSurface> nb_surface_tfs 
    //    = surface::cNurbsSurface::TransformControlPointsOfNurbsSurface(nb_surface,
    //    WorldLocalTransformation.cast<double>()); //

    /*surface::Ray_Intersect r_it(*nb_surface_tfs);
    r_it.x0 = nb_surface.Knot(0, 0);
    r_it.x1 = nb_surface.Knot(0, nb_surface.KnotCount(0) - 1);
    r_it.w = r_it.x1 - r_it.x0;
    r_it.y0 = nb_surface.Knot(1, 0);
    r_it.y1 = nb_surface.Knot(1, nb_surface.KnotCount(1) - 1);
    r_it.h = r_it.y1 - r_it.y0;*/
    Eigen::Vector3d a0, a1;
    double scale;
   // surface::ComputeBoundingBoxAndScaleUsingNurbsCurve(nb_curve, a0, a1, scale);
    
    float  min_value = INFINITY;
    float  max_value = -INFINITY;
    double total_trim_time = 0; double total_eval_time = 0;
    //
    int countItr = 0;
    int ray_intersector = 1000;
    //#pragma omp parallel for
    for (int i = 0; i < num_division_row; ++i)
    {

        for (int j = 0; j < col_reduced; ++j)
        {
            /*  if (i == 248 && j == 62)
              {
                  std::cout << "here" << std::endl;
              }*/
            int curr_position = i * num_division_col + j;
            double x = static_cast<double>((j * rad_resolution) * cos(-(i - 1)* angle_resolution));
            double y = static_cast<double>((j * rad_resolution) * sin(-(i - 1)* angle_resolution));
            Eigen::Vector2d qPoint(x, y);
            // Eigen::Vector2d pt(x, y);
            std::vector<size_t>indices_list;
            cGrid2D.query(&indices_list, qPoint, SORT_MULTI, 2.0 *avgpoint_dist);
            if (indices_list.size() > 0)
            {
                for (int itr = 0; itr < indices_list.size(); ++itr)
                {
                    // get the cluster to which the surface belongs
                    int pt_idx = indices_list[itr];
                    int ci;
                    Eigen::Vector2i cluster_pt = Cluster_labels[pt_idx];
                    if (pt_idx == cluster_pt(0))
                        ci = cluster_pt(1);
                    else
                        std::cout << "Wrong Index:" << cluster_pt(0) << std::endl;
                    //get the corresponding nurb surface and transform it into local frame;
                    // initiliaze the ray interesection with the transformed surface
                    surface::Ray_Intersect r_it(*nb_surface_tfs[ci]);
                    r_it.x0 = nb_surface[ci].Knot(0, 0);
                    r_it.x1 = nb_surface[ci].Knot(0, nb_surface[ci].KnotCount(0) - 1);
                    r_it.w = r_it.x1 - r_it.x0;
                    r_it.y0 = nb_surface[ci].Knot(1, 0);
                    r_it.y1 = nb_surface[ci].Knot(1, nb_surface[ci].KnotCount(1) - 1);
                    r_it.h = r_it.y1 - r_it.y0;
                   

                    r_it.xyz.head<2>() = Eigen::Vector2d(x, y);
                    r_it.init_param = st_params[pt_idx];

                    Eigen::Vector2d optim_parameter;
                    if (true) // col_reduced
                    {
                        auto start = std::chrono::high_resolution_clock::now();
                        double error = surface::cNurbsSurface::ComputeOptimizedParameterSpaceValue(r_it, 1e-5,
                            optim_parameter, false);

                        //  bool inside = surface::TrimInputSurfaceUsingCurveBoundary(optim_parameter, *nb_surface_tfs, 
                         //     nb_curve, a0, a1, scale);
                        bool inside = _pGrid.query(optim_parameter);

                        auto end = std::chrono::high_resolution_clock::now();
                        double trim_time = std::chrono::duration_cast<
                            std::chrono::duration<double, std::milli>>(end - start).count();
                        trim_time = trim_time / double(1000);

                        /*if (trim_time >= 0.001)
                            std::cout << " optimization time per cell:" << trim_time << std::endl;*/
                        Eigen::Vector3d vec3[3];
                        if (inside)
                        {
                            auto startopto = std::chrono::high_resolution_clock::now();

                            nb_surface_tfs[ci]->Evaluate(optim_parameter(0), optim_parameter(1), 1, 3, &vec3[0][0]);

                            auto endopto = std::chrono::high_resolution_clock::now();

                            double executeopto = std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli>>(endopto - startopto).count();
                            executeopto = executeopto / double(1000);

                            //  std::cout << " optimization time per cell:" << executeopto << std::endl;
                            error = (r_it.xyz.head<2>() - vec3[0].head<2>()).squaredNorm();

                            if (error < 1e-4)
                            {
                                vec3[1].normalize();
                                vec3[2].normalize();
                                // Eigen::VectorXd pt = Eigen::VectorXd::Zero(6);
                                PointNormalType pt;
                                pt.getVector3fMap() = vec3[0].cast<float>();
                                pt.getNormalVector3fMap() = (vec3[1].cross(vec3[2])).normalized().cast<float>();
                                /*   pt.head<3>() = vec3[0];
                                   pt.tail<3>() = (vec3[1].cross(vec3[2])).normalized();*/
                                   //currently store only z-values
                                 float val = std::round(vec3[0][2] / height_resolution);
                               // double step_size = max_radius / 512;
                                 //float val = vec3[0][2]/step_size;

                                /*  Eigen::Matrix3f rot = temp_wl.block<3, 3>(0, 0);
                                  Eigen::Vector3f trans = temp_wl.block<3, 1>(0, 3);*/
                                PointNormalType pt_dash; /*= pcl::transformPoint(pt, Eigen::Affine3f(temp_wl));*/

                                pt_dash.getVector3fMap() = temp_wl.block<3, 3>(0, 0) * pt.getVector3fMap() +
                                    temp_wl.block<3, 1>(0, 3);

                                pt_dash.getNormalVector3fMap() = temp_wl.block<3, 3>(0, 0) *  pt.getNormalVector3fMap();
                                /*                    Eigen::VectorXf pt_dash = Eigen::VectorXf::Zero(6);
                                                pt_dash.head<3>() = temp_wl.block<3, 3>(0, 0) * pt.head<3>().cast<float>() +
                                                    temp_wl.block<3, 1>(0, 3);
                                                pt_dash.tail<3>() = temp_wl.block<3, 3>(0, 0) * pt.tail<3>().cast<float>();*/
                                if (true)
                                    Image2D.addCellAt(i, j, num_division_col, val);

                                // temp_image2d.addCellAt(i, j, num_division_col, val);
                                int linearized_cell_index = i * num_division_col + j;

                                _dV data(/*i, j,*/cellSate::VALID, linearized_cell_index, pt_idx,
                                    val, optim_parameter, pt_dash/*.cast<double>()*/);

                                temp_descriptor_content[i][j] = std::move(data);
                                total_eval_time += executeopto;
                                total_trim_time += trim_time;

                                if (val < min_value)
                                {
                                    min_value = val;
                                }
                                if (val > max_value)
                                {
                                    max_value = val;
                                }
                                /* if (num_division_row == 512 && itr >=5)
                                     std::cout << "Exist iterator:" << itr << "/" << indices_list.size() << std::endl;*/
                                break;
                            }

                        }
                    }

                }
            }
            ////  kdTree.radiusSearch(qPoint, radius, indices_list, dist_list);
            //  if (!use_z_max && indices_list.size() > 0)
            //  {
            //      for (int itr = 0; itr < indices_list.size(); itr++)
            //      {
            //          int pt_idx = indices_list[itr];
            //         // float dist = dist_list[itr];
            //         /* if (dist < 0.3 * rad_resolution)
            //          {*/
            //          float val = pTarget->points[pt_idx].z;
            //          vector_of_maps[curr_position][val] = pt_idx;
            //             // int_uv_map[curr_position] = { pt_idx };
            //   
            //              
            //         /* }*/
            //      }
            //      countItr++;
            //  }

        }
    }
    /* std::cout << "total evaluation time:" << total_eval_time << std::endl;
     std::cout << "total trim time:" << total_trim_time << std::endl;*/
     // std::cout << " optimize and trimming time:" << executeopto << std::endl;
     // std::cout << countItr << "/" <<vector_of_maps.size() << std::endl;

     /* float  min_value = INFINITY;
      float  max_value = -INFINITY;*/
    if (use_z_max)
    {
        std::vector<std::vector<float>>data = Image2D.getImageData();


        int valid_index = 0;

        for (int itr = 0; itr < num_division_row; itr++)
        {
            for (int icx = 0; icx < num_division_col; icx++)
            {
                if (data[itr][icx] != -INFINITY)
                {
                    valid_index++;
                    //  refined_descriptor_content.push_back(descriptor_content[itr]);
                    if (data[itr][icx] < min_value)
                    {
                        min_value = data[itr][icx];
                    }
                    if (data[itr][icx] > max_value)
                    {
                        max_value = data[itr][icx];
                    }
                }
                else
                    continue;
            }
        }
    }

    // quantize the z-value & store
   /* float max_z = max_value - min_value;
    height_resolution = max_z / num_height_division;
    for (int i = 0; i < num_division_row; ++i)
    {

        for (int j = 0; j < col_reduced; ++j)
        {

            if (temp_descriptor_content[i][j].state == cellSate::VALID)
            {
                float val = std::round(temp_descriptor_content[i][j].val_at_pixel / height_resolution);
                temp_descriptor_content[i][j].val_at_pixel = val;
                Image2D.addCellAt(i, j, num_division_col, val);
            }
        }
    }*/
    max_value_image = max_value/* / height_resolution*/;
    min_value_image = min_value /*/ height_resolution*/;
    // descriptor_content = refined_descriptor_content;
    descriptor_content = std::move(temp_descriptor_content);
    //Image2D = std::move(temp_image2d);

}

std::vector<std::unique_ptr<ON_NurbsSurface>>  CirconImageDescriptor::TransformNurbSurfacesInDescriptorFrame(const std::vector<ON_NurbsSurface>& nb_surface,
    const Eigen::Matrix4d &Wl)
{
    std::vector<std::unique_ptr<ON_NurbsSurface>>nurb_surfaces_tfs;
    if (nb_surface.size() > 0)
    {
        nurb_surfaces_tfs.reserve(nb_surface.size());
        for (int i = 0 ; i < nb_surface.size(); i++)
        {
            std::unique_ptr<ON_NurbsSurface> nb_surface_tfs = surface::cNurbsSurface::TransformControlPointsOfNurbsSurface(nb_surface[i], Wl);
            nurb_surfaces_tfs.emplace_back(std::move(nb_surface_tfs));
        }

    }
    return nurb_surfaces_tfs;
}
void CirconImageDescriptor::ComputeFeature(int i)
{
    int col_reduced = 16;
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*transformed_inputCloud, *pTarget);
    size_t total_num_points = pTarget->points.size();
    descriptor_content.clear();
    _dV dv;
    descriptor_content.resize(num_division_row, std::vector<_dV>(num_division_col, dv));
    for (int itr = 0; itr < total_num_points; itr++)
    {
        if (true/*itr != basic_point_index*/)
        {
            // Compute i-index for image
            float x_y = atan2(pTarget->points[itr].y, pTarget->points[itr].x);
            x_y = x_y / angle_resolution;
            float ang = static_cast<float>(num_division_row) - x_y;
            int row = static_cast<int>(std::round(ang)) % num_division_row;
          
            int row_index = static_cast<int>(std::round(ang)) % num_division_row;

            // compute j-index for image
            float sq_root_sum = std::sqrtf(pow(pTarget->points[itr].y, 2.0) + pow(pTarget->points[itr].x, 2.0));
            sq_root_sum = std::round(sq_root_sum / rad_resolution);
            int col_index = static_cast<int>(sq_root_sum);
           
            if (col_index < 0)
                std::cout << "wrong col_index:" << sq_root_sum << itr << std::endl;

            if (row_index < num_division_row && col_index < num_division_col)
            {
                int curr_position = row_index * num_division_col + col_index;
                float c_ij_current = (std::round(pTarget->points[itr].z / height_resolution));
                // prev value at posn(i,j)
                float c_ij_prev = Image2D.getCell(row_index, col_index, num_division_col);
                // store index of poi
                if (itr == basic_point_index)
                {
                    basic_cell_index = curr_position;
                }
                if (c_ij_current > c_ij_prev)
                {
                    float dist = (pTarget->points[itr].getVector3fMap() - Eigen::Vector3f(0.0, 0.0, 0.0)).norm();
                    if (true)  // bypass
                    {
                        Image2D.addCellAt(row_index, col_index, num_division_col, c_ij_current);
                        Eigen::Vector2d vec2d(0, 0);
                       // Eigen::VectorXd pt = Eigen::VectorXd::Zero(6);
                        PointNormalType pt;
                        pt.getVector3fMap() = original_cloud_with_normal->points[itr].getVector3fMap();
                        pt.getNormalVector3fMap() = original_cloud_with_normal->points[itr].getNormalVector3fMap();

                      /*  pt.head<3>() = original_cloud_with_normal->points[itr].getVector3fMap().cast<double>();
                        pt.tail<3>() = original_cloud_with_normal->points[itr].getNormalVector3fMap().cast<double>();*/
                        cellSate state_(INVALID);
                        descriptor_content[row_index][col_index] = _dV(/*row_index, col_index,*/ state_,
                            curr_position, itr, c_ij_current, vec2d, pt);
                        // i++;
                    }
 
                }


            }
        }
    }
    std::vector<std::vector<float>>data = Image2D.getImageData();
    float  min_value = INFINITY;
    float  max_value = -INFINITY;
    int valid_index = 0;

    for (int itr = 0; itr < num_division_row; itr++)
    {
        for (int icx = 0; icx < num_division_col; icx++)
        {
            if (data[itr][icx] != -INFINITY)
            {
                valid_index++;
                //  refined_descriptor_content.push_back(descriptor_content[itr]);
                if (data[itr][icx] < min_value)
                {
                    min_value = data[itr][icx];
                }
                if (data[itr][icx] > max_value)
                {
                    max_value = data[itr][icx];
                }
            }
            else
                continue;
        }
    }
    max_value_image = max_value;
    min_value_image = min_value;
}
PointNormalType CirconImageDescriptor::ComputeBasicPointOfInterest()
{
    /*CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*inputCloud, *pTarget);*/
    Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
    Eigen::Matrix3f cov_matrix = Eigen::Matrix3f::Identity();
    pcl::compute3DCentroid(*original_cloud_with_normal, centroid);
    //pcl::computeCovarianceMatrixNormalized(*pTarget, centroid, cov_matrix);

    // compute nearest point to centroid for point of interest
    int K = 2;
    std::vector<int> SearchResults(K);
    std::vector<float> SearchDistances(K);
    pcl::KdTreeFLANN<PointNormalType> KTree;
    KTree.setInputCloud(original_cloud_with_normal);
    PointNormalType centroid_point;
    centroid_point.getVector3fMap() = centroid.head<3>();
    KTree.nearestKSearch(centroid_point, K, SearchResults, SearchDistances);
    if (SearchResults[0] < original_cloud_with_normal->points.size() || SearchResults.size() != 0)
    {
        RotationAxisPoint = original_cloud_with_normal->points[SearchResults[0]];
        basic_point_index = SearchResults[0]; // static_cast<size_t>(
        return original_cloud_with_normal->points[SearchResults[0]];
    }
    else
    {
        throw std::runtime_error("No closest point found.");
    }
    
}
void CirconImageDescriptor::setParametergrid(const cParameterGrid &pGrid)
{
   // _pGrid = pGrid;
}
//cParameterGrid CirconImageDescriptor::GetParameterGrid()
//{
//    return _pGrid;
//}
std::vector<Eigen::Vector2d>CirconImageDescriptor::GetInitialParameters()
{
    return st_params;
}
void CirconImageDescriptor::SetUpResolutionCount(int res)
{
    up_resolution_count = res;
}
void CirconImageDescriptor::SetFlagForHighResolution(const bool &flag)
{
    high_res_flag = flag;
}
void CirconImageDescriptor::ResetFlagForHighResolution()
{
    high_res_flag = false;
}
Eigen::Vector2d CirconImageDescriptor::TransformAndScaleParametricCoordinate(const Eigen::Vector2d & vec2d, const double &w, const double &h)
{
    Eigen::Vector4d vec4d(vec2d(0),vec2d(1),0,1.0);
   
    Eigen::Vector4d lw_pt = WorldLocalTransformation.inverse().cast<double>() * vec4d;
    Eigen::Matrix4d local = bbs.BuildLocalTransform();
    Eigen::Vector4d pc_domain = local * lw_pt;

    // scale point in between [0,1];
    double input_range_x = bbs.mx_pt(0) - bbs.mn_pt(0);
    double input_range_y = bbs.mx_pt(1) - bbs.mn_pt(1);
   // double output_range = 1.0 - 0.0;
    Eigen::Vector2d op;
    op(0) = (pc_domain(0) - bbs.mn_pt(0)) * w / (input_range_x + 0.0);
    op(1) = (pc_domain(1) - bbs.mn_pt(1)) * h / (input_range_y + 0.0);
    return op;
}
void CirconImageDescriptor::SetParametersforDescriptor(std::vector<Eigen::Vector2d> st_parameter)
{
    st_params = st_parameter;
}
void CirconImageDescriptor::SetBoundingBoxInformation(const surface::CloudBoundingBox &box)
{
    bbs = box;
  
}
void CirconImageDescriptor::SetBasicPointIndex(int Idx)
{
    basic_point_index = Idx;
}

int CirconImageDescriptor::GetBasicPointIndex()
{
    return basic_point_index;
}
int CirconImageDescriptor::GetDescriptorCellIndexForInterestPoint()
{
    return basic_cell_index;
}
void CirconImageDescriptor::SetRotationAxisPoint(PointNormalType rotpoint)
{
    RotationAxisPoint = rotpoint;
}
void CirconImageDescriptor::ConstructLocalFrameOfReference()
{
    Eigen::Vector3f query_idx_normal = RotationAxisPoint.getNormalVector3fMap();
    Eigen::Vector3f new_vec = Eigen::Vector3f(0.0, 1.0, 0.0).cross(query_idx_normal);
    Eigen::Vector3f x_bar = new_vec.normalized();// / new_vec.norm();
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
    rotation_matrix.row(0) = x_bar;
    rotation_matrix.row(1) = (query_idx_normal.cross(x_bar)).normalized();
    rotation_matrix.row(2) = query_idx_normal;
    Local_Coordinate_Frame.resize(3);
    Local_Coordinate_Frame[0] = rotation_matrix.row(0);
    Local_Coordinate_Frame[1] = rotation_matrix.row(1);
    Local_Coordinate_Frame[2] = rotation_matrix.row(2);
    
   // std::cout << query_idx_normal.transpose().rows() << "&" << query_idx_normal.transpose().cols() << std::endl;
    WorldLocalTransformation.setIdentity();
    WorldLocalTransformation.block<3, 3>(0, 0) = rotation_matrix;
    Eigen::Vector3f trans = - (rotation_matrix * RotationAxisPoint.getVector3fMap());
    WorldLocalTransformation.block<3, 1>(0, 3) = trans;
   // WorldLocalTransformation.col(3) = Eigen::Vector4f(trans(0), trans(1), trans(2), 1.0);
   // std::cout << WorldLocalTransformation * RotationAxisPoint.getVector4fMap() << std::endl;
   
}
 Eigen::Matrix4f CirconImageDescriptor::ConstructLocalCoordinateAxes(CirconImageDescriptor &cid, PointNormalType &axis_point)
{
     Eigen::Matrix4f localFrame;
     Eigen::Vector3f query_idx_normal = axis_point.getNormalVector3fMap();
     Eigen::Vector3f new_vec = Eigen::Vector3f(0.0, 1.0, 0.0).cross(query_idx_normal);
     Eigen::Vector3f x_bar = new_vec / new_vec.norm();
     Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
     rotation_matrix.row(0) = x_bar;
     rotation_matrix.row(1) = query_idx_normal.cross(x_bar);
     rotation_matrix.row(2) = query_idx_normal;
     localFrame.setIdentity();
     localFrame.block<3, 3>(0, 0) = rotation_matrix;
     Eigen::Vector3f trans = -(rotation_matrix * axis_point.getVector3fMap());
     localFrame.col(3) = Eigen::Vector4f(trans(0), trans(1), trans(2), 1.0);
     return localFrame;
}
 void CirconImageDescriptor::SetLocalFrame(const Eigen::Matrix4f &l_frame)
 {
     WorldLocalTransformation = l_frame;
 }
float CirconImageDescriptor::ComputeMaximumRadius(const CloudWithoutType& input_Cloud)
{
    Eigen::Vector3f Z_axis = Local_Coordinate_Frame[2]; // or use rot_matrix.row(2);
    float distance = 0.0f;
    float dist_pt = 0.0f;
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*input_Cloud, *pTarget);
    for (size_t i = 0; i < pTarget->points.size(); i++)
    {
        Eigen::Vector3f diff = RotationAxisPoint.getVector3fMap() - pTarget->points[i].getVector3fMap();
        Eigen::Vector3f cross = (diff.cross(Z_axis));
        float dist_pt = cross.norm() / Z_axis.norm();

        if (dist_pt > distance)
        {
            distance = dist_pt;
        }
    }
    return distance;
}
float CirconImageDescriptor::ComputeheightFromPointCloud(const CloudWithNormalPtr& input_Cloud)
{
    /*CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
   
    pcl::fromPCLPointCloud2(*input_Cloud, *pTarget);*/
    size_t size = input_Cloud->points.size();
    std::vector<PointNormalType>points(input_Cloud->points.begin(), input_Cloud->points.end());
   std::sort(points.begin(), points.end(), SortOnZ);
   float height = points[0].z - points[size - 1].z;
    return height;
}
void CirconImageDescriptor::Set2DCloud(const CloudPtr &cloud)
{
    pcl::copyPointCloud(*cloud, *Cloud2D);
    kdTree.setInputCloud(Cloud2D);
 
}
void CirconImageDescriptor::Set2DUniformGrid(CloudWithNormalPtr &in_cloud)
{
    /*cGrid2D.init( avgpoint_dist);
    cGrid2D.SetInputCloud(in_cloud);*/
}
CloudWithoutType CirconImageDescriptor::TransformPointToLocalFrame()
{
    transformed_inputCloud =  tool::TransFormationOfCloud(inputCloud, WorldLocalTransformation);
  //  CloudWithNormalPtr transformed_point_cloud(new pcl::PointCloud <PointNormalType>);
  //  pcl::transformPointCloudWithNormals(*original_cloud_with_normal, *transformed_point_cloud, WorldLocalTransformation);
  
    return transformed_inputCloud;

}
void CirconImageDescriptor::SetAngularResolution(float full_angle , int num_division)
{
    angle_resolution = full_angle / float(num_division);
}

void CirconImageDescriptor::SetRadialResolution(float distance, int num_division)
{
    rad_resolution = distance /float( num_division);
}
void CirconImageDescriptor::HeightResolution(float hei_resolution, int num_division)
{
    height_resolution = hei_resolution /float( num_division);
}
float CirconImageDescriptor::GetRadialResolution()
{
    return rad_resolution;
}
float CirconImageDescriptor::GetColumnDivision()
{
    return static_cast<float>(num_division_col);
}
float CirconImageDescriptor::GetAngularResolution()
{
    return angle_resolution;
}
float CirconImageDescriptor::GetHeightResolution()
{
    return height_resolution;
}
float CirconImageDescriptor::GetRowDivision()
{
    return num_division_row;
}
float CirconImageDescriptor::GetHeightDivision()
{
    return num_height_division;
}
void CirconImageDescriptor::SetImageDescriptorResolution(float full_angle, float max_radius, float height_max)
{
    int shortened_row_count = 64;
    if (false)
    {
        SetAngularResolution(full_angle, up_resolution_count);
        SetRadialResolution(max_radius, up_resolution_count);
        HeightResolution(height_max, num_height_division);
       // HeightResolution(height_max, num_height_division);
    }
    else
    {
        SetAngularResolution(full_angle, num_division_row);
        /*  if (num_division_row >= 256)
        SetRadialResolution(max_radius, num_division_row);
        else*/
       // std::cout << " I am here:" << std::endl;
        SetRadialResolution(max_radius, num_division_col);
        HeightResolution(height_max, num_division_col);
    }
}
void CirconImageDescriptor::SetDivision(int num_rows, int num_cols, int num_hei)
{
    num_division_row = num_rows;
    num_division_col = num_cols;
    num_height_division = num_hei;
}
void CirconImageDescriptor::SetInputAsTransformedCloud(CloudWithoutType InputCloud)
{
    transformed_inputCloud = inputCloud;
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*transformed_inputCloud, *pTarget);
}

void CirconImageDescriptor::GetRangeForImage(float &max, float &min)
{
    max = max_value_image;
    min = min_value_image;
}
void CirconImageDescriptor::WriteDescriptorAsImage(std::string FileName)
{
   std::vector<std::vector<float>>img_data = Image2D.getImageData();
   if (img_data.size() > 0)
   {
       //send the data after scaling it in the range of  0-255
       std::vector<std::vector<float>>scaled_data(num_division_row, std::vector<float>(num_division_col));
      
       float input_range = max_value_image - min_value_image;
       float output_range = 255.0 - 0.0;
       for (int i = 0; i < num_division_row; i++)
       {
           for (int j = 0; j < num_division_col; j++)
           {
               scaled_data[i][j] = std::roundf((img_data[i][j] - min_value_image) * output_range / input_range + 0.0);

           }
       }
       Image2D.WriteImage(FileName, scaled_data);
    //   //////////intrepolated data//////////////////
    //  std::vector<float>interpolated_image = tool::ReScaleImageBilinear(img_data, num_division_row, num_division_col, 8, 8);
    //  CirconImage intp_image(8, 8);
    //  intp_image.SetImageData(interpolated_image);
    //  float  min_value = INFINITY;
    //  int valid_index = 0;

    //  for (int itr = 0; itr < interpolated_image.size(); itr++)
    //  {
    //      if (interpolated_image[itr] != -INFINITY)
    //      {
    //          valid_index++;
    //          if (interpolated_image[itr] < min_value)
    //          {
    //              min_value = interpolated_image[itr];
    //          }
    //          else
    //          {
    //              continue;
    //          }
    //      }
    //      else
    //          continue;
    //  }
    //  std::vector<float>scaled_data_new;
    //  scaled_data_new.reserve(interpolated_image.size());
    //float  max_value_image = *(std::max_element(interpolated_image.begin(), interpolated_image.end()));
    //  float input_range_new = max_value_image - min_value;
    //  for (int i = 0; i < interpolated_image.size(); i++)
    //  {
    //      float output = std::roundf((interpolated_image[i] - min_value) * output_range / input_range_new + 0.0);
    //      scaled_data_new.push_back(output);
    //  }
    // /* int memsize = 64;
    //  unsigned char *data = new unsigned char[memsize];
    // 
    //  
    //  for (int i = 0; i < memsize; i++)
    //  {
    //      data[i] = static_cast<unsigned char>(interpolated_image[i]);
    //  }
    //  Image2D.WriteBitMap(pszFileName, data);*/
    //  std::string subName = "interpolated_image.bmp";
    //  std::string pszFileName = "Z:/staff/SDutta/GlobalRegistration/" + subName;
    //  intp_image.WriteImage(pszFileName, scaled_data_new);
      
   }
   else
   {
       std::runtime_error("no image data to write\n");
   }
}
std::vector<std::vector<float>> CirconImageDescriptor::GetDescriptorImage()
{
    return Image2D.getImageData();
}

Eigen::Matrix4f CirconImageDescriptor::GetTransformation()
{
    return WorldLocalTransformation;
}
void CirconImageDescriptor::ReconstructPointCloud()
{
    std::vector<std::vector<float>>img_data = Image2D.getImageData();
   
    reconstructed_points.clear();
    reconstructed_points.shrink_to_fit();
    reconstructed_points.reserve(num_division_row * num_division_col);
    int count = 0;
    for (int idx = 0; idx < num_division_row; idx++)
    {
        for (int icx = 0; icx < num_division_col; icx++)
        {
            if (img_data[idx][icx] != -INFINITY)
            {
                double step_size = max_radius / 512;
                float x_value = (icx * rad_resolution) * cos(-(idx - 1)* angle_resolution);
                float y_value = (icx * rad_resolution) * sin(-(idx - 1)* angle_resolution);
                float z_value = img_data[idx][icx] *  height_resolution;
                Eigen::Vector4f recon_pt(x_value, y_value, z_value, 1.0);
                recon_pt = WorldLocalTransformation.inverse() * recon_pt;
                reconstructed_points.emplace_back(recon_pt.head<3>());
                /* Eigen::Vector3f recon_pt = mat.inverse() * Eigen::Vector3f(x_value, y_value, z_value)  + RotationAxisPoint.getVector3fMap();
                 reconstructed_points.push_back(Eigen::Vector3f(recon_pt));*/
                 // std::cout << count << "," << i_idx << "," << j_idx << std::endl;
                count++;
            }
        }
    }
}
void CirconImageDescriptor::WritePointCloud(std::string fileName)
{
    std::string  fileNameWithLocation = fileName + ".ply";
    FILE *pFile;
    pFile = fopen(fileNameWithLocation.c_str(), "wb");
    fprintf(pFile, "ply\n");
    fprintf(pFile, "format ascii 1.0\n");
    fprintf(pFile, "element vertex %d\n", static_cast<int>(reconstructed_points.size()));
    fprintf(pFile, "property float x \n");
    fprintf(pFile, "property float y \n");
    fprintf(pFile, "property float z \n");
    fprintf(pFile, "end_header\n");
    for (int i = 0; i < reconstructed_points.size(); i++)
    {
        fprintf(pFile, "%f %f %f\r\n", reconstructed_points[i](0), reconstructed_points[i](1), reconstructed_points[i](2));
    }
    fclose(pFile);
}

std::vector<float> CirconImageDescriptor::TransformImageData(std::vector<float>prev_data, int query_index)
{
    // query_index -> index of the 2d image linearized as 1d array of floats
    std::vector<float>new_image_data(prev_data.size(), -INFINITY);
    std::map<int, size_t>::iterator iterator = index_index_map.find(query_index);
    if (iterator == index_index_map.end())
    {
        std::runtime_error("No point found to corresponding index\n");

    }
    size_t pointIndx = iterator->second;  // actual point index of the original input cloud
    Eigen::Vector3f query_idx_normal = original_cloud_with_normal->points[pointIndx].getNormalVector3fMap();
    Eigen::Vector3f new_vec = Eigen::Vector3f(0.0, 1.0, 0.0).cross(query_idx_normal);
    Eigen::Vector3f x_bar = new_vec / new_vec.norm();
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
    rotation_matrix.row(0) = x_bar;
    rotation_matrix.row(1) = query_idx_normal.cross(x_bar);
    rotation_matrix.row(2) = query_idx_normal;
    rot_matrix = rotation_matrix;  // update rotation matrix

    //
    //clear the old map and update it
    index_index_map.clear();
    for (int img_idx = 0; img_idx < prev_data.size(); img_idx++)
    {
        if ( prev_data[query_index] != -INFINITY)
        {
            int i_idx_curr = img_idx / num_division_col;
            int j_idx_curr = img_idx % num_division_col;

            int i_idx = query_index / num_division_col;
            int j_idx = query_index % num_division_col;

            float x_ij_dash = rad_resolution *(j_idx_curr *cos(angle_resolution *(i_idx_curr - 1)) - j_idx * cos(angle_resolution *(i_idx - 1)));
            float y_ij_dash = -rad_resolution *(j_idx_curr *sin(angle_resolution *(i_idx_curr - 1)) + j_idx * sin(angle_resolution *(i_idx - 1)));

            // Compute i-index for image
            float x_y = atan2(y_ij_dash, x_ij_dash);
            x_y = x_y / angle_resolution;
            float ang = static_cast<float>(num_division_row) - x_y;
            int row = static_cast<int>(std::round(ang)) % num_division_row;
            int row_index = row + 1;

            // compute j-index for image
            float sq_root_sum = std::sqrtf(pow(y_ij_dash, 2.0) + pow(x_ij_dash, 2.0));
            sq_root_sum = std::round(sq_root_sum / rad_resolution);
            int col_index = static_cast<int>(sq_root_sum);

            int updated_index = row_index * num_division_col + col_index;
            if ( updated_index < prev_data.size() && prev_data[img_idx] != -INFINITY)
            {
                float new_cij_value = prev_data[img_idx] - prev_data[query_index];
                float c_ij_prev = Image2D.getCell(row_index, col_index, num_division_col);
                new_image_data[updated_index] = new_cij_value;
         
                // rotation
              
                float z_value = new_cij_value * height_resolution;
                Eigen::Vector3f transformed_point = rotation_matrix * Eigen::Vector3f(x_ij_dash, y_ij_dash, z_value);
                float rotated_cij_value = std::round(transformed_point(2) / height_resolution);
           
                new_image_data[updated_index] = rotated_cij_value;
                index_index_map.insert(std::pair<int, size_t>(updated_index, img_idx));
            }
           
           
        }
        else
            continue;
    }

   float theta  =  GetRotationAngle();
   int num_rows_shift = std::round(theta / angle_resolution);  //number of rows to shift
   int row_index_shift = num_division_row - num_rows_shift - 1; // the actual row index w.r.t descriptor image
   int first_index_shift = row_index_shift * num_division_col;

   std::vector<float>revised_data;
   revised_data.reserve(new_image_data.size());
  // std::copy(new_image_data.begin() + first_index_shift, new_image_data.end(), std::back_inserter(revised_data));
   revised_data.insert(std::begin(revised_data), new_image_data.begin() + first_index_shift, new_image_data.end());
   revised_data.insert(std::end(revised_data), new_image_data.begin(), new_image_data.begin() + first_index_shift );
    Image2D.SetImageData(revised_data);

    //update range of image
    max_value_image = *(std::max_element(revised_data.begin(), revised_data.end()));
    float  min_value = INFINITY;
    for (int itr = 0; itr < revised_data.size(); itr++)
    {
        if (revised_data[itr] < min_value && revised_data[itr] != -INFINITY)
        {
            min_value = revised_data[itr];
        }
    }
    min_value_image = min_value;
    //update local frame information
    RotationAxisPoint = original_cloud_with_normal->points[pointIndx];
    ConstructLocalFrameOfReference();  // update lcoal frame
    rotation_index = num_rows_shift;
    return revised_data;
 
}
void CirconImageDescriptor::UpdateImageDataAfterTransformation(int index)
{
    // query_index -> index of the 2d image linearized as 1d array of floats
    sigma_threshold = rad_resolution / 16.0;  // set up threshold for non valid pts
  
    RotationAxisPoint = original_cloud_with_normal->points[index];
    ConstructLocalFrameOfReference();
    auto startItr = std::chrono::high_resolution_clock::now();
    CloudWithoutType transformed_cloud = TransformPointToLocalFrame();
   // float max_radius = GetRadiusFromCloud();
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*transformed_cloud, *pTarget);
    float height = ComputeheightFromPointCloud(pTarget);
    auto finishItr = std::chrono::high_resolution_clock::now();
    double executeTime = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
    executeTime = executeTime / double(1000);
  //  std::cout << "Time consumed current:" << executeTime << "sec" << std::endl;
    SetImageDescriptorResolution(2.0 *M_PI, max_radius, height);
    Image2D.clearMatrix();
    ////////prepare secondary descriptor///////////////////////
    ComputeFeature(index);
  
    rot_matrix = WorldLocalTransformation.block<3, 3>(0, 0);
   
    float theta = GetRotationAngle();
    int num_rows_shift = std::round((theta * num_division_row) / (2 * M_PI));  //number of rows to shift
    rotation_index = num_rows_shift;
}

void CirconImageDescriptor::UpdateDeescriptor(const PointNormalType /*Eigen::VectorXd*/ &pt)
{
    // query_index -> index of the 2d image linearized as 1d array of floats
    sigma_threshold = rad_resolution / 16.0;  // set up threshold for non valid pts
// Eigen::Vector3f query_idx_normal = original_cloud_with_normal->points[index].getNormalVector3fMap();
    RotationAxisPoint = pt;
    /*RotationAxisPoint.getVector3fMap() = pt.head<3>().cast<float>();
    RotationAxisPoint.getNormalVector3fMap() = pt.tail<3>().cast<float>();*/
    ConstructLocalFrameOfReference();
    auto startItr = std::chrono::high_resolution_clock::now();
    CloudWithoutType transformed_cloud = TransformPointToLocalFrame();
    // float max_radius = GetRadiusFromCloud();// ComputeMaximumRadius(transformed_cloud);
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*transformed_cloud, *pTarget);
    float height = ComputeheightFromPointCloud(pTarget);
    auto finishItr = std::chrono::high_resolution_clock::now();
    double executeTime = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
    executeTime = executeTime / double(1000);
    //  std::cout << "Time consumed current:" << executeTime << "sec" << std::endl;
    SetImageDescriptorResolution(2.0 *M_PI, max_radius, height);
    Image2D.clearMatrix();

    ComputeFeature(1);

    rot_matrix = WorldLocalTransformation.block<3, 3>(0, 0);

    float theta = GetRotationAngle();
    int num_rows_shift = std::round((theta * num_division_row) / (2 * M_PI));  //number of rows to shift
    rotation_index = num_rows_shift;
}
void CirconImageDescriptor::CreateSecondaryDescriptor(const PointNormalType /*Eigen::VectorXd*/ &pt,const cParameterGrid &pGrid, const CloudWithNormalPtr
&pTarget)
{
    // query_index -> index of the 2d image linearized as 1d array of floats
    sigma_threshold = rad_resolution / 16.0;  // set up threshold for non valid pts


   
  //  CloudWithoutType transformed_cloud = TransformPointToLocalFrame();

   // CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
  //  pcl::fromPCLPointCloud2(*transformed_cloud, *pTarget);

    Eigen::Vector3f min_pt;
    Eigen::Vector3f max_pt;
   // float diag_length1 = tool::ComputeOrientedBoundingBoxOfCloud(pTarget, min_pt, max_pt);
 //  pcl::transformPointCloudWithNormals(*original_cloud_with_normal, *pTarget, WorldLocalTransformation);
   // float height =  ComputeheightFromPointCloud(pTarget);
    CUniformGrid2D cGrid2D(avgpoint_dist,
        /* point accessor */
        [&pTarget](Eigen::Vector2d *out_pnt, double *out_attrib, size_t idx) -> bool
    {
        if (idx < pTarget->points.size())
        {
            Eigen::Vector2d vec(pTarget->points[idx].x, pTarget->points[idx].y);
            *out_pnt = vec;
            *out_attrib = pTarget->points[idx].z;
            return true;
        }
        else
            return false;
    }
        ); 
    SetImageDescriptorResolution(2.0 *M_PI, max_radius, max_radius);
    Image2D.clearMatrix();
    auto startItr = std::chrono::high_resolution_clock::now();
    ComputeFeature(cGrid2D, pGrid);
    auto finishItr = std::chrono::high_resolution_clock::now();
    double executeTime = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
    executeTime = executeTime / double(1000);
   // std::cout << "Time consumed for secondary descriptor:" << executeTime << "sec" << std::endl;

    rot_matrix = WorldLocalTransformation.block<3, 3>(0, 0);

    float theta = GetRotationAngle();
    int num_rows_shift = std::round((theta * num_division_row) / (2 * M_PI));  //number of rows to shift
    rotation_index = num_rows_shift;
}
float CirconImageDescriptor::GetAverageDist()
{
    return avgpoint_dist;
}
void  CirconImageDescriptor::SetMaxSearchColumn(int max_value)
{
    no_col_search = max_value;
}
void  CirconImageDescriptor::UpdateLocalFrameafterTransformation(PointNormalType new_origin, std::vector<Eigen::Vector3f> new_local_frame)
{
    RotationAxisPoint = new_origin;
    Local_Coordinate_Frame = new_local_frame;
}
 float CirconImageDescriptor::GetRejectionRatioOfPoints()
{
     return sigma_threshold;
}
 void CirconImageDescriptor::SetRotationIndex(int index)
 {
     rotation_index = index;
 }
int CirconImageDescriptor::GetRotationIndex()
{
    return rotation_index;
}
int CirconImageDescriptor::GetPointIndexFromCloud(int img_idx)
{
    std::map<int, size_t>::iterator iterator = index_index_map.find(img_idx);
    if (iterator == index_index_map.end())
    {
        return (-1);

    }
    int pointIndx = size_t (iterator->second);  // actual point index of the original input cloud
    return pointIndx;
}
CloudWithNormalPtr CirconImageDescriptor::GetoriginalCloud()
{
    return original_cloud_with_normal;
}

std::map<int, size_t> CirconImageDescriptor::GetImagePointMap()
{
    return index_index_map;
}


PointNormalType CirconImageDescriptor::GetRotationAXisPoint()
{
    return RotationAxisPoint;
}

float CirconImageDescriptor::GetRotationAngle()
{
    Eigen::Quaternionf qf(rot_matrix);
    qf.normalize();  // unit quaternion
    // quaternion q = cos(theta/2)+ sin(theta/2)u_xi + sin(theta/2)u_yj + sin(theta/2)u_zk, (u_x, u_y, u_z)-> rotation axis
   // float theta = 2.0 * acos(qf.w());  
    float y = sqrt(pow(qf.x(), 2.0) + pow(qf.y(), 2.0)+ pow( qf.z(), 2.0));
    float x = qf.w();
   float theta = 2.0 * atan2(y, x);
    return theta;
   /* if (x > 0.0)
    {
        theta = 2.0 * atan2(y, x);
        return theta;
    }
    else if (x < 0 && y >= 0.0)
    {
        theta = 2.0 *(atan2(y, x) + M_PI);
        return theta;
    }
    else if (x < 0 && y < 0.0)
    {
        theta = 2.0 *(atan2(y, x) - M_PI);
        return theta;
    }*/
}
void CirconImageDescriptor::SetMaximumAverageDistance(const float &dist)
{
    avgpoint_dist = dist;
}
void CirconImageDescriptor::SetMaximumRadius(float rad)
{
    max_radius = rad;
}
float CirconImageDescriptor::GetRadiusFromCloud()
{
    Eigen::Vector3f min_pt;
    Eigen::Vector3f max_pt;
    float diag_length = tool::ComputeOrientedBoundingBoxOfCloud(original_cloud_with_normal, min_pt, max_pt);
    return diag_length;
}
void CirconImageDescriptor::UpdateImageDimension(int col,int row)
{
    max_value_image = -INFINITY;
    min_value_image = -INFINITY;
    Image2D.SetImage(col, row);
}
std::vector<std::vector<_dV>>CirconImageDescriptor::GetDescriptorContent()
{
    return descriptor_content;
}

std::vector<ON_NurbsSurface> CirconImageDescriptor::GetNurbsSurface()
{
    return nb_surface;
}

std::vector<ON_NurbsCurve> CirconImageDescriptor::GetNurbsCurve()
{
    return nb_curve;
}
void CirconImageDescriptor::SetNurbsSurfaceAndCurve(const ON_NurbsSurface &nbs)
{
   // nb_surface = nbs;
 
}

std::vector<Eigen::Vector2i>CirconImageDescriptor::GetClusterLabels()
{
    return Cluster_labels;
}

CirconImage::CirconImage(int columns, int rows)
{

    this->columns = columns;
    this->rows = rows;
    int total_size = rows *columns;
    cell.resize(total_size, -INFINITY);
    image.resize(rows, std::vector<float>(columns, -INFINITY));

}

CirconImage::CirconImage(const CirconImage &img)
{
    columns = img.columns;
    rows = img.rows;
    cell = img.cell;
    image = img.image;
}
CirconImage::CirconImage(CirconImage &&img)
{
    columns = std::move(img.columns);
    rows = std::move(img.rows);
    cell = std::move(img.cell);
    image = std::move(img.image);
}
CirconImage& CirconImage:: operator=(const CirconImage &img)
{
    if (this == &img)
        return *this;
    columns = img.columns;
    rows = img.rows;
    cell = img.cell;
    image = img.image;
    return *this;
}
CirconImage& CirconImage:: operator=( CirconImage &&img)
{
    if (this == &img)
        return *this;
    columns = std::move(img.columns);
    rows = std::move(img.rows);
    cell = std::move(img.cell);
    image = std::move(img.image);
    img.columns = -1;
    img.rows = -1;
    return *this;
}
CirconImage::~CirconImage()
{
    cell.clear();
    image.clear();
}
void CirconImage::addCellAt(int row, int col, int col_div, float entry)
{
    int index = row * col_div + col;
    cell[index] = entry;
  //  std::cout << "row:" << row << std::endl;
    image[row][col] = entry;
}
float CirconImage::getCell(int row, int column, int col_div)
{
    int index = row * col_div + column;
    // image[row][column] = entry;
    return cell[index];
}

void CirconImage::clearMatrix()
{
    cell.clear();
    this->columns = columns;
    this->rows = rows;
    int total_size = rows *columns;
    cell.resize(total_size, -INFINITY);
    image.resize(rows, std::vector<float>(columns, -INFINITY));
}
std::vector<std::vector<float>> CirconImage::getImageData()
{
    return image;
}
void CirconImage::WriteImage(std::string &pszFileName, std::vector<std::vector<float>>ImageData)
{
    int memsize = rows * columns;
    unsigned char *data = new unsigned char[memsize];
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            data[i * columns + j] = static_cast<unsigned char>(ImageData[i][j]);
        }
    }
    WriteBitMap(pszFileName, data);
    delete[] data;
    data = NULL;
    
}
unsigned int CirconImage::WriteBitMap(std::string &pszFileName, unsigned char *pImgData)
{
    /* if (PIX_FORMAT_8BPP != format.PixFmt)
    return -1;*/

#pragma pack(push, 2)
    typedef struct tagBMPFILEHEADER {
        unsigned short    bfType;
        unsigned long   bfSize;
        unsigned short    bfReserved1;
        unsigned short    bfReserved2;
        unsigned long   bfOffBits;
    } BMPFILEHEADER, *PBMPFILEHEADER;
#pragma pack(pop)

    typedef struct tagBMPINFOHEADER {
        unsigned long  biSize;
        long   biWidth;
        long   biHeight;
        unsigned short   biPlanes;
        unsigned short   biBitCount;
        unsigned long  biCompression;
        unsigned long  biSizeImage;
        long   biXPelsPerMeter;
        long   biYPelsPerMeter;
        unsigned long  biClrUsed;
        unsigned long  biClrImportant;
    } BMPINFOHEADER, *PBMPINFOHEADER;


    typedef struct tagRGBQUAD {
        unsigned char    rgbBlue;
        unsigned char    rgbGreen;
        unsigned char    rgbRed;
        unsigned char    rgbReserved;
    } RGBQU;

    BMPFILEHEADER	bmpFileHeader;
    BMPINFOHEADER	bmpInfoHeader;
    RGBQU			rgbQuad[256];

    bmpFileHeader.bfType = 'B' + 256 * 'M';		// ASCII-string "BM"
    bmpFileHeader.bfSize = sizeof(BMPFILEHEADER) + sizeof(BMPINFOHEADER) + sizeof(rgbQuad)
        + rows * columns * 1;// format.BytesPerPixel;
    bmpFileHeader.bfReserved1 = 0;
    bmpFileHeader.bfReserved2 = 0;
    bmpFileHeader.bfOffBits = sizeof(BMPFILEHEADER) + sizeof(BMPINFOHEADER) + sizeof(rgbQuad);

    bmpInfoHeader.biSize = sizeof(BMPINFOHEADER);
    bmpInfoHeader.biWidth = columns;
    bmpInfoHeader.biHeight = rows;
    bmpInfoHeader.biPlanes = 1;
    bmpInfoHeader.biBitCount = 1 * 8;        //format.BytesPerPixel * 8;
    bmpInfoHeader.biCompression = 0; // BI_RGB
    bmpInfoHeader.biSizeImage = 0;
    bmpInfoHeader.biXPelsPerMeter = 1000;
    bmpInfoHeader.biYPelsPerMeter = 1000;
    bmpInfoHeader.biClrUsed = 0;
    bmpInfoHeader.biClrImportant = 0;

    for (int c = 0; c < 256; c++)
    {
        rgbQuad[c].rgbRed = c;
        rgbQuad[c].rgbGreen = c;
        rgbQuad[c].rgbBlue = c;
        rgbQuad[c].rgbReserved = 0;
    }

    FILE *fp = NULL;
    if (fopen_s(&fp, pszFileName.c_str(), "wb")) return -1;
    fwrite(&bmpFileHeader, sizeof(BMPFILEHEADER), 1, fp);
    fwrite(&bmpInfoHeader, sizeof(bmpInfoHeader), 1, fp);
    fwrite(&rgbQuad, sizeof(rgbQuad), 1, fp);


    for (int	y = rows - 1, stride = y * columns;
        y >= 0;
        y--, stride -= columns)
    {
        fwrite(pImgData + stride, columns, 1, fp);
    }

    fclose(fp);
    return 0;
}
void CirconImage::SetImageData(std::vector<float>new_img_data)
{
    cell = new_img_data;
}
void CirconImage::SetImage(int col, int row)
{
    if (cell.size() != 0)
    {
        cell.clear();
        image.clear();
    }
    this->columns = col;
    this->rows = row;
    int total_size = row *col;
    cell.resize(total_size, -INFINITY);
    image.resize(rows, std::vector<float>(col, -INFINITY));
}