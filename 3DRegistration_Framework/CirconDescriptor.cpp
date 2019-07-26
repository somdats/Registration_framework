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
  
    descriptor_content = cid.descriptor_content;
    reconstructed_points = cid.reconstructed_points;
    nb_curve = cid.nb_curve;
    nb_surface = cid.nb_surface;
    no_col_search = cid.no_col_search;
    bbs = cid.bbs;
    vector_of_maps = cid.vector_of_maps;
    st_params = cid.st_params;
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
    reconstructed_points = cid.reconstructed_points;
    descriptor_content = cid.descriptor_content;
    nb_curve = cid.nb_curve;
    nb_surface = cid.nb_surface;
    no_col_search = cid.no_col_search;
    vector_of_maps = cid.vector_of_maps;
    st_params = cid.st_params;
    bbs = cid.bbs;

}
void CirconImageDescriptor:: ComputeFeature()
{
    int col_reduced = 16;
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
   pcl::fromPCLPointCloud2(*transformed_inputCloud, *pTarget);
  
    size_t total_num_points = pTarget->points.size();
    index_index_map.clear();
   // int i = 0;
   // Eigen::Vector3f test1(0.0, 0.0, 0.0);
    descriptor_content.clear();
    descriptor_content.shrink_to_fit();
    _dV dv;
    descriptor_content.resize(num_division_row *num_division_col, dv);
    std::map<float, int>d_map = { {-INFINITY, -1} };
    vector_of_maps.resize(num_division_row *num_division_col, d_map);
    bool use_z_max = false;
   
     Eigen::Matrix4d tfs = Eigen::Matrix4d::Identity();
    Eigen::Matrix4f temp_wl = WorldLocalTransformation.inverse();
    std::unique_ptr<ON_NurbsSurface> nb_surface_tfs = surface::cNurbsSurface::TransformControlPointsOfNurbsSurface(nb_surface, WorldLocalTransformation.cast<double>()); //
 
    surface::Ray_Intersect r_it(*nb_surface_tfs);
    r_it.x0 = nb_surface.Knot(0, 0);
    r_it.x1 = nb_surface.Knot(0, nb_surface.KnotCount(0) - 1);
    r_it.w = r_it.x1 - r_it.x0;
    r_it.y0 = nb_surface.Knot(1, 0);
    r_it.y1 = nb_surface.Knot(1, nb_surface.KnotCount(1) - 1);
    r_it.h = r_it.y1 - r_it.y0;
    std::map < int, std::pair<int, int>> int_uv_map;
//#pragma omp parallel for
    for (int itr = 0; itr < total_num_points; itr++)
    {
        if (true/*itr != basic_point_index*/)
        {
        
            // Compute i-index for image
            float x_y = (atan2(pTarget->points[itr].y, pTarget->points[itr].x))/angle_resolution;
            float ang = static_cast<float>(num_division_row) - x_y;
            int row_index = static_cast<int>(std::round(ang)) % num_division_row;
    
            // compute j-index for image
            float sq_root_sum = std::sqrtf(pow(pTarget->points[itr].y, 2.0) + pow(pTarget->points[itr].x, 2.0));
            int col_index = static_cast<int>(std::round(sq_root_sum / rad_resolution));
        
            if (col_index < 0)
                std::cout << "wrong col_index:" << sq_root_sum << itr << std::endl;

            if (row_index < num_division_row && col_index < num_division_col)
            {
                int curr_position = row_index * num_division_col + col_index;
                // compute current value at(i,j)
                float c_ij_current = (std::round(pTarget->points[itr].z / height_resolution));
                // prev value at posn(i,j)
                float c_ij_prev = Image2D.getCell(row_index, col_index, num_division_col);
                // store index of poi
                if (itr == basic_point_index)
                {
                    basic_cell_index = curr_position;
                }



                if ( use_z_max  && c_ij_current > c_ij_prev)
                {
                    float dist = (pTarget->points[itr].getVector3fMap() - Eigen::Vector3f(0.0, 0.0, 0.0)).norm();
                    if (dist > eps)  // bypass
                    {
                        Image2D.addCellAt(row_index, col_index, num_division_col, c_ij_current);
                        #pragma omp critical
                        index_index_map[curr_position] = itr;
                        Eigen::Vector2d vec2d(0, 0);
                        Eigen::VectorXd pt =Eigen::VectorXd::Zero(6);
                         /*   pt
                                = original_cloud_with_normal->points[itr].getVector3fMap();*/
                        descriptor_content[curr_position]= (_dV(row_index, col_index, curr_position, itr, c_ij_current, vec2d, pt));
                       // i++;
                    }
                    else 
                        continue;
                }
                else if (!use_z_max && c_ij_current != -INFINITY)
                {
                    vector_of_maps[curr_position][pTarget->points[itr].z] = itr;
                    int_uv_map[itr] = {row_index, col_index};
                }

            }
            else
                continue;
        }
        else
            continue;
    }
   
    float  min_value = INFINITY;
    if (use_z_max)
    {
        std::vector<float>data = Image2D.getImageData();
        max_value_image = *(std::max_element(data.begin(), data.end()));
       
        int valid_index = 0;
        std::vector<_dV>refined_descriptor_content;

        for (int itr = 0; itr < data.size(); itr++)
        {
            if (data[itr] != -INFINITY)
            {
                valid_index++;
                //  refined_descriptor_content.push_back(descriptor_content[itr]);
                if (data[itr] < min_value)
                {
                    min_value = data[itr];
                }
                else
                {
                    continue;
                }

            }
            else
                continue;
        }
    }
    else
    {
        std::vector<std::map<float, int>>::iterator it;
        int n = 1;

        for (it = vector_of_maps.begin(); it != vector_of_maps.end(); it++)
        {
            if (it->size() > 1)
            {
                // std:: map<float, int>::iterator itr;
                for (auto itt = it->rbegin(); itt != --it->rend(); ++itt)
                {

                    r_it.xyz.head<2>() = Eigen::Vector2d(pTarget->points[itt->second].x, pTarget->points[itt->second].y);
                    r_it.init_param = st_params[itt->second];
                    Eigen::Vector2d optim_parameter;
                    std::pair<int, int> uv = int_uv_map[itt->second];
                    if (uv.second < col_reduced)
                    {
                        double error = surface::cNurbsSurface::ComputeOptimizedParameterSpaceValue(r_it, 1e-5, optim_parameter, false);
                        if (error < 1e-4)
                        {

                            float val = std::round(itt->first / height_resolution);

                            Image2D.addCellAt(uv.first, uv.second, num_division_col, val);
                            int linearized_cell_index = uv.first * num_division_col + uv.second;
                            Eigen::VectorXd pt = Eigen::VectorXd::Zero(6);
                            pt.head<3>() = original_cloud_with_normal->points[itt->second].getVector3fMap().cast<double>();
                            pt.tail<3>() = original_cloud_with_normal->points[itt->second].getNormalVector3fMap().cast<double>();
                            descriptor_content[linearized_cell_index] = _dV(uv.first, uv.second, linearized_cell_index, itt->second,
                                val, optim_parameter, pt);
                            if (val < min_value)
                            {
                                min_value = val;
                            }
                            break;
                        }
                    }

                }

            }
        }
     
    }
    std::vector<float>data = Image2D.getImageData();
    max_value_image = *(std::max_element(data.begin(), data.end()));
    min_value_image =  min_value;
   // descriptor_content = refined_descriptor_content;
  
}
void CirconImageDescriptor::ComputeFeature(int i)
{
    auto startItr = std::chrono::high_resolution_clock::now();
    index_index_map.clear();
    descriptor_content.clear();
    descriptor_content.shrink_to_fit();
    _dV dv;
    descriptor_content.resize(num_division_row *num_division_col,dv);
    Eigen::Matrix4d tfs = Eigen::Matrix4d::Identity();
    Eigen::Matrix4f temp_wl = WorldLocalTransformation.inverse();
    std::unique_ptr<ON_NurbsSurface> nb_surface_tfs = surface::cNurbsSurface::TransformControlPointsOfNurbsSurface(nb_surface, WorldLocalTransformation.cast<double>()); //
   
   /* std::unique_ptr<ON_NurbsCurve> nb_curve_tfs = surface::cNurbsCurve::TransformControlPointsOfNurbsCurve(nb_curve,
        WorldLocalTransformation.cast<double>());*/
    surface::Ray_Intersect r_it(*nb_surface_tfs);
    r_it.x0 = nb_surface.Knot(0, 0);
    r_it.x1 = nb_surface.Knot(0, nb_surface.KnotCount(0) - 1);
    r_it.w = r_it.x1 - r_it.x0;
    r_it.y0 = nb_surface.Knot(1, 0);
    r_it.y1 = nb_surface.Knot(1, nb_surface.KnotCount(1) - 1);
    r_it.h = r_it.y1 - r_it.y0;
    auto finishItr = std::chrono::high_resolution_clock::now();
    double executeTime = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
    executeTime = executeTime / double(1000);
   // std::cout << "Time consumed for optimization:" << executeTime << "sec" << std::endl;
    bool init_param_change = false;
    Eigen::Vector2d optim_paramter;
    for (int i = 0; i < num_division_row; i++)
    {
       
        for (int j = 0; j < num_division_col; j++)
        {
            int curr_position = i * num_division_col + j;
            //if (curr_position == 661 || curr_position == 662)
            //   // std::cout << " Iam here:" << std::endl;
            double x = static_cast<double>((j * rad_resolution) * cos(-(i - 1)* angle_resolution));
            double y = static_cast<double>((j * rad_resolution) * sin(-(i - 1)* angle_resolution));
            r_it.xyz.head<2>() = Eigen::Vector2d(x, y);
            
            if (!init_param_change)
            {
                r_it.init_param = TransformAndScaleParametricCoordinate(Eigen::Vector2d(x, y), r_it.w, r_it.h);

                surface::cNurbsSurface::ComputeOptimizedParameterSpaceValue(r_it, 1e-5, optim_paramter);
            }
            else
            { 
                r_it.init_param = optim_paramter;
                surface::cNurbsSurface::ComputeOptimizedParameterSpaceValue(r_it, 1e-5, optim_paramter,false);
            }
        
           /*if (min_error > 1e-4)
           {
               std::cout << "current posn:" << curr_position << " " << "error minimized value after optimization"
                   << min_error << std::endl;>	3DRegistration_Framework_release.dll!CirconImageDescriptor::ComputeFeature(int i) Line 225	C++

               continue;
           }*/
            Eigen::VectorXd pt;
            Eigen::Vector2d parameter;
            std::vector<Eigen::Vector2d> op_parameter = { optim_paramter };
            bool status_inside = surface::cNurbsSurface::EvaluateParameterForCoordinate(op_parameter, *nb_surface_tfs, nb_curve, pt, parameter);
            float c_ij_current;
           
            if (status_inside)
            {
                c_ij_current = std::round(pt(2) / height_resolution);
                Image2D.addCellAt(i, j, num_division_col, c_ij_current);
                if (j < no_col_search)
                {
                    Eigen::Matrix3f rot = temp_wl.block<3, 3>(0, 0);
                    Eigen::Vector3f trans = temp_wl.block<3, 1>(0, 3);
                    Eigen::VectorXf pt_dash = Eigen::VectorXf::Ones(6);
                    pt_dash.head<3>() = rot * pt.head<3>().cast<float>() + trans;
                    pt_dash.tail<3>() = rot * pt.tail<3>().cast<float>();
                    descriptor_content[curr_position] = (_dV(i, j, -1, -1, c_ij_current, parameter, pt_dash.cast<double>()));  // replace ) by itr
                }
                init_param_change = true;
            }
            else
            {
                init_param_change = false;
            }
          
            // prev value at posn(i,j)
          /*  float c_ij_prev = Image2D.getCell(i, j, num_division_col);
            if (c_ij_current > c_ij_prev)
            {*/
               // int curr_position = i * num_division_col + j;
               
                // i++;

           // }

        }
    }
    std::vector<float>data = Image2D.getImageData();
    max_value_image = *(std::max_element(data.begin(), data.end()));
    float  min_value = INFINITY;
    int valid_index = 0;
//std::vector<_dV>refined_descriptor_content;

    for (int itr = 0; itr < data.size(); itr++)
    {
        if (data[itr] != -INFINITY)
        {
            valid_index++;
            //  refined_descriptor_content.push_back(descriptor_content[itr]);
            if (data[itr] < min_value)
            {
                min_value = data[itr];
            }
            else
            {
                continue;
            }

        }
        else
            continue;
    }

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
    pcl::KdTreeFLANN<PointNormalType> KdTree;
    KdTree.setInputCloud(original_cloud_with_normal);
    PointNormalType centroid_point;
    centroid_point.getVector3fMap() = centroid.head<3>();
    KdTree.nearestKSearch(centroid_point, K, SearchResults, SearchDistances);
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
    Eigen::Vector3f x_bar = new_vec / new_vec.norm();
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
    rotation_matrix.row(0) = x_bar;
    rotation_matrix.row(1) = query_idx_normal.cross(x_bar);
    rotation_matrix.row(2) = query_idx_normal;
    Local_Coordinate_Frame.resize(3);
    Local_Coordinate_Frame[0] = rotation_matrix.row(0);
    Local_Coordinate_Frame[1] = rotation_matrix.row(1);
    Local_Coordinate_Frame[2] = rotation_matrix.row(2);

    
    WorldLocalTransformation.setIdentity();
    WorldLocalTransformation.block<3, 3>(0, 0) = rotation_matrix;
    Eigen::Vector3f trans = - (rotation_matrix * RotationAxisPoint.getVector3fMap());
    WorldLocalTransformation.col(3) = Eigen::Vector4f(trans(0), trans(1), trans(2), 1.0);
   
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
float CirconImageDescriptor::ComputeheightFromPointCloud(const CloudWithoutType& input_Cloud)
{
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
   
    pcl::fromPCLPointCloud2(*input_Cloud, *pTarget);
    size_t size = pTarget->points.size();
    std::vector<PointNormalType>points(pTarget->points.begin(), pTarget->points.end());
   std::sort(points.begin(), points.end(), SortOnZ);
   float height = points[0].z - points[size - 1].z;
    return height;
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
    SetAngularResolution(full_angle, num_division_row);
    SetRadialResolution(max_radius, num_division_col);
    HeightResolution(height_max, num_height_division);
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
void CirconImageDescriptor::WriteDescriptorAsImage(std::string FileName)
{
   std::vector<float>img_data = Image2D.getImageData();
   if (img_data.size() > 0)
   {
       //send the data after scaling it in the range of  0-255
       std::vector<float>scaled_data;
       scaled_data.reserve(img_data.size());
       float input_range = max_value_image - min_value_image;
       float output_range = 255.0 - 0.0;
       for (int i = 0; i < img_data.size(); i++)
       {
           float output = std::roundf((img_data[i] - min_value_image) * output_range / input_range + 0.0);
           scaled_data.push_back(output);
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
std::vector<float> CirconImageDescriptor::GetDescriptorImage()
{
    return Image2D.getImageData();
}

Eigen::Matrix4f CirconImageDescriptor::GetTransformation()
{
    return WorldLocalTransformation;
}
void CirconImageDescriptor::ReconstructPointCloud()
{
    std::vector<float>img_data = Image2D.getImageData();
    /*Eigen::Matrix3f mat = Eigen::Matrix3f::Identity();
    mat.row(0) = Local_Coordinate_Frame[0];
    mat.row(1) = Local_Coordinate_Frame[1];
    mat.row(2) = Local_Coordinate_Frame[2];*/
    reconstructed_points.clear();
    reconstructed_points.shrink_to_fit();
    int count = 0;
    for (int idx = 0; idx < img_data.size(); idx++)
    {
        if (img_data[idx] != -INFINITY)
        {
            int i_idx = idx / num_division_col;
            int j_idx = idx % num_division_col;
            float x_value = (j_idx * rad_resolution) * cos(-(i_idx - 1)* angle_resolution) ;
            float y_value = (j_idx * rad_resolution) * sin(-(i_idx - 1)* angle_resolution) ;
            float z_value = img_data[idx] * height_resolution ;
            Eigen::Vector4f recon_pt(x_value, y_value, z_value, 1.0);
            recon_pt = WorldLocalTransformation.inverse() * recon_pt;
            reconstructed_points.push_back(recon_pt.head<3>());
           /* Eigen::Vector3f recon_pt = mat.inverse() * Eigen::Vector3f(x_value, y_value, z_value)  + RotationAxisPoint.getVector3fMap();
            reconstructed_points.push_back(Eigen::Vector3f(recon_pt));*/
           // std::cout << count << "," << i_idx << "," << j_idx << std::endl;
            count++;
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
    
   // Eigen::Vector3f query_idx_normal = original_cloud_with_normal->points[index].getNormalVector3fMap();
    RotationAxisPoint = original_cloud_with_normal->points[index];
    ConstructLocalFrameOfReference();
    auto startItr = std::chrono::high_resolution_clock::now();
    CloudWithoutType transformed_cloud = TransformPointToLocalFrame();
   // float max_radius = GetRadiusFromCloud();// ComputeMaximumRadius(transformed_cloud);
    float height = ComputeheightFromPointCloud(transformed_cloud);
    auto finishItr = std::chrono::high_resolution_clock::now();
    double executeTime = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
    executeTime = executeTime / double(1000);
  //  std::cout << "Time consumed current:" << executeTime << "sec" << std::endl;
    SetImageDescriptorResolution(2.0 *M_PI, max_radius, height);
    Image2D.clearMatrix();
  
    ComputeFeature();
  
    rot_matrix = WorldLocalTransformation.block<3, 3>(0, 0);
   
    float theta = GetRotationAngle();
    int num_rows_shift = std::round((theta * num_division_row) / (2 * M_PI));  //number of rows to shift
    rotation_index = num_rows_shift;
}

void CirconImageDescriptor::UpdateDeescriptor(const Eigen::VectorXd &pt)
{
    // query_index -> index of the 2d image linearized as 1d array of floats
    sigma_threshold = rad_resolution / 16.0;  // set up threshold for non valid pts

                                              // Eigen::Vector3f query_idx_normal = original_cloud_with_normal->points[index].getNormalVector3fMap();
    RotationAxisPoint.getVector3fMap() = pt.head<3>().cast<float>();
    RotationAxisPoint.getNormalVector3fMap() = pt.tail<3>().cast<float>();
    ConstructLocalFrameOfReference();
    auto startItr = std::chrono::high_resolution_clock::now();
    CloudWithoutType transformed_cloud = TransformPointToLocalFrame();
    // float max_radius = GetRadiusFromCloud();// ComputeMaximumRadius(transformed_cloud);
    float height = ComputeheightFromPointCloud(transformed_cloud);
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
void CirconImageDescriptor::CreateSecondaryDescriptor(const Eigen::VectorXd &pt)
{
    // query_index -> index of the 2d image linearized as 1d array of floats
    sigma_threshold = rad_resolution / 16.0;  // set up threshold for non valid pts

    RotationAxisPoint.getVector3fMap() = pt.head<3>().cast<float>();
    RotationAxisPoint.getNormalVector3fMap()= pt.tail<3>().cast<float>();
    ConstructLocalFrameOfReference();
    auto startItr = std::chrono::high_resolution_clock::now();
    CloudWithoutType transformed_cloud = TransformPointToLocalFrame();
    // float max_radius = GetRadiusFromCloud();// ComputeMaximumRadius(transformed_cloud);
    float height = ComputeheightFromPointCloud(transformed_cloud);
    auto finishItr = std::chrono::high_resolution_clock::now();
    double executeTime = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
    executeTime = executeTime / double(1000);
    //  std::cout << "Time consumed current:" << executeTime << "sec" << std::endl;
    SetImageDescriptorResolution(2.0 *M_PI, max_radius, height);
    Image2D.clearMatrix();

    ComputeFeature();

    rot_matrix = WorldLocalTransformation.block<3, 3>(0, 0);

    float theta = GetRotationAngle();
    int num_rows_shift = std::round((theta * num_division_row) / (2 * M_PI));  //number of rows to shift
    rotation_index = num_rows_shift;
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
CirconImage::CirconImage(int columns, int rows)
{

        this->columns = columns;
        this->rows = rows;
        int total_size = rows *columns;
        cell.resize(total_size, -INFINITY);
        image.resize(rows,std::vector<float>(columns,-INFINITY));
 
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
std::vector<_dV>CirconImageDescriptor::GetDescriptorContent()
{
    return descriptor_content;
}

std::unique_ptr<ON_NurbsSurface> CirconImageDescriptor::GetNurbsSurface()
{
    return std::make_unique<ON_NurbsSurface>(nb_surface);
}

std::unique_ptr<ON_NurbsCurve> CirconImageDescriptor::GetNurbsCurve()
{
    return std::make_unique<ON_NurbsCurve>(nb_curve);
}
void CirconImageDescriptor::SetNurbsSurfaceAndCurve(const ON_NurbsSurface &nbs)
{
    nb_surface = nbs;
 
}
CirconImage::~CirconImage()
{
    cell.clear();
}
void CirconImage::addCellAt(int row, int col, int col_div, float entry)
{
    int index = row * col_div + col;
    cell[index] = entry;
}
float CirconImage::getCell(int row, int column, int col_div)
{
    int index = row * col_div + column;
    return cell[index];
}

void CirconImage::clearMatrix()
{
    cell.clear();
    this->columns = columns;
    this->rows = rows;
    int total_size = rows *columns;
    cell.resize(total_size, -INFINITY);
}
std::vector<float> CirconImage::getImageData()
{
    return cell;
}
void CirconImage::WriteImage(std::string &pszFileName, std::vector<float>ImageData)
{
    int memsize = rows * columns;
    unsigned char *data = new unsigned char[memsize];
    for (int i = 0; i < memsize; i++)
    {
        data[i] = static_cast<unsigned char>(ImageData[i]);
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
    }
    this->columns = col;
    this->rows = row;
    int total_size = row *col;
    cell.resize(total_size, -INFINITY);
}