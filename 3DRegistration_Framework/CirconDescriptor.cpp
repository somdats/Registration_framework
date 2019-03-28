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

    transformed_inputCloud.reset(new pcl::PCLPointCloud2);
    *transformed_inputCloud = *cid.transformed_inputCloud;
    original_cloud_with_normal.reset(new pcl::PointCloud <PointNormalType>);
    *original_cloud_with_normal = *cid.original_cloud_with_normal;
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
    transformed_inputCloud.reset(new pcl::PCLPointCloud2);
    *transformed_inputCloud = *cid.transformed_inputCloud;
    original_cloud_with_normal.reset(new pcl::PointCloud <PointNormalType>);
    *original_cloud_with_normal = *cid.original_cloud_with_normal;

}
void CirconImageDescriptor:: ComputeFeature()
{
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
   pcl::fromPCLPointCloud2(*transformed_inputCloud, *pTarget);
   //pcl::copyPointCloud(*original_cloud_with_normal, *pTarget);
    size_t total_num_points = pTarget->points.size();
    index_index_map.clear();
    int i = 0;
    Eigen::Vector3f test1(0.0, 0.0, 0.0);
    for (size_t itr = 0; itr < total_num_points; itr++)
    {
        if (true/*itr != basic_point_index*/)
        {
         /*   if (pTarget->points[itr].getVector3fMap() == test1)
            {
                std::cout << " I am at basic index point" << std::endl;
            }*/

            // Compute i-index for image
            float x_y = atan2(pTarget->points[itr].y, pTarget->points[itr].x);
            x_y = x_y / angle_resolution;
            float ang = static_cast<float>(num_division_row) - x_y;
            int row = static_cast<int>(std::round(ang)) % num_division_row;
            int row_index = row + 1;

            // compute j-index for image
            float sq_root_sum = std::sqrtf(pow(pTarget->points[itr].y, 2.0) + pow(pTarget->points[itr].x, 2.0));
            sq_root_sum = std::round(sq_root_sum / rad_resolution);
            int col_index = static_cast<int>(sq_root_sum);
            if (row_index < num_division_row && col_index < num_division_col)
            {
                // compute value at(i,j)
                float c_ij_current = (std::round(pTarget->points[itr].z / height_resolution));
                int curr_position = row_index * num_division_col + col_index;
             /*   if (curr_position == num_division_col)
                    std::cout << "At column division position" << std::endl;*/
                float c_ij_prev = Image2D.getCell(row_index, col_index, num_division_col);
                if (itr == basic_point_index)
                {
                    basic_cell_index = curr_position;
                }
                if (c_ij_current > c_ij_prev)
                {
                    float dist = (pTarget->points[itr].getVector3fMap() - test1).norm();
                    if (dist > eps)  // bypass
                    {
                        Image2D.addCellAt(row_index, col_index, num_division_col, c_ij_current);
                        index_index_map[curr_position] = itr;
                        /* index_index_map.insert(std::pair<int, size_t>(curr_position, itr));
                         std::map<int, size_t>::iterator it = index_index_map.find(curr_position);
                         if (it != index_index_map.end())
                             (*it).second = itr;*/
                        i++;
                    }
                    else
                        continue;
                }
                else
                    continue;
                //if ((pTarget->points[itr].getVector3fMap()- test1).norm() < eps)  // bypass
                //{
                //    index_index_map[curr_position] = itr;
                //}
            }
            else
                continue;
        }
        else
            continue;
    }
    std::vector<float>data = Image2D.getImageData();
    max_value_image = *(std::max_element(data.begin(), data.end()));
   float  min_value = INFINITY;
   int valid_index = 0;
    for (int itr = 0; itr < data.size(); itr++)
    {
        if (data[itr] != -INFINITY)
        {
            valid_index++;
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
        basic_point_index = static_cast<size_t>(SearchResults[0]);
        return original_cloud_with_normal->points[SearchResults[0]];
    }
    else
    {
        throw std::runtime_error("No closest point found.");
    }
    
}
void CirconImageDescriptor::SetBasicPointIndex(size_t Idx)
{
    basic_point_index = Idx;
}

size_t CirconImageDescriptor::GetBasicPointIndex()
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

    //std::vector<Eigen::Vector3f>local_frame;
    //local_frame.resize(3);
    //Eigen::Vector3f Z_Local = RotationAxisPoint.getNormalVector3fMap();
    //local_frame[2] = Z_Local; // Z-Axis for rotation

    //Eigen::Vector3f Y_World(0.0, 1.0, 0.0);
    //Eigen::Vector3f X_Local = Z_Local.cross(Y_World);
    //X_Local.normalize();
    //local_frame[0] = X_Local; // X-Axis for rotation

    //Eigen::Vector3f Y_Local = (Z_Local.cross(X_Local)).normalized();
    //local_frame[1] = Y_Local; // Y-Axis for rotation

    //Local_Coordinate_Frame = local_frame;
  /*  Eigen::Matrix3f rot;
    rot.row(0) = Local_Coordinate_Frame[0];
    rot.row(1) = Local_Coordinate_Frame[1];
    rot.row(2) = Local_Coordinate_Frame[2];*/
    WorldLocalTransformation.setIdentity();
    WorldLocalTransformation.block<3, 3>(0, 0) = rotation_matrix;
    Eigen::Vector3f trans = - (rotation_matrix * RotationAxisPoint.getVector3fMap());
    WorldLocalTransformation.col(3) = Eigen::Vector4f(trans(0), trans(1), trans(2), 1.0);
   
}
float CirconImageDescriptor::ComputeMaximumRadius(CloudWithoutType input_Cloud)
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
float CirconImageDescriptor::ComputeheightFromPointCloud(CloudWithoutType input_Cloud)
{
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*input_Cloud, *pTarget);
    size_t size = pTarget->points.size();
    std::vector<PointNormalType>points;
    points.reserve(size);
    for (size_t i = 0; i < size; i++)
    {
        points.push_back(pTarget->points[i]);
    }
    std::sort(points.begin(), points.end(), SortOnZ);
    float height = points[0].z - points[size - 1].z;
    return height;
}
CloudWithoutType CirconImageDescriptor::TransformPointToLocalFrame()
{
    transformed_inputCloud =  tool::TransFormationOfCloud(inputCloud, WorldLocalTransformation);
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
    Eigen::Matrix3f mat = Eigen::Matrix3f::Identity();
    mat.row(0) = Local_Coordinate_Frame[0];
    mat.row(1) = Local_Coordinate_Frame[1];
    mat.row(2) = Local_Coordinate_Frame[2];
    reconstructed_points.clear();
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
  /*  std::map<int, size_t>::iterator iterator = index_index_map.find(index);
    if (iterator == index_index_map.end())
    {
        std::runtime_error("No point found to corresponding index\n");

    }*/
    size_t pointIndx = index; // iterator->second;  // actual point index of the original input cloud
    if (pointIndx == basic_point_index)
    {
        //throw std::runtime_error("repeat of index\n");
        std::cout << " Warning!" << ":: repeat of Index" << std::endl;
    }
    else
    {
        basic_point_index = pointIndx;
    }
    Eigen::Vector3f query_idx_normal = original_cloud_with_normal->points[pointIndx].getNormalVector3fMap();
    RotationAxisPoint = original_cloud_with_normal->points[pointIndx];
    ConstructLocalFrameOfReference();
    CloudWithNormalPtr pNew(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*inputCloud, *pNew);

    CloudWithoutType transformed_cloud = TransformPointToLocalFrame();
    float max_radius = ComputeMaximumRadius(transformed_cloud);
    float height = ComputeheightFromPointCloud(transformed_cloud);
    SetImageDescriptorResolution(2.0 *M_PI, max_radius, height);
    Image2D.clearMatrix();
    ComputeFeature();
    rot_matrix = WorldLocalTransformation.block<3, 3>(0, 0);

    float theta = GetRotationAngle();
    int num_rows_shift = std::round((theta * num_division_row)/(2*M_PI));  //number of rows to shift
    rotation_index = num_rows_shift;
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
void CirconImageDescriptor::UpdateImageDimension(int col,int row)
{
    max_value_image = -INFINITY;
    min_value_image = -INFINITY;
    Image2D.SetImage(col, row);
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