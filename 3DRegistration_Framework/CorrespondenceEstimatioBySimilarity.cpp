#include"pch.h"
#include"CorrespondenceEstimatioBySimilarity.h"
#include"ErrorMetric.h"
#include"Voronoi_diagram.h"
#include"wfuncs.h"
#include"MLSSearch.h"
#include"Transformation_tool.h"
#include<chrono>
#include<algorithm>


namespace
{
   
    Eigen::Vector3f project(Eigen::Vector3f origin, Eigen::Vector3f normal, Eigen::Vector3f point)
    {
        // Bring the point to the origin
        Eigen::Vector3f p = point - origin;
        Eigen::Vector3f n = normal;

        n.normalize();
        const float projection = static_cast<float>(p.dot(n));

        return p - (projection * n);
    }

    struct sorter
    {

        inline bool operator()(std::pair<int, float> &p1, std::pair<int, float>&p2) const
        {
            return p1.second < p2.second;
        }
    };
    int FindElementInMap(int img_idx, std::map<int, size_t>index_map)
    {
        std::map<int, size_t>::iterator iterator = index_map.find(img_idx);
        if (iterator == index_map.end())
        {
            return (-1);

        }
        int pointIndx = static_cast<int>(iterator->second);  // actual point index of the original input cloud
        return pointIndx;
    }

    bool FindElementInList(int img_idx, std::vector<int>invalid)
    {
        if (invalid.size() == 0)
            return true;
     
        if (std::find(invalid.begin(), invalid.end(), img_idx) == invalid.end())
        {
            return true;

        }
        else
            return false;
    }

    float DistanceBetweenPoints(std::vector<PointNormalType> list, PointNormalType query_point)
    {
        float threshold = INFINITY;
        float dist = 0;
        for (auto pt : list)
        {
           dist = pcl::geometry::distance(pt, query_point);
            if (dist < threshold)
            {
                threshold = dist;
            }
        }
        return dist;
    }
    PointNormalType ComputeCentroid(std::vector<PointNormalType>array_of_points)
    {
        PointNormalType Centroid;
        for (const auto &points : array_of_points)
        {
            Centroid.getVector3fMap() += points.getVector3fMap();
            Centroid.getNormalVector3fMap() += points.getNormalVector3fMap();
        }
        Centroid.getVector3fMap() /= array_of_points.size();
        Centroid.getNormalVector3fMap() /= array_of_points.size();
        Centroid.getNormalVector3fMap().normalize();

        return Centroid;
    }
    std::vector<std::pair<float, std::pair<int, int>>>  RemoveElementsFromList(std::vector<std::pair<float, std::pair<int, int>>>dot_prodct_list )
    {
        int index1 = dot_prodct_list[0].second.first;
        int index2 = dot_prodct_list[0].second.second;
        std::vector<int> invalid_list;
        invalid_list.push_back(index1);
        invalid_list.push_back(index2);
        std::vector<std::pair<float, std::pair<int, int>>>updated_list;
        updated_list.push_back(std::make_pair(dot_prodct_list[0].first,
            std::make_pair(dot_prodct_list[0].second.first, dot_prodct_list[0].second.second)));
        for (int idx = 1; idx < dot_prodct_list.size(); idx++)
        {
            bool status1 = FindElementInList(dot_prodct_list[idx].second.first, invalid_list);
            bool status2 = FindElementInList(dot_prodct_list[idx].second.second, invalid_list);
            if (status1 && status2)
            {
                updated_list.push_back(std::make_pair(dot_prodct_list[idx].first,
                    std::make_pair(dot_prodct_list[idx].second.first, dot_prodct_list[idx].second.second)));
                invalid_list.push_back(dot_prodct_list[idx].second.first);
                invalid_list.push_back(dot_prodct_list[idx].second.second);
            }
            else
                continue;
        }
        return updated_list;
    }
}
void CirconCorrespondence :: PrepareDataForCorrespondenceEstimation(CloudWithoutType &srcCloud, CloudWithoutType &tarCloud)
{
    srcCloudCorrespondence_ = srcCloud;
    targetCloudCorrespondence_ = tarCloud;
    CloudWithNormalPtr _cloud(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*input_source, *_cloud);
    Eigen::Vector3f min_pt, max_pt;
    diagonal_length = tool::ComputeOrientedBoundingBoxOfCloud(_cloud, min_pt, max_pt);
}
search::ISearch* CirconCorrespondence::getSearchStrategy()const
{
    return newsearch;
}
Eigen::Matrix4f CirconCorrespondence::ComputeTransformation()
{
    CloudWithNormalPtr corrs_source(new pcl::PointCloud<PointNormalType>);
    CloudWithNormalPtr corrs_target(new pcl::PointCloud<PointNormalType>);
    pcl::fromPCLPointCloud2(*srcCloudCorrespondence_, *corrs_source);
    pcl::fromPCLPointCloud2(*targetCloudCorrespondence_, *corrs_target);
    float tow_ms = 0.0;
    Eigen::Matrix4f T2_T1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T2 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Final_Transformation = Eigen::Matrix4f::Identity();
    PointNormalType src_corres,tar_corres;
    Measure ms_max;
    bool reset = false;
    CirconImageDescriptor src_corres_descriptor;
    int level_itr = 0;
    int tc_corres, sc_corres;
    for (int tIndex = 16; tIndex < 20; tIndex++)
    {
        tar_corres = corrs_target->at(tIndex);
        target_basic_index = original_target_index[tIndex];
        cid_target.SetBasicPointIndex(target_basic_index);
        for (int sIndex = 21; sIndex < 25; sIndex++)
        {
           int incr_reslolution = 0;
           int level_itr = 0;
           src_corres =  corrs_source->at(sIndex);
           source_basic_index = original_source_index[sIndex];
          

          /*  float row = cid_source.GetRadialResolution();
            Epsilon = row / cid_source.GetColumnDivision();*/
            while ( level_itr != num_resolution )
            {
                cid_source.SetBasicPointIndex(source_basic_index);
                PrepareDescriptor(cid_source, src_corres);
                source_descriptor_image = cid_source.GetDescriptorImage();

              
                PrepareDescriptor(cid_target, tar_corres);
                target_descriptor_image = cid_target.GetDescriptorImage();
                Epsilon = cid_source.GetRadialResolution(); /// 16.0;

                Measure ms = CompareDescriptorWithSimilarityMeasure(source_descriptor_image, target_descriptor_image, incr_reslolution);
                if (ms.similarity_value < tow_ms)
                {
                    reset = true;
                }
                else
                {
                    tow_ms = std::pow(0.80, level_itr + 1) * ms.similarity_value;
                    ms_max = ms;
                    T2 = ms_max.WlTransform; // cid_source.GetTransformation();
                    T1 = cid_target.GetTransformation();
                    float theta = cid_source.GetAngularResolution();
                    float angle = cid_source.GetAngularResolution() * ms_max.rotation_index ;
                    T2_T1.row(0) = Eigen::Vector4f(cos(angle), sin(angle), 0, 0);
                    T2_T1.row(1) = Eigen::Vector4f(-sin(angle), cos(angle), 0, 0);
                    T2_T1.row(2) = Eigen::Vector4f(0, 0, 1, 0);
                   /* Eigen::Vector4f trans = -(T2_T1 * ms_max.point_of_interest.getVector4fMap());
                    T2_T1.col(3) = trans;*/
                    src_corres_descriptor = cid_source;
                 
                }
                if (level_itr == num_resolution - 1)
                {
                    //find transformation and check stopping criterion

                  /*  float theta = cid_source.GetAngularResolution();
                    float angle = cid_source.GetAngularResolution() * ms_max.rotation_index * M_PI / 180.0f;
                    T2_T1.row(0) = Eigen::Vector4f(cos(angle), sin(angle), 0, 0);
                    T2_T1.row(1) = Eigen::Vector4f(-sin(angle), cos(angle), 0, 0);
                    T2_T1.row(2) = Eigen::Vector4f(0,0, 1, 0);*/
                    Final_Transformation = T1.inverse()* T2_T1 * T2;
                    std::cout << "Final_Transformation:" << "\n" << Final_Transformation << std::endl;
                    Eigen::Vector3f tar_norm = cid_target.GetRotationAXisPoint().getNormalVector3fMap();
                    Eigen::Vector3f src_norm = ms_max.point_of_interest.getNormalVector3fMap();
                    CloudWithNormalPtr srx =  cid_source.GetoriginalCloud();
                    CloudWithNormalPtr tgx = cid_target.GetoriginalCloud();
                    std::map<int, size_t> map_tgx = cid_target.GetImagePointMap();
                    int cell_index = cid_target.GetDescriptorCellIndexForInterestPoint();
                    std::vector<std::pair<int, float>>src1 = ComputeFictitiousCorrespondence(ms_max.img_pt_index_map, srx, ms_max.cell_index, ms_max.point_of_interest);
                    std::vector<std::pair<int, float>>tar1 = ComputeFictitiousCorrespondence(map_tgx, tgx, cell_index, tar_corres);
                    std::pair<float, float>error = ComputeStoppingCriteria(src1, tar1, ms_max.point_of_interest, tar_corres, T2_T1, Final_Transformation);
                    if (error.first < 0.0040 && error.second < 0.025 * diagonal_length)//tar_norm.dot(src_norm) >= 0.9)
                    {
                        tc_corres = tIndex;
                        sc_corres = sIndex;
                        std::cout << "Target Index:" << tc_corres << "," << "source_index:" << sc_corres << std::endl;
                        return Final_Transformation;
                    }
                    else
                        reset = true;
                }
                else  // increase resolution
                {
                    src_corres = ms.point_of_interest;
                    source_basic_index = ms.point_index;
                    // reduce the no. of cols to evaluate by half and modify the resolution
                    incr_reslolution = level_itr + 1;
                    int num_row = 2.0 * cid_source.GetRowDivision(); //  std::pow(2, incr_reslolution)
                    int num_col = cid_source.GetColumnDivision() / 2.0; // std::pow(2, incr_reslolution);  // may be changed
                    int num_height = cid_source.GetHeightDivision(); //std::pow(2, incr_reslolution) * 
                    SetResolutionOfDescriptor(cid_source, num_row, num_col, num_height);
                    SetResolutionOfDescriptor(cid_target, num_row, num_col, num_height);
                    cid_source.UpdateImageDimension(num_col, num_row);
                    cid_target.UpdateImageDimension(num_col, num_row);
                    this->division_col = num_col;
                    this->division_row = num_row;
                    level_itr++;
                    continue;
                }
                if (reset)
                {
                    int num_row = cid_source.GetRowDivision() / std::pow(2, incr_reslolution);
                    int num_col = num_row; // cid_source.GetColumnDivision() / std::pow(2, incr_reslolution);  // may be changed
                    int num_height = cid_source.GetHeightDivision();// / std::pow(2, incr_reslolution);
                    SetResolutionOfDescriptor(cid_source, num_row, num_col, num_height);
                    SetResolutionOfDescriptor(cid_target, num_row, num_col, num_height);
                    cid_source.UpdateImageDimension(num_col, num_row);
                    cid_target.UpdateImageDimension(num_col, num_row);
                    this->division_col = num_col;
                    this->division_row = num_row;
                    source_basic_index = -1;
                    reset = false;
                    break;
                }
              
            }
     
        }
       
    }
   // std::cout << "Target Index:" << tc_corres << "," << "source_index:" << sc_corres << std::endl;
    Final_Transformation = T1.inverse()* T2_T1 * T2;
    return Final_Transformation;
}
std::vector<std::pair<int, float>>CirconCorrespondence::ComputeFictitiousCorrespondence(std::map<int, size_t>img_point_map,
    CloudWithNormalPtr cloud, int corres_1_source_idx, PointNormalType corres_point)
{
    std::vector<std::pair<int, float>>neighborhood_dot_products;
   // std::vector<int>neighbor_point_idx;
    // collect 8-neighbor around source correspondence
    int i_index = corres_1_source_idx / division_col;
    int j_index = corres_1_source_idx % division_col;
   
    for (int r_idx = 0; r_idx < division_row; r_idx++)
    {
        if (neighborhood_dot_products.size() >= 0 && r_idx != i_index) //&& neighborhood_dot_products.size() 
        {
            int r = r_idx * division_col + j_index;
            int index = FindElementInMap(r, img_point_map);
            if (index != -1)
            {
                // neighbor_point_idx.push_back(index);
                float norm_product = cloud->points[index].getNormalVector3fMap().dot(corres_point.getNormalVector3fMap());
                neighborhood_dot_products.push_back(std::make_pair(index, norm_product));
            }
            else
                continue;
        }
        else
            continue;
    }
    //int r1, r2, r3, r4, r5, r6, r7, r8;
  
    //          /* neighborhood pattern 
    //            r1  r2  r3
    //            r4  p   r5 
    //            r6  r7  r8*/
    //if (i_index != 0 && j_index != 0)
    //{
    //    r1 = (i_index - 1)*division_col + j_index - 1;
    //    int index = FindElementInMap(r1, img_point_map);
    //    if (index != -1)
    //    {
    //        neighbor_point_idx.push_back(index);
    //        float norm_product = cloud->points[index].getNormalVector3fMap().dot(corres_point.getNormalVector3fMap());
    //        neighborhood_dot_products.push_back(std::make_pair(index, norm_product));
    //    }
    //}
    //if (i_index != 0 )
    //{
    //    r2 = (i_index - 1)*division_col + j_index;
    //    int index = FindElementInMap(r2, img_point_map);
    //    if (index != -1)
    //    {
    //        neighbor_point_idx.push_back(index);
    //        float norm_product = cloud->points[index].getNormalVector3fMap().dot(corres_point.getNormalVector3fMap());
    //        neighborhood_dot_products.push_back(std::make_pair(index, norm_product));
    //    }
    //}
    //if (i_index != 0 && j_index != division_col - 1)
    //{
    //    r3 = (i_index - 1)*division_col + j_index + 1;
    //    int index = FindElementInMap(r3, img_point_map);
    //    if (index != -1)
    //    {
    //        neighbor_point_idx.push_back(index);
    //        float norm_product = cloud->points[index].getNormalVector3fMap().dot(corres_point.getNormalVector3fMap());
    //        neighborhood_dot_products.push_back(std::make_pair(index, norm_product));
    //    }
    //}
    //if ( j_index != 0)
    //{
    //    r4 = i_index *division_col + j_index - 1;
    //    int index = FindElementInMap(r4, img_point_map);
    //    if (index != -1)
    //    {
    //        neighbor_point_idx.push_back(index);
    //        float norm_product = cloud->points[index].getNormalVector3fMap().dot(corres_point.getNormalVector3fMap());
    //        neighborhood_dot_products.push_back(std::make_pair(index, norm_product));
    //    }
    //}
    //if ( j_index != division_col - 1)
    //{
    //    r5 = i_index * division_col + j_index + 1;
    //    int index = FindElementInMap(r5, img_point_map);
    //    if (index != -1)
    //    {
    //        neighbor_point_idx.push_back(index);
    //        float norm_product = cloud->points[index].getNormalVector3fMap().dot(corres_point.getNormalVector3fMap());
    //        neighborhood_dot_products.push_back(std::make_pair(index, norm_product));
    //    }
    //}
    //if (i_index != division_row - 1 && j_index != 0)
    //{
    //    r6= (i_index + 1)*division_col + j_index - 1;
    //    int index = FindElementInMap(r6, img_point_map);
    //    if (index != -1)
    //    {
    //        neighbor_point_idx.push_back(index);
    //        float norm_product = cloud->points[index].getNormalVector3fMap().dot(corres_point.getNormalVector3fMap());
    //        neighborhood_dot_products.push_back(std::make_pair(index, norm_product));
    //    }
    //}
    //if (i_index != division_row - 1)
    //{
    //    r7 = (i_index + 1)*division_col + j_index;
    //    int index = FindElementInMap(r7, img_point_map);
    //    if (index != -1)
    //    {
    //        neighbor_point_idx.push_back(index);
    //        float norm_product = cloud->points[index].getNormalVector3fMap().dot(corres_point.getNormalVector3fMap());
    //        neighborhood_dot_products.push_back(std::make_pair(index, norm_product));
    //    }
    //}
    //if (i_index != division_row - 1 && j_index != division_col - 1)
    //{
    //    r8 = (i_index + 1)*division_col + j_index + 1;
    //    int index = FindElementInMap(r8, img_point_map);
    //    if (index != -1)
    //    {
    //        neighbor_point_idx.push_back(index);
    //        float norm_product = cloud->points[index].getNormalVector3fMap().dot(corres_point.getNormalVector3fMap());
    //        neighborhood_dot_products.push_back(std::make_pair(index, norm_product));
    //    }
    //}
    return neighborhood_dot_products;

}
std::vector<std::pair<float, std::pair<int, int>>> CirconCorrespondence::ComputeCorrespondencePair(std::vector<std::pair<int, float>>src_poi, std::vector<std::pair<int, float>>tar_poi)
{
    float comp_epsilon = 0.10;
    std::vector < std::pair<float,std::pair<int, int>>>corres_pair;
    int it1, it2;

    for (auto const& itr : src_poi)
    {
        float init_val = INFINITY;
        for (auto const& it : tar_poi)
        {
            float diff = std::fabs(itr.second - it.second);
         /*   if (diff < init_val)
            {*/
               // init_val = diff;
                it1 = itr.first;
                it2 = it.first;
            corres_pair.push_back(std::make_pair(diff, std::make_pair(it1, it2)));
          
        }
       
    }
    typedef std::pair<float, std::pair<int, int>> value;
    if (corres_pair.size() > 0)
    {
        std::sort(corres_pair.begin(), corres_pair.end(),
            [&](const value& v1, const value& v2)
        {
            return v1.first < v2.first;
        });
    }
    else
    {
        throw std::runtime_error("correspondence pair for fictitiuos correspondence is zero");
    }
    std::vector<std::pair<float, std::pair<int, int>>>update_data = RemoveElementsFromList(corres_pair);
    return update_data;
}

Eigen::Matrix4f CirconCorrespondence::FindTransformationUsingFictitiousCorrespondence(std::vector<std::pair<float, std::pair<int, int>>>
    corres_pair, PointNormalType src_corres, PointNormalType tar_corres, Eigen::Matrix4f Pair_Transform)
{
    Eigen::Matrix4f FictTransformation = Eigen::Matrix4f::Identity();
    std::vector<PointNormalType>fictitiuous_pairs_source, fictitiuous_pairs_target;
    if (corres_pair.size() >= 2)
    {
       int s_index_1 =  corres_pair[0].second.first;
       int s_index_2 = corres_pair[1].second.first;

       int t_index_1 = corres_pair[0].second.second;
       int t_index_2 = corres_pair[1].second.second;
       // source cloud
       fictitiuous_pairs_source.push_back(src_corres);
       fictitiuous_pairs_source.push_back(cid_source.GetoriginalCloud()->points[s_index_1]);
       fictitiuous_pairs_source.push_back(cid_source.GetoriginalCloud()->points[s_index_2]);
       PointNormalType source_corres_centroid = ComputeCentroid(fictitiuous_pairs_source);

       // target cloud
       fictitiuous_pairs_target.push_back(tar_corres);
       fictitiuous_pairs_target.push_back(cid_target.GetoriginalCloud()->points[t_index_1]);
       fictitiuous_pairs_target.push_back(cid_target.GetoriginalCloud()->points[t_index_2]);
       PointNormalType target_corres_centroid = ComputeCentroid(fictitiuous_pairs_target);

       // Prepare Descriptors
       int r = cid_source.GetRowDivision();
       int c = cid_source.GetColumnDivision();
       int h = cid_source.GetHeightDivision();
       CirconImageDescriptor src_corres_descriptor(input_source, r, c, h);
       CirconImageDescriptor tgx_corres_descriptor(input_target, r, c, h);
       PrepareDescriptor(src_corres_descriptor, source_corres_centroid);
       PrepareDescriptor(tgx_corres_descriptor, target_corres_centroid);
       Eigen::Matrix4f sc_transform = src_corres_descriptor.GetTransformation();
       Eigen::Matrix4f tr_transform = tgx_corres_descriptor.GetTransformation();

       // fictititous transformation
       FictTransformation = tr_transform.inverse() * Pair_Transform * sc_transform;

       return FictTransformation;
    }
    else
    {
        return FictTransformation;
    }
}
std::pair<float, float> CirconCorrespondence::ComputeStoppingCriteria(std::vector<std::pair<int, float>>src_poi, 
    std::vector<std::pair<int, float>>tar_poi, PointNormalType src_corres, PointNormalType tar_corres,
    Eigen::Matrix4f Pair_Transform, Eigen::Matrix4f Corres_Transform)
{
    std::vector<std::pair<float, std::pair<int, int>>>correspondence_pair = ComputeCorrespondencePair(src_poi, tar_poi);
    Eigen::Matrix4f fict_transfrom =  FindTransformationUsingFictitiousCorrespondence(correspondence_pair, src_corres, tar_corres, Pair_Transform);
    Eigen::Matrix4f diff_transform = fict_transfrom.inverse() * Corres_Transform;
    Eigen::Matrix3f rot_diff = diff_transform.block<3, 3>(0, 0);
    Eigen::Vector3f euler_angles = rot_diff.eulerAngles(2, 1, 0);
    float rot_error = sqrtf(euler_angles.squaredNorm() / 3.0);

    float trans_error = diff_transform.col(3).head <3>().norm();

    return std::make_pair(rot_error, trans_error);
   
}

void CirconCorrespondence ::PrepareDescriptor(CirconImageDescriptor& cid, PointNormalType rotpoint)
{
    if (basic_descriptor)
    {
        cid.ComputeBasicPointOfInterest();  // considers closest point to centroid as the point around which descriptor is built
    }
    else
        cid.SetRotationAxisPoint(rotpoint);
    cid.ConstructLocalFrameOfReference();
    CloudWithoutType transformed_cloud = cid.TransformPointToLocalFrame();

    float max_radius = cid.ComputeMaximumRadius(transformed_cloud);
    float height = cid.ComputeheightFromPointCloud(transformed_cloud);
    cid.SetImageDescriptorResolution(FULL_ANGLE, max_radius, height);
    cid.ComputeFeature();
   
   // Epsilon = max_radius / (float(division_col * 16));  // currently no. of row steps  = no.of col steps ( square image)
}

void CirconCorrespondence::PrepareTargetDescriptor(PointNormalType rotpoint)
{
    if (basic_descriptor)
    {
        cid_source.ComputeBasicPointOfInterest();  // considers closest point to centroid as the point around which descriptor is built
    }
    else
        cid_source.SetRotationAxisPoint(rotpoint);
    cid_target.ConstructLocalFrameOfReference();
    CloudWithoutType transformed_cloud = cid_target.TransformPointToLocalFrame();

    float max_radius = cid_target.ComputeMaximumRadius(transformed_cloud);
    float height = cid_target.ComputeheightFromPointCloud(transformed_cloud);
    cid_target.SetImageDescriptorResolution(FULL_ANGLE, max_radius, height);
    cid_target.ComputeFeature();
    target_descriptor_image = cid_target.GetDescriptorImage();
}
float CirconCorrespondence::ComputeMeasureOfSimilarity(std::vector<float> src_image, std::vector<float>tar_image)
{
    float similarity_value = 0.0;
    float overlap_set_sum = 0.0; 
    float union_set_sum = 0.0;
    float sigma_ov = 1.0;
    float dist_overlap = 0.0f;
    if (src_image.size() == tar_image.size())
    {
        for (int itr = 0; itr < src_image.size(); itr++)
        {
            int j_index = itr % division_col;
           /* if (j_index <= max_column_nr)
            {*/
                if (src_image[itr] != -INFINITY && tar_image[itr] != -INFINITY)
                {
                    overlap_set_sum += j_index;
                   // union_set_sum += j_index;
                    dist_overlap += j_index * std::fabs(src_image[itr] - tar_image[itr]);
                }
                if (src_image[itr] != -INFINITY || tar_image[itr] != -INFINITY)
                {
                    union_set_sum += j_index;
                }
           /* }
            else
                continue;*/
        }
      
    }
    else
    {
        std::runtime_error("source and  target descriptor unequal in size\n");
        return -INFINITY;
    }
    dist_overlap = dist_overlap / overlap_set_sum;
    sigma_ov = static_cast<float>(overlap_set_sum) / static_cast<float>(union_set_sum);
    float lambda_dash = rho_ * lambda_;
    similarity_value = (sigma_ov) / ((rho_ * dist_overlap + lambda_dash) + sigma_ov * (1 - lambda_dash));
    return similarity_value;
}

CirconImageDescriptor CirconCorrespondence::TransformCirconDescriptor(int index)
{
    CirconImageDescriptor transformed_descriptor = cid_source;
   // std::vector<float>imagedata = transformed_descriptor.GetDescriptorImage();
   // transformed_descriptor.TransformImageData(imagedata, index);
    transformed_descriptor.UpdateImageDataAfterTransformation(index);
    return transformed_descriptor;
}

CirconCorrespondence ::Measure CirconCorrespondence::CompareDescriptorWithSimilarityMeasure
(std::vector<float> src_image, std::vector<float>tar_image, int resolution)
{
    float current_epsilon = INFINITY;
    float sm = 0.0f;
    CirconCorrespondence::Measure mes;
    PointNormalType init_point = cid_source.GetRotationAXisPoint();
    // take basic descriptor into a temp. array
    std::vector<float>current_array_of_data = src_image;
    std::vector<float>new_image;
    //int curr_point_idx;
    int prev_point_idx = -1;
    float init_ratio_nv = 0.0;
    float tow_nv = 0.9;
    int max_cell_index = 0;
   // Epsilon = cid_source.GetRejectionRatioOfPoints();
    std::map<int, size_t>image_point_index_map = cid_source.GetImagePointMap();
    std::map<int, size_t>initial_start_map = image_point_index_map; // keeping acopy just for verification, may be deleted afterwards
    std::vector<int> invalid_pt_list;
    if (resolution != 0)
    {
       // max_cell_index = division_row * (division_col/2);
        prev_point_idx = source_basic_index; //
        max_column_nr = division_col / 2;
    }
    else
    {
        prev_point_idx = source_basic_index;// FindElementInMap(division_col, image_point_index_map);
       // max_cell_index = division_row * division_col;
        max_column_nr = division_col;
    }
    
    std::map<int, size_t>image_point_index_map_current;
    int non_valid_count = 0;
    std::vector<PointNormalType>previous_pts_list;
    int current_row_idx = 0;
    while (current_epsilon >= Epsilon)
    {
        current_row_idx = (image_point_index_map.begin()->first) /division_col;
        std::vector<size_t>temp_non_valid;
        CirconImageDescriptor transformed_source;
       // for each cell in an array, if the cell value is  valid: construct a descriptor around that point
        int valid_count = 0;
      /*  for (int itr  = 0; itr < num_cell; itr++)*/
        for (auto const& itr : image_point_index_map)
        {
             // neglect the first column as they resemble the same basic point with zero radius
            /* if (init_ratio_nv < tow_nv)
             {*/
             // curr_point_idx = FindElementInMap(itr, image_point_index_map); // cid_source.GetPointIndexFromCloud(itr);
            //  bool status = std::find(non_valid_points.begin(), non_valid_points.end(), curr_point_idx) != non_valid_points.end();
             /* if (itr.second == prev_point_idx)
                  std::cout << "Repeat of index" << std::endl;*/
            bool valid = FindElementInList(itr.second, invalid_pt_list);
            int max_col_index_to_check = itr.first % division_col;
            if ( itr.second != prev_point_idx ) //&& max_col_index_to_check <= max_column_nr)
            {
                if (/*status == false &&*/ itr.second >= 0)
                {
                    transformed_source = TransformCirconDescriptor(static_cast<int>(itr.second)); //itr
                    new_image = transformed_source.GetDescriptorImage();

                }
                else
                    continue;

            }
            else
                continue;
            // check the similarity measure between new descriptor and the target image descriptor     

            float updated_sm = ComputeMeasureOfSimilarity(new_image, tar_image);
            if (updated_sm > sm)
            {
                sm = updated_sm;
                mes.cell_index = transformed_source.GetDescriptorCellIndexForInterestPoint();//itr.first;
                mes.cell_values = new_image;
                mes.similarity_value = sm;
                mes.rotation_index = division_row - mes.cell_index / this->division_col + 1;  //  transformed_source.GetRotationIndex()
                mes.point_of_interest = transformed_source.GetRotationAXisPoint();
                image_point_index_map_current = transformed_source.GetImagePointMap();
                mes.point_index = itr.second; // FindElementInMap(itr, image_point_index_map); //transformed_source.GetPointIndexFromCloud(itr);
                mes.WlTransform = transformed_source.GetTransformation();
                mes.img_pt_index_map = image_point_index_map_current;
               // valid_count++;
            }
            else
            {
                non_valid_count++;
                invalid_pt_list.push_back(itr.second);
                // size_t _idx = transformed_source.GetPointIndexFromCloud(itr);
                // temp_non_valid.push_back(curr_point_idx);
                continue;
            }

            /* }
             else
                 continue;*/
        }
        /*non_valid_points = temp_non_valid;
        float non_valid_ratio = float(non_valid_count) / float(num_cell);
        init_ratio_nv = non_valid_ratio;
        pt_validity_ratio.insert(std::pair<float, size_t>(non_valid_ratio,curr_point_idx));*/
        // update descriptor data for next comparison

        current_array_of_data = mes.cell_values;
        PointNormalType current_point  = mes.point_of_interest;
        previous_pts_list.push_back(init_point);
        float dist = DistanceBetweenPoints(previous_pts_list, current_point);// pcl::geometry::distance(init_point, current_point);
       
        if (dist <= Epsilon)
            break;
        else
        {
            current_epsilon = dist;
            init_point = current_point;
            image_point_index_map = image_point_index_map_current;
            prev_point_idx = static_cast<int>(mes.point_index);  // takes care so that the descriptor is not created again with the same point of previous iteration
        }
    }
    max_measure = mes;  // keeps a copy of present measure as a member variable
    return mes;
}
void CirconCorrespondence::SetResolutionOfDescriptor(CirconImageDescriptor &dsc, int num_rows, int num_cols, int hei_div)
{
    dsc.SetDivision(num_rows, num_cols, hei_div);
}
 std::vector<float> CirconCorrespondence::ComputelaplacianOfNormals(CloudWithoutType &inputCloud)
{
    const double avg_dist = metric::EstimatePointAvgDistance(inputCloud);
    const double h = 5.0 * avg_dist;

    CloudWithNormalPtr _cloud(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*inputCloud, *_cloud);

    pcl::KdTreeFLANN<PointNormalType>::Ptr kdt(new pcl::KdTreeFLANN < PointNormalType>);
    kdt->setInputCloud(_cloud);
    std::vector<float>voronoi_area;
    voronoi_area.resize(_cloud->size());
    std::vector<float>laplacian;
    laplacian.resize(_cloud->size());
    for (size_t i = 0; i < _cloud->size(); i++)
    {
        const auto& point = _cloud->at(i);
        const auto& normal = _cloud->points[i].getNormalVector3fMap();
        const auto& norm_vector = normal;

        if (!pcl::isFinite(point)) continue;

        std::vector<int> indices;
        std::vector<float> distances;
        kdt->radiusSearch(point, h, indices, distances);
        if (indices.size() < 4) {
            voronoi_area[i] = 0.0;
            continue;
        }
        // Project the neighbor points in the tangent plane at p_i with normal n_i
        std::vector<Eigen::Vector3f> projected_points;
        for (const auto& neighbor_index : indices)
        {
            if (neighbor_index != i)
            {
                const auto& neighbor_point = _cloud->at(neighbor_index);
                projected_points.push_back(project(point.getVector3fMap(), normal, neighbor_point.getVector3fMap()));

            }
        }
        assert(projected_points.size() >= 3);

        // Use the first vector to create a 2D basis
        Eigen::Vector3f u = projected_points[0];
        u.normalize();
        Eigen::Vector3f v = (u.cross(norm_vector));
        v.normalize();

        // Add the points to a 2D plane
        std::vector<Eigen::Vector2f> plane;

        // Add the point at the center
        plane.push_back(Eigen::Vector2f::Zero());

        // Add the rest of the points
        for (const auto& projected : projected_points)
        {

            float x = projected.dot(u);
            float y = projected.dot(v);

            // Add the 2D point to the vector
            plane.push_back(Eigen::Vector2f(x, y));
        }
        assert(plane.size() >= 4);
        // Compute the voronoi cell area of the point
        float area = Voronoidiagram::Area(plane);
        voronoi_area[i] = area;
    }

    for (int i = 0; i < _cloud->size(); i++)
    {
        const auto& point = _cloud->at(i);
        if (!pcl::isFinite(point)) continue;

        std::vector<int> indices;
        std::vector<float> distances;
        kdt->radiusSearch(point, h, indices, distances);
        float lbo = 0.0;
        for (const auto& j : indices)
        {
            if (j != i)
            {
                const auto& neighbor = _cloud->at(j);

                float d = (neighbor.getNormalVector3fMap() - point.getNormalVector3fMap()).norm();
                float w = voronoi_area[j] * exp(-(d * d) / (4.0 * h));
                lbo += w;

            }
        }
        lbo = lbo * (1.0 / (4.0 * M_PI * h * h));
        laplacian[i] = lbo;

    }

    return laplacian;

}
 std::map<float, int> CirconCorrespondence::ComputeMeanCurvatureFromPointCloud(CloudWithoutType inputCloud)
 {
     CloudWithNormalPtr _cloud(new pcl::PointCloud <PointNormalType>);
     pcl::fromPCLPointCloud2(*inputCloud, *_cloud);
     CloudPtr cloudxyz(new pcl::PointCloud <PointType>);
     pcl::copyPointCloud(*_cloud, *cloudxyz);
     //pcl::fromPCLPointCloud2(*inputCloud, *cloudxyz);

     Eigen::Vector3f min_pt, max_pt;
     const double avg_dist = tool::ComputeOrientedBoundingBoxOfCloud(_cloud, min_pt, max_pt);  //metric::EstimatePointAvgDistance(inputCloud);
     const double h = 0.02 * avg_dist;
     const double h_up = 0.10 *avg_dist;

     CWendlandWF<float> wendLandWeightF;
     cMLSearch mls(inputCloud, wendLandWeightF, 5.0, 3);

     pcl::KdTreeFLANN<PointNormalType>::Ptr kdt(new pcl::KdTreeFLANN < PointNormalType>);
     kdt->setInputCloud(_cloud);
    
     Eigen::Vector3f eigen_values(0.0f, 0.0f, 0.0f);
     std::vector<float>mean_curvatures;
     mean_curvatures.resize(_cloud->size());
     std::vector<float>mean_curvatures_up_scale;
     mean_curvatures_up_scale.resize(_cloud->size());
     std::map<float, int>mean_curvature_ratio;
     auto start = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
     for (int i = 0; i < _cloud->size(); i++)
     {
         std::vector<int> indices;
         std::vector<float> distances;
         std::vector<int> indices_up;
         std::vector<float> distances_up;
       
         PointNormalType point = _cloud->at(i);
      
         if (!pcl::isFinite(point))
             continue;

         kdt->radiusSearch(point, h, indices, distances);
         if (indices.size() < 4)
             continue;
        
         std::vector<float>weights = mls.ComputeWeightFromLocalNeighborhood(point, cloudxyz, indices, wendLandWeightF, h);
        
         if (weights[0] < 0.01)
             continue;
         Eigen::Vector3f meanOfNeighborhood(0.0f, 0.0f, 0.0f);
         Eigen::MatrixXf cov_matrix;
         std::vector<Eigen::Vector3f>eigen_vectors;
         eigen_vectors.resize(3);
         eigen_values = mls.ComputeWeightedCovariancematrix(cloudxyz, weights, indices,
             meanOfNeighborhood, eigen_vectors, cov_matrix);
       
         mean_curvatures[i] = eigen_values[0];

      /* up scale curvature computation*/
         kdt->radiusSearch(point, h_up, indices_up, distances_up);

         if (indices_up.size() < 4)
             continue;
         
         std::vector<float>weights_up = mls.ComputeWeightFromLocalNeighborhood(point, cloudxyz, indices_up, wendLandWeightF, h_up);

         if (weights_up[0] < 0.01)
             continue;

         Eigen::Vector3f meanOfNeighborhood_up(0.0f, 0.0f, 0.0f);
         Eigen::MatrixXf cov_matrix_up;
         std::vector<Eigen::Vector3f>eigen_vectors_up;
         eigen_vectors_up.resize(3);
         Eigen::Vector3f eig_values(0.0f, 0.0f, 0.0f);

         eig_values = mls.ComputeWeightedCovariancematrix(cloudxyz, weights_up, indices_up,
             meanOfNeighborhood_up, eigen_vectors_up, cov_matrix_up);
      
         mean_curvatures_up_scale[i] = eig_values[0];
       
         if (mean_curvatures_up_scale[i] != 0.0)
         {
             float m_curv_ratio = mean_curvatures[i] / mean_curvatures_up_scale[i];
             mean_curvature_ratio.insert(std::pair<float, int>(m_curv_ratio,i));
         }

     }
     auto end = std::chrono::high_resolution_clock::now();
     double executeopto = std::chrono::duration_cast<
         std::chrono::duration<double, std::milli>>(end - start).count();
     executeopto = executeopto / double(1000);
     std::cout << "weight_compute_up_scale:" << executeopto << std::endl;
    /* for (size_t i = 0; i < _cloud->size(); i++)
     {
        
         auto& point = _cloud->at(i);
         const auto& normal = _cloud->points[i].getNormalVector3fMap();
         const auto& norm_vector = normal;

         if (!pcl::isFinite(point))
             continue;

         kdt->radiusSearch(point, h_up, indices, distances);
         if (indices.size() < 4)
             continue;

         std::vector<float>weights = mls.ComputeWeightFromLocalNeighborhood(point, cloudxyz, indices, wendLandWeightF, h_up);
         if (weights[0] < 0.01)
             continue;
         Eigen::Vector3f meanOfNeighborhood(0.0f, 0.0f, 0.0f);
         Eigen::MatrixXf cov_matrix;
         std::vector<Eigen::Vector3f>eigen_vectors;
         eigen_vectors.resize(3);
         Eigen::Vector3f eig_values(0.0f, 0.0f, 0.0f);
         eig_values = mls.ComputeWeightedCovariancematrix(cloudxyz, weights, indices,
             meanOfNeighborhood, eigen_vectors, cov_matrix);
        
         mean_curvatures_up_scale[i] = eig_values[0];
         if (mean_curvatures_up_scale[i] != 0.0)
             mean_curvature_ratio[i] = mean_curvatures[i] / mean_curvatures_up_scale[i];

     }*/
     return mean_curvature_ratio;
 }
 CloudWithNormalPtr CirconCorrespondence::WriteFilteredPoints(std::string FileName, CloudWithoutType inputCloud, std::map<float, int>curvature_ratio, float threshold)
 {
     CloudWithNormalPtr _cloud(new pcl::PointCloud <PointNormalType>);
     pcl::fromPCLPointCloud2(*inputCloud, *_cloud);
     CloudWithNormalPtr _filteredcloud(new pcl::PointCloud <PointNormalType>);
     size_t count = 0;
    // std::sort(curvature_ratio.begin(), curvature_ratio.end(),sorter());
     size_t posSlash = FileName.rfind('.');
     std::string subfileName = FileName.substr(0, posSlash);
     subfileName = subfileName + ".txt";
     FILE *pFile;
     pFile = fopen(subfileName.c_str(), "wb");
     for (std::pair<float,int>entry: curvature_ratio)
     {
         if (entry.first < threshold && count < 50)
         {
             int index = entry.second;
             _filteredcloud->points.push_back(_cloud->at(index));
             fprintf(pFile, "%d\n", index);
             count++;
         }
     }
     fclose(pFile);
     _filteredcloud->width = count;
     _filteredcloud->height = 1;
    // pcl::io::savePLYFile(FileName, *_filteredcloud);
     return _filteredcloud;
     
 }
 int CirconCorrespondence:: ReadCorrespondenceIndex(std::string FileNameA, std::string fileNameB)
 {
     size_t posSlash = FileNameA.rfind('.');
     std::string subfileNameA = FileNameA.substr(0, posSlash);
     subfileNameA = subfileNameA + ".txt";
     unsigned noOfLines_A = pct::readLines(subfileNameA);

     size_t posBackSlash = fileNameB.rfind('.');
     std::string subfileNameB = fileNameB.substr(0, posBackSlash);
     subfileNameB = subfileNameB + ".txt";
     unsigned noOfLines_B = pct:: readLines(subfileNameB);
     FILE *pFile, *nFile;
     pFile = fopen(subfileNameA.c_str(), "rb");
     nFile = fopen(subfileNameB.c_str(), "rb");
   
     original_source_index.resize(noOfLines_A);
     original_target_index.resize(noOfLines_B);
     if (NULL == pFile || NULL == nFile)
     {
         throw std::runtime_error("index file not found");
     }
     if (noOfLines_A != noOfLines_B)
     {
         return -1;
     }
     char szParam1[50], szParam2[50];
     for (int i = 0; i < noOfLines_A; i++)
     {
         fscanf(pFile, "%s\n", szParam1);
         fscanf(nFile, "%s\n", szParam2);
         original_source_index[i] = atoi(szParam1);
         original_target_index[i] = atoi(szParam2);
     }
     fclose(pFile);
     fclose(nFile);
     return 1;
 }
void CirconCorrespondence::WriteLaplacianImage( std::string fileName, CloudWithoutType inputCloud, std::vector <UVData<float>>pixelIndex)
{
    std::vector<float>laplacians = ComputelaplacianOfNormals(inputCloud);
    std::vector<int> uCoord, vCoord;
    for (int i = 0; i < pixelIndex.size(); i++)
    {
        uCoord.push_back(pixelIndex[i].u);
        vCoord.push_back(pixelIndex[i].v);
    }
    int maxUCoord = *std::max_element(uCoord.begin(), uCoord.end()); // maxrows
    int maxVCoord = *std::max_element(vCoord.begin(), vCoord.end());  // maxcol
    CirconImage image(maxVCoord, maxUCoord);

    //sort the laplacian values
    std::vector<float>data = laplacians;
   float max_value_image = *(std::max_element(data.begin(), data.end()));
    float  min_value = INFINITY;
    for (int itr = 0; itr < data.size(); itr++)
    {
        if (data[itr] < min_value && data[itr] != -INFINITY)
        {
            min_value = data[itr];
        }
    }
   float min_value_image = min_value;

   // for scaling to write as image
   float input_range = max_value_image - min_value_image;
   float output_range = 255.0 - 0.0;

    if (laplacians.size() != pixelIndex.size())
    {
        assert(laplacians.size() != pixelIndex.size());
    }
    for (int j = 0; j < pixelIndex.size(); j++)
    {
        int posn = pixelIndex[j].u  * maxVCoord + pixelIndex[j].v;
        float output = std::roundf((laplacians[j] - min_value_image) * output_range / input_range + 0.0);
        image.addCellAt(pixelIndex[j].u, pixelIndex[j].v, maxVCoord, output);
    }
    std::vector<float>img_data = image.getImageData();
    image.WriteImage(fileName, img_data);

}
