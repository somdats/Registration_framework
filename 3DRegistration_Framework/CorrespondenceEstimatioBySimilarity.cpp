#include"pch.h"
#include"ErrorMetric.h"
#include"Voronoi_diagram.h"
#include"wfuncs.h"
#include"MLSSearch.h"
#include"Transformation_tool.h"
#include<chrono>
#include<algorithm>
#include"UniformGrid.h"

#include"CorrespondenceEstimatioBySimilarity.h"
namespace
{
    bool SortFunction(const cEle &a, const cEle &b)
    {
         return a.similarity_value > b.similarity_value;
    }
        
    bool cmp(const CirconCorrespondence::Measure& lhs, const CirconCorrespondence::Measure& rhs)
    {
        return lhs.similarity_value < rhs.similarity_value;
    }
   
    size_t ToreduceIndex( const size_t &curr_value ,const size_t &input_min,
        const size_t &input_max, const size_t &op_min, const size_t &op_max)
    {
        float input_range = input_max - input_min;
        float output_range = op_max - op_min;
        size_t opValue = (curr_value - input_min) * output_range / input_range ;
        return opValue;
    }
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

    float DistanceBetweenPoints(const std::vector<PointNormalType>& list, PointNormalType query_point)
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
        return threshold;
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

    void WritePointCloud(std::vector<PointNormalType>reconstructed_points, std::string fileName)
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
        fprintf(pFile, "property float nx \n");
        fprintf(pFile, "property float ny \n");
        fprintf(pFile, "property float nz \n");
        fprintf(pFile, "end_header\n");
        for (int i = 0; i < reconstructed_points.size(); i++)
        {
            fprintf(pFile, "%f %f %f %f %f %f\r\n", reconstructed_points[i].x, reconstructed_points[i].y, 
                reconstructed_points[i].z, reconstructed_points[i].normal_x, reconstructed_points[i].normal_y, reconstructed_points[i].normal_z);
        }
        fclose(pFile);
    }

    void WritePointFromDescriptor(const std::vector<std::vector<_dV>> &descriptors, std::string fileName)
    {
        std::vector<PointNormalType>reconstructed_points;
        for (int ir = 0; ir < descriptors.size(); ir++)
        {
            for (int ic = 0; ic < descriptors[0].size(); ic++)
            {
                if (descriptors[ir][ic].val_at_pixel != -INFINITY )
                {
                    PointNormalType pt = descriptors[ir][ic].pt;
                  /* pt.getVector3fMap() =  descriptors[ir][ic].pt.head<3>().cast<float>();
                   pt.getNormalVector3fMap() = descriptors[ir][ic].pt.tail<3>().cast<float>();*/
                   /* PointNormalType pt(descriptors[ir][ic].pt(0), descriptors[ir][ic].pt(1), descriptors[ir][ic].pt(2), descriptors[ir][ic].pt(3),
                        descriptors[ir][ic].pt(4), descriptors[ir][ic].pt(5));*/
                    reconstructed_points.push_back(pt);
                   
                }
            }
        }
        std::string  fileNameWithLocation = fileName + ".ply";
        FILE *pFile;
        pFile = fopen(fileNameWithLocation.c_str(), "wb");
        fprintf(pFile, "ply\n");
        fprintf(pFile, "format ascii 1.0\n");
        fprintf(pFile, "element vertex %d\n", static_cast<int>(reconstructed_points.size()));
        fprintf(pFile, "property float x \n");
        fprintf(pFile, "property float y \n");
        fprintf(pFile, "property float z \n");
        fprintf(pFile, "property float nx \n");
        fprintf(pFile, "property float ny \n");
        fprintf(pFile, "property float nz \n");
        fprintf(pFile, "end_header\n");
        for (int i = 0; i < reconstructed_points.size(); i++)
        {
           
            fprintf(pFile, "%f %f %f %f %f %f\r\n", reconstructed_points[i].x, reconstructed_points[i].y,
                reconstructed_points[i].z, reconstructed_points[i].normal_x, reconstructed_points[i].normal_y, reconstructed_points[i].normal_z);
        }
        fclose(pFile);
    }

    size_t FindMaximumSimilarityInIterations(const std::vector <CirconCorrespondence::Measure>& m)
    {

      /*  auto max_it = std::max_element(m.begin(), m.end(), cmp);
        size_t idx_count = max_it - m.begin();*/
        auto it = m.begin();
        float value = it++->similarity_value;
      //  CirconCorrespondence::Measure max_similarity;
        size_t idx_count = 0;
        for (; it != m.end(); it++)
        {
            if (it->similarity_value == -1.0)
                continue;
            if (it->similarity_value > value)
            {
                value = it->similarity_value;
                idx_count = it - m.begin();
            }
        }
     
      //  std::cout << "similarity_value:" << max_it->similarity_value << "," << m[idx_count].similarity_value << std::endl;
      //  return *max_it;
        return idx_count;
    }
    std::vector<std::vector<float>> ScaleImageData(const std::vector<std::vector<float>>&data, int num_division_row, int num_division_col)
    {

        float  min_value = INFINITY;
        float  max_value = -INFINITY;
 
        if (data.size() > 0)
        {

            for (int itr = 0; itr < num_division_row; itr++)
            {
                for (int icx = 0; icx < num_division_col; icx++)
                {
                    if (data[itr][icx] != -INFINITY)
                    {
                        if (data[itr][icx] < min_value)
                        {
                            min_value = data[itr][icx];
                        }
                        if (data[itr][icx] > max_value)
                        {
                            max_value = data[itr][icx];
                        }
                    }
                }
            }

        }
        //send the data after scaling it in the range of  0-255
        std::vector<std::vector<float>>scaled_data(num_division_row, std::vector<float>(num_division_col));

        float input_range = max_value - min_value;
        float output_range = 255.0 - 0.0;
        for (int i = 0; i < num_division_row; i++)
        {
            for (int j = 0; j < num_division_col; j++)
            {
                float output = std::roundf((data[i][j] - min_value) * output_range / input_range + 0.0);
                scaled_data[i][j] = output;
            }
        }

        return scaled_data;
    }

    std::vector<int>CopyElementsfromMaptoVector(int size, const std::map<int, size_t>& index_map)
    {
        std::vector<int>point_indices(size, -1);
        if (size > 0)
        {
            for (auto const& it : index_map)
            {
                point_indices.at(it.first) = static_cast<int>(it.second);
            }
        }
        return point_indices;
    }

    std::map<int, size_t> SortMapElementsUsingKeyValue(const std::map<int, size_t>& index_map, const int& nr_col, const int& division_row)
    {
        std::map<int, size_t>sorted_output_map;
        
        int key = division_row  * nr_col;
        //#pragma omp parallel 
        //        {
        //#pragma omp single
        //            {
        for (auto const& itr : index_map)
        {
            if (itr.first > key)
            {
                return sorted_output_map;
            }
            else if (itr.first < key)
            {
                //#pragma omp critical
                sorted_output_map[itr.first] = itr.second;
            }

            else
                continue;
        }

        /*        }

            }*/
        return sorted_output_map;
    }

    int modulo(int val, int divisor)
    {
        int out = (val%divisor + divisor) % divisor;
        return out;
    }

    std::vector<cEle> RejectCorrespondenceBasedOnSimilarityValue(const std::vector<cEle> 
        &SimilarityValues, const float &Sim_Max, const float &Sim_Min, const float &ThresholdMultiplier)
    {
        auto diff = Sim_Max - Sim_Min;
        auto cnt = 0;
        std::vector<cEle>Idx;
        for (auto sim : SimilarityValues)
        {
            if (Sim_Max - sim.similarity_value <= ThresholdMultiplier * diff)
            {
                Idx.emplace_back(cEle(sim.similarity_value,sim.src_idx,sim.tgt_idx));
                cnt++;
               
            }
       }
        std::sort(Idx.begin(), Idx.end(), [](const cEle &a, const cEle &b)
        {
            if (a.similarity_value != -INF || b.similarity_value != -INF)
                return a.similarity_value > b.similarity_value;
        });
        return Idx;
    }
    Eigen::Matrix4f create_affine_matrix(float a, float b, float c, Eigen::Vector3f trans =Eigen::Vector3f(0,0,0))
    {
        Transform<float, 3, Eigen::Affine> t;
        t = Translation<float, 3>(trans);
        t.rotate(AngleAxis<float>(a, Vector3f::UnitX()));
        t.rotate(AngleAxis<float>(b, Vector3f::UnitY()));
        t.rotate(AngleAxis<float>(c, Vector3f::UnitZ()));
        return t.matrix();
    }
}
std::pair<int, float> CirconCorrespondence::ComputeRotationIndex(const std::vector<std::vector<float>> &src_desc,  std::vector<std::vector<float>> &tar_desc,
    int row, int col, float max_src, float min_src, float max_tgt, float min_tgt)
{
    float sm = -INFINITY;
    // coarse resolution similarity
    Eigen::MatrixXf mat = tool::CreateMatrixFromStlVector(src_desc, row, col);
    int min_res = src_desc.size();
    int max_rot_idx;
    for (int itr = 0; itr < min_res; itr++)
    {
        Eigen::MatrixXf out_matrix = tool::RowShiftMatrix(mat, itr);
        std::vector<std::vector<float>>mat_vec(row, std::vector<float>(col));
        tool::Create2DStlVectorFromMatrix(out_matrix, mat_vec, row, col);

        float new_sm = ComputeMeasureOfSimilarity(mat_vec, tar_desc/*, max_src, min_src, max_tgt, min_tgt*/);

        if (new_sm > sm)
        {
            sm = new_sm;
            max_rot_idx = itr;
        }

    }
 /*   std::cout << "max_rotation_index at 32X32 from exhaustive search:" << max_rot_idx << std::endl;
    std::cout << "simailarity value from exhaustive search:" << sm << std::endl;*/
    std::pair<int, float> val = std::make_pair(max_rot_idx, sm);
    return val;

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
surface::CloudBoundingBox CirconCorrespondence::initNurbsPCABoundingBox(int order, pcl::on_nurbs::NurbsDataSurface &data, Eigen::Vector3d &mean, Eigen::Matrix3d &eigenvectors, Eigen::Vector3d &V_max,
    Eigen::Vector3d &V_min)
{
     Eigen::Vector3d eigenvalues;
     unsigned s = static_cast<unsigned> (data.interior.size());
     data.interior_param.clear();

     pcl::on_nurbs::NurbsTools::pca(data.interior, mean, eigenvectors, eigenvalues);

     data.mean = mean;
     data.eigenvectors = eigenvectors;


     eigenvalues = eigenvalues / s; // seems that the eigenvalues are dependent on the number of points (???)
     Eigen::Matrix3d eigenvectors_inv = eigenvectors.inverse();

     Eigen::Vector3d v_max(-DBL_MAX, -DBL_MAX, -DBL_MAX);
     Eigen::Vector3d v_min(DBL_MAX, DBL_MAX, DBL_MAX);
     for (unsigned i = 0; i < s; i++)
     {
         Eigen::Vector3d p(eigenvectors_inv * (data.interior[i] - mean));
         data.interior_param.push_back(Eigen::Vector2d(p(0), p(1)));

         if (p(0) > v_max(0))
             v_max(0) = p(0);
         if (p(1) > v_max(1))
             v_max(1) = p(1);
         if (p(2) > v_max(2))
             v_max(2) = p(2);

         if (p(0) < v_min(0))
             v_min(0) = p(0);
         if (p(1) < v_min(1))
             v_min(1) = p(1);
         if (p(2) < v_min(2))
             v_min(2) = p(2);
     }

     for (unsigned i = 0; i < s; i++)
     {
         Eigen::Vector2d &p = data.interior_param[i];
         if (v_max(0) > v_min(0) && v_max(0) > v_min(0))
         {
             p(0) = (p(0) - v_min(0)) / (v_max(0) - v_min(0));
             p(1) = (p(1) - v_min(1)) / (v_max(1) - v_min(1));
         }
         else
         {
             throw std::runtime_error("[NurbsTools::initNurbsPCABoundingBox] Error: v_max <= v_min");
         }
     }

     V_max = v_max;
     V_min = v_min;
     surface::CloudBoundingBox bb(eigenvectors_inv, v_max,v_min, mean);
     return bb;
     

}
 void CirconCorrespondence::SetBoundingBoxForDescriptors(surface::CloudBoundingBox &src_bbs, surface::CloudBoundingBox &tgt_bbs)
 {
     cid_source.SetBoundingBoxInformation(src_bbs);
     cid_target.SetBoundingBoxInformation(tgt_bbs);
 }

 CloudPtr CirconCorrespondence::Construct2DCloudForKdtreeSearch(const CloudWithNormalPtr &inputCloud, const Eigen::Matrix4f &WorldLocalTransformation)
 {
     CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
      pcl::transformPointCloudWithNormals(*inputCloud, *pTarget, WorldLocalTransformation);
      CloudPtr Cloud2D(new pcl::PointCloud<PointType>);
     Cloud2D->width = pTarget->width;
     Cloud2D->height = pTarget->height;
     Cloud2D->points.clear();
     Cloud2D->points.shrink_to_fit();
     Cloud2D->points.reserve(Cloud2D->width *  Cloud2D->height);
     for (int i = 0; i < pTarget->points.size(); i++)
     {
         PointType pt_2d(pTarget->points[i].x, pTarget->points[i].y, 0.0);
         Cloud2D->points.push_back(pt_2d);
     }
     return Cloud2D;
 }

 //std::vector<cEle> CirconCorrespondence::SortFeaturePointsOnSimilarityValue(const CloudWithNormalPtr &inputSrcCloud,
 //    const CloudWithNormalPtr &inputTgtCloud, CirconImageDescriptor &cid_src, CirconImageDescriptor &cid_tgt,
 //    const float &max_averg_dist)
 std::vector<cEle>CirconCorrespondence::SortFeaturePointsOnSimilarityValue(const CloudWithNormalPtr &inputSrcCloud,
     const CloudWithNormalPtr &inputTgtCloud, const float &max_averg_dist)
 {
     std::vector<ON_NurbsSurface> nbs = cid_target.GetNurbsSurface();
     std::vector<ON_NurbsCurve> ncs = cid_target.GetNurbsCurve();
     std::vector<ON_NurbsSurface> nbs_src = cid_source.GetNurbsSurface();
     std::vector<ON_NurbsCurve> ncs_src = cid_source.GetNurbsCurve();
     std::vector<Eigen::Vector2i>cluster_label_src = cid_source.GetClusterLabels();
     std::vector<Eigen::Vector2i>cluster_label_tgt  = cid_target.GetClusterLabels();
   
    
     std::vector<cEle> idx_similarity_pair;
     idx_similarity_pair.resize(inputTgtCloud->points.size() *inputSrcCloud->points.size(), cEle());
     bool use_nurbs_strategy = true;
    
    
     // does not print the descriptor building time
     setDebug(false);
     int itr = 0;
     auto start_sort = std::chrono::high_resolution_clock::now();
     /* CirconImageDescriptor cid_src_local = cid_src;
      CirconImageDescriptor cid_tgt_local = cid_tgt;*/
     std::vector<std::pair<int, std::vector<std::vector<float>>>>src_interest_point_descriptors;
     src_interest_point_descriptors.resize(inputSrcCloud->points.size());
   
   
 #pragma omp parallel for  
     for (int iSrc = 0; iSrc < inputSrcCloud->points.size(); iSrc++)
     {
         CirconImageDescriptor cid_src(input_source, 16, 16, 16, src_cloud_parameter, nbs_src, ncs_src, cluster_label_src);
         cid_src.SetMaximumAverageDistance(max_averg_dist);
         PointNormalType src_corres = inputSrcCloud->points[iSrc];
         cid_src.SetBasicPointIndex(original_source_index[iSrc]);
         PrepareDescriptor(cid_src, src_corres, use_nurbs_strategy, true);
         std::vector<std::vector<float>> src_descriptor_image = cid_src.GetDescriptorImage();
         src_interest_point_descriptors[iSrc] = std::make_pair(original_source_index[iSrc], src_descriptor_image);
     }
     auto end_sort = std::chrono::high_resolution_clock::now();
     double extime = std::chrono::duration_cast<
         std::chrono::duration<double, std::milli>>(end_sort - start_sort).count();
     extime = extime / double(1000);
     std::cout << " sorting time:" << extime << "sec" << std::endl;

     auto start_sort_nest = std::chrono::high_resolution_clock::now();
      #pragma omp parallel for /*collapse(2)*/
     for (int iTgt = 0; iTgt < inputTgtCloud->points.size(); iTgt++)
     {
         CirconImageDescriptor cid_tgt(input_target, 16, 16, 16, tgt_cloud_parameter, nbs, ncs, cluster_label_tgt);
         cid_tgt.SetMaximumAverageDistance(max_averg_dist);
         PointNormalType tar_corres = inputTgtCloud->points[iTgt];
         cid_tgt.SetBasicPointIndex(original_target_index[iTgt]);
         PrepareDescriptor(cid_tgt, tar_corres, use_nurbs_strategy, false);
         std::vector<std::vector<float>> target_descriptor_image = cid_tgt.GetDescriptorImage();
         for (int iSrc = 0; iSrc < inputSrcCloud->points.size(); iSrc++)
         {
           
           /*  PointNormalType src_corres = inputSrcCloud->points[iSrc];
             cid_src.SetBasicPointIndex(original_source_index[iSrc]);
             PrepareDescriptor(cid_src, src_corres, use_nurbs_strategy, true);
             std::vector<std::vector<float>> src_descriptor_image = cid_src.GetDescriptorImage();
             cid_src.GetRangeForImage(max_src, min_src);*/
             float max_src, min_src, max_tgt, min_tgt;
             cid_tgt.GetRangeForImage(max_tgt, min_tgt);
             
             auto result = ComputeRotationIndex(src_interest_point_descriptors[iSrc].second, target_descriptor_image, 16, 16,
                     max_src, min_src, max_tgt, min_tgt);

                 if (result.second != -INF)
                     //    idx_similarity_pair.emplace_back(cEle{ result.second, original_source_index[iSrc],
                     //        original_target_index[iTgt] }); //src_basic_index, target_basic_index
                     idx_similarity_pair[itr] = (cEle{ result.second, src_interest_point_descriptors[iSrc].first,
                         original_target_index[iTgt] }); //src_basic_index, target_basic_index
             itr++;

         }
     }
     auto end_sort_nest = std::chrono::high_resolution_clock::now();
     double extime_nest = std::chrono::duration_cast<
         std::chrono::duration<double, std::milli>>(end_sort_nest - start_sort_nest).count();
     extime_nest = extime_nest / double(1000);
     std::cout << " sorting time:" << extime_nest << "sec" << std::endl;
     //  if (debug)

     std::sort(idx_similarity_pair.begin(), idx_similarity_pair.end(), [](const cEle &a, const cEle &b)
     {
         if (a.similarity_value != -INF || b.similarity_value != -INF)
             return a.similarity_value > b.similarity_value;
     });
     return idx_similarity_pair;

 }
Eigen::Matrix4f CirconCorrespondence::ComputeTransformation()
{
    auto start_algo = std::chrono::high_resolution_clock::now();
   
    CloudWithNormalPtr corrs_source(new pcl::PointCloud<PointNormalType>);
    CloudWithNormalPtr corrs_target(new pcl::PointCloud<PointNormalType>);
    pcl::fromPCLPointCloud2(*srcCloudCorrespondence_, *corrs_source);
    pcl::fromPCLPointCloud2(*targetCloudCorrespondence_, *corrs_target);
    float tow_ms = 0.0;
    Eigen::Matrix4f final2_transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T2_T1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T2 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Final_Transformation = Eigen::Matrix4f::Identity();
    PointNormalType src_corres,tar_corres;
    Measure ms_max;
    bool reset = false;
    CirconImageDescriptor src_corres_descriptor;
  //  int level_itr = 0;
    int tc_corres, sc_corres;
    std::vector<PointNormalType>feature_points;
    int tIndex, sIndex;
    CloudWithNormalPtr srx = cid_source.GetoriginalCloud();
    CloudWithNormalPtr tgx = cid_target.GetoriginalCloud();
    PointNormalType test_src = srx->points[182126];
    Eigen::Vector3f min_pt;
    Eigen::Vector3f max_pt;
    float diag_length1 = tool::ComputeOrientedBoundingBoxOfCloud(srx, min_pt, max_pt);
    float diag_length2 = tool::ComputeOrientedBoundingBoxOfCloud(tgx, min_pt, max_pt);
    maximum_radius = diag_length1;
    if (diag_length2 > diag_length1)
        maximum_radius = diag_length2;

    float avg_dst_sample1 = metric::EstimatePointAvgDistance(input_source);
    float avg_dst_sample2 = metric::EstimatePointAvgDistance(input_target);
    float avg_dist = avg_dst_sample1;
    if (avg_dst_sample1 < avg_dst_sample2)
        avg_dist = avg_dst_sample2;

    cid_source.SetMaximumAverageDistance(avg_dist);
    cid_target.SetMaximumAverageDistance(avg_dist);

 /*   pcl::KdTreeFLANN<PointNormalType>::Ptr kdt_src(new pcl::KdTreeFLANN < PointNormalType>);
    kdt_src->setInputCloud(srx);
    pcl::KdTreeFLANN<PointNormalType>::Ptr kdt_tgt(new pcl::KdTreeFLANN < PointNormalType>);
    kdt_tgt->setInputCloud(tgx);*/
    std::string filePath = "Z:/staff/SDutta/GlobalRegistration/";
    std::vector<ON_NurbsSurface> nbs = cid_target.GetNurbsSurface();
    std::vector<ON_NurbsCurve> ncs = cid_target.GetNurbsCurve();
    std::vector<ON_NurbsSurface> nbs_src = cid_source.GetNurbsSurface();
    std::vector<ON_NurbsCurve> ncs_src = cid_source.GetNurbsCurve();
    std::vector<Eigen::Vector2i>cluster_label_src = cid_source.GetClusterLabels();
    std::vector<Eigen::Vector2i>cluster_label_tgt = cid_target.GetClusterLabels();

    std::vector<Eigen::Vector2d> std_params;
    bool use_nurbs_strategy = true;
    std::vector<Eigen::Matrix4f>similar_transformation;
    similar_transformation.reserve(2);
    int repeat_counter = 2;

    double gSize = 1.0 / 128.0;
    std::vector<Eigen::Vector2d> st_param = cid_source.GetInitialParameters();
    cParameterGrid cParam(gSize,
        /* point accessor */
        [&st_param](Eigen::Vector2d *out_pnt, double *out_attrib, size_t idx) -> bool
    {
        if (idx < st_param.size())
        {
            Eigen::Vector2d vec(st_param[idx](0), st_param[idx](1));
            *out_pnt = vec;
            *out_attrib = 0.0;
            return true;
        }
        else
            return false;
    }
    );
    cParam_Source = cParam;
   // cid_source.setParametergrid(cParam);
    // set target parameter-grid
    std::vector<Eigen::Vector2d> st_param_tar = cid_target.GetInitialParameters();
    cParameterGrid cParam_tar(gSize,
        /* point accessor */
        [&st_param_tar](Eigen::Vector2d *out_pnt, double *out_attrib, size_t idx) -> bool
    {
        if (idx < st_param_tar.size())
        {
            Eigen::Vector2d vec(st_param_tar[idx](0), st_param_tar[idx](1));
            *out_pnt = vec;
            *out_attrib = 0.0;
            return true;
        }
        else
            return false;
    }
    );
   // cid_target.setParametergrid(cParam_tar);
    cParam_target = cParam_tar;

    // addition of new interestpoint detection method//
    // create feature descriptor at coarse resolution////////////////////  
   /* CirconImageDescriptor cid_src_feature(input_source, 16, 16, 512, src_cloud_parameter, *nbs_src, *ncs_src);
    CirconImageDescriptor cid_tgt_feature(input_target, 16, 16, 512, tgt_cloud_parameter, *nbs, *ncs);*/
    // descriptor initialization ends///////////////////
  
    /* Build descriptor and sort the interest points pair in decreasing order of similarity          */
  /* std::vector<cEle>corres_pair = SortFeaturePointsOnSimilarityValue(corrs_source, corrs_target, cid_src_feature,
       cid_tgt_feature, avg_dist);*/

    // sort pair of correspondence at coarse resolution , i.e 16x16 to obtain initial set of correspondences
   std::vector<cEle>corres_pair = SortFeaturePointsOnSimilarityValue(corrs_source, corrs_target, avg_dist);
   int count_sim = 0;
   std::vector<cEle>sim_values;
   sim_values.reserve(6);
   float sim_max = -INF; float sim_min = INF;
   int min_idx, max_idx, MaxCorrespondencePair = 6;
   Measure Max_Correspondence_Measure;
   bool update_measure = false;
   int max_tar_Index;
  
   // get initial 7 pair of correspondence
   std::vector<cEle>batch_corres_pair = { corres_pair.begin(), corres_pair.begin() + 7};
   std::vector<cEle>::iterator it = corres_pair.begin();
   /*std::advance(it, 6);
   batch_corres_pair.insert(batch_corres_pair.begin(), it, it + 7);*/
   int iitr = 0;
   std::vector<Measure>batch_max_measures;
   int batch_max_measure_idx = -1;
   //  print the descriptor building time
   setDebug(false);
   int incr_reslolution = 0;
   int level_itr = 0;
   int batch_count = 1;

   // iterate over a batch of correspondence across hierarachy
   for (; iitr < batch_corres_pair.size(); ++iitr)
   // for (tIndex = 40; tIndex < 45; tIndex++)//

    {
       std::cout << "iitr:" << iitr << std::endl;
      // std::cout << batch_corres_pair[iitr].src_idx << "," << batch_corres_pair[iitr].tgt_idx << std::endl;
       target_basic_index = batch_corres_pair[iitr].tgt_idx;
       tar_corres =  original_tgt_cloud_with_normal->points[target_basic_index];// corrs_target->at(tIndex);
           // original_target_index[tIndex];
       cid_target.SetBasicPointIndex(target_basic_index);
        int start = 0;
        non_valid_index.clear();  
            // start point of interest
        source_basic_index = batch_corres_pair[iitr].src_idx;
        src_corres = original_src_cloud_with_normal->points[source_basic_index]; // corrs_source->at(sIndex);
         //  source_basic_index = sIndex ;// original_source_index[sIndex];
           // src_corres = srx->points[original_source_index[sIndex]];
            //if (use_nurbs_strategy)
            //{
            //    Eigen::VectorXd pt_source = Eigen::VectorXd::Zero(6);
            //    pt_source.head<3>() = src_corres.getVector3fMap().cast<double>();
            //    pt_source.tail<3>() = src_corres.getNormalVector3fMap().cast<double>();
            //    Eigen::Vector2d init_param(0.0, 0.0);
            //    Eigen::Vector2d optim_param;
            //    Eigen::VectorXd cp_source = surface::cNurbsSurface::ComputeClosetPointOnNurbSurface(pt_source, *nbs_src, *ncs_src, init_param, optim_param,
            //        1e-8);
            //    if (cp_source.head<3>() == Eigen::Vector3d::Zero())
            //        continue;

            //    float dp = (src_corres.getVector3fMap() - cp_source.head<3>().cast<float>()).dot(cp_source.tail<3>().cast<float>());
            //    if (dp == 1.0f || dp == -1.0f)
            //    {
            //        std::cout << "closest points correct" << std::endl;
            //    }
            //    else
            //    {
            //        std::cout << "dp:" << dp << std::endl;
            //    }
            //   
            //    src_corres.getVector3fMap() = cp_source.head<3>().cast<float>(); // corrs_source->at(sIndex);
            //    src_corres.getNormalVector3fMap() = cp_source.tail<3>().cast<float>();
            //}
          
        
          /* for a correspondence pair, iterate over resolution to find the correct match ( high similarity) 
          for a target point until highest resolution*/
     
            while (level_itr != num_resolution)
            {
                // prepare source descriptor
                std::cout << "Current Level:" << level_itr << std::endl;
                cid_source.SetBasicPointIndex(source_basic_index);
               /* if (level_itr == num_resolution - 1)
                {
                    up_resolution_count *= 2;
                }*/
                PrepareDescriptor(cid_source, src_corres, use_nurbs_strategy, true);
                source_descriptor_image = cid_source.GetDescriptorImage();

               
                PrepareDescriptor(cid_target, tar_corres, use_nurbs_strategy,false);
                target_descriptor_image = cid_target.GetDescriptorImage();
                Epsilon = cid_source.GetRadialResolution(); /// 16.0;

                if (_write)
                {
                    std::string filePath = "Z:/staff/SDutta/GlobalRegistration/";
                    char subfile[100], subImg[100], subImgTar[100], subTarFile[100];
                    sprintf(subfile, "%d_src_descriptor",source_basic_index);
                    sprintf(subImg, "%d_src_img.bmp",source_basic_index);
                    sprintf(subImgTar, "%d_tar_img.bmp",target_basic_index);
                    std::string ImageName = filePath + subImg;
                   // cid_source.WriteDescriptorAsImage(ImageName);

                    std::string CloudName = filePath + subfile;
                    cid_source.ReconstructPointCloud();
                    cid_source.WritePointCloud(CloudName);

                    //write points from descriptos -> to verify the contents :sanity check
                    /*std::string desc_source_file_name = filePath + "desc_source";
                    WritePointFromDescriptor(cid_source.GetDescriptorContent(), desc_source_file_name);*/

                    std::string TarImageName = filePath + subImgTar;
                   // cid_target.WriteDescriptorAsImage(TarImageName);
                    sprintf(subTarFile, "%d_tar_descriptor", target_basic_index);
                    std::string TarCloudName = filePath + subTarFile;
                    cid_target.ReconstructPointCloud();
                    cid_target.WritePointCloud(TarCloudName);
                }
                // Compare descriptors : create secondary descriptors and retrurn max measure along with rotation index
                CompareDescriptor(source_descriptor_image, target_descriptor_image, 1);
                // store the similarity measure of corresondence pair greater than 1 and less than maximum
                if (count_sim < MaxCorrespondencePair && MaxCorrespondencePair > 1)
                {
                    sim_values.emplace_back(cEle(max_measure.similarity_value, max_measure.point_index,
                        batch_corres_pair[iitr].tgt_idx));
                    batch_max_measures.emplace_back(max_measure);

                    // keep tracks which pair of source and target point gives max- similarity for every six pair of points
                    if (max_measure.similarity_value > sim_max)
                    {
                        sim_max = max_measure.similarity_value;
                        max_idx = max_measure.point_index;
                      //  Max_Correspondence_Measure = max_measure;
                        batch_max_measure_idx = iitr;
                        max_tar_Index = target_basic_index;
                        update_measure = true;
                    
                    }

                    if (max_measure.similarity_value < sim_min)
                    {
                        sim_min = max_measure.similarity_value;
                        min_idx = max_measure.point_index;
                    }

                    ++count_sim;
                    if (iitr == MaxCorrespondencePair - 1)
                    {
                        std::vector<cEle> indices;

                        // update to get the source pt and target pt with max similarity
                        if (update_measure)
                        {
                            std::cout << " max_idx for batch processing:" << batch_max_measure_idx << std::endl;
                            max_measure = std::move(batch_max_measures[batch_max_measure_idx]);// std::move(Max_Correspondence_Measure);
                            tar_corres = original_tgt_cloud_with_normal->points[max_tar_Index];
                            target_basic_index = max_tar_Index;
                            cid_target.SetBasicPointIndex(max_tar_Index);
                            update_measure = false;
                        }
                        if (count_sim > 1)
                        {
                            indices = RejectCorrespondenceBasedOnSimilarityValue(sim_values, sim_max, sim_min, 0.25);
                        }
                        count_sim = 0; // reset counter
                        sim_values.clear();
                        sim_values.shrink_to_fit();
                        batch_max_measures.clear();
                        batch_max_measures.shrink_to_fit();
                        batch_max_measure_idx = -1;
                        MaxCorrespondencePair = indices.size() >= 1 ? indices.size() : 0;
                        sim_max = -INF;  sim_min = INF;

                        /// current hack, move to higher resolution when at a given resolution , the correspondence pair is more than 1
                        if (MaxCorrespondencePair > 1)
                        {
                            sim_max = -INF;  sim_min = INF;
                            std::sort(indices.begin(), indices.end(), SortFunction);
                            batch_corres_pair.erase(batch_corres_pair.begin(), batch_corres_pair.end());
                            batch_corres_pair.insert(batch_corres_pair.begin(), indices.begin(), indices.end());
                            iitr = -1;
                            // reduce the no. of cols to evaluate by half and modify the resolution
                            nr_search_col = nr_search_col / 2;
                            incr_reslolution = level_itr + 1;
                            int num_row = 2.0 * cid_source.GetRowDivision(); //  std::pow(2, incr_reslolution)
                            int num_col = cid_source.GetColumnDivision()* 2.0; // std::pow(2, incr_reslolution);  // may be changed
                            int num_height = cid_source.GetHeightDivision(); //std::pow(2, incr_reslolution) * 
                            SetResolutionOfDescriptor(cid_source, num_row, num_col, num_height);
                            SetResolutionOfDescriptor(cid_target, num_row, num_col, num_height);
                            cid_source.UpdateImageDimension(num_col, num_row);
                            cid_target.UpdateImageDimension(num_col, num_row);
                            this->division_col = num_col;
                            this->division_row = num_row;
                            source_basic_index = -1;
                            target_basic_index = -1;
                            reset = false;
                            level_itr++;
                            break;
                        }
                    }
                    else
                        break;
                }
              
                ////////////////

                  //  tow_ms = std::pow(0.80, level_itr + 1) * max_measure.similarity_value;
                  
                    T2 = max_measure.WlTransform;
                    T1 = cid_target.GetTransformation();
                   /* if (level_itr == num_resolution - 1 && max_measure.rotation_index == division_row - 1)
                        max_measure.rotation_index = 0;*/
                    float angle = cid_source.GetAngularResolution() * (max_measure.rotation_index);
                    //  std::cout << "rotation_angle:" << angle << endl;
                    T2_T1.row(0) = Eigen::Vector4f(cos(angle), sin(angle), 0, 0);
                    T2_T1.row(1) = Eigen::Vector4f(-sin(angle), cos(angle), 0, 0);
                    T2_T1.row(2) = Eigen::Vector4f(0, 0, 1, 0);
                    Final_Transformation = T1.inverse()* T2_T1 *  T2;
     
                    if (division_row == 128)
                    {
                      
                                                                                                                                                                                                //  std::cout << T1.inverse() << "\n" << T2_T1 << "\n" <<  T2 << std::endl;
                        std::cout << "src_corres:" << max_measure.point_of_interest.getVector3fMap().transpose() <<std::endl;
                        std::cout << "tar_corres:" << tar_corres.getVector3fMap().transpose() << std::endl;
                        std::cout << "transformation at resolution(128 X 128):" << "\n" << Final_Transformation << std::endl;
                        std::cout << " source transformation at resolution(128 X 128):" << "\n" << T2 << std::endl;
                        std::cout << " target transformation at resolution(128 X 128):" << "\n" << T1.inverse() << std::endl;
                        std::cout << " rotation transformation@ 128 x 128:" << "\n" << T2_T1 << std::endl;
    
                    }
                  
                if (level_itr == num_resolution - 1)
                {
                    T2.setIdentity();
                    T1.setIdentity();
                    T2 = max_measure.WlTransform;
                    T1 = cid_target.GetTransformation();
                    //if (max_measure.rotation_index == division_row - 1)
                    //    max_measure.rotation_index += 1;
                    //T2 = max_measure.WlTransform; // cid_source.GetTransformation();
                    //T1 = cid_target.GetTransformation();
                    ////  float theta = cid_source.GetAngularResolution();
                    //float angle = cid_source.GetAngularResolution() * (max_measure.rotation_index);
                    ////  std::cout << "rotation_angle:" << angle << endl;
                    //T2_T1.row(0) = Eigen::Vector4f(cos(angle), sin(angle), 0, 0);
                    //T2_T1.row(1) = Eigen::Vector4f(-sin(angle), cos(angle), 0, 0);
                    //T2_T1.row(2) = Eigen::Vector4f(0, 0, 1, 0);
                    Eigen::Matrix4f Tg = Eigen::Matrix4f::Identity();
                    Tg << 0.867416, 0.003512,-0.497571,-0.622662,
                        -0.003265, 0.999994,0.001367, 0.000760,
                        0.497573,0.000439, 0.867422, -0.166305,
                        0, 0, 0, 1;

                

                    /////////////Create Descriptor @ max resolution/////////////////////
                    CirconImageDescriptor cid_source_max(input_source, division_row, division_col, 
                        cid_source.GetHeightDivision(), src_cloud_parameter,
                        nbs_src, ncs_src,cluster_label_src);
                    PointNormalType src_corres_at_max = max_measure.point_of_interest;
                    cid_source_max.SetBasicPointIndex(max_measure.point_index);
                    cid_source_max.SetMaximumAverageDistance(avg_dist);
                    PrepareDescriptor(cid_source_max, src_corres_at_max, use_nurbs_strategy, true);
                    char max_file_name[100];
                    sprintf(max_file_name, "src_max_descriptor_%d", max_measure.point_index);
                    std::string Src_max_file = filePath + max_file_name;
                    cid_source_max.ReconstructPointCloud();
                    cid_source_max.WritePointCloud(Src_max_file);
                        /////////////////////////////////////////////////////////////////

                    //find transformation and check stopping criterion
                    auto startItr = std::chrono::high_resolution_clock::now();
                    std::cout << " source transformation at max:" << "\n" << T2 << std::endl;
                    std::cout << " target transformation at max" << "\n" << T1.inverse() << std::endl;
                    std::cout << " rotation transformation@" << division_row << "X" << division_col << "\n" << T2_T1 << std::endl;
                    Final_Transformation = T1.inverse()* T2_T1 *  T2;
                   
                    Eigen::Matrix4f delta_transform = Final_Transformation *  Tg.inverse();
                    std::cout << "\n";
                   // std::cout << "Difference Transformation:\n" << delta_transform << std::endl;
                    std::cout << "\n";
                  //  std::cout << Final_Transformation.inverse()  * Tg << std::endl;

                    Eigen::Vector3f tar_norm = cid_target.GetRotationAXisPoint().getNormalVector3fMap();
                    Eigen::Vector3f src_norm = max_measure.point_of_interest.getNormalVector3fMap();

                    std::pair<PointNormalType, PointNormalType>pt_corres;
                    // compute fictituous correspondence pair for source & target clouds
                    pt_corres.first =   ComputeFictitiousCorrespondence(max_measure.descriptor_content, max_measure.rotation_index);
                    pt_corres.second = ComputeFictitiousCorrespondence(cid_target.GetDescriptorContent(), max_measure.rotation_index);
                    std::pair<int, int> index_pair(0,0);
        
                    int col_indx = max_measure.cell_index%division_col;
                    // std::pair<float, float>error = ComputeStoppingCriteria(src1, tar1, max_measure.point_of_interest, tar_corres, T2_T1, Final_Transformation);
                    if (index_pair.first == -1 || index_pair.second == -1)
                    {
                        reset = true;
                    }
                    else
                    {
                        std::cout << "src_corres:" << max_measure.point_of_interest.getVector3fMap().transpose() << std::endl;
                        std::cout << "tar_corres:" << tar_corres.getVector3fMap() .transpose()<< std::endl;
                        std::cout << "Final_Transformation:" << "\n" << Final_Transformation << std::endl;
                        Eigen::Vector4f fict_translation = Eigen::Vector4f::Identity();
                        std::pair<float, float>error = EstimateStopParameter(pt_corres, index_pair, T2_T1, Final_Transformation,
                            fict_translation);
                       /* Final_Transformation.col(3) = fict_translation;
                        Final_Transformation.col(3)(1) = Final_Transformation.col(3)(1) /5.0f;*/
                       
                      //  std::cout << "Final_Transformation after modification:" << "\n" << Final_Transformation << std::endl;
#ifdef LOGDATA
                        error_log("cell index at max resolution = %d\n", max_measure.cell_index);
                        error_log("Column index at max resolution = %d\n", col_indx);
                        error_log(" rotation error = %f, translation error = %f\n", error.first, error.second);
#endif
                        Eigen::Affine3f mat(Final_Transformation);
                        PointNormalType tfs_src_corres = pcl::transformPointWithNormal(pt_corres.first, mat);
                        float norm_pdt = pt_corres.second.getNormalVector3fMap().dot(tfs_src_corres.getNormalVector3fMap());
                      //  std::cout << "norm_pdt:" << norm_pdt << std::endl;
                        if (norm_pdt >= 0.92)
                        {  // 0.025 & 0.025 previuosly
                            if (error.first <= 0.050 && error.second <= 0.045 * maximum_radius)//tar_norm.dot(src_norm) >= 0.9) // 0.025 0.0045
                            {
                                PointNormalType tfs_src_corres = pcl::transformPointWithNormal(max_measure.point_of_interest, mat);
                                feature_points.push_back(tfs_src_corres);
                                feature_points.push_back(tar_corres);
                                tc_corres = target_basic_index;
                                sc_corres = source_basic_index;

                                if (_write)
                                {

                                    std::string ImageName = filePath + "best.bmp";
                                    std::string CloudName = filePath + "best_cloud";
  
                                   /* std::cout << "Target Index:" << target_basic_index << "," << "source_index:"
                                        << max_measure.point_of_interest << std::endl;*/

                                    std::string CorresName = filePath + "src_corres_best_found";
                                    WritePointCloud(feature_points, CorresName);
                                }
                                feature_points.clear();
                                std::cout << "Number of batch processed:" << batch_count << std::endl;
                                auto end_algo = std::chrono::high_resolution_clock::now();
                                double extime = std::chrono::duration_cast<
                                    std::chrono::duration<double, std::milli>>(end_algo - start_algo).count();
                                extime = extime / double(1000);
                                std::cout << " total execution time:" << extime << "sec" << std::endl;
                                return Final_Transformation;
                              /*  similar_transformation.push_back(Final_Transformation);
                                repeat_counter--;
                                if (repeat_counter == 0)
                                {

                                    Eigen::Matrix3f mat2 = similar_transformation[0].block<3, 3>(0, 0);
                                    Eigen::Matrix3f mat = similar_transformation[1].block<3, 3>(0, 0);
                                    Eigen::AngleAxisf newAngleAxis(mat);
                                    Eigen::AngleAxisf newAngleAxis_2(mat2);
                                    std::cout << newAngleAxis.angle() << "," << newAngleAxis_2.angle() << std::endl;
                                    std::cout << newAngleAxis.axis() << "," << newAngleAxis_2.axis() << std::endl;
                                    Eigen::Matrix3f rot = (mat2 + mat) * 0.5;
                                    Eigen::Vector3f avg_translation = (similar_transformation[0].col(3).head<3>() + similar_transformation[1].col(3).head<3>()) * 0.5;
                                    std::cout << rot << std::endl;
                                    Final_Transformation.block<3, 3>(0, 0) = rot;
                                    Final_Transformation.col(3) = Eigen::Vector4f(avg_translation(0), avg_translation(1), avg_translation(2), 1.0);
                                    std::cout << "transformation at final outcome:" << "\n" << Final_Transformation << std::endl;
                                    return Final_Transformation;
                                }
                                else
                                    reset = true;*/
                            }
                            else
                                reset = true;
                        }
                        else
                            reset = true;
                        // }
                    }

                    auto finishItr = std::chrono::high_resolution_clock::now();
                    double executeTime = std::chrono::duration_cast<
                        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
                    executeTime = executeTime / double(1000);
                    //  std::cout << "Time consumed for rotation shift similarity:" << executeTime << "sec" << std::endl;
                }
                else  // increase resolution
                {
                    src_corres = max_measure.point_of_interest;
                    source_basic_index = max_measure.point_index;
                    // reduce the no. of cols to evaluate by half and modify the resolution
                    nr_search_col = nr_search_col / 2;
                    incr_reslolution = level_itr + 1;
                    int num_row = 2.0 * cid_source.GetRowDivision(); //  std::pow(2, incr_reslolution)
                    int num_col = cid_source.GetColumnDivision()* 2.0; // std::pow(2, incr_reslolution);  // may be changed
                    int num_height = cid_source.GetHeightDivision(); //std::pow(2, incr_reslolution) * 
                    SetResolutionOfDescriptor(cid_source, num_row, num_col, num_height);
                    SetResolutionOfDescriptor(cid_target, num_row, num_col, num_height);
                    cid_source.UpdateImageDimension(num_col, num_row);
                    cid_target.UpdateImageDimension(num_col, num_row);
                    this->division_col = num_col;
                    this->division_row = num_row;
                    // nr_search_col = num_col / 2;
                    level_itr++;
                    continue;
                }
                if (reset)
                {
                   /* cid_source.ResetFlagForHighResolution();
                    cid_target.ResetFlagForHighResolution();
                    high_flag = false;*/
                    MaxCorrespondencePair = 6;
                    int num_row = cid_source.GetRowDivision() / std::pow(2, level_itr); //incr_reslolution
                    int num_col = cid_source.GetColumnDivision() / std::pow(2, level_itr);  // may be changed num_row;
                    int num_height = cid_source.GetHeightDivision();// / std::pow(2, incr_reslolution);
                    nr_search_col = cid_source.GetRowDivision() / std::pow(2, level_itr);// cid_source.GetHeightDivision();
                    SetResolutionOfDescriptor(cid_source, num_row, num_col, num_height);
                    SetResolutionOfDescriptor(cid_target, num_row, num_col, num_height);
                    cid_source.UpdateImageDimension(num_col, num_row);
                    cid_target.UpdateImageDimension(num_col, num_row);
                    this->division_col = num_col;
                    this->division_row = num_row;
                    source_basic_index = -1;
                    reset = false;
                    /////////////////
                    if (it!= corres_pair.end())
                    {
                        batch_corres_pair.erase(batch_corres_pair.begin(), batch_corres_pair.end());
                        std::advance(it, 6);
                        batch_corres_pair.insert(batch_corres_pair.begin(), it, it + 7);
                      //  std::cout << batch_corres_pair[0].src_idx << batch_corres_pair[0].tgt_idx << std::endl;
                        iitr = -1;
                        sim_min = INF;
                        sim_max = -INF;
                     
                    }
                    ////////////////
                    if (_write)
                    {
                        char subfile[100];
                        sprintf(subfile, "src_corres_%d_%d", target_basic_index, source_basic_index);
                        std::string CorresName = filePath + subfile;
                        WritePointCloud(feature_points, CorresName);
                    }
                    feature_points.clear();
                    incr_reslolution = 0;
                    level_itr = 0;
                    ++batch_count;
                    break;
                }


            }

      //  }


    }
   // std::cout << "Target Index:" << tc_corres << "," << "source_index:" << sc_corres << std::endl;
    std::string fileName = "Z:/staff/SDutta/GlobalRegistration/column_half";
    WritePointCloud(feature_points, fileName);
   // Final_Transformation = T1.inverse()* T2_T1 * T2;
    std::cout << "Target Index:" << target_basic_index << "," << "source_index:" << max_measure.point_of_interest << std::endl;
    return Final_Transformation;
}
//Eigen::Matrix4f CirconCorrespondence::ComputeAlignmentMatrixinLocalFrame(Eigen::Matrix4f T1, Eigen::Vector3f Trans)
//{
//    Eigen::Matrix4f Align_Matrix = Eigen::Matrix4f::Identity();
//    Align_Matrix.block<3, 3>(0, 0) = T1.block<3, 3>(0, 0);
//    Align_Matrix.col(3) = Trans;
//    return Align_Matrix;
//   
//}
//void CirconCorrespondence::ComputeSimilarityBetweenTwoCloud( std::string dirname, std::string OutputFile)
//{
//    CloudWithNormalPtr srx = cid_source.GetoriginalCloud();
//    CloudWithNormalPtr tgx = cid_target.GetoriginalCloud();
//    int tIndex, sIndex;
//    std::string  fileNameWithLocation = OutputFile + ".txt";
//    FILE *pFile;
//    pFile = fopen(fileNameWithLocation.c_str(), "wb");
//    fprintf(pFile, "src\ttar\tms\trotation_index\n");
//    char subsrcfile[100], subtarfile[100], subImg[100], subImgTar[100];
//    float max_sim = 0;
//    
//    std::string  fileWithLocation = OutputFile + "_Max_Similarity" + ".txt";
//    FILE *nFile;
//    nFile = fopen(fileWithLocation.c_str(), "wb");
//    fprintf(nFile, "src\ttar\tms\trotation_index\n");
//    Eigen::Vector3f min_pt; 
//    Eigen::Vector3f max_pt;
//    std::vector<int> SearchResults;
//    std::vector<float> SearchDistances;
//   float diag_length = tool::ComputeOrientedBoundingBoxOfCloud(srx, min_pt, max_pt);
//   pcl::KdTreeFLANN<PointNormalType> KdTree;
//   KdTree.setInputCloud(srx);
//   PointNormalType query_point = srx->at(23958);
//   KdTree.radiusSearch(query_point, 0.1 * diag_length, SearchResults, SearchDistances);
//  // SetParameterForSimilarityMeasure(0.11, 0.01);
//    for (tIndex = 7078; tIndex < 7079; ++tIndex)
//    {
//        int rotation_idx, max_rotation_idx = 0;
//        float max_sim = 0;
//        int max_src_index = -1, max_tgt_index = -1;
//        cid_target.SetBasicPointIndex(tIndex);
//        PointNormalType pt_tgt = tgx->at(tIndex);
//        PrepareDescriptor(cid_target, pt_tgt);
//       /* sprintf(subImgTar, "tar_img_%d.bmp", tIndex);
//        std::string TarImageName = dirname + subImgTar;
//        cid_target.WriteDescriptorAsImage(TarImageName);
//
//        sprintf(subtarfile, "tar_recons_%d", tIndex);
//        std::string TarCloudName = dirname + subtarfile;
//        cid_target.ReconstructPointCloud();
//        cid_target.WritePointCloud(TarCloudName);*/
//
//        target_descriptor_image = cid_target.GetDescriptorImage();
//        for (sIndex = 0; sIndex < SearchResults.size(); ++sIndex)
//        {
//            cid_source.SetBasicPointIndex(SearchResults[sIndex]);
//            PointNormalType pt_src = srx->at(SearchResults[sIndex]);
//            PrepareDescriptor(cid_source, pt_src);
//           /* sprintf(subImg, "src_img_%d.bmp", sIndex);
//            std::string SrcImageName = dirname + subImg;
//            sprintf(subsrcfile, "src_recons_%d", sIndex);
//            std::string SrcCloudName = dirname + subsrcfile;
//            if (iospace::ExistsFile(SrcImageName) == false)
//            {
//                cid_source.WriteDescriptorAsImage(SrcImageName);
//                cid_source.ReconstructPointCloud();
//                cid_source.WritePointCloud(SrcCloudName);
//            }*/
//            source_descriptor_image = cid_source.GetDescriptorImage();
//            float ms = ComputeMeasureOfSimilarity(source_descriptor_image, target_descriptor_image);
//            std::vector<float>max_shift_descriptor;
//            std::map<int, size_t>point_img_map = cid_source.GetImagePointMap();
//            int idx = cid_source.GetDescriptorCellIndexForInterestPoint();
//            ms = ComputeRotationIndexFromShift(source_descriptor_image, target_descriptor_image, max_shift_descriptor, rotation_idx, point_img_map, idx);
//
//            fprintf(pFile, "%d\t%d\t%f\t%d\r\n", SearchResults[sIndex], tIndex, ms, rotation_idx);
//            if (ms > max_sim)
//            {
//                max_sim = ms;
//                max_src_index = SearchResults[sIndex];
//                max_tgt_index = tIndex;
//                max_rotation_idx = rotation_idx;
//            }
//            cid_source.UpdateImageDimension(division_col, division_row);
//        }
//        fprintf(nFile, "%d\t%d\t%f\t%d\r\n", max_src_index, max_tgt_index, max_sim, max_rotation_idx);
//        fflush(nFile);
//        fflush(pFile);
//        cid_target.UpdateImageDimension(division_col, division_row);
//    }
//    fclose(pFile);
//    fclose(nFile);
//
 PointNormalType CirconCorrespondence::ComputeFictitiousCorrespondence(const std::vector<std::vector<_dV>> &descriptor_source, int rot_idx)
{

    int src_count = 0;
    Eigen::Vector3f src_centroid(0.0,0.0,0.0), src_centroid_normal(0.0,0.0,0.0);
    int col = (max_measure.cell_index ) % division_col;
    bool status = false;
    int shortened_row;
    if (false)
    {
        shortened_row = 16;
    }
    else
        shortened_row = division_row;
    int incr = (shortened_row / 3) + 1; // division_row
    
    int count = 0;
    int r = 1;
   /* for (int c = 1; c < division_col; c++)
    {*/
       
      /*  if (src_count == 3)
            break;*/
    while (src_count < 3)
    {
        for (r; r < shortened_row; r += incr) //division_row
        {
          
            auto itr = descriptor_source[r][col];
            if (itr.state == cellSate::INVALID) // itr.col_idx < 0 && itr.row_idx < 0
                continue;
            else
            {
                /*if (src_count < 3)
                {*/
                src_centroid = src_centroid + itr.pt.getVector3fMap();
                src_centroid_normal = src_centroid_normal + itr.pt.getNormalVector3fMap();
     /*               src_centroid = src_centroid + itr.pt.head<3>().cast<float>();
                    src_centroid_normal = src_centroid_normal + itr.pt.tail<3>().cast<float>();*/
                    src_count++;
                    std::cout << " fictituous increment:" << incr << "," << "row_index:" << r << std::endl;
               /* }
                else
                {
                    break;
                }*/
            }
           
        }
      /*  if (src_count < 3)
        {*/
            count++;
            r = count;
            incr++;
        /*}*/
    }
        
 /*   }*/
    src_centroid /= 3.0;
    src_centroid_normal /= 3.0;
    PointNormalType pt;
    pt.getVector3fMap() = src_centroid;
    pt.getNormalVector3fMap() = src_centroid_normal.normalized();
    return pt;
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
    std::vector<Eigen::Vector2d> params;
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
       CirconImageDescriptor src_corres_descriptor(input_source, r, c, h, params);
       CirconImageDescriptor tgx_corres_descriptor(input_target, r, c, h, params);
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
    std::cout << "Fictituous transformation is:" << "\n" << fict_transfrom << std::endl;
    Eigen::Matrix4f diff_transform = Corres_Transform *fict_transfrom.inverse();
    std::cout << "difference transformation earlier is:" << "\n" << fict_transfrom.inverse() * Corres_Transform << std::endl;
    std::cout << "difference transformation is:" << "\n" << diff_transform << std::endl;
    Eigen::Matrix3f rot_diff = diff_transform.block<3, 3>(0, 0);
    Eigen::Vector3f euler_angles = rot_diff.eulerAngles(2, 1, 0);
    Eigen::AngleAxisf axan(rot_diff);
   float ang = axan.angle();
   if (ang > M_PI)
       ang = 2 * M_PI - ang;
   else if (ang < -M_PI)
       ang = 2 * M_PI + ang;
   Eigen::Vector3f ax = axan.axis();
   std::cout << " diff angle:" << ang << std::endl;
    float rot_error = sqrtf(euler_angles.squaredNorm() / 3.0);

    float trans_error = diff_transform.col(3).head <3>().norm();
      
    return std::make_pair(ang, trans_error);
   
}

void CirconCorrespondence::prepareDescriptorForInterestPoint(CirconImageDescriptor& cid, const cParameterGrid &pGrid, 
    PointNormalType rotpoint,const bool &use_nurbs_strategy)
{
    
    cid.SetRotationAxisPoint(rotpoint);
    cid.ConstructLocalFrameOfReference();


    auto startItr = std::chrono::high_resolution_clock::now();
    CloudWithoutType transformed_cloud = cid.TransformPointToLocalFrame();
    /////////////////////

    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*transformed_cloud, *pTarget);
    float height = cid.ComputeheightFromPointCloud(pTarget);

    cid.SetMaxSearchColumn(nr_search_col);
    float max_radius = maximum_radius;
    cid.SetImageDescriptorResolution(FULL_ANGLE, max_radius, height);

    if (use_nurbs_strategy)
    {
        CUniformGrid2D cGrid2D(cid.GetAverageDist(),
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
        // parameter grid to decide on the optim parameter
        auto start = std::chrono::high_resolution_clock::now();

        auto end = std::chrono::high_resolution_clock::now();
        double execute = std::chrono::duration_cast<
            std::chrono::duration<double, std::milli>>(end - start).count();
        execute = execute / double(1000);
        //  std::cout << " parameter execution time:" << execute << std::endl;


        auto startopto = std::chrono::high_resolution_clock::now();

        cid.ComputeFeature(cGrid2D, pGrid);
      
        auto endopto = std::chrono::high_resolution_clock::now();
        double executeopto = std::chrono::duration_cast<
            std::chrono::duration<double, std::milli>>(endopto - startopto).count();
        executeopto = executeopto / double(1000);
        std::cout << " Descriptor time:" << executeopto << std::endl;

    }
    else
    {
        auto startopto = std::chrono::high_resolution_clock::now();
        cid.ComputeFeature(1);
        auto endopto = std::chrono::high_resolution_clock::now();
        double executeopto = std::chrono::duration_cast<
            std::chrono::duration<double, std::milli>>(endopto - startopto).count();
        executeopto = executeopto / double(1000);
        std::cout << " Descriptor time:" << executeopto << std::endl;
    }
    float theta = cid.GetRotationAngle();
    int num_rows_shift = std::round((theta * division_row) / (2 * M_PI));  //number of rows to shift
    cid.SetRotationIndex(num_rows_shift);

    // Epsilon = max_radius / (float(division_col * 16));  // currently no. of row steps  = no.of col steps ( square image)
}
void CirconCorrespondence ::PrepareDescriptor(CirconImageDescriptor& cid, PointNormalType rotpoint, 
    const bool &use_nurbs_strategy, const bool &cloud_type)
{
    // reset image2d
   cid.UpdateImageDimension(cid.GetColumnDivision(), cid.GetRowDivision());
   
    if (basic_descriptor)
    {
        cid.ComputeBasicPointOfInterest();  // considers closest point to centroid as the point around which descriptor is built
    }
    else
        cid.SetRotationAxisPoint(rotpoint);
    cid.ConstructLocalFrameOfReference();
 
   
    auto startItr = std::chrono::high_resolution_clock::now();
   // CloudWithoutType transformed_cloud = cid.TransformPointToLocalFrame();
    /////////////////////
   
   /* CloudPtr cloud =  Construct2DCloudForKdtreeSearch(cid.GetoriginalCloud(), cid.GetTransformation());
    cid.Set2DCloud(cloud);*/
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
  //  pcl::fromPCLPointCloud2(*transformed_cloud, *pTarget);
    if (cloud_type)
        pcl::transformPointCloudWithNormals(*original_src_cloud_with_normal, *pTarget, cid.GetTransformation());
    else
        pcl::transformPointCloudWithNormals(*original_tgt_cloud_with_normal, *pTarget, cid.GetTransformation());
   
    Eigen::Vector3f min_pt;
    Eigen::Vector3f max_pt;
    float diag_length1 = tool::ComputeOrientedBoundingBoxOfCloud(pTarget, min_pt, max_pt);
   // float height =  cid.ComputeheightFromPointCloud(pTarget);
  /*  std::cout << "diagonal length:" << diag_length1 << std::endl;
     std::cout << "height_divide:" << height << std::endl;
      std::cout << "radius_divide:" << maximum_radius  << std::endl;*/

    cid.SetMaxSearchColumn(nr_search_col);
    float max_radius = maximum_radius;// cid.GetRadiusFromCloud();// ComputeMaximumRadius(transformed_cloud);
// cid.SetUpResolutionCount(up_resolution_count);
    cid.SetImageDescriptorResolution(FULL_ANGLE,  max_radius, max_radius);

    if (use_nurbs_strategy)
    {
        CUniformGrid2D cGrid2D( cid.GetAverageDist(),
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
      auto startopto = std::chrono::high_resolution_clock::now();
        if (cloud_type)
            cid.ComputeFeature(cGrid2D, cParam_Source);
        else
            cid.ComputeFeature(cGrid2D, cParam_target);
        auto endopto = std::chrono::high_resolution_clock::now();
        double executeopto = std::chrono::duration_cast<
            std::chrono::duration<double, std::milli>>(endopto - startopto).count();
        executeopto = executeopto / double(1000);
        if (debug) //debug
            std::cout << " Descriptor time:" << executeopto << std::endl;
        
    }
    else
    {
        auto startopto = std::chrono::high_resolution_clock::now();
        cid.ComputeFeature(1);
        auto endopto = std::chrono::high_resolution_clock::now();
        double executeopto = std::chrono::duration_cast<
            std::chrono::duration<double, std::milli>>(endopto - startopto).count();
        executeopto = executeopto / double(1000);
        std::cout << " Descriptor time:" << executeopto << std::endl;
    }
    float theta = cid.GetRotationAngle();
    int num_rows_shift = std::round((theta * division_row) / (2 * M_PI));  //number of rows to shift
    cid.SetRotationIndex(num_rows_shift);
   
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
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*transformed_cloud, *pTarget);
    float max_radius = cid_target.ComputeMaximumRadius(transformed_cloud);
    float height = cid_target.ComputeheightFromPointCloud(pTarget);
    cid_target.SetImageDescriptorResolution(FULL_ANGLE, max_radius, height);
    //cid_target.ComputeFeature();
    target_descriptor_image = cid_target.GetDescriptorImage();
}
float CirconCorrespondence::ComputeMeasureOfSimilarity(std::vector<std::vector<float>> &src_image,  std::vector<std::vector<float>> &tar_image)
{
    float similarity_value = 0.0;
    float overlap_set_sum = 0.0; 
    float union_set_sum = 0.0;
    float sigma_ov = 1.0;
    float dist_overlap = 0.0f;
    int r = src_image.size();
    int c = src_image[0].size();
    if (c != tar_image[0].size())
    {
        std::cout << "MisMatch in Image Size:" << c << "!=" << tar_image[0].size() << std::endl;
    }
    for (int ir = 0; ir < r; ir++)
    {
        for (int ic = 0; ic < c; ic++)
        {
            if (src_image[ir][ic] != -INFINITY && tar_image[ir][ic] != -INFINITY)
            {
                overlap_set_sum += ic;
                dist_overlap += ic * std::fabs(src_image[ir][ic] - tar_image[ir][ic]);
            }
            if (src_image[ir][ic] != -INFINITY || tar_image[ir][ic] != -INFINITY)
            {
                union_set_sum += ic;
            }
        }
    }
    dist_overlap = dist_overlap / overlap_set_sum;
    sigma_ov = static_cast<float>(overlap_set_sum) / static_cast<float>(union_set_sum);
    float lambda_dash = rho_ * lambda_;
    similarity_value = (sigma_ov) / ((rho_ * dist_overlap + lambda_dash) + sigma_ov * (1 - lambda_dash));
    return similarity_value;

}

float CirconCorrespondence::ComputeSimilarityScore(std::vector<std::vector<float>> &src_image,
    const std::vector<std::vector<float>> &tar_image, float max_src, float min_src, float max_tgt, float min_tgt)
{
    int r = src_image.size();
    int c = src_image[0].size();
    if (c != tar_image[0].size())
    {
        std::cout << "MisMatch in Image Size:" << c << "!=" << tar_image[0].size() << std::endl;
    }
    float  sum_XY = 0, sum_X = 0 , sum_Y = 0, sum_X_sq = 0.0, sum_Y_sq= 0.0;
   // int squareSum_X = 0, squareSum_Y = 0;
    size_t count_overlap = 0;
    // first compute correlation coefficient 
    std::vector<float>src_values, tgt_values;
    
    for (int ir = 0; ir < r; ir++)
    {
        for (int ic = 0; ic < c; ic++)
        {
            if (src_image[ir][ic] != -INFINITY && tar_image[ir][ic] != -INFINITY)
            {
                ++count_overlap;
                float x = /*std::fabs*/(src_image[ir][ic] - min_src)/(max_src -min_src);
                float y = /*std::fabs*/(tar_image[ir][ic] - min_tgt) / (max_tgt - min_tgt);
                // sum of overlapping pixels :sourec and target
                sum_X = sum_X + x ;

                sum_Y = sum_Y + y;
                sum_XY = sum_XY +  x * y;
                sum_X_sq = sum_X_sq + (x * x);
                sum_Y_sq = sum_Y_sq + (y * y);
            
               // sum_XY = sum_XY + src_image[ir][ic] * tar_image[ir][ic];
                // sum of square of values. 
               /* squareSum_X = squareSum_X + src_image[ir][ic] * src_image[ir][ic];
                squareSum_Y = squareSum_Y + tar_image[ir][ic] * tar_image[ir][ic];*/
                src_values.emplace_back(std::fabs(src_image[ir][ic]));
                tgt_values.emplace_back(std::fabs(tar_image[ir][ic]));
            }
        }
    }
   // sum_XX =  count_overlap * sum_X_sq - std::powf(sum_X, 2.0)/* / static_cast<float>(count_overlap)*/;
   // sum_YY = count_overlap * sum_Y_sq - std::powf(sum_Y, 2.0) /*/ static_cast<float>(count_overlap)*/;
   //// sum_XY = sum_XY - (sum_X * sum_Y)/*/ static_cast<float>(count_overlap)*/;
   // float corr = (count_overlap * sum_XY - sum_X * sum_Y) / (sqrt(sum_XX * sum_YY));
    float ratio = 1.0 / (float)(count_overlap);
    float sq_ratio = 1.0 / (float)(count_overlap * count_overlap);
    sum_X = ratio *sum_X;
    sum_Y = ratio * sum_Y;
    sum_XY = ratio * sum_XY;
    sum_X_sq = ratio * sum_X_sq;
   sum_Y_sq = ratio * sum_Y_sq;
    float corr = (float)( sum_XY - sq_ratio *sum_X * sum_Y)
        / sqrt(( sum_X_sq -  sum_X * sum_X)
            * (sum_Y_sq -  sum_Y * sum_Y));
    

    //float variance_src = 0;
    //float variance_tgt = 0;
    //float num = static_cast<float>(src_values.size());
    //float meanX = sum_X /num;
    //float meanY = sum_Y /num;
    //
    //for (int i = 0; i< src_values.size(); i++)
    //{
    //    variance_src += std::pow(src_values[i] - meanX, 2.0);
    //    variance_tgt += std::pow(tgt_values[i] - meanY, 2.0);
    //}
    //float stddeviation_src = sqrt(variance_src);/*std::sqrt((std::inner_product(src_values.begin(), src_values.end(), src_values.begin(), 0)/num)
    //    - (meanX * meanX))*/ 
    //float stddeviation_tgt = sqrt(variance_tgt);/*std::sqrt((std::inner_product(tgt_values.begin(), tgt_values.end(), tgt_values.begin(), 0)/num)
    //                         - (meanY * meanY))*/ 
    //float covar = 0;
    //for (int itr = 0; itr < src_values.size(); itr++)
    //{
    //    covar += (src_values[itr] - meanX) * (tgt_values[itr] - meanY);
    //}
    //covar = covar /num ;
    //// use formula for calculating correlation coefficient. 
    //float corr = covar / (stddeviation_src * stddeviation_tgt);
    /*(float)(count_overlap * sum_XY - sum_X * sum_Y)/sqrt((count_overlap * squareSum_X - sum_X * sum_X)
            * (count_overlap * squareSum_Y - sum_Y * sum_Y))*/;
  //  std::cout << "new corr:" << corr << std::endl;
    float lambda = 3.0;
    float v = lambda / ((static_cast<float>(count_overlap) - 3.0));
    /*std::cout << "v:" << v << std::endl;
    std::cout << "new corr:" << std::atanh(corr) << std::endl;*/
    float sim = std::fabs((std::atanh(corr) * std::atanh(corr)) - v);
    if (sim == INFINITY)
        std::cout << sum_X_sq << "," << sum_Y_sq << "," << sum_XY << "," << sum_Y << std::endl;
    return sim;

}
CirconImageDescriptor CirconCorrespondence::TransformCirconDescriptor(int index)
{
    auto startopto = std::chrono::high_resolution_clock::now();
    CirconImageDescriptor transformed_descriptor = cid_source;
    auto endopto = std::chrono::high_resolution_clock::now();
    double executeopto = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(endopto - startopto).count();
    executeopto = executeopto / double(1000);
   // std::cout << " constructor time:" << executeopto << std::endl; 
    transformed_descriptor.SetBasicPointIndex(index);
    transformed_descriptor.SetMaximumRadius(maximum_radius);
    /////////////////////
    
   /* CloudPtr cloud =  Construct2DCloudForKdtreeSearch(transformed_descriptor.GetoriginalCloud(), transformed_descriptor.GetTransformation());
    transformed_descriptor.Set2DCloud(cloud);*/
    //////////////////////////////
    transformed_descriptor.UpdateImageDataAfterTransformation(index);
 
    return transformed_descriptor;
}
CirconImageDescriptor CirconCorrespondence::TransformCirconDescriptor(const PointNormalType/*Eigen::VectorXd*/ &pt)
{
    CirconImageDescriptor transformed_descriptor = cid_source;
    transformed_descriptor.SetBasicPointIndex(-1);
    transformed_descriptor.SetMaximumRadius(maximum_radius);
   // PointNormalType rotpoint = pt;
   /* rotpoint.getVector3fMap() = pt.head<3>().cast<float>();
    rotpoint.getNormalVector3fMap() = pt.tail<3>().cast<float>();*/
    transformed_descriptor.SetRotationAxisPoint(pt);
    transformed_descriptor.ConstructLocalFrameOfReference();
 /*  auto grid = cid_source.GetParameterGrid();
    transformed_descriptor.setParametergrid(*grid);*/
  /* CloudPtr cloud =  Construct2DCloudForKdtreeSearch(transformed_descriptor.GetoriginalCloud(), transformed_descriptor.GetTransformation());
    transformed_descriptor.Set2DCloud(cloud);*/
  // transformed_descriptor.SetUpResolutionCount(up_resolution_count);
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::transformPointCloudWithNormals(*original_src_cloud_with_normal, *pTarget, transformed_descriptor.GetTransformation());
    transformed_descriptor.CreateSecondaryDescriptor(pt,cParam_Source,pTarget);

    return transformed_descriptor;
}

void CirconCorrespondence::CompareDescriptor(const std::vector<std::vector<float>> &src_image,
    std::vector<std::vector<float>>&tar_image, int with_nurbs)
{
    float current_epsilon = INFINITY;

    PointNormalType init_point = cid_source.GetRotationAXisPoint();
    int prev_point_idx = -1;
    float init_ratio_nv = 0.0;
    float tow_nv = 0.9;
    int max_cell_index = 0;
    std::vector< CirconCorrespondence::Measure>similarity_structure_list;
    int non_valid_count = 0;
    std::vector<PointNormalType>previous_pts_list;
    int current_row_idx = 0;
    // set temp index for testing prev index
    prev_point_idx = source_basic_index;

    // calculate initial similarity value 

   // new_image = cid_source.GetDescriptorImage();
    std::vector<std::vector<float>>init_max_shift_descriptor;
    int init_rot_idx = 0;
    std::map<int, size_t>init_mage_point_index_map = cid_source.GetImagePointMap();
    int init_cell_idx = cid_source.GetDescriptorCellIndexForInterestPoint();
    std::vector<std::vector<_dV>> descriptor_source = cid_source.GetDescriptorContent();
    std::vector<std::vector<_dV>> init_descriptor_source = descriptor_source;
    int start_rotation_index;
    int secondary_rot_idx;
    if (division_row > 32)
    {

        init_rot_idx = 2 * max_measure.rotation_index;
        std::cout << "start_rotation_index" << "@" << division_row << "=" << init_rot_idx << std::endl;
        start_rotation_index = 2 * max_measure.rotation_index;

    }

    // float sm = 0.0f;
    float sm = ComputeMaximumRotationShiftParameter(cid_source.GetDescriptorImage(), tar_image,
        init_max_shift_descriptor, init_mage_point_index_map, descriptor_source, init_rot_idx,
        init_cell_idx);
    // CirconCorrespondence::Measure mes;
     // initialize similarity measure
     // reset the previous max simialrity measure only after initial assignment
    max_measure.Reset();
    max_measure.similarity_value = sm;
    max_measure.rotation_index = init_rot_idx;
    max_measure.cell_index = 0;// init_cell_idx;//itr.first;
  //  mes.cell_values = init_max_shift_descriptor;
    max_measure.point_of_interest = cid_source.GetRotationAXisPoint();
    max_measure.point_index = cid_source.GetBasicPointIndex();
    max_measure.WlTransform = cid_source.GetTransformation();
    max_measure.primary_idx = max_measure.cell_index % this->division_col;
    // image_point_index_map = init_mage_point_index_map; 
    max_measure.descriptor_content = std::move(descriptor_source);
    // previous_pts_list.push_back(mes.point_of_interest);
    int shortened_division_row;
    if (false)//division_row >= 256 || high_flag
    {
        shortened_division_row = 32;
    }
    else
        shortened_division_row = division_row;
    /* while (current_epsilon >= Epsilon)
     {*/
    int count = 0;
    std::vector<size_t>temp_non_valid;
    // CirconImageDescriptor transformed_source; // put inside for loop 
    auto start = std::chrono::high_resolution_clock::now();
    std::vector <CirconCorrespondence::Measure>similarity_measure;
    similarity_measure.resize(shortened_division_row * division_col, Measure());
    // std::cout << "I am here:" << std::endl;
    int reduced_lIdx = 0;

    // initialize max similarity value

    Measure secondary_max_measure;
#pragma omp parallel for schedule(dynamic, 1)
    for (int lin_idx = 0; lin_idx < shortened_division_row * division_col; lin_idx++)
    {
        int row_idx = lin_idx / division_col;
        int col_idx = lin_idx % division_col;
        if (col_idx > nr_search_col - 1)
            continue;
        auto itr = init_descriptor_source[row_idx][col_idx];
        if (itr.state == cellSate::INVALID) //itr.col_idx < 0 || itr.row_idx < 0
            continue;
        if (division_row >= 64 && col_idx == 0)
        {
            continue;
        }
       
        int secondary_rot_idx;
        if (division_row > 32)
            secondary_rot_idx = start_rotation_index;

        CirconImageDescriptor transformed_source;
        std::vector<std::vector<float>>new_secondary_image;//init_max_shift_descriptor;

        Eigen::Vector3f point_normal = itr.pt.getNormalVector3fMap()/*tail<3>()*/;
        float dp = (point_normal.cast<float>()).dot(Eigen::Vector3f(0.0, 1.0, 0.0));
        if (dp == -1.0f)
        {
            std::cout << itr.pt.getNormalVector3fMap() << std::endl;
            continue;

        }
        else if (dp == 1.0f)
        {
            std::cout << itr.pt.getNormalVector3fMap() << std::endl;
            continue;
        }

        else
        {
            // transformed_source.SetFlagForHighResolution(high_flag);
            transformed_source = TransformCirconDescriptor(itr.pt); //itr


          // transformed_source = TransformCirconDescriptor(itr.pt_idx);
            new_secondary_image = transformed_source.GetDescriptorImage();


        }

        std::vector<std::vector<float>>secondary_max_shift_image;

        std::map<int, size_t>secondary_mage_point_index_map = transformed_source.GetImagePointMap();
        int second_cell_idx = transformed_source.GetDescriptorCellIndexForInterestPoint();
        // check the similarity measure between new descriptor and the target image descriptor and return the similarity with max possible rotation   
        std::vector<std::vector<_dV>>desc_content = transformed_source.GetDescriptorContent(); 

        float updated_sm = ComputeMaximumRotationShiftParameter(new_secondary_image, tar_image, secondary_max_shift_image,
            secondary_mage_point_index_map, desc_content, secondary_rot_idx, second_cell_idx);
        
#pragma omp critical 
        if (updated_sm > secondary_max_measure.similarity_value)//updated_sm >= sm
        {

            secondary_max_measure.similarity_value = updated_sm;
            secondary_max_measure.rotation_index = secondary_rot_idx;
            secondary_max_measure.point_of_interest = transformed_source.GetRotationAXisPoint();
            secondary_max_measure.point_index = itr.pt_idx;
            secondary_max_measure.WlTransform = transformed_source.GetTransformation();
            secondary_max_measure.primary_idx = row_idx * division_col + col_idx;
            secondary_max_measure.descriptor_content = std::move(desc_content);
            secondary_max_measure.cell_index = col_idx;

            //       similarity_measure[lin_idx].similarity_value = updated_sm;
            //       // mes.cell_index = itr.row_idx * division_col + itr.col_idx;// itr.first;//
            //      // similarity_measure[lin_idx].cell_values = secondary_max_shift_image;

            //       similarity_measure[lin_idx].rotation_index = secondary_rot_idx;  //  transformed_source.GetRotationIndex()
            //      /* Eigen::Matrix4f inv_lW = transformed_source.GetTransformation().inverse();*/
            //       similarity_measure[lin_idx].point_of_interest = transformed_source.GetRotationAXisPoint();
            //       // image_point_index_map_current = std::move(secondary_mage_point_index_map);
            //       similarity_measure[lin_idx].point_index = itr.pt_idx; // FindElementInMap(itr, image_point_index_map); //transformed_source.GetPointIndexFromCloud(itr);
            //       similarity_measure[lin_idx].WlTransform = transformed_source.GetTransformation();
            //       // mes.img_pt_index_map = image_point_index_map_current;
            //       similarity_measure[lin_idx].primary_idx = itr.row_idx * division_col + itr.col_idx;
            //       // mes.pix_value = current_array_of_data[itr.first];
            //     //  desc_content_current = /*std::move*/(desc_content);
            //       similarity_measure[lin_idx].descriptor_content = std::move(desc_content);
            //     //  count++;

            ////  reduced_lIdx+= 1;

        }
        // similarity_measure[lin_idx] = mes;
         //else
         //{
         //    non_valid_count++;
         //    // invalid_pt_list.push_back(itr.second);
         //     // size_t _idx = transformed_source.GetPointIndexFromCloud(itr);
         //     // temp_non_valid.push_back(curr_point_idx);
         //   // count++;
         //    continue;
         //}
      //  reduced_lIdx++;
       // count++;
        /*      }
          }*/
          // }
    }
    auto end = std::chrono::high_resolution_clock::now();
    double execute = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(end - start).count();
    execute = execute / double(1000);
  //  std::cout << "total secondary cell iteration time @" << division_row << ":"<< execute << "sec" << std::endl;
    //  std::cout << " parameter execution time:" << execute << std::endl;
//    current_array_of_data = mes.cell_values;
//    PointNormalType current_point = mes.point_of_interest;
//    previous_pts_list.push_back(init_point);
//    std::cout << previous_pts_list.size() << std::endl;
//    float dist = DistanceBetweenPoints(previous_pts_list, current_point);// pcl::geometry::distance(init_point, current_point);
//    std::cout << dist << std::endl;
//    similarity_structure_list.push_back(mes);
//    float diff = std::fabs(dist - Epsilon);
//    if (diff <= 1e-4f)
//    {
//        mes = FindMaximumSimilarityInIterations(similarity_structure_list);
//        break;
//    }
//    else
//    {
//        current_epsilon = dist;
//        init_point = current_point;
//        descriptor_source = std::move(desc_content_current);
//       // image_point_index_map = image_point_index_map_current;
//        // image_point_index_map = SortMapElementsUsingKeyValue(image_point_index_map_current,nr_search_col, division_col);
//        prev_point_idx = static_cast<int>(mes.point_index);  // takes care so that the descriptor is not created again with the same point of previous iteration

//    }

//}

  /*  if (similarity_measure.size() > 0)
    {
        std::cout << "similarity_measure size:" << similarity_measure.size() << std::endl;
        auto startopto = std::chrono::high_resolution_clock::now();
        size_t idx = FindMaximumSimilarityInIterations(similarity_measure);
        auto endopto = std::chrono::high_resolution_clock::now();
        double executeopto = std::chrono::duration_cast<
            std::chrono::duration<double, std::milli>>(endopto - startopto).count();
        executeopto = executeopto / double(1000);
        std::cout << " assignment time:" << executeopto << std::endl;
        max_measure = similarity_measure[idx];
    }
*/
// max_measure = mes;  // keeps a copy of present measure as a member variable

    auto startmove = std::chrono::high_resolution_clock::now();
    if (secondary_max_measure.similarity_value > max_measure.similarity_value)
    {
        max_measure = std::move(secondary_max_measure);
    }
    auto endmove = std::chrono::high_resolution_clock::now();
    double executemove = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(endmove - startmove).count();
    executemove = executemove / double(1000);
    //  std::cout << " move time:" << executemove << std::endl;
  
   std::cout << "max_rotation_index" << "@" << division_row << "=" << max_measure.rotation_index << std::endl;
    /*  auto startmove = std::chrono::high_resolution_clock::now();
      similarity_measures_content = std::move(similarity_measure);
      auto endmove = std::chrono::high_resolution_clock::now();
      double executemove = std::chrono::duration_cast<
          std::chrono::duration<double, std::milli>>(endmove - startmove).count();
      executemove = executemove / double(1000);
      std::cout << " move time:" << executemove << std::endl;

*/
/*  return max_measure;*/
}

CirconCorrespondence::Measure CirconCorrespondence::CompareDescriptorWithSimilarityMeasure(const std::vector<std::vector<float>> &src_image, 
    const std::vector<std::vector<float>> &tar_image)
{
    float current_epsilon = INFINITY;

    CirconCorrespondence::Measure mes;
    PointNormalType init_point = cid_source.GetRotationAXisPoint();
    // take basic descriptor into a temp. array
    std::vector<std::vector<float>>current_array_of_data = src_image;
    std::vector<std::vector<float>>new_image;
    //int curr_point_idx;
    int prev_point_idx = -1;
    float init_ratio_nv = 0.0;
    float tow_nv = 0.9;
    int max_cell_index = 0;

    std::map<int, size_t>image_point_index_map = cid_source.GetImagePointMap();
    std::map<int, size_t>initial_start_map = image_point_index_map; // keeping acopy just for verification, may be deleted afterwards
    std::vector<int> invalid_pt_list;

    std::vector< CirconCorrespondence::Measure>similarity_structure_list;
    std::map<int, size_t>image_point_index_map_current;
    int non_valid_count = 0;
    std::vector<PointNormalType>previous_pts_list;
    int current_row_idx = 0;
    // set temp index for testing prev index
    prev_point_idx = source_basic_index;

    // calculate initial similarity value 
    new_image = cid_source.GetDescriptorImage();
    std::vector<std::vector<float>>init_max_shift_descriptor;
    int init_rot_idx;
    std::map<int, size_t>init_mage_point_index_map = cid_source.GetImagePointMap();
    int init_cell_idx = cid_source.GetDescriptorCellIndexForInterestPoint();
   // float sm = 0.0f;

    std::vector<std::vector<_dV>>desc_content = cid_source.GetDescriptorContent();

    float sm = ComputeMaximumRotationShiftParameter(new_image, tar_image, init_max_shift_descriptor, init_mage_point_index_map, desc_content,
        init_rot_idx,init_cell_idx);

    mes.cell_index = init_cell_idx;//itr.first;
  //  mes.cell_values = init_max_shift_descriptor;
    mes.rotation_index = init_rot_idx;
    image_point_index_map_current = init_mage_point_index_map;
  //  mes.img_pt_index_map = image_point_index_map_current;
    mes.point_of_interest = cid_source.GetRotationAXisPoint();
    mes.point_index = cid_source.GetBasicPointIndex();
    mes.WlTransform = cid_source.GetTransformation();
    mes.primary_idx = mes.cell_index % this->division_col;
    new_image = init_max_shift_descriptor;
    image_point_index_map = init_mage_point_index_map;
    mes.similarity_value = sm;
    previous_pts_list.push_back(mes.point_of_interest);
    auto startItr = std::chrono::high_resolution_clock::now();
    // std::map<int, size_t>image_point_index_map_refined;
    while (current_epsilon >= Epsilon)
    {
        // float sm = 0.0f;
        /* current_row_idx = (image_point_index_map.begin()->first) /division_col;*/
        std::vector<size_t>temp_non_valid;
        CirconImageDescriptor transformed_source;// = cid_source;
        // for each cell in an array, if the cell value is  valid: construct a descriptor around that point
        int valid_count = 0;

        //image_point_index_map_refined = SortMapElementsUsingKeyValue(image_point_index_map, nr_search_col, division_row);

        for (auto const& itr : image_point_index_map)
        {
            // neglect the first column as they resemble the same basic point with zero radius

            bool valid = FindElementInList(itr.second, invalid_pt_list);
            int curr_col_idx = itr.first % division_col;
            if (valid)
            {
                if (itr.second != prev_point_idx && curr_col_idx < nr_search_col)
                {
                    float dp = original_src_cloud_with_normal->points[itr.second].getNormalVector3fMap().dot(Eigen::Vector3f(0.0, 1.0, 0.0));
                    if (dp == -1.0f)
                    {
                        std::cout << itr.second << std::endl;
                        continue;

                    }
                    else if (dp == 1.0f)
                    {
                        std::cout << itr.second << std::endl;
                        continue;
                    }
                    else
                    {

                        transformed_source = TransformCirconDescriptor(static_cast<int>(itr.second)); //itr
                        new_image = transformed_source.GetDescriptorImage();

                    }

                }
                else
                    continue;

                std::vector<std::vector<float>>secondary_max_shift_descriptor;
                int secondary_rot_idx;
                std::map<int, size_t>secondary_mage_point_index_map = transformed_source.GetImagePointMap();
                int second_cell_idx = transformed_source.GetDescriptorCellIndexForInterestPoint();
                // check the similarity measure between new descriptor and the target image descriptor and return the similarity with max possible rotation   
                std::vector<std::vector<_dV>>desc_content; // TODO Use the update version
                float updated_sm = ComputeMaximumRotationShiftParameter(new_image, tar_image, secondary_max_shift_descriptor, 
                    secondary_mage_point_index_map,  desc_content, secondary_rot_idx, second_cell_idx);

                if (updated_sm > sm)
                {
                    sm = updated_sm;
                    mes.similarity_value = sm;
                    mes.cell_index = second_cell_idx;// itr.first;//
                   // mes.cell_values = secondary_max_shift_descriptor;

                    mes.rotation_index = secondary_rot_idx;  //  transformed_source.GetRotationIndex()
                    mes.point_of_interest = transformed_source.GetRotationAXisPoint();
                    image_point_index_map_current = std::move(secondary_mage_point_index_map);
                    mes.point_index = itr.second; // FindElementInMap(itr, image_point_index_map); //transformed_source.GetPointIndexFromCloud(itr);
                    mes.WlTransform = transformed_source.GetTransformation();
                  //  mes.img_pt_index_map = image_point_index_map_current;
                    mes.primary_idx = itr.first%division_col;
                   // mes.pix_value = current_array_of_data[itr.first];

                }
                else
                {
                    non_valid_count++;
                    invalid_pt_list.push_back(itr.second);
                    // size_t _idx = transformed_source.GetPointIndexFromCloud(itr);
                    // temp_non_valid.push_back(curr_point_idx);
                    continue;
                }

            }
            else
                continue;
        }

        // std::cout << "Time consumed for rotation shift similarity:" << executeTime << "sec" << std::endl;
        /*non_valid_points = temp_non_valid;
        float non_valid_ratio = float(non_valid_count) / float(num_cell);
        init_ratio_nv = non_valid_ratio;
        pt_validity_ratio.insert(std::pair<float, size_t>(non_valid_ratio,curr_point_idx));*/
        // update descriptor data for next comparison

      //  current_array_of_data = mes.cell_values;
        PointNormalType current_point = mes.point_of_interest;
        previous_pts_list.push_back(init_point);
        float dist = DistanceBetweenPoints(previous_pts_list, current_point);// pcl::geometry::distance(init_point, current_point);
        similarity_structure_list.push_back(mes);
        if (dist <= Epsilon)
        {
            size_t idx = FindMaximumSimilarityInIterations(similarity_structure_list);
            mes = similarity_structure_list[idx];
            break;
        }
        else
        {
            current_epsilon = dist;
            init_point = current_point;
            image_point_index_map = image_point_index_map_current;
            // image_point_index_map = SortMapElementsUsingKeyValue(image_point_index_map_current,nr_search_col, division_col);
            prev_point_idx = static_cast<int>(mes.point_index);  // takes care so that the descriptor is not created again with the same point of previous iteration

        }
    }
    auto finishItr = std::chrono::high_resolution_clock::now();
    double executeTime = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
    executeTime = executeTime / double(1000);
   // std::cout << "Time consumed for rotation shift similarity:" << executeTime << "sec" << std::endl;
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
 double CirconCorrespondence::ComputeRadiusForSampling(CloudWithoutType CloudA, CloudWithoutType CloudB, double sf)
 {
     CloudWithNormalPtr corrs_source(new pcl::PointCloud<PointNormalType>);
     CloudWithNormalPtr corrs_target(new pcl::PointCloud<PointNormalType>);
     pcl::fromPCLPointCloud2(*CloudA, *corrs_source);
     pcl::fromPCLPointCloud2(*CloudB, *corrs_target);
     Eigen::Vector3f min_pt;
     Eigen::Vector3f max_pt;
     double maximum_radius;
     float diag_length1 = tool::ComputeOrientedBoundingBoxOfCloud(corrs_source, min_pt, max_pt);
     float diag_length2 = tool::ComputeOrientedBoundingBoxOfCloud(corrs_target, min_pt, max_pt);
     if (diag_length1 > diag_length2)
         maximum_radius = static_cast<double>(diag_length1);
     else
         maximum_radius = static_cast<double>(diag_length2);
    
     return (sf *maximum_radius);
 }
 void CirconCorrespondence::ComputePointOfInterestbasedOnUniformSampling(CloudWithoutType inputCloud, double radius, std::string OutputFileName)
 {
     CloudWithNormalPtr _cloud(new pcl::PointCloud <PointNormalType>);
     pcl::fromPCLPointCloud2(*inputCloud, *_cloud);
     CloudWithNormalPtr cloudxyz(new pcl::PointCloud <PointNormalType>);
     CloudWithNormalPtr _filteredcloud(new pcl::PointCloud <PointNormalType>);
     pcl::UniformSampling<PointNormalType>uni_sampling/*(true)*/;
     pcl::search::KdTree<PointNormalType>::Ptr tree(new pcl::search::KdTree<PointNormalType>());
     pcl::PointCloud<int> keypoints_indices;
     uni_sampling.setInputCloud(_cloud);
     uni_sampling.setRadiusSearch(radius);
    // uni_sampling.
 /*    uni_sampling.setSearchMethod(tree);
     
     uni_sampling*/
     uni_sampling.filter(*cloudxyz);
     pcl::PointIndices ptIndx;
     uni_sampling.getRemovedIndices(ptIndx);
 
     size_t posSlash = OutputFileName.rfind('.');
     std::string subfileName = OutputFileName.substr(0, posSlash);
     subfileName = subfileName + ".txt";
     FILE *pFile;
     pFile = fopen(subfileName.c_str(), "wb");
  
  int itr;
  pcl::KdTreeFLANN<PointNormalType>kt;
  kt.setInputCloud(_cloud);
  std::vector<int>indxs;
  std::vector<float>dist;
   for (itr= 0; itr!= cloudxyz->points.size(); itr++)
   {
       kt.nearestKSearch(cloudxyz->points[itr], 1, indxs, dist);
       _filteredcloud->points.push_back(_cloud->points[indxs[0]]);
       fprintf(pFile, "%d\n", indxs[0]);
      /* _filteredcloud->points.push_back(cloudxyz->points[itr]);
       fprintf(pFile, "%d\n", ptIndx.indices[itr]);*/
   }
   fclose(pFile);
   _filteredcloud->width = cloudxyz->points.size();
   _filteredcloud->height = 1;
   pcl::io::savePLYFile(OutputFileName, *_filteredcloud);
 }

 void CirconCorrespondence::ComputeFeaturePoints(const CloudWithoutType &inputCloud, const ON_NurbsSurface &nb_surface,
     const double &radius, std::string OutputFileName)
 {
     CloudWithNormalPtr _cloud(new pcl::PointCloud <PointNormalType>);
     pcl::fromPCLPointCloud2(*inputCloud, *_cloud);
     CloudWithNormalPtr cloudxyz(new pcl::PointCloud <PointNormalType>);
     CloudWithNormalPtr _filteredcloud(new pcl::PointCloud <PointNormalType>);
     pcl::UniformSampling<PointNormalType>uni_sampling;
     uni_sampling.setInputCloud(_cloud);
     uni_sampling.setRadiusSearch(radius);
     uni_sampling.filter(*cloudxyz);
     pcl::PointIndices ptIndx;
     uni_sampling.getRemovedIndices(ptIndx);

     size_t posSlash = OutputFileName.rfind('.');
     std::string subfileName = OutputFileName.substr(0, posSlash);
     subfileName = subfileName + ".txt";
     FILE *pFile;
     pFile = fopen(subfileName.c_str(), "wb");

     // Evaluate nurbsurface at at u,v(0.5,0.5)
     Eigen::Vector3d vec3[3];
     nb_surface.Evaluate(0.5, 0.5, 1, 3, &vec3[0][0]);
     vec3[1].normalize();
     vec3[2].normalize();
     
     PointNormalType qPt;
     qPt.getVector3fMap() = vec3[0].cast<float>();
     qPt.getNormalVector3fMap() = (vec3[1].cross(vec3[2])).normalized().cast<float>();
     
     pcl::KdTreeFLANN<PointNormalType>kt;
     kt.setInputCloud(cloudxyz);  // initialize kdtree with uniformly sampled cloud points
     std::vector<int>indxs;
     std::vector<float>dist;
     double nn = cloudxyz->points.size();
     kt.nearestKSearch(qPt, nn, indxs, dist);
     _filteredcloud->points.push_back(qPt);
     int itr;
     fprintf(pFile, "%d\n", 0);
     for (itr = 0; itr < indxs.size(); itr++)
     {
         _filteredcloud->points.push_back(cloudxyz->points[indxs[itr]]);
         fprintf(pFile, "%d\n", indxs[itr]);
       
     }
     fclose(pFile);
     _filteredcloud->width = indxs.size() + 1;
     _filteredcloud->height = 1;
     pcl::io::savePLYFile(OutputFileName, *_filteredcloud);
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
         if (entry.first > threshold && count < 50)
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
    pcl::io::savePLYFile(FileName, *_filteredcloud);
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
   /*  if (noOfLines_A != noOfLines_B)
     {
         return -1;
     }*/
     char szParam1[50], szParam2[50];
     for (int i = 0; i < noOfLines_A; i++)
     {
         fscanf(pFile, "%s\n", szParam1);
         original_source_index[i] = atoi(szParam1);
      
     }
     for (int i = 0; i < noOfLines_B; i++)
     {
         fscanf(nFile, "%s\n", szParam2);
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
    std::vector<std::vector<float>>img_data = image.getImageData();
    image.WriteImage(fileName, img_data);

}
//float CirconCorrespondence::ComputeRotationIndexFromShift(std::vector<float>secondary_descriptor, std::vector<float>target_point_descriptor, std::vector<float>&max_shift_descriptor,
//    int &rotation_idx, std::map<int, size_t>&image_point_index_map_current, int& secondary_dsc_posn)
//{
//    std::string OutputDirectory = "X:/staff/SDutta/GlobalRegistration/Comparison/";
//    std::map<int, size_t>updated_map;
//    std::map<int, size_t>max_sim_updated_map;
//    if (secondary_descriptor.size() == 0)
//    {
//        return -1.0;
//    }
//    int desc_size = division_row * division_col;
//    max_shift_descriptor.clear();
//    max_shift_descriptor.resize(desc_size, -INFINITY);
//    std::vector<float>iterative_descriptor;
//    float max_similarity = ComputeMeasureOfSimilarity(secondary_descriptor, target_point_descriptor);
//    rotation_idx = 0;
//    max_sim_updated_map = image_point_index_map_current;  // initialize map
//    max_shift_descriptor = secondary_descriptor;  // keep the current descriptor as the max descriptor
//    
//  /*  CirconImage img_desc(division_col, division_row);
//    CirconImage img_desc2(division_col, division_row);
//    char subsrcfile[100], subitrfile[100];*/
//    
//  
//    int max_second_desc_idx = secondary_dsc_posn;
//    int updated_second_desc_idx;
//    std::vector<int>secondary_pt_indices = CopyElementsfromMaptoVector(desc_size, image_point_index_map_current);
//    for (int nr = 1; nr < division_row; nr++)
//    {
//       
//        iterative_descriptor.resize(division_row * division_col, -INFINITY);
//        for (int img_pix = 0; img_pix < secondary_descriptor.size(); img_pix++)
//        {
//            float value_at_idx = secondary_descriptor.at(img_pix);
//            int row_idx = img_pix / division_col;
//            int col_idx = img_pix%division_col;
//        
//            int new_row_idx = (row_idx + nr) % division_row;
//            int idx = new_row_idx * division_col + col_idx;
//            iterative_descriptor[idx] = value_at_idx;
//            if (value_at_idx != -INFINITY)
//            {
//               // int point_idx = FindElementInMap(img_pix, image_point_index_map_current);
//                updated_map[idx] = secondary_pt_indices[img_pix];
//            }
//
//            int up_row_idx = secondary_dsc_posn / division_col;
//            int up_col_idx = secondary_dsc_posn %division_col;
//          updated_second_desc_idx  = ((up_row_idx + nr) % division_row) * division_col + up_col_idx;
//        
//        }
//        float similarity_value = ComputeMeasureOfSimilarity(iterative_descriptor, target_point_descriptor);
//        if (similarity_value > max_similarity)
//        {
//            max_similarity = similarity_value;
//            max_shift_descriptor = iterative_descriptor;
//            rotation_idx = nr;
//           max_sim_updated_map = updated_map;
//           max_second_desc_idx = updated_second_desc_idx;
//           
//        }
//       /* sprintf(subitrfile, "src_img_itr_%d.bmp", nr);
//        std::vector<float>scaled_iterative_descriptor = ScaleImageData(iterative_descriptor);
//        img_desc2.SetImageData(scaled_iterative_descriptor);
//        std::string SrcImgName = OutputDirectory + subitrfile;
//        img_desc2.WriteImage(SrcImgName, scaled_iterative_descriptor);*/
//        iterative_descriptor.clear();
//        updated_map.clear();
//
//    }
//   /* sprintf(subsrcfile, "max_src_img_%d.bmp", rotation_idx);
//    std::vector<float>scaled_max_shift_descriptor = ScaleImageData(max_shift_descriptor);
//    img_desc.SetImageData(scaled_max_shift_descriptor);
//    std::string SrcCloudName = OutputDirectory + subsrcfile;
//    img_desc.WriteImage(SrcCloudName, scaled_max_shift_descriptor);
//    */
//   
//    image_point_index_map_current = max_sim_updated_map;  // update the current map 
//    secondary_dsc_posn = max_second_desc_idx;
//    return max_similarity;
//}
void CirconCorrespondence::EvaluateSimilarityMeasureParameter(int src_idx, int tar_idx, float delta_step, std::string dirname, std::string  OutputFile)
{
    CloudWithNormalPtr srx = cid_source.GetoriginalCloud();
    CloudWithNormalPtr tgx = cid_target.GetoriginalCloud();
  
    std::string  fileNameWithLocation = OutputFile + ".txt";
    FILE *pFile;
    pFile = fopen(fileNameWithLocation.c_str(), "wb");
    fprintf(pFile, "row\tlambda\tms\n");
    char subsrcfile[100], subtarfile[100], subImg[100], subImgTar[100];
    float max_sim = 0;
    int tIndex = tar_idx + 1;
    int sIndex = src_idx + 1;
    for (tar_idx; tar_idx < tIndex; ++tar_idx)
    {
        int rotation_idx, max_rotation_idx = 0;
        float max_sim = 0;
        int max_src_index = -1, max_tgt_index = -1;
        cid_target.SetBasicPointIndex(tar_idx);
        PointNormalType pt_tgt = tgx->at(tar_idx);
        PrepareDescriptor(cid_target, pt_tgt);
        target_descriptor_image = cid_target.GetDescriptorImage();
        for (src_idx; src_idx < sIndex; ++src_idx)
        {
            cid_source.SetBasicPointIndex(src_idx);
            PointNormalType pt_src = srx->at(src_idx);
            PrepareDescriptor(cid_source, pt_src);
            source_descriptor_image = cid_source.GetDescriptorImage();
            std::vector<float>max_shift_descriptor;
            std::map<int, size_t>point_img_map = cid_source.GetImagePointMap();
            int idx = cid_source.GetDescriptorCellIndexForInterestPoint();
            float rho = 0.01, lambda;
            for (int x = 1; x <= 100; x++)
            {
                lambda = 0.01;
                for (int y = 1; y <= 100; y++)
                {

                    SetParameterForSimilarityMeasure(rho, lambda);
                    float ms;/* = ComputeRotationIndexFromShift(source_descriptor_image, target_descriptor_image, max_shift_descriptor, rotation_idx, point_img_map, idx);*/
                    fprintf(pFile, "%f\t%f\t%f\r\n", rho, lambda, ms);
                    lambda += delta_step;
                    
                }
                rho += delta_step;
            }
        }
        fflush(pFile);
    }
    fclose(pFile);
}
void CirconCorrespondence::SetParameterForSimilarityMeasure(float row, float lambda)
{
    rho_ = row;
    lambda_ = lambda;
}
std::pair<PointNormalType, PointNormalType>CirconCorrespondence::ComputeFictituousCorrespondencePair(const PointNormalType &qSrcPt, const PointNormalType &qtgtPt,
    const pcl::KdTreeFLANN<PointNormalType> &ktree_src, const  pcl::KdTreeFLANN<PointNormalType> &ktree_tar, float avgDist, Eigen::Matrix4f curr_transform, std::pair<int, int>&index_pair)
{
    std::vector<int>indices_list;
    std::vector<float>dist_list;
    std::vector<int>indices_list_tgt;
    std::vector<float>dist_list_tgt;
    float qRadius = 0.04 * maximum_radius;
   // ktree_src.radiusSearch(qSrcPt, qRadius, indices_list, dist_list);
    CloudWithNormalPtr srx = cid_source.GetoriginalCloud();
    CloudWithNormalPtr tgx = cid_target.GetoriginalCloud();
    Eigen::Affine3f mat(curr_transform);
    std::pair<PointNormalType, PointNormalType>fict_corres_pair;
   /* index_pair.first = -1;
    index_pair.second = -1;*/
    PointNormalType src_centroid, tgt_centroid;
    ktree_src.radiusSearch(qSrcPt, qRadius, indices_list, dist_list);
    ktree_tar.radiusSearch(qtgtPt, qRadius, indices_list_tgt, dist_list_tgt);
    for (int i = 1; i < 4; i++)
    {
       
        src_centroid.getVector3fMap() += srx->points[indices_list[i]].getVector3fMap();
        tgt_centroid.getVector3fMap() += tgx->points[indices_list_tgt[i]].getVector3fMap();
        src_centroid.getNormalVector3fMap() += srx->points[indices_list[i]].getNormalVector3fMap();
        tgt_centroid.getNormalVector3fMap() += tgx->points[indices_list_tgt[i]].getNormalVector3fMap();
       
    }
    src_centroid.getVector3fMap() = src_centroid.getVector3fMap() / 3.0;
    tgt_centroid.getVector3fMap() = tgt_centroid.getVector3fMap() / 3.0;

    src_centroid.getNormalVector3fMap() = src_centroid.getNormalVector3fMap() / 3.0;
    tgt_centroid.getNormalVector3fMap() = tgt_centroid.getNormalVector3fMap() / 3.0;

    src_centroid.getNormalVector3fMap().normalize();
    tgt_centroid.getNormalVector3fMap().normalize();
    fict_corres_pair.first = src_centroid;
    fict_corres_pair.second = tgt_centroid;
 /*   for (int index = indices_list.size() - 1 ; index > 0; index--)
    {
        if (indices_list[index] < srx->points.size())
        {
            PointNormalType tfs_src_corres = pcl::transformPointWithNormal(srx->points[indices_list[index]], mat);
            ktree_tar.nearestKSearch(tfs_src_corres, 1, indices_list_tgt, dist_list_tgt);
            float dist = pcl::geometry::distance(tfs_src_corres, tgx->points[indices_list_tgt[0]]);
            float norm_pdt = tfs_src_corres.getNormalVector3fMap().dot(tgx->points[indices_list_tgt[0]].getNormalVector3fMap());
            if (dist < 0.05 * avgDist && norm_pdt > 0.97)
            {
                fict_corres_pair.first = srx->points[indices_list[index]];
                fict_corres_pair.second = tgx->points[indices_list_tgt[0]];
                index_pair.first = indices_list[index];
                index_pair.second = indices_list_tgt[0];
                std::cout << "target distance for fictituous correspodence:" << dist << std::endl;
                return fict_corres_pair;
            }
            else
                continue;

        }
        else
            continue;
    }*/
    return fict_corres_pair;
}
std::pair<float, float>CirconCorrespondence::EstimateStopParameter(const std::pair<PointNormalType, PointNormalType> &corres_pair, 
    const std::pair<int, int> &index_pair, const  Eigen::Matrix4f &Pair_Transform, const  Eigen::Matrix4f &Corres_Transform, Eigen::Vector4f &trans_fict)
{
    std::vector<Eigen::Vector2d> st_param;
    // Prepare Descriptors
    int r = cid_source.GetRowDivision();
    int c = cid_source.GetColumnDivision();
    int h = cid_source.GetHeightDivision();
    CirconImageDescriptor src_corres_descriptor(input_source, r, c, h);
    CirconImageDescriptor tgx_corres_descriptor(input_target, r, c, h);
    src_corres_descriptor.SetRotationAxisPoint(corres_pair.first);
    tgx_corres_descriptor.SetRotationAxisPoint(corres_pair.second);
    src_corres_descriptor.ConstructLocalFrameOfReference();
    tgx_corres_descriptor.ConstructLocalFrameOfReference();
  /*  src_corres_descriptor.SetBasicPointIndex(index_pair.first);
    tgx_corres_descriptor.SetBasicPointIndex(index_pair.second);
    PrepareDescriptor(src_corres_descriptor, corres_pair.first);
    PrepareDescriptor(tgx_corres_descriptor, corres_pair.second);*/
    Eigen::Matrix4f sc_transform = src_corres_descriptor.GetTransformation();
    Eigen::Matrix4f tr_transform = tgx_corres_descriptor.GetTransformation();
  //  std::cout << "Source Fictituous Transformation:\n" << sc_transform << std::endl;
  //  std::cout << "target Fictituous Transformation:\n" << tr_transform << std::endl;

    Eigen::Matrix4f  FictTransformation = tr_transform.inverse() * Pair_Transform * sc_transform;
    std::cout << "Fictituous transformation is:" << "\n" << FictTransformation << std::endl;

    Eigen::Matrix4f diff_transform = Corres_Transform *FictTransformation.inverse();
    Eigen::Matrix3f rot_diff = diff_transform.block<3, 3>(0, 0);
    Eigen::Vector3f euler_angles = rot_diff.eulerAngles(2, 1, 0);
    Eigen::AngleAxisf axan(rot_diff);
    float ang = axan.angle();
    if (ang > M_PI)
        ang = 2 * M_PI - ang;
    else if (ang < -M_PI)
        ang = 2 * M_PI + ang;
    Eigen::Vector3f ax = axan.axis();
    std::cout << " diff angle:" << ang << std::endl;
    float rot_error = sqrtf(euler_angles.squaredNorm() / 3.0);

    float trans_error = diff_transform.col(3).head <3>().norm();

    trans_fict = FictTransformation.col(3);
    return std::make_pair(ang, trans_error);

}
//std::vector<std::vector<float>>CirconCorrespondence::ResScaleTargetDescriptor(const std::vector<std::vector<float>> &Descriptor_Current)
//{
//    int row_min = 8;
//    int col_min = 8;
// 
//    std::vector<std::vector<float>>ImageDescriptors;
//    for (int it = row_min; it < division_row; it*=2)
//    {
//        std::vector<std::vector<float>>curr_img = tool::ReScaleImageBilinear(Descriptor_Current, division_row, division_col, row_min, col_min);
//        ImageDescriptors.push_back(curr_img);
//        row_min *= 2;
//        col_min *= 2;
//    }
//    return ImageDescriptors;
//}
std::vector<float>CirconCorrespondence::RotateImageByIndexNumber(const Eigen::MatrixXf & vector_matrix, int rot_idx)
{
   Eigen::MatrixXf out_matrix = tool::RowShiftMatrix(vector_matrix, rot_idx);
   std::vector<float>matrix_vector = tool::CreateStlVectorFromMatirx(out_matrix);
   return matrix_vector;
}

float CirconCorrespondence::ReScaleAndEvaluateDescriptorForMaximumRotation(const std::vector<std::vector<float>>& src_descriptor,
    const std::vector<std::vector<float>>& target_descriptor, 
    int &rotation_idx, std::vector<std::vector<float>> &max_descriptor,int min_res)
{

    auto startItr = std::chrono::high_resolution_clock::now();
    int row_min = 8;
    int col_min = 8;
    int curr_image_row = src_descriptor.size();
    int  curr_image_col = src_descriptor[0].size();
    std::vector<std::vector<std::vector<float>>>src_descriptors;
    std::vector<std::vector<std::vector<float>>>tar_descriptors;
  /*  if (curr_image_row >= 256)
    {
        col_min = curr_image_col;
    }*/
    int vec_size = std::log2(curr_image_row) - 2;// (curr_image_row / row_min) + 1;
    src_descriptors.resize(vec_size);
    tar_descriptors.resize(vec_size);
    if (division_row <= 32)
    {
        for (int itr = 0; itr < vec_size -1; itr++) //int it = row_min; it < curr_image_row; it *= 2
        {
            std::vector<std::vector<float>>curr_img_source = tool::ReScaleImageBilinear(src_descriptor, curr_image_row, curr_image_col, row_min, col_min);
            src_descriptors[itr] = std::move(curr_img_source);

            std::vector<std::vector<float>>curr_img_target = tool::ReScaleImageBilinear(target_descriptor, curr_image_row, curr_image_col, row_min, col_min);
            tar_descriptors[itr] = std::move(curr_img_target);
            row_min *= 2;
            if (true) //curr_image_row <= 128
                col_min *= 2;
        }
        src_descriptors[vec_size- 1] = (src_descriptor);
        tar_descriptors[vec_size - 1] = (target_descriptor);
    }
   

    int row = 8; int col = 8;
    int row_updated;
    int col_updated;
    /*if (division_row >= 256)
    {
        col_updated = division_col;
        row_updated = division_row;
       
    }
    else
    {*/
        col_updated = curr_image_col;
        row_updated = curr_image_row;
    //}
    std::vector<Eigen::MatrixXf>vec_matrices;
    float sm = -INFINITY;
    int max_rot_idx;
    std::vector<std::vector<float>>max_secondary_descriptor;
    int updated_rot_idx;// = rotation_idx;
    int shift_up, shift_down;
    int shift_next;
    int dir;
    std::vector <std::vector<float>>tg = target_descriptor;

    if (division_row > 32)
    {
        ////////////sanity check/////////////////////////////
        if (false)
        {
            Eigen::MatrixXf mat_new = tool::CreateMatrixFromStlVector(src_descriptor, row_updated, col_updated);
            std::pair<int, float> val = ComputeRotationIndex(src_descriptor, tg, row_updated, col_updated);
            Eigen::MatrixXf out_exhaustive = tool::RowShiftMatrix(mat_new, val.first);
            std::vector<std::vector<float>>mat_vec_exhaustive(row_updated, std::vector<float>(row_updated));
            tool::Create2DStlVectorFromMatrix(out_exhaustive, mat_vec_exhaustive, row_updated, row_updated);
            max_descriptor = std::move(mat_vec_exhaustive);
            rotation_idx = val.first;
            return val.second;
        }
        //////////////////////////////////////////////////
       // rotation_idx = updated_rot_idx;
        max_descriptor = src_descriptor;
        Eigen::MatrixXf mat = tool::CreateMatrixFromStlVector(src_descriptor, row_updated, col_updated);
        updated_rot_idx =  rotation_idx;
       
        Eigen::MatrixXf mat_curr = tool::RowShiftMatrix(mat, rotation_idx);
        std::vector<std::vector<float>>mat_vec_curr(row_updated, std::vector<float>(col_updated));
        tool::Create2DStlVectorFromMatrix(mat_curr, mat_vec_curr, row_updated, col_updated);
        float  similarity_value_curr = ComputeMeasureOfSimilarity(mat_vec_curr, tg);


        Eigen::MatrixXf out_matrix_prev = tool::RowShiftMatrix(mat, modulo(rotation_idx - 1, row_updated));
        std::vector<std::vector<float>>mat_vec_prev(row_updated, std::vector<float>(col_updated));
        tool::Create2DStlVectorFromMatrix(out_matrix_prev, mat_vec_prev, row_updated, col_updated);
        float  similarity_value_A = ComputeMeasureOfSimilarity(mat_vec_prev, tg);

        Eigen::MatrixXf out_matrix_next = tool::RowShiftMatrix(mat, modulo(rotation_idx + 1, row_updated));
        std::vector<std::vector<float>>mat_vec_next(row_updated, std::vector<float>(col_updated));
        tool::Create2DStlVectorFromMatrix(out_matrix_next, mat_vec_next, row_updated, col_updated);
        float similarity_value_B = ComputeMeasureOfSimilarity(mat_vec_next, tg);

        if (similarity_value_A <= similarity_value_curr && similarity_value_curr >= similarity_value_B)
        {
            sm = similarity_value_curr;
            max_descriptor = std::move(mat_vec_curr);
 /*           if (modulo(rotation_idx + 1, row_updated) == 0 && similarity_value_curr == similarity_value_B)
            {
                rotation_idx = 0;
                max_descriptor = std::move(mat_vec_next);
            }
            else if (modulo(rotation_idx - 1, row_updated) == 0 && similarity_value_curr == similarity_value_A)
            {
                rotation_idx = 0;
                max_descriptor = std::move(mat_vec_prev);
            }*/
           // std::cout << "current rot_idx:" << rotation_idx << std::endl;
          //  rotation_idx = current_rot_idx;
            return sm;
        }
     /*   if (updated_rot_idx == 0)
        {
            shift_next = ((updated_rot_idx + dir) % division_row + division_row) % division_row;
            shift_up = (updated_rot_idx - 1) % division_row + division_row;
            shift_down = updated_rot_idx + 1;
        }
        else if (updated_rot_idx >= division_row - 1)
        {
            shift_up = updated_rot_idx - 1;
            shift_down = (updated_rot_idx + 1) % division_row;
        }
        else
        {
            shift_down = updated_rot_idx + 1;
            shift_up = updated_rot_idx - 1;
        }
    
         */
        bool status_up = true;
        bool status_down = true;
        int itr = 0;
        if (similarity_value_A > similarity_value_B)
        {
            dir = -1;
           
        }
        else
        {
            dir = 1;
            similarity_value_A = similarity_value_B;
            mat_vec_prev = std::move(mat_vec_next);
        }
        shift_next = rotation_idx +  dir;
        while (true)
        {
           
            Eigen::MatrixXf out_matrix = tool::RowShiftMatrix(mat, modulo(shift_next + dir, row_updated));

            tool::Create2DStlVectorFromMatrix(out_matrix, mat_vec_curr, row_updated, col_updated);
            similarity_value_curr = ComputeMeasureOfSimilarity(mat_vec_curr, tg);
           
            if (similarity_value_curr < similarity_value_A)
            {
                break;
            }
            else if (similarity_value_curr == similarity_value_A )
            {
                if (modulo(shift_next + dir, row_updated) != 0)
                    break;
            }
            
           
            similarity_value_A = similarity_value_curr;
            mat_vec_prev = std::move(mat_vec_curr);
            shift_next = shift_next + dir;
         
        }
        rotation_idx = modulo(shift_next, row_updated); // shift_next - dir
    
      //  std::cout << "current rot_idx:" << rotation_idx << std::endl;
        max_descriptor = std::move(mat_vec_prev);
        sm = similarity_value_A;
        return sm;
    }
  /*  for (int i = 0 ; i < src_descriptors.size(); i++)
    {*/
       // std::cout << "current size:" << src_descriptors.size() << std::endl;

        //Eigen::MatrixXf mat = tool::CreateMatrixFromStlVector(src_descriptors[0], row, col);
        ////get the lowest resolution descriptor .i.e, currenlty 8x8
        ///*if (i == 0)
        //{*/
        //    // coarse resolution similarity
        //    for (int itr = 0; itr < min_res; itr++)
        //    {
        //        Eigen::MatrixXf out_matrix = tool::RowShiftMatrix(mat, itr);
        //        std::vector<std::vector<float>>mat_vec(row, std::vector<float>(col));
        //       tool:: Create2DStlVectorFromMatrix(out_matrix, mat_vec, row, col);
        //     
        //        float new_sm = ComputeMeasureOfSimilarity(mat_vec, tar_descriptors[0]);
        //      
        //        if (new_sm > sm)
        //        {
        //            sm = new_sm;
        //            max_rot_idx = itr;
        //        }

        //    }
       /* }
        else
        {*/
            ////////////earlier technique/////////////////////////////////
            int nr = 1;
            //for (int it = 2 * min_res; it <= division_row; it *= 2) //division_row
            //{
            //    row *= 2;
            //    col *= 2;
            //   // std::cout << "current it:" << it << std::endl;
            //    Eigen::MatrixXf mat = tool::CreateMatrixFromStlVector(src_descriptors[nr], row, col);

            //    int new_rot_idx = 2 * max_rot_idx;
            //    Eigen::MatrixXf out_matrix = tool::RowShiftMatrix(mat, new_rot_idx);
            //    std::vector<std::vector<float>>mat_vec(row, std::vector<float>(col));
            //   tool:: Create2DStlVectorFromMatrix(out_matrix, mat_vec, row, col);

            //    int new_rot_idx_next = 2 * max_rot_idx + 1;
            //    Eigen::MatrixXf out_next = tool::RowShiftMatrix(mat, new_rot_idx_next);
            //    std::vector<std::vector<float>>mat_vec_next(row, std::vector<float>(col));
            //    tool::Create2DStlVectorFromMatrix(out_next, mat_vec_next, row, col);

            //    float similarity_value_A = ComputeMeasureOfSimilarity(mat_vec, tar_descriptors[nr]);
            //    float similarity_value_B = ComputeMeasureOfSimilarity(mat_vec_next, tar_descriptors[nr]);
            //    if(similarity_value_A > similarity_value_B)
            //    {
            //        sm = similarity_value_A;
            //        max_rot_idx = new_rot_idx;
            //        max_secondary_descriptor = std::move(mat_vec);
            //    }
            //    else if (similarity_value_B > similarity_value_A)
            //    {
            //        sm = similarity_value_B;
            //        max_rot_idx = new_rot_idx_next;
            //        max_secondary_descriptor = std::move(mat_vec_next);
            //    }
            //    else if (similarity_value_A == similarity_value_B)
            //    {
            //        int new_rot_idx_prev = 2 * max_rot_idx - 1;
            //        Eigen::MatrixXf out_prev = tool::RowShiftMatrix(mat, new_rot_idx_prev);
            //        std::vector<std::vector<float>>mat_vec_prev(row, std::vector<float>(col));
            //        tool::Create2DStlVectorFromMatrix(out_prev, mat_vec_prev, row, col);
            //        float similarity_value_C = ComputeMeasureOfSimilarity(mat_vec_prev, tar_descriptors[nr]);

            //        if (similarity_value_B > similarity_value_C)
            //        {
            //            sm = similarity_value_B;
            //            max_rot_idx = new_rot_idx_next;
            //            max_secondary_descriptor = std::move(mat_vec_next);
            //        }
            //        else
            //        {
            //            sm = similarity_value_A;
            //            max_rot_idx = new_rot_idx;
            //            max_secondary_descriptor = std::move(mat_vec);
            //        }
            // 
            //    }
            //    nr++;
            // 
            //}
        //}
        //row *= 2;
        //if (true) //curr_image_row <= 128
        //    col *= 2;
   // }
    auto finishItr = std::chrono::high_resolution_clock::now();
    double executeTime = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
    executeTime = executeTime / double(1000);
    // std::cout << "Time for shift:" << executeTime << "sec" << std::endl;
 ////////////////////earlier techniques end here////////////////////////////
  // exhaustive search ouput
    int curr_row = src_descriptors[2].size();
    int curr_col = src_descriptors[2][0].size();
    Eigen::MatrixXf mat_32 = tool::CreateMatrixFromStlVector(src_descriptors[2], curr_row, curr_col);
    std::pair<int, float> val = ComputeRotationIndex(src_descriptors[2], tar_descriptors[2], curr_row, curr_col);
  Eigen::MatrixXf out_exhaustive = tool::RowShiftMatrix(mat_32, val.first);
  std::vector<std::vector<float>>mat_vec_exhaustive(row, std::vector<float>(curr_col));
  tool::Create2DStlVectorFromMatrix(out_exhaustive, mat_vec_exhaustive, curr_row, curr_col);
  rotation_idx = val.first; // max_rot_idx;
   /* std::cout << "rotation computed by hierarachy:" << max_rot_idx << std::endl;
    std::cout << "rotation computed by exhaustive search:" << rotation_idx << std::endl;*/
    max_descriptor = std::move(mat_vec_exhaustive);
    sm = val.second;
    ////////////////end exhaustive/////////////////////////
    //max_descriptor = std::move(max_secondary_descriptor);
  
    return sm;
}
//float CirconCorrespondence::EvaluateSimilarityByRotationShift(const std::vector<std::vector<float>> &src_descriptors, 
//    const std::vector<std::vector<float>> &target_descriptors,int &rotation_idx, std::vector<std::vector<float>> &max_descriptor, int min_res)
//{
//    //get the lowest resolution descriptor .i.e, currenlty 8x8
//    std::vector<float>coarse_src_descriptor = src_descriptors[0];
//    std::vector<float>coarse_tar_descriptor = target_descriptors[0];
//    std::vector<Eigen::MatrixXf>vec_matrices;
//    int row = 8; int col = 8;
//    for (auto const& src_desc : src_descriptors)
//    {
//        Eigen::MatrixXf mat = tool::CreateMatrixFromStlVector(src_desc, row, col);
//        vec_matrices.push_back(mat);
//        row *= 2;
//        col *= 2;
//     }
//
//   
//    float sm = -INFINITY;
//    int max_rot_idx;
//    // coarse resolution similarity
//    for (int i = 0; i <  min_res; i++)
//    {
//    
//      std::vector<float> coarse_res_out = RotateImageByIndexNumber(vec_matrices[0], i);
//      float new_sm = ComputeMeasureOfSimilarity(coarse_res_out, target_descriptors[0]);
//      if (new_sm > sm)
//      {
//          sm = new_sm;
//          max_rot_idx = i;
//      }
//
//    }
//    int it;
//        // iterate through all the resolution
//    int nr = 1;
//    std::vector<float>max_secondary_descriptor;
//    for (int it = 2 * min_res; it <= division_row; it *= 2)
//    {
//
//        // test the similarity for current and next row shift by doubling it in the next resolution
//        int new_rot_idx = 2 * max_rot_idx;
//        Eigen::MatrixXf out = tool::RowShiftMatrix(vec_matrices[nr], new_rot_idx);
//        std::vector<float> coarse_res_out = tool::CreateStlVectorFromMatirx(out);
//
//        int new_rot_idx_next = 2 * max_rot_idx + 1;
//        Eigen::MatrixXf out_next = tool::RowShiftMatrix(vec_matrices[nr], new_rot_idx_next);
//        std::vector<float> coarse_res_out_next = tool::CreateStlVectorFromMatirx(out_next);
//
//        float similarity_value_A = ComputeMeasureOfSimilarity(coarse_res_out, target_descriptors[nr]);
//        float similarity_value_B = ComputeMeasureOfSimilarity(coarse_res_out_next, target_descriptors[nr]);
//        if (similarity_value_A > similarity_value_B)
//        {
//            sm = similarity_value_A;
//            max_rot_idx = new_rot_idx;
//            max_secondary_descriptor = coarse_res_out;
//        }
//        else
//        {
//            sm = similarity_value_B;
//            max_rot_idx = new_rot_idx_next;
//            max_secondary_descriptor = coarse_res_out_next;
//        }
//        nr++;
//    }
//    rotation_idx = max_rot_idx;
//    max_descriptor = max_secondary_descriptor;
//    return sm;
//}
float CirconCorrespondence::ComputeMaximumRotationShiftParameter(const std::vector<std::vector<float>> &secondary_descriptor,
    const std::vector<std::vector<float>> &target_point_descriptor, std::vector<std::vector<float>> &max_shift_descriptor,
    std::map<int, size_t>&image_point_index_map_current,std::vector<std::vector<_dV>> &descriptor_content,
    int &rotation_idx, int& secondary_dsc_posn)
{
    // first rescale source and target descriptors to coarser level starting 8x8
  
   float max_similarity = ReScaleAndEvaluateDescriptorForMaximumRotation(secondary_descriptor, target_point_descriptor,
       rotation_idx, max_shift_descriptor);

   int up_row_idx = secondary_dsc_posn / division_col;
   int up_col_idx = secondary_dsc_posn %division_col;
   secondary_dsc_posn = ((up_row_idx + rotation_idx) % division_row) * division_col + up_col_idx;
 
   _dV dv;
   std::vector<std::vector<_dV>> refined_descriptor_content;
   int shortened_row;
   if (false) // division_row >= 256 || high_flag
   {
       shortened_row = 32;
       refined_descriptor_content.resize(shortened_row, std::vector<_dV>(shortened_row));
   }
   else
   {
       shortened_row = division_row;
       refined_descriptor_content.resize(division_row, std::vector<_dV>(division_col));
   }

 
  /* std::map<int, bool>status;
   for (int i = 0; i < division_row; i++)
   {
       status[i] = false;
   }*/
  
   for (int row_idx = 0; row_idx < shortened_row; row_idx++)
   {
       int new_row_idx = (row_idx + rotation_idx) % shortened_row;  // division_row
       if (new_row_idx < 0 || row_idx < 0)
       {
           std::cout << rotation_idx << std::endl;
           std::cout << "wrong data" << std::endl;
       }
       else
           refined_descriptor_content[new_row_idx] = std::move(descriptor_content[row_idx]);
       //if (!status[row_idx] && !status[new_row_idx])
       //{
       //    
       // /*   std::vector<_dV>temp = descriptor_content[row_idx];
       //    descriptor_content[row_idx] = descriptor_content[new_row_idx];
       //    descriptor_content[new_row_idx] = temp;
       //    status[row_idx] = true;
       //    status[new_row_idx] = true;*/
       //}

   }

   
   descriptor_content = std::move(refined_descriptor_content);
   return max_similarity;
}

void CirconCorrespondence::SampleDataUniformly(const CloudWithoutType &pc, 
    const float &gSize, const std::string &OutPutFileName)
{
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*pc, *pTarget);
    CUniformGrid3D grid3D(gSize,
        [&pTarget](Eigen::Vector3d *out_pnt/*, double *out_attrib*/, size_t idx) -> bool
    {
        if (idx < pTarget->points.size())
        {
            Eigen::Vector3d vec(pTarget->points[idx].x, pTarget->points[idx].y, pTarget->points[idx].z);
            *out_pnt = vec;
           // *out_attrib = pTarget->points[idx].z;
            return true;
        }
        else
            return false;
    }
    );
  auto voxel_centers =  grid3D.Compute();
  CUniformGrid3D::GetSampledCloud(pc, voxel_centers, OutPutFileName);
}