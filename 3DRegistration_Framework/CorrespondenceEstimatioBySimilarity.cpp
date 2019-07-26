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

    CirconCorrespondence::Measure FindMaximumSimilarityInIterations(const std::vector <CirconCorrespondence::Measure>& m)
    {

        float value = -INFINITY;
        CirconCorrespondence::Measure max_similarity;
        for (auto const& ms : m)
        {
            if (ms.similarity_value > value)
            {
                value = ms.similarity_value;
                max_similarity = ms;
            }
        }
        return max_similarity;
    }
    std::vector<float> ScaleImageData(std::vector<float>data)
    {
        int max_value_image, min_value_image;
        float  min_value = INFINITY;
        std::vector<float>scaled_data;
        if (data.size() > 0)
        {
            max_value_image = *(std::max_element(data.begin(), data.end()));


            for (int itr = 0; itr < data.size(); itr++)
            {
                if (data[itr] != -INFINITY)
                {

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
            //send the data after scaling it in the range of  0-255

            scaled_data.reserve(data.size());
            float input_range = max_value_image - min_value_image;
            float output_range = 255.0 - 0.0;
            for (int i = 0; i < data.size(); i++)
            {
                float output = std::roundf((data[i] - min_value_image) * output_range / input_range + 0.0);
                scaled_data.push_back(output);
            }
        }
        else
        {
            std::runtime_error("no image data to write\n");
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
    std::vector<PointNormalType>feature_points;
    int tIndex, sIndex;
    CloudWithNormalPtr srx = cid_source.GetoriginalCloud();
    CloudWithNormalPtr tgx = cid_target.GetoriginalCloud();
    Eigen::Vector3f min_pt;
    Eigen::Vector3f max_pt;
    float diag_length1 = tool::ComputeOrientedBoundingBoxOfCloud(srx, min_pt, max_pt);
    float diag_length2 = tool::ComputeOrientedBoundingBoxOfCloud(tgx, min_pt, max_pt);
    if (diag_length1 > diag_length2)
        maximum_radius = diag_length1;
    else
        maximum_radius = diag_length2;
    float avg_dst_sample1 = metric::EstimatePointAvgDistance(input_source);
    float avg_dst_sample2 = metric::EstimatePointAvgDistance(input_target);
    float avg_dist;
    if (avg_dst_sample1 > avg_dst_sample2)
        avg_dist = avg_dst_sample1;
    else
        avg_dist = avg_dst_sample2;

    pcl::KdTreeFLANN<PointNormalType>::Ptr kdt_src(new pcl::KdTreeFLANN < PointNormalType>);
    kdt_src->setInputCloud(srx);
    pcl::KdTreeFLANN<PointNormalType>::Ptr kdt_tgt(new pcl::KdTreeFLANN < PointNormalType>);
    kdt_tgt->setInputCloud(tgx);
    std::string filePath = "Z:/staff/SDutta/GlobalRegistration/";
    std::unique_ptr<ON_NurbsSurface> nbs = cid_target.GetNurbsSurface();
    std::unique_ptr<ON_NurbsCurve> ncs = cid_target.GetNurbsCurve();
    std::unique_ptr<ON_NurbsSurface> nbs_src = cid_source.GetNurbsSurface();
    std::unique_ptr<ON_NurbsCurve> ncs_src = cid_source.GetNurbsCurve();
    std::vector<Eigen::Vector2d> std_params;
    for (tIndex = 66; tIndex < 73; tIndex++)
    {
        // find closet point on nurb surface for a give pt. of interest
        tar_corres = corrs_target->at(tIndex);
        Eigen::VectorXd pt_target = Eigen::VectorXd::Zero(6);
        pt_target.head<3>() = tar_corres.getVector3fMap().cast<double>();
        pt_target.tail<3>() = tar_corres.getNormalVector3fMap().cast<double>();
        Eigen::VectorXd cp_target = surface::cNurbsSurface::ComputeClosetPointOnNurbSurface(pt_target, *nbs, *ncs, 1e-8);

   
        // 
        tar_corres.getVector3fMap() = cp_target.head<3>().cast<float>(); // corrs_source->at(sIndex);
        tar_corres.getNormalVector3fMap() = cp_target.tail<3>().cast<float>();
        target_basic_index = original_target_index[tIndex];
        cid_target.SetBasicPointIndex(target_basic_index);
        int start = 0;
        non_valid_index.clear();  //recently added

    /*  Eigen::Matrix4f local_tfs_target =  CirconImageDescriptor::ConstructLocalCoordinateAxes(cid_target, tar_corres);
      cid_target.SetLocalFrame(local_tfs_target);
      std::unique_ptr<ON_NurbsSurface> nb_surface_tfs = surface::cNurbsSurface::TransformControlPointsOfNurbsSurface(*nbs,
          local_tfs_target.cast<double>());
      cid_target.SetNurbsSurfaceAndCurve(*nb_surface_tfs);*/

        for (sIndex = 75; sIndex < 82; sIndex++)
        {
            // find closet point on nurb surface for a give pt. of interest
    
            src_corres = corrs_source->at(sIndex);
            Eigen::VectorXd pt_source = Eigen::VectorXd::Zero(6);
            pt_source.head<3>() = src_corres.getVector3fMap().cast<double>();
            pt_source.tail<3>() = src_corres.getNormalVector3fMap().cast<double>();
            Eigen::VectorXd cp_source = surface::cNurbsSurface::ComputeClosetPointOnNurbSurface(pt_source, *nbs_src, *ncs_src, 1e-8);

            float dp = (src_corres.getVector3fMap() - cp_source.head<3>().cast<float>()).dot(cp_source.tail<3>().cast<float>());
            if (dp == 1.0f || dp == -1.0f)
            {
                std::cout << "closest points correct" << std::endl;
            }
            else
            {
                std::cout << "dp:" << dp << std::endl;
            }
            int incr_reslolution = 0;
            int level_itr = 0;
            src_corres.getVector3fMap() = cp_source.head<3>().cast<float>(); // corrs_source->at(sIndex);
            src_corres.getNormalVector3fMap() = cp_source.tail<3>().cast<float>();
            source_basic_index = original_source_index[sIndex];
    
            /*Eigen::Matrix4f local_tfs_source = CirconImageDescriptor::ConstructLocalCoordinateAxes(cid_source, src_corres);
            cid_source.SetLocalFrame(local_tfs_source);
            std::unique_ptr<ON_NurbsSurface> nb_surface_tfs_source = surface::cNurbsSurface::TransformControlPointsOfNurbsSurface(*nbs_src,
                local_tfs_source.cast<double>());
            cid_source.SetNurbsSurfaceAndCurve(*nb_surface_tfs_source);*/

            while (level_itr != num_resolution)
            {
                // prepare source descriptor
                cid_source.SetBasicPointIndex(source_basic_index);
              
                PrepareDescriptor(cid_source, src_corres);
                source_descriptor_image = cid_source.GetDescriptorImage();

                PrepareDescriptor(cid_target, tar_corres);
                target_descriptor_image = cid_target.GetDescriptorImage();
                Epsilon = cid_source.GetRadialResolution(); /// 16.0;

                if (_write)
                {
                    std::string filePath = "Z:/staff/SDutta/GlobalRegistration/";
                    char subfile[100], subImg[100], subImgTar[100], subTarFile[100];
                    sprintf(subfile, "src_descriptor_%d_%d", tIndex, source_basic_index);
                    sprintf(subImg, "src_img_%d_%d.bmp", tIndex, source_basic_index);
                    sprintf(subImgTar, "tar_img_%d_%d.bmp", tIndex, target_basic_index);
                    std::string ImageName = filePath + subImg;
                    cid_source.WriteDescriptorAsImage(ImageName);
                    std::string CloudName = filePath + subfile;
                    cid_source.ReconstructPointCloud();
                    cid_source.WritePointCloud(CloudName);

                    std::string TarImageName = filePath + subImgTar;
                    cid_target.WriteDescriptorAsImage(TarImageName);
                    sprintf(subTarFile, "tar_descriptor_%d", target_basic_index);
                    std::string TarCloudName = filePath + subTarFile;
                    cid_target.ReconstructPointCloud();
                    cid_target.WritePointCloud(TarCloudName);
                }


                Measure ms = CompareDescriptor(source_descriptor_image, target_descriptor_image,1); //CompareDescriptorWithSimilarityMeasure

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
                    float angle = cid_source.GetAngularResolution() * ms_max.rotation_index;
                    //  std::cout << "rotation_angle:" << angle << endl;
                    T2_T1.row(0) = Eigen::Vector4f(cos(angle), sin(angle), 0, 0);
                    T2_T1.row(1) = Eigen::Vector4f(-sin(angle), cos(angle), 0, 0);
                    T2_T1.row(2) = Eigen::Vector4f(0, 0, 1, 0);

                    /*  int i_idx = ms_max.cell_index / division_col;
                      int j_idx = ms_max.cell_index % division_col;
                      float v0 = (j_idx * cid_source.GetRadialResolution()) * cos(-(i_idx - 1)* theta);
                      float v1 = (j_idx * cid_source.GetRadialResolution()) * sin(-(i_idx - 1)* theta);
                      float v2 = ms_max.pix_value * cid_source.GetHeightResolution();
                      T2_T1.col(3) = T2_T1 * Eigen::Vector4f(v0, v1, v2,1.0);
                      Eigen::Vector4f trans = -(T2_T1 * ms_max.point_of_interest.getVector4fMap());
                      T2_T1.col(3) = T2_T1 * trans;*/

                    src_corres_descriptor = cid_source;
                    // feature_points.push_back(ms.point_of_interest);

                }
                if (level_itr == num_resolution - 1)
                {
                    //find transformation and check stopping criterion
                    auto startItr = std::chrono::high_resolution_clock::now();
                    /*  float theta = cid_source.GetAngularResolution();
                      float angle = cid_source.GetAngularResolution() * ms_max.rotation_index * M_PI / 180.0f;
                      T2_T1.row(0) = Eigen::Vector4f(cos(angle), sin(angle), 0, 0);
                      T2_T1.row(1) = Eigen::Vector4f(-sin(angle), cos(angle), 0, 0);
                      T2_T1.row(2) = Eigen::Vector4f(0,0, 1, 0);*/
                    Final_Transformation = T1.inverse()* T2_T1 *  T2;
                    //  std::cout << T1.inverse() << "\n" << T2_T1 << "\n" <<  T2 << std::endl;
                    std::cout << "src_corres:" << ms_max.point_index << "," << "tgt corres:" << target_basic_index << std::endl;
                    std::cout << "Final_Transformation:" << "\n" << Final_Transformation << std::endl;
                    Eigen::Vector3f tar_norm = cid_target.GetRotationAXisPoint().getNormalVector3fMap();
                    Eigen::Vector3f src_norm = ms_max.point_of_interest.getNormalVector3fMap();


                    /*std::map<int, size_t> map_tgx = cid_target.GetImagePointMap();
                    int cell_index = cid_target.GetDescriptorCellIndexForInterestPoint();
                    std::vector<std::pair<int, float>>src1 = ComputeFictitiousCorrespondence(ms_max.img_pt_index_map, srx, ms_max.cell_index, ms_max.point_of_interest);
                    std::vector<std::pair<int, float>>tar1 = ComputeFictitiousCorrespondence(map_tgx, tgx, cell_index, tar_corres);*/
                    std::pair<int, int> index_pair;
                    std::pair<PointNormalType, PointNormalType>pt_corres = ComputeFictituousCorrespondencePair(ms_max.point_of_interest, *kdt_src, *kdt_tgt, avg_dist,
                        Final_Transformation, index_pair);
                    /*  if (src1.size() == 0 || tar1.size() == 0)
                      {
                          reset = true;
                      }
                      else
                      {*/
                    int col_indx = ms_max.cell_index%division_col;
                    // std::pair<float, float>error = ComputeStoppingCriteria(src1, tar1, ms_max.point_of_interest, tar_corres, T2_T1, Final_Transformation);
                    if (index_pair.first == -1 || index_pair.second == -1)
                    {
                        reset = true;
                    }
                    else
                    {
                        std::pair<float, float>error = EstimateStopParameter(pt_corres, index_pair, T2_T1, Final_Transformation);
#ifdef LOGDATA
                        error_log("cell index at max resolution = %d\n", ms_max.cell_index);
                        error_log("Column index at max resolution = %d\n", col_indx);
                        error_log(" rotation error = %f, translation error = %f\n", error.first, error.second);
#endif


                        if (error.first < 0.031 && error.second < 0.03 * maximum_radius)//tar_norm.dot(src_norm) >= 0.9) // 0.025
                        {
                            Eigen::Affine3f mat(Final_Transformation);
                            PointNormalType tfs_src_corres = pcl::transformPointWithNormal(ms_max.point_of_interest, mat);
                            feature_points.push_back(tfs_src_corres);
                            feature_points.push_back(tar_corres);
                            tc_corres = tIndex;
                            sc_corres = sIndex;

                            CirconImageDescriptor circ_best(input_source, cid_source.GetRowDivision(), cid_source.GetColumnDivision(),
                                cid_source.GetHeightDivision());
                            circ_best.SetBasicPointIndex(ms_max.point_index);
                            PrepareDescriptor(circ_best, ms_max.point_of_interest);
                            if (_write)
                            {

                                std::string ImageName = filePath + "best.bmp";
                                std::string CloudName = filePath + "best_cloud";
                                circ_best.WriteDescriptorAsImage(ImageName);
                                circ_best.ReconstructPointCloud();
                                circ_best.WritePointCloud(CloudName);
                                std::cout << "Target Index:" << tc_corres << "," << "source_index:" << sc_corres << std::endl;
                                /* std::string fileName = "Z:/staff/SDutta/GlobalRegistration/column_half.ply";
                                 WritePointCloud(feature_points, fileName);*/
                                std::string CorresName = filePath + "src_corres_ best_found";
                                WritePointCloud(feature_points, CorresName);
                            }
                            feature_points.clear();
                            return Final_Transformation;
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
                    src_corres = ms.point_of_interest;
                    source_basic_index = ms.point_index;
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
                    if (_write)
                    {
                        char subfile[100];
                        sprintf(subfile, "src_corres_%d_%d", tIndex, sIndex);
                        std::string CorresName = filePath + subfile;
                        WritePointCloud(feature_points, CorresName);
                    }
                    feature_points.clear();
                    break;
                }


            }

        }


    }
   // std::cout << "Target Index:" << tc_corres << "," << "source_index:" << sc_corres << std::endl;
    std::string fileName = "Z:/staff/SDutta/GlobalRegistration/column_half";
    WritePointCloud(feature_points, fileName);
    Final_Transformation = T1.inverse()* T2_T1 * T2;
    std::cout << "Target Index:" << tIndex << "," << "source_index:" << sIndex << std::endl;
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
void CirconCorrespondence::ComputeSimilarityBetweenTwoCloud( std::string dirname, std::string OutputFile)
{
    CloudWithNormalPtr srx = cid_source.GetoriginalCloud();
    CloudWithNormalPtr tgx = cid_target.GetoriginalCloud();
    int tIndex, sIndex;
    std::string  fileNameWithLocation = OutputFile + ".txt";
    FILE *pFile;
    pFile = fopen(fileNameWithLocation.c_str(), "wb");
    fprintf(pFile, "src\ttar\tms\trotation_index\n");
    char subsrcfile[100], subtarfile[100], subImg[100], subImgTar[100];
    float max_sim = 0;
    
    std::string  fileWithLocation = OutputFile + "_Max_Similarity" + ".txt";
    FILE *nFile;
    nFile = fopen(fileWithLocation.c_str(), "wb");
    fprintf(nFile, "src\ttar\tms\trotation_index\n");
    Eigen::Vector3f min_pt; 
    Eigen::Vector3f max_pt;
    std::vector<int> SearchResults;
    std::vector<float> SearchDistances;
   float diag_length = tool::ComputeOrientedBoundingBoxOfCloud(srx, min_pt, max_pt);
   pcl::KdTreeFLANN<PointNormalType> KdTree;
   KdTree.setInputCloud(srx);
   PointNormalType query_point = srx->at(23958);
   KdTree.radiusSearch(query_point, 0.1 * diag_length, SearchResults, SearchDistances);
  // SetParameterForSimilarityMeasure(0.11, 0.01);
    for (tIndex = 7078; tIndex < 7079; ++tIndex)
    {
        int rotation_idx, max_rotation_idx = 0;
        float max_sim = 0;
        int max_src_index = -1, max_tgt_index = -1;
        cid_target.SetBasicPointIndex(tIndex);
        PointNormalType pt_tgt = tgx->at(tIndex);
        PrepareDescriptor(cid_target, pt_tgt);
       /* sprintf(subImgTar, "tar_img_%d.bmp", tIndex);
        std::string TarImageName = dirname + subImgTar;
        cid_target.WriteDescriptorAsImage(TarImageName);

        sprintf(subtarfile, "tar_recons_%d", tIndex);
        std::string TarCloudName = dirname + subtarfile;
        cid_target.ReconstructPointCloud();
        cid_target.WritePointCloud(TarCloudName);*/

        target_descriptor_image = cid_target.GetDescriptorImage();
        for (sIndex = 0; sIndex < SearchResults.size(); ++sIndex)
        {
            cid_source.SetBasicPointIndex(SearchResults[sIndex]);
            PointNormalType pt_src = srx->at(SearchResults[sIndex]);
            PrepareDescriptor(cid_source, pt_src);
           /* sprintf(subImg, "src_img_%d.bmp", sIndex);
            std::string SrcImageName = dirname + subImg;
            sprintf(subsrcfile, "src_recons_%d", sIndex);
            std::string SrcCloudName = dirname + subsrcfile;
            if (iospace::ExistsFile(SrcImageName) == false)
            {
                cid_source.WriteDescriptorAsImage(SrcImageName);
                cid_source.ReconstructPointCloud();
                cid_source.WritePointCloud(SrcCloudName);
            }*/
            source_descriptor_image = cid_source.GetDescriptorImage();
            float ms = ComputeMeasureOfSimilarity(source_descriptor_image, target_descriptor_image);
            std::vector<float>max_shift_descriptor;
            std::map<int, size_t>point_img_map = cid_source.GetImagePointMap();
            int idx = cid_source.GetDescriptorCellIndexForInterestPoint();
            ms = ComputeRotationIndexFromShift(source_descriptor_image, target_descriptor_image, max_shift_descriptor, rotation_idx, point_img_map, idx);

            fprintf(pFile, "%d\t%d\t%f\t%d\r\n", SearchResults[sIndex], tIndex, ms, rotation_idx);
            if (ms > max_sim)
            {
                max_sim = ms;
                max_src_index = SearchResults[sIndex];
                max_tgt_index = tIndex;
                max_rotation_idx = rotation_idx;
            }
            cid_source.UpdateImageDimension(division_col, division_row);
        }
        fprintf(nFile, "%d\t%d\t%f\t%d\r\n", max_src_index, max_tgt_index, max_sim, max_rotation_idx);
        fflush(nFile);
        fflush(pFile);
        cid_target.UpdateImageDimension(division_col, division_row);
    }
    fclose(pFile);
    fclose(nFile);

}
std::vector<std::pair<int, float>>CirconCorrespondence::ComputeFictitiousCorrespondence(std::map<int, size_t>img_point_map,
    CloudWithNormalPtr cloud, int corres_1_source_idx, PointNormalType corres_point)
{
    std::vector<std::pair<int, float>>neighborhood_dot_products;
   // std::vector<int>neighbor_point_idx;
    // collect 8-neighbor around source correspondence
    int i_index = corres_1_source_idx / division_col;
    int j_index = corres_1_source_idx % division_col;
   /* int nr_column = nr_search_col;
    while (nr_column != 0)
    {*/
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
    /*    nr_column--;
        j_index = j_index + 1;
    }*/
 
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

void CirconCorrespondence ::PrepareDescriptor(CirconImageDescriptor& cid, PointNormalType rotpoint)
{
    if (basic_descriptor)
    {
        cid.ComputeBasicPointOfInterest();  // considers closest point to centroid as the point around which descriptor is built
    }
    else
        cid.SetRotationAxisPoint(rotpoint);
    cid.ConstructLocalFrameOfReference();  
    auto startItr = std::chrono::high_resolution_clock::now();
    CloudWithoutType transformed_cloud = cid.TransformPointToLocalFrame();
    cid.SetMaxSearchColumn(nr_search_col);
    float max_radius = maximum_radius;// cid.GetRadiusFromCloud();// ComputeMaximumRadius(transformed_cloud);
    float height = cid.ComputeheightFromPointCloud(transformed_cloud);
    cid.SetImageDescriptorResolution(FULL_ANGLE, max_radius, height);
    auto finishItr = std::chrono::high_resolution_clock::now();
    double executeTime = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
    executeTime = executeTime / double(1000);
   // std::cout << "Time consumed for rotation shift similarity:" << executeTime << "sec" << std::endl;
    /*  points.reserve(size);
    for (size_t i = 0; i < size; i++)
    {
    points.push_back(pTarget->points[i]);
    }*/
    auto startopto = std::chrono::high_resolution_clock::now();
    cid.ComputeFeature();
    auto endopto = std::chrono::high_resolution_clock::now();
    double executeopto = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(endopto - startopto).count();
    executeopto = executeopto / double(1000);
    std::cout << " Descriptor time:" << executeopto <<  std::endl;
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

    float max_radius = cid_target.ComputeMaximumRadius(transformed_cloud);
    float height = cid_target.ComputeheightFromPointCloud(transformed_cloud);
    cid_target.SetImageDescriptorResolution(FULL_ANGLE, max_radius, height);
    cid_target.ComputeFeature();
    target_descriptor_image = cid_target.GetDescriptorImage();
}
float CirconCorrespondence::ComputeMeasureOfSimilarity(const std::vector<float>& src_image, const std::vector<float>& tar_image)
{
    float similarity_value = 0.0;
    float overlap_set_sum = 0.0; 
    float union_set_sum = 0.0;
    float sigma_ov = 1.0;
    float dist_overlap = 0.0f;
   /* if (src_image.size() == tar_image.size())
    {*/
#pragma omp parallel for
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
      
   /* }
    else
    {
        std::runtime_error("source and  target descriptor unequal in size\n");
        return -INFINITY;
    }*/
    dist_overlap = dist_overlap / overlap_set_sum;
    sigma_ov = static_cast<float>(overlap_set_sum) / static_cast<float>(union_set_sum);
    float lambda_dash = rho_ * lambda_;
    similarity_value = (sigma_ov) / ((rho_ * dist_overlap + lambda_dash) + sigma_ov * (1 - lambda_dash));
    return similarity_value;
}

CirconImageDescriptor CirconCorrespondence::TransformCirconDescriptor(int index)
{
   
    CirconImageDescriptor transformed_descriptor = cid_source;
    transformed_descriptor.SetBasicPointIndex(index);
    transformed_descriptor.SetMaximumRadius(maximum_radius);
    transformed_descriptor.UpdateImageDataAfterTransformation(index);
 
    return transformed_descriptor;
}
CirconImageDescriptor CirconCorrespondence::TransformCirconDescriptor(const Eigen::VectorXd &pt)
{

    CirconImageDescriptor transformed_descriptor = cid_source;
    transformed_descriptor.SetBasicPointIndex(-1);
    transformed_descriptor.SetMaximumRadius(maximum_radius);
    transformed_descriptor.CreateSecondaryDescriptor(pt);

    return transformed_descriptor;
}

CirconCorrespondence::Measure CirconCorrespondence::CompareDescriptor(const std::vector<float>& src_image, const std::vector<float>& tar_image,
    int with_nurbs)
{
    float current_epsilon = INFINITY;
    CirconCorrespondence::Measure mes;
    PointNormalType init_point = cid_source.GetRotationAXisPoint();
    // take basic descriptor into a temp. array
    std::vector<float>current_array_of_data = src_image;
    std::vector<float>new_image;
    std::map<int, size_t>image_point_index_map_current;
    std::vector<_dV>desc_content_current;
    int prev_point_idx = -1;
    float init_ratio_nv = 0.0;
    float tow_nv = 0.9;
    int max_cell_index = 0;
    std::vector< CirconCorrespondence::Measure>similarity_structure_list;
    std::map<int, size_t>image_point_index_map = cid_source.GetImagePointMap();
    int non_valid_count = 0;
    std::vector<PointNormalType>previous_pts_list;
    int current_row_idx = 0;
    // set temp index for testing prev index
    prev_point_idx = source_basic_index;

    // calculate initial similarity value 

    new_image = cid_source.GetDescriptorImage();
    std::vector<float>init_max_shift_descriptor;
    int init_rot_idx = 0;
    std::map<int, size_t>init_mage_point_index_map = cid_source.GetImagePointMap();
    int init_cell_idx = cid_source.GetDescriptorCellIndexForInterestPoint();
    std::vector<_dV> descriptor_source = cid_source.GetDescriptorContent();

    // float sm = 0.0f;
    float sm = ComputeMaximumRotationShiftParameter(new_image, tar_image, init_max_shift_descriptor, init_mage_point_index_map, descriptor_source, init_rot_idx,
        init_cell_idx);

    // initialize similarity measure
    mes.similarity_value = sm;
    mes.cell_index = init_cell_idx;//itr.first;
    mes.cell_values = init_max_shift_descriptor;
    mes.rotation_index = init_rot_idx;
    image_point_index_map_current = init_mage_point_index_map;
    mes.img_pt_index_map = image_point_index_map_current;
    mes.point_of_interest = cid_source.GetRotationAXisPoint();
    mes.point_index = cid_source.GetBasicPointIndex();
    mes.WlTransform = cid_source.GetTransformation();
    mes.primary_idx = mes.cell_index % this->division_col;
    new_image = init_max_shift_descriptor;
    image_point_index_map = init_mage_point_index_map;
    mes.similarity_value = sm;
    previous_pts_list.push_back(mes.point_of_interest);
   
    while (current_epsilon >= Epsilon)
    {
        int count = 0;
        std::vector<size_t>temp_non_valid;
        CirconImageDescriptor transformed_source;
        for (auto const& itr : descriptor_source)
        {
           // std::cout << "present index:" << count << std::endl;
            if (itr.col_idx >= 0 && itr.col_idx < nr_search_col)
            {
              
                Eigen::Vector3d point_normal = itr.pt.tail<3>();
                float dp = (point_normal.cast<float>()).dot(Eigen::Vector3f(0.0, 1.0, 0.0));
                if (dp == -1.0f)
                {
                    std::cout << itr.pt.head<3>() << std::endl;
                    continue;

                }
                else if (dp == 1.0f)
                {
                    std::cout << itr.pt.head<3>() << std::endl;
                    continue;
                }

                else
                {

                    transformed_source = TransformCirconDescriptor(itr.pt); //itr
                    new_image = transformed_source.GetDescriptorImage();

                }
            }

            else
                continue;

            std::vector<float>secondary_max_shift_descriptor;
            int secondary_rot_idx;
            std::map<int, size_t>secondary_mage_point_index_map = transformed_source.GetImagePointMap();
            int second_cell_idx = transformed_source.GetDescriptorCellIndexForInterestPoint();
            // check the similarity measure between new descriptor and the target image descriptor and return the similarity with max possible rotation   
            std::vector<_dV>desc_content = transformed_source.GetDescriptorContent(); // TODO Use the update version

            float updated_sm = ComputeMaximumRotationShiftParameter(new_image, tar_image, secondary_max_shift_descriptor,
                secondary_mage_point_index_map, desc_content, secondary_rot_idx, second_cell_idx);

            if (updated_sm > sm)
            {
                sm = updated_sm;
                mes.similarity_value = sm;
               // mes.cell_index = itr.row_idx * division_col + itr.col_idx;// itr.first;//
                mes.cell_values = secondary_max_shift_descriptor;

                mes.rotation_index = secondary_rot_idx;  //  transformed_source.GetRotationIndex()
               /* Eigen::Matrix4f inv_lW = transformed_source.GetTransformation().inverse();*/
                mes.point_of_interest = transformed_source.GetRotationAXisPoint();
               // image_point_index_map_current = std::move(secondary_mage_point_index_map);
               // mes.point_index = itr.second; // FindElementInMap(itr, image_point_index_map); //transformed_source.GetPointIndexFromCloud(itr);
                mes.WlTransform = transformed_source.GetTransformation();
               // mes.img_pt_index_map = image_point_index_map_current;
                mes.primary_idx = itr.row_idx * division_col + itr.col_idx;
               // mes.pix_value = current_array_of_data[itr.first];
                desc_content_current = desc_content;

            }
            else
            {
                non_valid_count++;
               // invalid_pt_list.push_back(itr.second);
                // size_t _idx = transformed_source.GetPointIndexFromCloud(itr);
                // temp_non_valid.push_back(curr_point_idx);
                count++;
                continue;
            }
            count++;
        }
        
        current_array_of_data = mes.cell_values;
        PointNormalType current_point = mes.point_of_interest;
        previous_pts_list.push_back(init_point);
        float dist = DistanceBetweenPoints(previous_pts_list, current_point);// pcl::geometry::distance(init_point, current_point);
        std::cout << dist << std::endl;
        similarity_structure_list.push_back(mes);
        if (dist <= Epsilon)
        {
            mes = FindMaximumSimilarityInIterations(similarity_structure_list);
            break;
        }
        else
        {
            current_epsilon = dist;
            init_point = current_point;
            descriptor_source = desc_content_current;
           // image_point_index_map = image_point_index_map_current;
            // image_point_index_map = SortMapElementsUsingKeyValue(image_point_index_map_current,nr_search_col, division_col);
            prev_point_idx = static_cast<int>(mes.point_index);  // takes care so that the descriptor is not created again with the same point of previous iteration

        }

    }
    max_measure = mes;  // keeps a copy of present measure as a member variable
    return mes;
}

CirconCorrespondence::Measure CirconCorrespondence::CompareDescriptorWithSimilarityMeasure(const std::vector<float>& src_image,
    const std::vector<float>& tar_image)
{
    float current_epsilon = INFINITY;

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
    std::vector<float>init_max_shift_descriptor;
    int init_rot_idx;
    std::map<int, size_t>init_mage_point_index_map = cid_source.GetImagePointMap();
    int init_cell_idx = cid_source.GetDescriptorCellIndexForInterestPoint();
   // float sm = 0.0f;

    std::vector<_dV>desc_content = cid_source.GetDescriptorContent();

    float sm = ComputeMaximumRotationShiftParameter(new_image, tar_image, init_max_shift_descriptor, init_mage_point_index_map, desc_content,
        init_rot_idx,init_cell_idx);

    mes.cell_index = init_cell_idx;//itr.first;
    mes.cell_values = init_max_shift_descriptor;
    mes.rotation_index = init_rot_idx;
    image_point_index_map_current = init_mage_point_index_map;
    mes.img_pt_index_map = image_point_index_map_current;
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
        CirconImageDescriptor transformed_source;
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

                std::vector<float>secondary_max_shift_descriptor;
                int secondary_rot_idx;
                std::map<int, size_t>secondary_mage_point_index_map = transformed_source.GetImagePointMap();
                int second_cell_idx = transformed_source.GetDescriptorCellIndexForInterestPoint();
                // check the similarity measure between new descriptor and the target image descriptor and return the similarity with max possible rotation   
                std::vector<_dV>desc_content; // TODO Use the update version
                float updated_sm = ComputeMaximumRotationShiftParameter(new_image, tar_image, secondary_max_shift_descriptor, 
                    secondary_mage_point_index_map,  desc_content, secondary_rot_idx, second_cell_idx);

                if (updated_sm > sm)
                {
                    sm = updated_sm;
                    mes.similarity_value = sm;
                    mes.cell_index = second_cell_idx;// itr.first;//
                    mes.cell_values = secondary_max_shift_descriptor;

                    mes.rotation_index = secondary_rot_idx;  //  transformed_source.GetRotationIndex()
                    mes.point_of_interest = transformed_source.GetRotationAXisPoint();
                    image_point_index_map_current = std::move(secondary_mage_point_index_map);
                    mes.point_index = itr.second; // FindElementInMap(itr, image_point_index_map); //transformed_source.GetPointIndexFromCloud(itr);
                    mes.WlTransform = transformed_source.GetTransformation();
                    mes.img_pt_index_map = image_point_index_map_current;
                    mes.primary_idx = itr.first%division_col;
                    mes.pix_value = current_array_of_data[itr.first];

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

        current_array_of_data = mes.cell_values;
        PointNormalType current_point = mes.point_of_interest;
        previous_pts_list.push_back(init_point);
        float dist = DistanceBetweenPoints(previous_pts_list, current_point);// pcl::geometry::distance(init_point, current_point);
        similarity_structure_list.push_back(mes);
        if (dist <= Epsilon)
        {
            mes = FindMaximumSimilarityInIterations(similarity_structure_list);
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
   }
   fclose(pFile);
   _filteredcloud->width = cloudxyz->points.size();
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
    std::vector<float>img_data = image.getImageData();
    image.WriteImage(fileName, img_data);

}
float CirconCorrespondence::ComputeRotationIndexFromShift(std::vector<float>secondary_descriptor, std::vector<float>target_point_descriptor, std::vector<float>&max_shift_descriptor,
    int &rotation_idx, std::map<int, size_t>&image_point_index_map_current, int& secondary_dsc_posn)
{
    std::string OutputDirectory = "Z:/staff/SDutta/GlobalRegistration/Comparison/";
    std::map<int, size_t>updated_map;
    std::map<int, size_t>max_sim_updated_map;
    if (secondary_descriptor.size() == 0)
    {
        return -1.0;
    }
    int desc_size = division_row * division_col;
    max_shift_descriptor.clear();
    max_shift_descriptor.resize(desc_size, -INFINITY);
    std::vector<float>iterative_descriptor;
    float max_similarity = ComputeMeasureOfSimilarity(secondary_descriptor, target_point_descriptor);
    rotation_idx = 0;
    max_sim_updated_map = image_point_index_map_current;  // initialize map
    max_shift_descriptor = secondary_descriptor;  // keep the current descriptor as the max descriptor
    
  /*  CirconImage img_desc(division_col, division_row);
    CirconImage img_desc2(division_col, division_row);
    char subsrcfile[100], subitrfile[100];*/
    
  
    int max_second_desc_idx = secondary_dsc_posn;
    int updated_second_desc_idx;
    std::vector<int>secondary_pt_indices = CopyElementsfromMaptoVector(desc_size, image_point_index_map_current);
    for (int nr = 1; nr < division_row; nr++)
    {
       
        iterative_descriptor.resize(division_row * division_col, -INFINITY);
        for (int img_pix = 0; img_pix < secondary_descriptor.size(); img_pix++)
        {
            float value_at_idx = secondary_descriptor.at(img_pix);
            int row_idx = img_pix / division_col;
            int col_idx = img_pix%division_col;
        
            int new_row_idx = (row_idx + nr) % division_row;
            int idx = new_row_idx * division_col + col_idx;
            iterative_descriptor[idx] = value_at_idx;
            if (value_at_idx != -INFINITY)
            {
               // int point_idx = FindElementInMap(img_pix, image_point_index_map_current);
                updated_map[idx] = secondary_pt_indices[img_pix];
            }

            int up_row_idx = secondary_dsc_posn / division_col;
            int up_col_idx = secondary_dsc_posn %division_col;
          updated_second_desc_idx  = ((up_row_idx + nr) % division_row) * division_col + up_col_idx;
        
        }
        float similarity_value = ComputeMeasureOfSimilarity(iterative_descriptor, target_point_descriptor);
        if (similarity_value > max_similarity)
        {
            max_similarity = similarity_value;
            max_shift_descriptor = iterative_descriptor;
            rotation_idx = nr;
           max_sim_updated_map = updated_map;
           max_second_desc_idx = updated_second_desc_idx;
           
        }
       /* sprintf(subitrfile, "src_img_itr_%d.bmp", nr);
        std::vector<float>scaled_iterative_descriptor = ScaleImageData(iterative_descriptor);
        img_desc2.SetImageData(scaled_iterative_descriptor);
        std::string SrcImgName = OutputDirectory + subitrfile;
        img_desc2.WriteImage(SrcImgName, scaled_iterative_descriptor);*/
        iterative_descriptor.clear();
        updated_map.clear();

    }
   /* sprintf(subsrcfile, "max_src_img_%d.bmp", rotation_idx);
    std::vector<float>scaled_max_shift_descriptor = ScaleImageData(max_shift_descriptor);
    img_desc.SetImageData(scaled_max_shift_descriptor);
    std::string SrcCloudName = OutputDirectory + subsrcfile;
    img_desc.WriteImage(SrcCloudName, scaled_max_shift_descriptor);
    */
   
    image_point_index_map_current = max_sim_updated_map;  // update the current map 
    secondary_dsc_posn = max_second_desc_idx;
    return max_similarity;
}
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
                    float ms = ComputeRotationIndexFromShift(source_descriptor_image, target_descriptor_image, max_shift_descriptor, rotation_idx, point_img_map, idx);
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
std::pair<PointNormalType, PointNormalType>CirconCorrespondence::ComputeFictituousCorrespondencePair(const PointNormalType &qSrcPt, const pcl::KdTreeFLANN<PointNormalType> &ktree_src,
    const  pcl::KdTreeFLANN<PointNormalType> &ktree_tar, float avgDist, Eigen::Matrix4f curr_transform, std::pair<int, int>&index_pair)
{
    std::vector<int>indices_list;
    std::vector<float>dist_list;
    std::vector<int>indices_list_tgt;
    std::vector<float>dist_list_tgt;
    float qRadius = 0.1 * maximum_radius;
    ktree_src.radiusSearch(qSrcPt, qRadius, indices_list, dist_list);
    CloudWithNormalPtr srx = cid_source.GetoriginalCloud();
    CloudWithNormalPtr tgx = cid_target.GetoriginalCloud();
    Eigen::Affine3f mat(curr_transform);
    std::pair<PointNormalType, PointNormalType>fict_corres_pair;
    index_pair.first = -1;
    index_pair.second = -1;
    for (int index = indices_list.size() - 1 ; index > 0; index--)
    {
        if (indices_list[index] < srx->points.size())
        {
            PointNormalType tfs_src_corres = pcl::transformPointWithNormal(srx->points[indices_list[index]], mat);
            ktree_tar.nearestKSearch(tfs_src_corres, 1, indices_list_tgt, dist_list_tgt);
            float dist = pcl::geometry::distance(tfs_src_corres, tgx->points[indices_list_tgt[0]]);
           // std::cout << "target distance for fictituous correspodence:" << dist <<  std::endl;
            if (dist < 0.1 * avgDist)
            {
                fict_corres_pair.first = srx->points[indices_list[index]];
                fict_corres_pair.second = tgx->points[indices_list_tgt[0]];
                index_pair.first = indices_list[index];
                index_pair.second = indices_list_tgt[0];
                return fict_corres_pair;
            }
            else
                continue;

        }
        else
            continue;
    }
    return fict_corres_pair;
}
std::pair<float, float>CirconCorrespondence::EstimateStopParameter(std::pair<PointNormalType, PointNormalType>corres_pair, std::pair<int, int>index_pair,
    Eigen::Matrix4f Pair_Transform, Eigen::Matrix4f Corres_Transform)
{
    std::vector<Eigen::Vector2d> st_param;
    // Prepare Descriptors
    int r = cid_source.GetRowDivision();
    int c = cid_source.GetColumnDivision();
    int h = cid_source.GetHeightDivision();
    CirconImageDescriptor src_corres_descriptor(input_source, r, c, h);
    CirconImageDescriptor tgx_corres_descriptor(input_target, r, c, h);
    src_corres_descriptor.ConstructLocalFrameOfReference();
    tgx_corres_descriptor.ConstructLocalFrameOfReference();
  /*  src_corres_descriptor.SetBasicPointIndex(index_pair.first);
    tgx_corres_descriptor.SetBasicPointIndex(index_pair.second);
    PrepareDescriptor(src_corres_descriptor, corres_pair.first);
    PrepareDescriptor(tgx_corres_descriptor, corres_pair.second);*/
    Eigen::Matrix4f sc_transform = src_corres_descriptor.GetTransformation();
    Eigen::Matrix4f tr_transform = tgx_corres_descriptor.GetTransformation();
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

    return std::make_pair(ang, trans_error);

}
std::vector<std::vector<float>>CirconCorrespondence::ResScaleTargetDescriptor(const std::vector<float>& Descriptor_Current)
{
    int row_min = 8;
    int col_min = 8;
 
    std::vector<std::vector<float>>ImageDescriptors;
    int it ;
    for (int it = row_min; it < division_row; it*=2)
    {
        std::vector<float>curr_img = tool::ReScaleImageBilinear(Descriptor_Current, division_row, division_col, row_min, col_min);
        ImageDescriptors.push_back(curr_img);
        row_min *= 2;
        col_min *= 2;
    }
    return ImageDescriptors;
}
std::vector<float>CirconCorrespondence::RotateImageByIndexNumber(const Eigen::MatrixXf & vector_matrix, int rot_idx)
{
   Eigen::MatrixXf out_matrix = tool::RowShiftMatrix(vector_matrix, rot_idx);
   std::vector<float>matrix_vector = tool::CreateStlVectorFromMatirx(out_matrix);
   return matrix_vector;
}
float CirconCorrespondence::EvaluateSimilarityByRotationShift(const std::vector<std::vector<float>>& src_descriptors, const std::vector<std::vector<float>>& target_descriptors,
    int& rotation_idx, std::vector<float>& max_descriptor, int min_res)
{
    //get the lowest resolution descriptor .i.e, currenlty 8x8
    std::vector<float>coarse_src_descriptor = src_descriptors[0];
    std::vector<float>coarse_tar_descriptor = target_descriptors[0];
    std::vector<Eigen::MatrixXf>vec_matrices;
    int row = 8; int col = 8;
    for (auto const& src_desc : src_descriptors)
    {
        Eigen::MatrixXf mat = tool::CreateMatrixFromStlVector(src_desc, row, col);
        vec_matrices.push_back(mat);
        row *= 2;
        col *= 2;
     }

   
    float sm = -INFINITY;
    int max_rot_idx;
    // coarse resolution similarity
    for (int i = 0; i <  min_res; i++)
    {
    
      std::vector<float> coarse_res_out = RotateImageByIndexNumber(vec_matrices[0], i);
      float new_sm = ComputeMeasureOfSimilarity(coarse_res_out, target_descriptors[0]);
      if (new_sm > sm)
      {
          sm = new_sm;
          max_rot_idx = i;
      }

    }
    int it;
        // iterate through all the resolution
    int nr = 1;
    std::vector<float>max_secondary_descriptor;
    for (int it = 2 * min_res; it <= division_row; it *= 2)
    {

        // test the similarity for current and next row shift by doubling it in the next resolution
        int new_rot_idx = 2 * max_rot_idx;
        Eigen::MatrixXf out = tool::RowShiftMatrix(vec_matrices[nr], new_rot_idx);
        std::vector<float> coarse_res_out = tool::CreateStlVectorFromMatirx(out);

        int new_rot_idx_next = 2 * max_rot_idx + 1;
        Eigen::MatrixXf out_next = tool::RowShiftMatrix(vec_matrices[nr], new_rot_idx_next);
        std::vector<float> coarse_res_out_next = tool::CreateStlVectorFromMatirx(out_next);

        float similarity_value_A = ComputeMeasureOfSimilarity(coarse_res_out, target_descriptors[nr]);
        float similarity_value_B = ComputeMeasureOfSimilarity(coarse_res_out_next, target_descriptors[nr]);
        if (similarity_value_A > similarity_value_B)
        {
            sm = similarity_value_A;
            max_rot_idx = new_rot_idx;
            max_secondary_descriptor = coarse_res_out;
        }
        else
        {
            sm = similarity_value_B;
            max_rot_idx = new_rot_idx_next;
            max_secondary_descriptor = coarse_res_out_next;
        }
        nr++;
    }
    rotation_idx = max_rot_idx;
    max_descriptor = max_secondary_descriptor;
    return sm;
}
float CirconCorrespondence::ComputeMaximumRotationShiftParameter(const std::vector<float>& secondary_descriptor, 
    const std::vector<float>& target_point_descriptor,std::vector<float>&max_shift_descriptor, std::map<int, size_t>&image_point_index_map_current, 
    std::vector<_dV> &descriptor_content, int &rotation_idx, int& secondary_dsc_posn)
{
    // first rescale source and target descriptors to coarser level starting 8x8
  
   std::vector<std::vector<float>>src_descriptors  = ResScaleTargetDescriptor(secondary_descriptor);
   src_descriptors.push_back(secondary_descriptor);
   std::vector<std::vector<float>>tar_descriptors = ResScaleTargetDescriptor(target_point_descriptor);
   tar_descriptors.push_back(target_point_descriptor);
   float max_similarity = EvaluateSimilarityByRotationShift(src_descriptors, tar_descriptors, rotation_idx, max_shift_descriptor);

   int up_row_idx = secondary_dsc_posn / division_col;
   int up_col_idx = secondary_dsc_posn %division_col;
   secondary_dsc_posn = ((up_row_idx + rotation_idx) % division_row) * division_col + up_col_idx;
   int desc_size = division_row * division_col;
   std::map<int, size_t>updated_map;
   std::vector<int>secondary_pt_indices = CopyElementsfromMaptoVector(desc_size, image_point_index_map_current);
 /*  secondary_descriptor_data.clear();
   secondary_descriptor_data.shrink_to_fit();
   secondary_descriptor_data.reserve(secondary_descriptor.size());*/

   std::vector<_dV>  refined_descriptor_content;
   refined_descriptor_content.reserve(max_shift_descriptor.size());
 
   for (int img_pix = 0; img_pix < secondary_descriptor.size(); img_pix++)
   {
       float value_at_idx = secondary_descriptor.at(img_pix);
       int row_idx = img_pix / division_col;
       int col_idx = img_pix%division_col;

       int new_row_idx = (row_idx + rotation_idx) % division_row;
       int idx = new_row_idx * division_col + col_idx;
      
       if (value_at_idx != -INFINITY)
       {
           // int point_idx = FindElementInMap(img_pix, image_point_index_map_current);
           if (secondary_pt_indices.size() > 0)
               updated_map[idx] = secondary_pt_indices[img_pix];
           if (descriptor_content.size() > 0 && col_idx < nr_search_col)
               refined_descriptor_content.push_back(_dV(new_row_idx, col_idx, -1, -1, max_shift_descriptor[idx], descriptor_content[img_pix].st,
                   descriptor_content[img_pix].pt));
       }
       else
       {
           _dV d;
          // descriptor_data.push_back(d);
       }

      
   }
 /*  auto finishItr = std::chrono::high_resolution_clock::now();
   double executeTime = std::chrono::duration_cast<
       std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
   executeTime = executeTime / double(1000);*/

   //update arguments
   image_point_index_map_current = updated_map;
   descriptor_content = refined_descriptor_content;
   return max_similarity;
}