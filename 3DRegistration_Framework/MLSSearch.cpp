#include"pch.h"
#include"MLSSearch.h"
#include"ErrorMetric.h"
#include <exception>
 // pcl includes
//#include<pcl/common/vector_average.h>
//#include<pcl/common/geometry.h>

#include<chrono>


using namespace Eigen;
namespace
{
    int indexVisited(std::vector<int> &anVisitedList, int nCurrentIndx)
    {
        int visitedNodes = (int)anVisitedList.size();
        bool flag = false;
        for (int i = 0; i < visitedNodes; i++)
        {
            if (nCurrentIndx == anVisitedList[i])
            {
                flag = true;
                break;
            }
        }

        if (flag)
            return -1;

        else
        {
            anVisitedList.push_back(nCurrentIndx);
            return 0;
        }

    }
    int generatefaceNormals(CloudPtr &MeshVertices, pcl::Vertices & cTri, Eigen::Vector3f  &faceNormals)
    {
        Eigen::Vector3f cBA, cCA, cA, cB, cC, cZdir(0, 0, 1);
        int nN1, nN2, nN3;
        nN1 = cTri.vertices[0];
        nN2 = cTri.vertices[1];
        nN3 = cTri.vertices[2];

        faceNormals.setZero();
        if (nN1 < 0 || nN2 < 0 || nN3 < 0)
            return 0;

        cA = MeshVertices->points[nN1].getVector3fMap();
        cB = MeshVertices->points[nN2].getVector3fMap();
        cC = MeshVertices->points[nN3].getVector3fMap();

        cCA = cC - cA;
        cBA = cB - cA;

        faceNormals = cBA.cross(cCA); //cCA.cross(cBA)

        if (faceNormals.norm() != 0)
            faceNormals.normalize();
        return 1;
    }

    bool pointInTriangle(const Eigen::Vector3f& query_point, const Eigen::Vector3f& triangle_vertex_0, const Eigen::Vector3f& triangle_vertex_1,
        const Eigen::Vector3f& triangle_vertex_2)
    {

        Eigen::Vector3f u = triangle_vertex_1 - triangle_vertex_0;

        Eigen::Vector3f v = triangle_vertex_2 - triangle_vertex_0;
        Eigen::Vector3f n = u.cross(v);

        Eigen::Vector3f w = query_point - triangle_vertex_0;
      
        float gamma = u.cross(w).dot(n) / n.dot(n);

        float beta = w.cross(v).dot(n) / n.dot(n);
        float alpha = 1 - gamma - beta;
        return ((0 <= alpha) && (alpha <= 1) && (0 <= beta) && (beta <= 1) && (0 <= gamma) && (gamma <= 1));
    }
}

cMLSearch::~cMLSearch()
{
    if (NULL != kdTree)
    {
        delete kdTree;
        kdTree = NULL;
    }

    // newly added
    if (NULL != kntree)
    {
        delete kntree;
        kntree = NULL;
    }

   /* if (NULL != pcNeigborhood)
    {
        delete pcNeigborhood;
        pcNeigborhood = NULL;
    }*/
}
void cMLSearch::PrepareSample(CloudWithoutType &targetCloud)
{
    inputCloud = targetCloud;
    CloudPtr pTarget(new pcl::PointCloud <PointType>);
    pcl::fromPCLPointCloud2(*targetCloud, *pTarget);
    inputTargetPointSet = pTarget;
    kdTree = new(pcl::KdTreeFLANN<PointType>);
    kdTree->setInputCloud(pTarget);
    CloudWithNormalPtr pTarget_Norm(new pcl::PointCloud <PointNormalType>);
    inputTargetWithNormal = pTarget_Norm;
    pcl::fromPCLPointCloud2(*targetCloud, *pTarget_Norm);
    // newly added
    kntree = new(pcl::KdTreeFLANN<PointNormalType>);
    kntree->setInputCloud(pTarget_Norm);
    avgDistanceTargetCloud = metric::EstimatePointAvgDistance(targetCloud);
  
}

std::vector<float> cMLSearch:: ComputeWeightFromLocalNeighborhood(PointNormalType &querySourcePoint, CloudPtr &targetCloud,
    std::vector<int> &nn_indices, IWeightingFunction<float> &weightingFunction, float kernelSize)
{
    /*CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*targetCloud, *pTarget);
*/
    std::vector<float>Weights;
    Weights.reserve(nn_indices.size());
    if (nn_indices.size() != 0)
    {

        for (int i = 0; i < nn_indices.size(); i++)
        {
            if (nn_indices[i] < targetCloud->points.size())
            {
                Eigen::Vector3f targetPoint = targetCloud->points[nn_indices[i]].getVector3fMap();
                Eigen::Vector3f sourcePoint = querySourcePoint.getVector3fMap();
                float distance = (sourcePoint - targetPoint).norm();
                Weights.push_back(weightingFunction(distance, kernelSize));
            }
        }
    }
    return Weights;
}

void cMLSearch::GenerateFaceNeighborhood(pcl::PolygonMesh &mesh, std::vector<SNeighbor> &ptNeighbors)
{
    int numFaces = mesh.polygons.size();
    std::vector<pcl::Vertices>Tri;
    int faceCount = 0;
//#pragma omp parallel for
    for (int it = 0; it<numFaces; ++it)
    {
        pcl::Vertices Tri = mesh.polygons[it];
        if (Tri.vertices[0] < 0 && Tri.vertices[1] < 0 && Tri.vertices[2] < 0)
            continue;
        else
        {
            ptNeighbors[Tri.vertices[0]].m_anFaceIndicesList.push_back(it);
            ptNeighbors[Tri.vertices[1]].m_anFaceIndicesList.push_back(it);
            ptNeighbors[Tri.vertices[2]].m_anFaceIndicesList.push_back(it);

            ptNeighbors[Tri.vertices[0]].m_anVertexIndxList.push_back(Tri.vertices[1]);
            ptNeighbors[Tri.vertices[0]].m_anVertexIndxList.push_back(Tri.vertices[2]);

            ptNeighbors[Tri.vertices[1]].m_anVertexIndxList.push_back(Tri.vertices[0]);
            ptNeighbors[Tri.vertices[1]].m_anVertexIndxList.push_back(Tri.vertices[2]);

            ptNeighbors[Tri.vertices[2]].m_anVertexIndxList.push_back(Tri.vertices[0]);
            ptNeighbors[Tri.vertices[2]].m_anVertexIndxList.push_back(Tri.vertices[1]);
            faceCount++;

        }
    }
}

void cMLSearch::setPolygonMesh(pcl::PolygonMesh mesh)
{
    if (triangle_projection == true)
    {
        Mesh = mesh;
    }
    CloudPtr cloud1(new pcl::PointCloud<PointType>());
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud1);

    std::vector<SNeighbor>Neigborhood;
    Neigborhood.resize(cloud1->points.size());
    GenerateFaceNeighborhood(mesh, Neigborhood);
    pcNeigborhood = Neigborhood;
}

bool cMLSearch::GetProjectedPointFromSurface(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint)
{
    PointRGBNormalType transforme_q_point;
    transforme_q_point.getVector3fMap() = transform * querypoint.getVector3fMap();
    transforme_q_point.getNormalVector3fMap() = transform.linear() * querypoint.getNormalVector3fMap();
    std::vector<int> pointIdxNearestSearch;
    std::vector<float> pointSquaredDistance;
    PointType point(transforme_q_point.x, transforme_q_point.y, transforme_q_point.z);
    kdTree->nearestKSearch(point, 1, pointIdxNearestSearch, pointSquaredDistance);
    if (pointIdxNearestSearch.size() == 0 || pointIdxNearestSearch[0] > inputTargetPointSet->points.size())
    {
        targetPoint.x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.z = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_z = std::numeric_limits<float>::quiet_NaN();
        targetPoint.curvature = std::numeric_limits<float>::quiet_NaN();
        return false;
    }
    int closestPointIndex = pointIdxNearestSearch[0];
    PointType closestTargetPoint = inputTargetPointSet->points[closestPointIndex];
    int numIncidentFaceList = pcNeigborhood[closestPointIndex].m_anFaceIndicesList.size();
    std::vector<int> anVisitedList;
    anVisitedList.reserve(numIncidentFaceList);
    pointNeigbor ptNeigborhood = pcNeigborhood[closestPointIndex];
    int nFlagStat;
    for (int faceIndexItr = 0; faceIndexItr < numIncidentFaceList; faceIndexItr++)
    {
        int anIndx = ptNeigborhood.m_anFaceIndicesList[faceIndexItr];
        pcl::Vertices Tri = Mesh.polygons[anIndx];
        if (Tri.vertices[0] < 0 && Tri.vertices[1] < 0 && Tri.vertices[2] < 0)
            continue;
        nFlagStat = indexVisited(anVisitedList, anIndx);
        if (nFlagStat == -1)
            continue;
        Eigen::Vector3f faceNormal;
        generatefaceNormals(inputTargetPointSet, Tri, faceNormal);
        Eigen::Vector3f projectedPoint;
        Eigen::Vector3f cA = inputTargetPointSet->points[Tri.vertices[0]].getVector3fMap();
        Eigen::Vector3f cB = inputTargetPointSet->points[Tri.vertices[1]].getVector3fMap();
        Eigen::Vector3f cC = inputTargetPointSet->points[Tri.vertices[2]].getVector3fMap();
        pcl::geometry::project(point.getVector3fMap(), cA, faceNormal, projectedPoint);
         
        bool onCurrTriangle = pointInTriangle(projectedPoint, cA, cB, cC);
        if (onCurrTriangle == true)
        {
            targetPoint.getVector3fMap() = projectedPoint;
            targetPoint.getNormalVector3fMap() = faceNormal;
            return true;
        }
        else
            continue;
    }
    return false;
}
void cMLSearch::findPoint(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint)
{
    if (triangle_projection == true)
    {
        bool status = GetProjectedPointFromSurface(querypoint, transform, targetPoint);
        if (status == false)
        {
            targetPoint.x = std::numeric_limits<float>::quiet_NaN();
            targetPoint.y = std::numeric_limits<float>::quiet_NaN();
            targetPoint.z = std::numeric_limits<float>::quiet_NaN();
            targetPoint.normal_x = std::numeric_limits<float>::quiet_NaN();
            targetPoint.normal_y = std::numeric_limits<float>::quiet_NaN();
            targetPoint.normal_z = std::numeric_limits<float>::quiet_NaN();
            targetPoint.curvature = std::numeric_limits<float>::quiet_NaN();
            return;
        }
        else
            return;
    }
    PointNormalType qPoint;
    qPoint.getVector3fMap() = transform * querypoint.getVector3fMap();  // transformed source
    qPoint.getNormalVector3fMap() = transform.linear() * querypoint.getNormalVector3fMap();
    bool status = false;
    bool mls_corresps = false;
   // bool _curvatureFiltering = true;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    Eigen::Vector3f eigen_values(0.0f, 0.0f, 0.0f);
    Eigen::Matrix3Xf pt(3, 1), norm(3, 1);
    PointNormalType pnOld;
    pnOld.getVector3fMap() = qPoint.getVector3fMap();      //querypoint.getVector3fMap();
    pnOld.getNormalVector3fMap() = qPoint.getNormalVector3fMap(); // querypoint.getNormalVector3fMap();
    pcl::PointNormal pnNew;
    float radius = 0.0f;
        if (m_resolution == 0.0f)
        {
            radius = scaleF * avgDistanceTargetCloud;
        }
        else
            radius = scaleF * m_resolution;
    int itr = 0;
    Eigen::VectorXf coeffs;
    Eigen::VectorXf pn;
    std::vector<Eigen::Vector3f>eigen_vectors;
    int mls_itr;
   
    for (mls_itr = 0; mls_itr < mls_itr_max; mls_itr++)
    {
        std::vector<float>weight;
        Eigen::Vector3f ref_domain_origin(0.0f, 0.0f, 0.0f);
        PointType point(pnOld.x, pnOld.y, pnOld.z);
        if (kdTree->radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 4) //point
        {
            weight = ComputeWeightFromLocalNeighborhood(pnOld, inputTargetPointSet, pointIdxRadiusSearch, WeightFunc, radius);

            if (weight[0] < 0.01)
            {
       
                break;
            }
            Eigen::Vector3f meanOfNeighborhood(0.0f, 0.0f, 0.0f);
            Eigen::MatrixXf cov_matrix;
            eigen_vectors.clear();
            eigen_vectors.resize(3);
            auto startItr = std::chrono::high_resolution_clock::now();
            // compute weighted covariance matrix of local neighborhood
            eigen_values = ComputeWeightedCovariancematrix(inputTargetPointSet, weight, pointIdxRadiusSearch,
                meanOfNeighborhood, eigen_vectors, cov_matrix);
            auto finishItr = std::chrono::high_resolution_clock::now();
            double executeTime = std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
            executeTime = executeTime / double(1000);
            //std::cout << "Time:" << executeTime << "secs" << std::endl;
            //  compute local reference domain origin
            ref_domain_origin = ComputeLocalReferenceDomainOrigin(pnOld, eigen_vectors,
                meanOfNeighborhood);  //pnOld

            std::vector<Eigen::Vector3f>localTransfromedPoints = transformNeighborhoodtoLocalFrame(pnOld,
                inputTargetPointSet, pointIdxRadiusSearch, meanOfNeighborhood, eigen_vectors, ref_domain_origin); //pnOld

            // std::vector<Eigen::Vector3f>LocalToWorld = ReProjectneighborstoWorldFrame(localTransfromedPoints, ref_domain_origin, eigen_vectors);
            coeffs = poly_fit(polynomialOrder_, localTransfromedPoints, weight);
            pn = ComputeCorrespondenceUsingPolyfit(coeffs, ref_domain_origin, eigen_vectors);
            pnNew.getVector3fMap() = pn.head<3>();
            pnNew.getNormalVector3fMap() = pn.tail<3>();
            
        }
        else
        {
            // mls_itr = mls_itr_max;
            break;
            // introduce findpoint kdtree-search here
        }
     
        float error = pcl::geometry::distance(pnNew, pnOld);
        if (error <= 2e-4f)
        {
            if (with_filtering == true)
            {

                Eigen::Vector3f viewDirection = pnNew.getVector3fMap() - Eigen::Vector3f(0.0f, 0.0f, 0.0f);
                viewDirection.normalize();
                if (pnNew.getNormalVector3fMap().dot(viewDirection) > 0.0f)
                {
                    eigen_vectors[0] *= -1.0f;
                }
                if (qPoint.getNormalVector3fMap().dot(eigen_vectors[0]) >= normal_filter_threshold)
                {
                    if (_curvatureFiltering == true)
                    {
                        Eigen::Vector3f eigen_val_source = getEigenValuesForPoint(querypoint);
                        float comp1 = eigen_val_source(1) / eigen_values(1);
                        float comp2 = eigen_val_source(2) / eigen_values(2);
                        if ((0.5f <= comp1 && comp1 <= 1.5f)  && (0.5f <= comp2 && comp2 <= 1.5f))
                        {
                            status = true;
                            break;
                        }
                        else
                        {
                            break;
                        }
                    }
                    else
                    {
                        status = true;
                        break;
                    }

                }
                else
                {
                    break;
                }

            }
            else
            {
                status = true;
                break;
            }

        }
        // update point, if the error condition not satisfied
        pnOld = pnNew;
    }

    //  if (mls_itr == mls_itr_max - 1)   // max itertaion of MLS reaches
    //  {
    //      if (with_filtering == true)
    //      {
    //          if (tranformed_source_normal.dot(eigen_vectors[0]) <= 0.7f)
    //          {

    //              Eigen::Vector3f eigen_val_source = getEigenValuesForPoint(querypoint);
    //              float comp1 = eigen_val_source(1) / eigen_values(1);
    //              float comp2 = eigen_val_source(2) / eigen_values(2);
    //              if (0.5f <= comp1 <= 1.5f && 0.5f <= comp2 <= 1.5f)
    //              {
    //                  status = true;
    //          
    //              }
    //             
    //          }
    //         
    //      }
    //     /* else
    //      {
    //          status = true;
    //      }
    //*/
    //  }
    if ( status == false)
    {
        targetPoint.x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.z = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_z = std::numeric_limits<float>::quiet_NaN();
        targetPoint.curvature = std::numeric_limits<float>::quiet_NaN();
        return;
    }
    else
    {

        Eigen::Vector3f viewDirection = pnNew.getVector3fMap() - Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        viewDirection.normalize();
        if (pnNew.getNormalVector3fMap().dot(viewDirection) > 0.0f)
        {
            pnNew.getNormalVector3fMap() *= -1.0f;
        }
        targetPoint.getVector3fMap() = pnNew.getVector3fMap();
        targetPoint.getNormalVector3fMap() = pnNew.getNormalVector3fMap();

        return;

    }
   
    /* targetPoint.getVector3fMap() = pnNew.getVector3fMap();
    targetPoint.getNormalVector3fMap() = pnNew.getNormalVector3fMap();
    std::cout << "count_m:" << count_m <<  std::endl;*/

}

Eigen::Vector3f cMLSearch :: ComputeWeightedCovariancematrix(CloudPtr &targetCloud, std::vector<float>& Weights,
    std::vector<int> nn_indices, Eigen::Vector3f &meanpoint, std::vector<Eigen::Vector3f>&Eigen_Vectors,
    Eigen::MatrixXf & cov_matrix)
{
   
    /*CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*targetCloud, *pTarget);*/
    
   // Eigen_Vectors.clear();
    Eigen::Matrix3f covMatrix = Eigen::Matrix3f::Identity();
    if (Weights.size() != nn_indices.size())
    {
        //return false;
        exit(0);
    }
    pcl::VectorAverage3f vAverageCov;
    for (int i = 0; i < nn_indices.size(); i++)
    {
        Eigen::Vector3f neighborPoints = targetCloud->points[nn_indices[i]].getVector3fMap();
        vAverageCov.add(neighborPoints, Weights[i]);
    }
    Eigen::Vector3f eigen_vector1(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f eigen_vector2(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f eigen_vector3(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f eigen_values(0.0f, 0.0f, 0.0f);
    if (vAverageCov.getNoOfSamples() < 3)
        throw   std::runtime_error("ERROR: unable to compute covariance-matrix");
    vAverageCov.doPCA(eigen_values, eigen_vector1, eigen_vector2, eigen_vector3);  // returns all eigen vector and values
    eigen_vector1.normalize(); eigen_vector2.normalize(); eigen_vector3.normalize();
    Eigen::Vector3f eigen_vector4;
    vAverageCov.getEigenVector1(eigen_vector4); //verify whether eigenvector4 is same as eigen_vector1
    eigen_vector4.normalize();
    meanpoint = vAverageCov.getMean();
    Eigen_Vectors[0] = eigen_vector1;
    Eigen_Vectors[1] = eigen_vector2;
    Eigen_Vectors[2] = eigen_vector3;
    
    //verification if eigen vectors are correct
    if (std::abs(eigen_vector1.dot(eigen_vector2)) < 1e-7f && std::abs(eigen_vector1.dot(eigen_vector3)) <1e-7f
        && std::abs(eigen_vector2.dot(eigen_vector3)) <1e-7f)
        // std::cout << "dot product is zero:" << "1" << std::endl;
        ///*else  if (std::abs(eigen_vector1.dot(eigen_vector3)) <1e-7)
        //    std::cout << "dot product is zero:" << "2" << std::endl;
        //else if (std::abs(eigen_vector2.dot(eigen_vector3)) <1e-7)
        //    std::cout << "dot product is zero:" << "3"<< std::endl;*/
        //else
        //{
        //  //  std::cout << "dot product is  not zero:" <<  std::endl;
        //}
        cov_matrix = vAverageCov.getCovariance();
    eigen_values /= eigen_values.sum();
    return eigen_values;
}

Vector3f cMLSearch :: ComputeLocalReferenceDomainOrigin(PointNormalType &querySourcePoint,
    std::vector<Vector3f>&Eigen_Vectors, Vector3f &meanpoint)
{
    Vector3f ProjectPoint = querySourcePoint.getVector3fMap();
    Vector3f ProjectedPoint(0.0f, 0.0f, 0.0f);
    pcl::geometry::project(ProjectPoint, meanpoint, Eigen_Vectors[0], ProjectedPoint);
    return ProjectedPoint;
}

std::vector<Vector3f> cMLSearch::transformNeighborhoodtoLocalFrame(PointNormalType &querySourcePoint,
    CloudPtr &targetCloud,
    std::vector<int>&nn_indices, Eigen::Vector3f &meanpoint, std::vector<Eigen::Vector3f>&Eigen_Vectors,
    Eigen::Vector3f &localOrigin)
{
   /* CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*targetCloud, *pTarget);*/

    std::vector<Eigen::Vector3f>localTransformedPoints;
    localTransformedPoints.reserve(nn_indices.size());
    for (int i = 0; i < nn_indices.size(); i++)
    {
        Eigen::Vector3f neighborPoints = targetCloud->points[nn_indices[i]].getVector3fMap();
        Eigen::Vector3f diff = neighborPoints - localOrigin;
        float zCoord = diff.dot(Eigen_Vectors[0]);
        float uCoord = diff.dot(Eigen_Vectors[1]);
        float vCoord = diff.dot(Eigen_Vectors[2]);

        localTransformedPoints.push_back(Eigen::Vector3f(uCoord, vCoord, zCoord));
    }

    //	// SANITY CHECK: q should have coordinates (0, 0) in reference domain
    /* float q_refdom1 = (localOrigin - localOrigin).dot(Eigen_Vectors[0]);
    float q_refdom2 = (localOrigin - localOrigin).dot(Eigen_Vectors[1]);*/
    return localTransformedPoints;
}

std::vector<Vector3f> cMLSearch::ReProjectneighborstoWorldFrame(std::vector<Eigen::Vector3f> &transformedlocalNeighbor,
    Eigen::Vector3f &localOrigin,
    std::vector<Eigen::Vector3f>&Eigen_Vectors)
{
    std::vector<Vector3f>mls_points_planeproj;
    mls_points_planeproj.reserve(transformedlocalNeighbor.size());
    for each (Eigen::Vector3f temp in transformedlocalNeighbor)
    {
        mls_points_planeproj.push_back(localOrigin + Eigen_Vectors[2] * temp(0) + Eigen_Vectors[1] * temp(1));
    }
    return mls_points_planeproj;
}

VectorXf cMLSearch:: poly_fit(int polynomial_order, std::vector<Eigen::Vector3f> &transformedlocalNeighbor,
    std::vector<float>&Weights)
{

    if (polynomial_order > 1)
    {
        int nr_coeff = (polynomial_order + 1) * (polynomial_order + 2) / 2;
        int num_neighbors = transformedlocalNeighbor.size();
        Eigen::VectorXf poly_coefficient = Eigen::VectorXf::Zero(nr_coeff);
        if (num_neighbors > nr_coeff)
        {
            float u_pow, v_pow;
            Eigen::MatrixXf P(nr_coeff, num_neighbors);
            Eigen::MatrixXf P_weight;         // (nr_coeff, num_neighbors);
            Eigen::MatrixXf P_weight_Pt(nr_coeff, nr_coeff);
            Eigen::VectorXf weight_vec(num_neighbors);
            Eigen::VectorXf f_vec(num_neighbors);

            for (int ni = 0; ni < num_neighbors; ++ni)
            {
                // Compute the polynomial's terms at the current point
                int j = 0;
                u_pow = 1;
                for (int ui = 0; ui <= polynomial_order; ++ui)
                {
                    v_pow = 1;
                    for (int vi = 0; vi <= polynomial_order - ui; ++vi)
                    {
                        P(j++, ni) = u_pow * v_pow;
                        v_pow *= transformedlocalNeighbor[ni](1);
                    }
                    u_pow *= transformedlocalNeighbor[ni](0);
                }
                weight_vec(ni) = Weights[ni];
                f_vec(ni) = transformedlocalNeighbor[ni](2);  // intially it was zero:: bug
            }
            // Computing coefficients
            P_weight = P * weight_vec.asDiagonal();
            P_weight_Pt = P_weight * P.transpose();
            poly_coefficient = P_weight * f_vec;
            P_weight_Pt.llt().solveInPlace(poly_coefficient);

        }
        return poly_coefficient;
    }

}

Eigen::VectorXf cMLSearch::ComputeCorrespondenceUsingPolyfit(Eigen::VectorXf &coefficients, Eigen::Vector3f &localOrigin,
    std::vector<Eigen::Vector3f> &Eigen_Vectors)
{
    Eigen::VectorXf PointNormal = Eigen::VectorXf::Zero(6);
    PointNormal.head<3>() = localOrigin + coefficients(0) * Eigen_Vectors[0];
    PointNormal.tail<3>() = Eigen_Vectors[0];
    return PointNormal;
}

int cMLSearch::findIndices(PointRGBNormalType &querypoint)
{
    // TODO
    return 0;
}
CloudWithoutType &cMLSearch::getTargetCloudForMLSSurface()
{
    return inputCloud;
}

Eigen::Matrix3f cMLSearch :: GetPrincipalFrame()
{
    return principalFrame;
}
void cMLSearch::findApproximatePointforMLSSmoothing(PointRGBNormalType &querypoint, Eigen::Affine3f &transform, PointRGBNormalType &targetPoint)
{
    Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
    bool status = true;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    static int count_m = 0;
    static  int count_n = 0;
    static  int count_o = 0;
    Eigen::Matrix3Xf pt(3, 1), norm(3, 1);
    PointNormalType pnOld;
    pnOld.getVector3fMap() =  transform * querypoint.getVector3fMap();
    pnOld.getNormalVector3fMap() = transform.linear() * querypoint.getNormalVector3fMap();
    pcl::PointNormal pnNew;
    float radius = 0.0f;
    if (m_resolution == 0.0f)
    {
         radius = scaleF * avgDistanceTargetCloud;
    }
    else
        radius = scaleF * m_resolution;
    int itr = 0;
    Eigen::VectorXf coeffs;
    Eigen::VectorXf pn;
    std::vector<Eigen::Vector3f>eigen_vectors;
    int mls_itr;
    Eigen::Vector3f viewDirection =  origin - querypoint.getVector3fMap();
    Eigen::Vector3f eigen_values(0.0f, 0.0f, 0.0f);
    for (mls_itr = 0; mls_itr < mls_itr_max; mls_itr++)
    {
        std::vector<float>weight;
        Eigen::Vector3f ref_domain_origin(0.0f, 0.0f, 0.0f);
        PointType point(pnOld.x, pnOld.y, pnOld.z);
        if (kdTree->radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 2)
        {
            weight = ComputeWeightFromLocalNeighborhood(pnOld, inputTargetPointSet, pointIdxRadiusSearch, WeightFunc, radius);

            /*if (weight[0] < 0.01)
            {
                mls_itr = mls_itr_max;
                int idx = pointIdxRadiusSearch[0];
                break;
            }*/
            Eigen::Vector3f meanOfNeighborhood(0.0f, 0.0f, 0.0f);
            Eigen::MatrixXf cov_matrix;
            eigen_vectors.clear();
            eigen_vectors.resize(3);
            auto startItr = std::chrono::high_resolution_clock::now();
            // compute weighted covariance matrix of local neighborhood
            eigen_values =  ComputeWeightedCovariancematrix(inputTargetPointSet, weight, pointIdxRadiusSearch,
                meanOfNeighborhood, eigen_vectors, cov_matrix);
            auto finishItr = std::chrono::high_resolution_clock::now();
            double executeTime = std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
            executeTime = executeTime / double(1000);
            //std::cout << "Time:" << executeTime << "secs" << std::endl;
            //  compute local reference domain origin
            ref_domain_origin = ComputeLocalReferenceDomainOrigin(pnOld, eigen_vectors,
                meanOfNeighborhood);

            std::vector<Eigen::Vector3f>localTransfromedPoints = transformNeighborhoodtoLocalFrame(pnOld,
                inputTargetPointSet, pointIdxRadiusSearch, meanOfNeighborhood, eigen_vectors, ref_domain_origin);

            // std::vector<Eigen::Vector3f>LocalToWorld = ReProjectneighborstoWorldFrame(localTransfromedPoints, ref_domain_origin, eigen_vectors);
            coeffs = poly_fit(polynomialOrder_, localTransfromedPoints, weight);
            pn = ComputeCorrespondenceUsingPolyfit(coeffs, ref_domain_origin, eigen_vectors);
            pnNew.getVector3fMap() = pn.head<3>();
            pnNew.getNormalVector3fMap() = pn.tail<3>();
            /*if (pnNew.getNormalVector3fMap().dot(viewDirection) < 0.0f)     //previously
            {
                pnNew.getNormalVector3fMap() = -1.0f * pnNew.getNormalVector3fMap();
            }*/
            count_m++;

        }
        else
        {
           // mls_itr = mls_itr_max;
           status = false;
            break;
            // introduce findpoint kdtree-search here
        }
       
        float error = pcl::geometry::distance(pnNew, pnOld);
        if (error <= 2e-4f)
        {
            break;
            //check if the required point satisfies filtering criteria
        }
        // update point, if the error condition not satisfied
        pnOld = pnNew;
    }
    if (status == false)
    {
        targetPoint.x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.z = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_z = std::numeric_limits<float>::quiet_NaN();
        targetPoint.curvature = std::numeric_limits<float>::quiet_NaN();
        principalFrame.col(0) = Eigen::Vector3f(0.0,0.0,0.0);
        principalFrame.col(1) = Eigen::Vector3f(0.0, 0.0, 0.0);
        principalFrame.col(2) = Eigen::Vector3f(0.0, 0.0, 0.0);
        Eigen_Values = Eigen::Vector3f(0.0, 0.0, 0.0);
        return;
    }
    if (mls_itr == mls_itr_max)
    {
     /*   targetPoint.x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.z = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_x = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_y = std::numeric_limits<float>::quiet_NaN();
        targetPoint.normal_z = std::numeric_limits<float>::quiet_NaN();
        targetPoint.curvature = std::numeric_limits<float>::quiet_NaN();*/
        Eigen::Vector3f viewDirection = pnNew.getVector3fMap() - Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        viewDirection.normalize();
        if (pnNew.getNormalVector3fMap().dot(viewDirection) > 0.0f)
        {
            pnNew.getNormalVector3fMap() = -1.0f * pnNew.getNormalVector3fMap();
        }
        targetPoint.getVector3fMap() = pnNew.getVector3fMap();
        targetPoint.getNormalVector3fMap() = pnNew.getNormalVector3fMap();
        principalFrame.col(0) = eigen_vectors[0];
        principalFrame.col(1) = eigen_vectors[1];
        principalFrame.col(2) = eigen_vectors[2];
        Eigen_Values = eigen_values;
    }
    else
    {
        Eigen::Vector3f viewDirection = pnNew.getVector3fMap() - Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        viewDirection.normalize();
        if (pnNew.getNormalVector3fMap().dot(viewDirection) > 0.0f)
        {
            pnNew.getNormalVector3fMap() = -1.0f * pnNew.getNormalVector3fMap();
        }
        targetPoint.getVector3fMap() = pnNew.getVector3fMap();
        targetPoint.getNormalVector3fMap() = pnNew.getNormalVector3fMap();
        principalFrame.col(0) = eigen_vectors[0];
        principalFrame.col(1) = eigen_vectors[1];
        principalFrame.col(2) = eigen_vectors[2];
        Eigen_Values = eigen_values;

    }
   
}
Eigen::Vector3f cMLSearch::GetEigenValues()
{
    return Eigen_Values;
}
Eigen::Matrix3f cMLSearch::getPrincipalFrameForPoint(PointRGBNormalType &querypoint)
{
    PointDataType pt(querypoint.x, querypoint.y, querypoint.z,
        querypoint.normal_x, querypoint.normal_y, querypoint.normal_z);
    Eigen::Matrix3f p_frame_source;
    std::map<PointDataType, int>::iterator itr = Principal_frame_Index_source_map.find(pt);
    if (itr != Principal_frame_Index_source_map.end())
    {
        int index = itr->second;
        if (index < Principal_frame_source.size() && index > -1)
        {
            p_frame_source = Principal_frame_source[index];
        }
    }
    return p_frame_source;
}

void cMLSearch::CreateIndicesVsPointMap(CloudWithoutType & PointCloud)
{
    CloudWithNormalPtr  temp_cloud(new pcl::PointCloud<PointNormalType>);
    pcl::fromPCLPointCloud2(*PointCloud, *temp_cloud);
    Principal_frame_Index_source_map.clear();
    for (int i = 0; i < temp_cloud->points.size(); i++)
    {
        PointDataType pt(temp_cloud->points[i].x, temp_cloud->points[i].y, temp_cloud->points[i].z,
            temp_cloud->points[i].normal_x, temp_cloud->points[i].normal_y, temp_cloud->points[i].normal_z);
        Principal_frame_Index_source_map.insert(std::pair<PointDataType, int>(pt, i));
    }
}

Eigen::Vector3f cMLSearch::getEigenValuesForPoint(PointRGBNormalType &querypoint)
{
    PointDataType pt(querypoint.x, querypoint.y, querypoint.z,
        querypoint.normal_x, querypoint.normal_y, querypoint.normal_z);
    Eigen::Vector3f eigenval_source;
    std::map<PointDataType, int>::iterator itr = Principal_frame_Index_source_map.find(pt);
    if (itr != Principal_frame_Index_source_map.end())
    {
        int index = itr->second;
        if (index < eigen_values_source.size() && index > -1)
        {
            eigenval_source = eigen_values_source[index];
        }
    }
    return eigenval_source;
}
Eigen::Vector3f cMLSearch::ComputeEigenValues(Eigen::MatrixXf covariance_matrix)
{
    
    Eigen::SelfAdjointEigenSolver< Eigen::MatrixXf> eigensolver(covariance_matrix);
    if (eigensolver.info() != Success)
        abort();
    Eigen::Vector3f  eigen_values = eigensolver.eigenvalues();
    return eigen_values;
}

void cMLSearch::SetFeatureAngleForSourceCloud(std::vector<double>feature_angles_s)
{
    Point_feature_angles_source = feature_angles_s;
   
}

void cMLSearch::getFeatureAngle(Eigen::VectorXf &querypointA,double &Fa_source)
{
    Eigen::Vector3f sx = querypointA.head(3);
    Eigen::Vector3f sn = querypointA.tail(3);
    PointDataType ptA(sx(0), sx(1), sx(2), sn(0), sn(1), sn(2));
    std::map<PointData<float>, int>::iterator itr = Principal_frame_Index_source_map.find(ptA);
    if (itr != Principal_frame_Index_source_map.end())
    {
        int index = itr->second;
        if (index < Point_feature_angles_source.size() && index > -1)
        {
            Fa_source = Point_feature_angles_source[index];
        }
    }
  
}
void cMLSearch::SetIndicesVsPointMap(std::map<PointDataType, int>pf_source_map)
{
    Principal_frame_Index_source_map = pf_source_map;
}