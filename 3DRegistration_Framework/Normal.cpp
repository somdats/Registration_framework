#include"pch.h"
#include"Normal.h"

Normal::Normal()
{
    m_dX = NULL;
    m_dY = NULL;
    m_dZ = NULL;
    Curvature = NULL;
    Index.resize(10);
    Dist.resize(10);
}

void Normal::computeRoots2(Eigen::Matrix3f::Scalar &b, Eigen::Matrix3f::Scalar &c, Eigen::Vector3f &roots)
{

    typedef Eigen::Matrix3f::Scalar Scalar;

    roots(0) = Scalar(0);
    Scalar d = Scalar((b) *(b)-4.0 * (c));
    if (d < 0.0) // no real roots!!!! THIS SHOULD NOT HAPPEN!
        d = 0.0;

    Scalar sd = sqrt(d);

    roots(2) = 0.5f * (b + sd);
    roots(1) = 0.5f * (b - sd);
}
void Normal::computeRoots(Eigen::Matrix3f &m, Eigen::Vector3f &roots)
{


    typedef Eigen::Matrix3f::Scalar Scalar;
    /* Eigen::Matrix3f temp;
    temp = m;*/
    /*typedef Eigen::Vector3f Scalar ;
    Eigen::Matrix3f temp;
    temp = *m;*/



    // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
    // eigenvalues are the roots to this equation, all guaranteed to be
    // real-valued, because the matrix is symmetric.
    Scalar c0 = m(0, 0) * m(1, 1) * m(2, 2)
        + Scalar(2) * m(0, 1) * m(0, 2) * m(1, 2)
        - m(0, 0) * m(1, 2) * m(1, 2)
        - m(1, 1) * m(0, 2) * m(0, 2)
        - m(2, 2) * m(0, 1) * m(0, 1);
    Scalar c1 = m(0, 0) * m(1, 1) -
        m(0, 1) * m(0, 1) +
        m(0, 0) * m(2, 2) -
        m(0, 2) * m(0, 2) +
        m(1, 1) * m(2, 2) -
        m(1, 2) * m(1, 2);
    Scalar c2 = m(0, 0) + m(1, 1) + m(2, 2);


    if (fabs(c0) < Eigen::NumTraits<Scalar>::epsilon())// one root is 0 -> quadratic equation
        computeRoots2(c2, c1, roots);
    else
    {
        const Scalar s_inv3 = Scalar(1.0 / 3.0);
        const Scalar s_sqrt3 = std::sqrt(Scalar(3.0));
        // Construct the parameters used in classifying the roots of the equation
        // and in solving the equation for the roots in closed form.
        Scalar c2_over_3 = c2*s_inv3;
        Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
        if (a_over_3 > Scalar(0))
            a_over_3 = Scalar(0);

        Scalar half_b = Scalar(0.5) * (c0 + c2_over_3 * (Scalar(2) * c2_over_3 * c2_over_3 - c1));

        Scalar q = half_b * half_b + a_over_3 * a_over_3*a_over_3;
        if (q > Scalar(0))
            q = Scalar(0);

        // Compute the eigenvalues by solving for the roots of the polynomial.
        Scalar rho = std::sqrt(-a_over_3);
        Scalar theta = std::atan2(std::sqrt(-q), half_b) * s_inv3;
        Scalar cos_theta = std::cos(theta);
        Scalar sin_theta = std::sin(theta);
        roots(0) = c2_over_3 + Scalar(2) * rho * cos_theta;
        roots(1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
        roots(2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

        // Sort in increasing order.
        if (roots(0) >= roots(1))
            std::swap(roots(0), roots(1));
        if (roots(1) >= roots(2))
        {
            std::swap(roots(1), roots(2));
            if (roots(0) >= roots(1))
                std::swap(roots(0), roots(1));
        }

        if (roots(0) <= 0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
            computeRoots2(c2, c1, roots);
    }
}

void Normal::eigen33(Eigen::Matrix3f &mat, Eigen::Vector3f::Scalar &eigenvalue, Eigen::Vector3f &eigenvector)
{
    //Eigen::Vector3f::Scalar Scalar;
    // Scale the matrix so its entries are in [-1,1].  The scaling is applied
    // only when at least one matrix entry has magnitude larger than 1.

    Eigen::Vector3f::Scalar scale = mat.cwiseAbs().maxCoeff();
    if (scale <= std::numeric_limits<Eigen::Vector3f::Scalar>::min())
        scale = Eigen::Vector3f::Scalar(1.0);

    Eigen::Matrix3f scaledMat = mat / scale;

    Eigen::Vector3f eigenvalues;
    computeRoots(scaledMat, eigenvalues);

    eigenvalue = eigenvalues(0) * scale;

    scaledMat.diagonal().array() -= eigenvalues(0);

    Eigen::Vector3f vec1 = scaledMat.row(0).cross(scaledMat.row(1));
    Eigen::Vector3f vec2 = scaledMat.row(0).cross(scaledMat.row(2));
    Eigen::Vector3f vec3 = scaledMat.row(1).cross(scaledMat.row(2));

    Eigen::Vector3f::Scalar len1 = vec1.squaredNorm();
    Eigen::Vector3f::Scalar len2 = vec2.squaredNorm();
    Eigen::Vector3f::Scalar len3 = vec3.squaredNorm();

    if (len1 >= len2 && len1 >= len3)
        eigenvector = vec1 / std::sqrt(len1);
    else if (len2 >= len1 && len2 >= len3)
        eigenvector = vec2 / std::sqrt(len2);
    else
        eigenvector = vec3 / std::sqrt(len3);
}

void Normal::centroid(std::vector<int> data, pcl::PointCloud<pcl::PointXYZ >&cld, int count, Eigen::Vector3f &centroid3d)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>centroid3d; 
    //Eigen::Vector4f centroid3d ;
    Eigen::Vector3f C3Ddata (0,0,0);
    centroid3d = C3Ddata;
    data.resize(count);
#pragma omp parallel for
    for (int i = 0; i < count; i++)
    {
        Eigen::Vector3f temp(cld.points[data[i]].x, cld.points[data[i]].y, cld.points[data[i]].z);

        //centroid3d.X  += cld.points[data[i]].x;
        centroid3d = centroid3d + temp;
    }
    centroid3d = centroid3d / (double)count;
    //return centroid3d;
}

unsigned int Normal::computecovmatrix( Eigen:: Vector3f &Cvec, std::vector<int>&Arr, CloudPtr &ptcld, Eigen::Matrix3f &covariance_matrix)
{
    unsigned point_count;
    //Eigen::Matrix3f temp = covariance_matrix;

    if (Arr.empty())
        return (0);

    // Initialize to 0
    covariance_matrix.setZero();


    // If the data is dense, we don't need to check for NaN
    if (ptcld->is_dense)
    {

        point_count = Arr.size();
        // For each point in the cloud
        for (size_t i = 0; i < point_count; ++i)
        {
            double tx = ptcld->points[Arr[i]].x - Cvec(0);
            double ty = ptcld->points[Arr[i]].y - Cvec(1);
            double tz = ptcld->points[Arr[i]].z - Cvec(2);
            Eigen::Vector4f pt(tx, ty, tz, 1);
            covariance_matrix(1, 1) += pt.y() * pt.y();
            covariance_matrix(1, 2) += pt.y() * pt.z();

            covariance_matrix(2, 2) += pt.z() * pt.z();

            // pt *= pt.x ();

            /*covariance_matrix(0, 0) += pt.x();
            covariance_matrix(0, 1) += pt.y ();
            covariance_matrix(0, 2) += pt.z ();*/
            covariance_matrix(0, 0) += pt.x() *  pt.x();
            covariance_matrix(0, 1) += pt.x() *  pt.y();
            covariance_matrix(0, 2) += pt.x()  *  pt.z();
            covariance_matrix(1, 0) += pt.x() *  pt.y();
            covariance_matrix(2, 0) += pt.x()  *  pt.z();
            covariance_matrix(2, 1) += pt.y() * pt.z();
        }
    }
    // NaN or Inf values could exist => check for them
    else
    {
        point_count = 0;
        // For each point in the cloud
        for (size_t i = 0; i < Arr.size(); ++i)
        {
            // Check if the point is invalid
            if (!isFinite(ptcld->points[Arr[i]]))
                continue;

            double tx = ptcld->points[Arr[i]].x - Cvec(0);
            double ty = ptcld->points[Arr[i]].y - Cvec(1);
            double tz = ptcld->points[Arr[i]].z - Cvec(2);
            Eigen::Vector4f pt(tx, ty, tz, 1);
            covariance_matrix(1, 1) += pt.y() * pt.y();
            covariance_matrix(1, 2) += pt.y() * pt.z();

            covariance_matrix(2, 2) += pt.z() * pt.z();

            pt *= pt.x();
            covariance_matrix(0, 0) += pt.x();
            covariance_matrix(0, 1) += pt.y();
            covariance_matrix(0, 2) += pt.z();
            ++point_count;
        }
    }
    /* covariance_matrix (1, 0) = covariance_matrix (0, 1);
    covariance_matrix (2, 0) = covariance_matrix (0, 2);
    covariance_matrix (2, 1) = covariance_matrix (1, 2);*/
    covariance_matrix = covariance_matrix / point_count;
    return (point_count);
}

void Normal::solve_plane(Eigen::Matrix3f &covariance_matrix, float &nx, float &ny, float &nz, float &curvature)
{

    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
    eigen33(covariance_matrix, eigen_value, eigen_vector);

    nx = eigen_vector[0];
    ny = eigen_vector[1];
    nz = eigen_vector[2];

    // Compute the curvature surface change
    float eig_sum = covariance_matrix.coeff(0) + covariance_matrix.coeff(4) + covariance_matrix.coeff(8);
    if (eig_sum != 0)
        curvature = fabs(eigen_value / eig_sum);
    else
        curvature = 0;
}
