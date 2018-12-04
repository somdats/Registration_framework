#include"pch.h"
#include"Noise.h"
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"


CloudWithNormalPtr noise :: addGaussianNoiseToPointCloud(CloudWithNormalPtr &inputCloud, float std_dev, unsigned int seedValue )
{
    boost::mt19937 rng; 
    rng.seed(seedValue);
    boost::normal_distribution<> nd(0, std_dev);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
   // float rand = static_cast<float> (var_nor());
    CloudWithNormalPtr outputcloud(new pcl::PointCloud<PointNormalType>);
    outputcloud = inputCloud;
    for (size_t point_i = 0; point_i < inputCloud->points.size(); ++point_i)
    {
        /*outputcloud->points[point_i].getVector3fMap() = inputCloud->points[point_i].getVector3fMap() +
        static_cast<float> (var_nor()) * inputCloud->points[point_i].getNormalVector3fMap();*/
        outputcloud->points[point_i].x = inputCloud->points[point_i].x +
            static_cast<float> (var_nor())  * inputCloud->points[point_i].normal_x;
        outputcloud->points[point_i].y = inputCloud->points[point_i].y +
            static_cast<float> (var_nor()) * inputCloud->points[point_i].normal_y;
        outputcloud->points[point_i].z = inputCloud->points[point_i].z +
            static_cast<float> (var_nor()) * inputCloud->points[point_i].normal_z;
    }
    return outputcloud;
}
