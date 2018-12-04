#include"pch.h"
// C++ STL
#include <vector>
#include <fstream>
#include <utility>

// Point Cloud Library (PCL)
//#include <pcl/point_types.h>
//#include <pcl/features/fpfh_omp.h>
//#include <pcl/io/ply_io.h>

// Implemented header
#include "FPFHfeatures.h"



//////
//
// Helper functions
//

template <class flt_type>
flt_type estimatePointAvgDistance(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
{
	// Init
	flt_type avg = 0.0; int K = 2;
	std::vector<int> searchResults(K);
	std::vector<float> searchDistances(K);
	pcl::KdTreeFLANN<pcl::PointNormal> kdTree;
	kdTree.setInputCloud(cloud);

	// Sum distances to nearest neighbors
	for (unsigned i=0; i<cloud->size(); i++)
	{
		kdTree.nearestKSearch(*cloud, i, K, searchResults, searchDistances);
		avg += searchDistances[1];
	}

	// Return average
	return std::sqrt(avg / flt_type(cloud->size()));
}


//////
//
// Class implementation
//

// FPFHfeatures
//

template <class flt_type>
FPFHfeatures<flt_type>::FPFHfeatures() {}

template <class flt_type>
FPFHfeatures<flt_type>::FPFHfeatures(const std::string &filename, real radiusMult)
	: cloud(new pcl::PointCloud<pcl::PointNormal>)
{
	generateFromCloud_ply(filename, radiusMult);
}

template <class flt_type>
FPFHfeatures<flt_type>::FPFHfeatures(CloudWithoutType &CloudInput, real radiusMult)
    : cloud(new pcl::PointCloud<pcl::PointNormal>)
{
    pcl::fromPCLPointCloud2(*CloudInput, *cloud);
    radius_multiplier = radiusMult;
}
template <class flt_type>
FPFHfeatures<flt_type>::~FPFHfeatures() {}

template <class flt_type>
void FPFHfeatures<flt_type>::generateFromCloud_ply(const std::string &filename, real radiusMult)
{
	// Tidy up
	cloud->clear();
	features.clear();

	// Load point cloud
	if (pcl::io::loadPLYFile<pcl::PointNormal>(filename, *cloud) < 0)
		throw std::runtime_error("[FPFHfeatures] Unable to read .ply file:\n\"" +
		                         filename + "\"");

    radius_multiplier = radiusMult;

}
template <class flt_type>
void FPFHfeatures<flt_type>::generateFromCloud(CloudWithoutType &CloudInput, real radiusMult)
{
    cloud->clear();
    features.clear();
    pcl::fromPCLPointCloud2(*CloudInput, *cloud);
    radius_multiplier = radiusMult;
}
template <class flt_type>
void FPFHfeatures<flt_type>::writeFeatures_bin(const std::string &filename) const
{
	// Open binary file stream
	std::ofstream binfile(filename, std::ofstream::binary);

	// Write header information
	int intVar = (int)features.size(); binfile.write((char*)&intVar, sizeof(int));
	    intVar = 33;                   binfile.write((char*)&intVar, sizeof(int));

	// Write features
	for (int i=0; i<features.size(); i++)
	{
		// Write 3D position
		const pcl::PointNormal &pt = cloud->points[i];
		float pos[3] = { pt.x/pt.data[3], pt.y/pt.data[3], pt.z/pt.data[3] };
		binfile.write((char*)pos, 3*sizeof(float));
		const pcl::FPFHSignature33 &feature = features.points[i];
		binfile.write((char*)feature.histogram, 33*sizeof(float));
	}
}
template <class flt_type>
void FPFHfeatures<flt_type>::ComputeFeature()
{
    // Estimate feature inclusion radius
    real radius = radius_multiplier * estimatePointAvgDistance<real>(cloud);

    // Do feature estimation
    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> festm;
    festm.setRadiusSearch(radius);
    festm.setInputCloud(cloud);
    festm.setInputNormals(cloud);
    festm.compute(features);
}


//////
//
// Explicit template instantiations
//

// Only floating point variants are intended
template REG3D_API FPFHfeatures<float>;
template REG3D_API FPFHfeatures<double>;
