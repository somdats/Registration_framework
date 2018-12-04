
// API config
#include"Config.h"
#include"Datatypes.h"
#include"IFeatureEstimator.h"

/**
 * @brief
 *		Class encapsulating PCL's Fast Point Feature Histogram (FPFH) functionality.
 */
template <class flt_type>
class REG3D_API FPFHfeatures :public FeatureEstimators::IFeature
{

public:

	////
	// Types

	/** @brief Real number type. */
	typedef flt_type real;


protected:

	////
	// Data members

	/** @brief The source point cloud. */
	pcl::PointCloud<PointNormalType>::Ptr cloud;

	/** @brief The feature point histograms. */
	pcl::PointCloud<pcl::FPFHSignature33> features;


public:

	////
	// Object construction / destruction

	/** @brief Default constructor. */
	FPFHfeatures();

	/** @brief Initializes from the given .ply file. */
	FPFHfeatures(const std::string &filename, real radiusMult);

    FPFHfeatures(CloudWithoutType &CloudInput, real radiusMult);

	/** @brief The destructor. */
	~FPFHfeatures();
    real radius_multiplier;

	////
	// Methods

	/**
	 * @brief Computes FPFH features for the point cloud stored in the given .ply file.
	 */
	void generateFromCloud_ply (const std::string &filename, real radiusMult);

    void generateFromCloud(CloudWithoutType &Cloud, real radiusMult);

	/** @brief Writes the given FPFH features to a binary file. */
	void writeFeatures_bin (const std::string &filename) const;
    void ComputeFeature();
};


