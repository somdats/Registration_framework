#include"pch.h"
#include<chrono>
#include<random>
#include"Common.h"
#include"Morton.h"
# include"NormalSpaceSampling.h"

// constructor

cNormalSpaceSampling::cNormalSpaceSampling()
{
    nr_samples_ = 0;
    seed_ = 0;
    binsx_ = 0;
    binsy_ = 0;
    binsz_ = 0;
  
}
// constructor

cNormalSpaceSampling::cNormalSpaceSampling(int samples, int seed, int x, int y, int z)
{
    nr_samples_ = samples;
    seed_ = seed;
    binsx_ = x;
    binsy_ = y;
    binsz_ = z;
}
 // copy constructor

cNormalSpaceSampling::cNormalSpaceSampling(cNormalSpaceSampling &cNmlSampling)
{
    nr_samples_ = cNmlSampling.nr_samples_;
    binsx_ = cNmlSampling.binsx_;
    binsy_ = cNmlSampling.binsy_;
    binsz_ = cNmlSampling.binsz_;
}

// Destructor

cNormalSpaceSampling::~cNormalSpaceSampling()
{
    nr_samples_ = 0;
    binsx_ = 0;
    binsy_ = 0;
    binsz_ = 0;
 
}
 // Prepare Data for Sampling 

 void cNormalSpaceSampling::PrepareSamplingData(CloudWithoutType &inputCloud)
 {
   
     CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
     pcl::fromPCLPointCloud2(*inputCloud, *pTarget);
     
     std::string normal = "normal_x";
     auto it = find_if(begin(inputCloud->fields), end(inputCloud->fields), [=](pcl::PCLPointField const& f) {
         return (f.name == normal);
     });
     bool found = (it != end(inputCloud->fields));
     if (!found)
     {
         throw std::runtime_error(" normal not found.");
     }
     nml_sampling.setInputCloud(pTarget);
     nml_sampling.setNormals(pTarget);
     nml_sampling.setSample(nr_samples_);
     nml_sampling.setSeed(seed_);
     setBins();
 
 }
  //  set bin size for sampling
 
 void cNormalSpaceSampling::setBins()
 {
    
     nml_sampling.setBins(binsx_, binsy_, binsz_);
 }

 // return sampling size 

 int cNormalSpaceSampling::getSampleSize()const
 {
     return nr_samples_;
 }

 // sample the input cloud
 // I/P : input cloud
 //o/p : sampled cloud

 void cNormalSpaceSampling::SamplePointCloud(CloudWithoutType &outputCloud)
 {
     CloudWithNormalPtr psampledCloud(new pcl::PointCloud <PointNormalType>);
     auto startItr = std::chrono::high_resolution_clock::now();
     nml_sampling.filter(*psampledCloud);
     auto finishItr = std::chrono::high_resolution_clock::now();
     double executeTime = std::chrono::duration_cast<
         std::chrono::duration<double, std::milli>>(finishItr - startItr).count();
     executeTime = executeTime / double(1000);
     std::cout << "normalspacesampling time :" << executeTime << "secs" << std::endl;
#ifdef LOGDATA
     error_log("normalspacesampling time:%f\n", executeTime);
#endif
     pcl::toPCLPointCloud2(*psampledCloud, *outputCloud);  
 }

 void cNormalSpaceSampling::Sampling(CloudWithoutType &inputCloud, CloudWithoutType &outputCloud)
 {

     CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
     pcl::fromPCLPointCloud2(*inputCloud, *pTarget);
     Eigen::Matrix3Xf MatNormal = Eigen::Matrix3Xf::Zero(3, pTarget->points.size());
     Eigen::Matrix3Xf MatCloud = Eigen::Matrix3Xf::Zero(3, pTarget->points.size());
     CloudWithNormalPtr psampledCloud(new pcl::PointCloud <PointNormalType>);
     //Distribute the points with a correspondence into normal buckets.
     std::map<MortonCode64, std::vector<size_t>> normalBucketsMap;
     for (int i = 0; i < pTarget->points.size(); ++i)
     {
         if (!std::isnan(pTarget->points[i].getVector3fMap().x()))
         {
             Eigen::Vector3i discrete = (pTarget->points[i].getNormalVector3fMap() * 10).cast<int>();
             MortonCode64 code(discrete.x(), discrete.y(), discrete.z());
             normalBucketsMap[code].push_back(i);
         }
         MatCloud.col(i) = pTarget->points[i].getVector3fMap();
         MatNormal.col(i) = pTarget->points[i].getNormalVector3fMap();
     }
     std::vector<std::vector<size_t>> normalBuckets;
     int potentialSamples = 0;
     for (auto& entry : normalBucketsMap)
     {
         potentialSamples += entry.second.size();
         normalBuckets.push_back(std::move(entry.second));
     }
     normalBucketsMap.clear();

     if (potentialSamples < 10)
     {
         std::cout << "Could not find enough overlap. Registration will abort." << std::endl;
         return;
     }
     int samples = (int)(potentialSamples * 0.02);
     std::uniform_int_distribution<size_t> bucketDist(0, normalBuckets.size() - 1);
     std::mt19937 rnd;
     psampledCloud->width = samples;
     psampledCloud->height = 1;
     psampledCloud->points.resize(samples * 1);
     //subsample the point cloud for ICP
     for (int i = 0; i < samples; ++i)
     {
         size_t sample;

         if (0.01 == 1)
             sample = i;
         else
         {
             //normal space sampling

             bool sampleOk = false;
             int attempt = 0;
             while (!sampleOk && attempt++ < 10)
             {
                 auto bucketIdx = bucketDist(rnd);
                 auto& bucket = normalBuckets[bucketIdx];

                 std::uniform_int_distribution<size_t> sampleDist(0, bucket.size() - 1);
                 auto sampleIdx = sampleDist(rnd);
                 sample = bucket[sampleIdx];

                 if (std::isnan(MatCloud.coeff(0, sample)) || std::isnan(MatNormal.coeff(0, sample)))
                     continue;

                 sampleOk = true;

                 bucket.erase(bucket.begin() + sampleIdx);
                 if (bucket.empty())
                 {
                     normalBuckets.erase(normalBuckets.begin() + bucketIdx);
                     bucketDist = std::uniform_int_distribution<size_t>(0, normalBuckets.size() - 1);
                 }
             }
         }
         psampledCloud->points[i].getVector3fMap() = pTarget->points[sample].getVector3fMap();
         psampledCloud->points[i].getNormalVector3fMap() = pTarget->points[sample].getNormalVector3fMap();
     }
     pcl::toPCLPointCloud2(*psampledCloud, *outputCloud);
 }
