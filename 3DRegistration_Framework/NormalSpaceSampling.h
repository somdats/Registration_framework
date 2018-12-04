#pragma once

#include"ISampling.h"
#include"Config.h"
#include"Datatypes.h"



// NormalSpaceSampling samples the input point cloud in the space of normal directions computed at every point

class REG3D_API cNormalSpaceSampling : public sampling::ISample
{
    public:
    cNormalSpaceSampling();
    cNormalSpaceSampling( int samples, int seed, int x, int y , int z); 
    cNormalSpaceSampling(cNormalSpaceSampling &cNmlSampling);
    ~cNormalSpaceSampling();
    void PrepareSamplingData(CloudWithoutType &inputCloud) override;
    void SamplePointCloud(CloudWithoutType &outputCloud ) override;
    void Sampling(CloudWithoutType &inputtCloud, CloudWithoutType &outputCloud);
    int getSampleSize()const;
    void setBins();
    typedef pcl::NormalSpaceSampling<PointNormalType, PointNormalType>normal_space_sampling;
  
 
protected:
    // brief Number of indices that will be returned.
          int nr_samples_;
      //brief Number of bins in x direction. 
          int binsx_;
       // brief Number of bins in y direction. 
          int binsy_;
      //brief Number of bins in z direction. 
          int binsz_;
          // set seed
          int seed_;
//  Instantiate  pcl normalSpaceSampling object
          normal_space_sampling nml_sampling;
  
};