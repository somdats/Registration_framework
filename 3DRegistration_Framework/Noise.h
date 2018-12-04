#pragma once
#include"Config.h"
#include"Datatypes.h"

namespace noise
{

REG3D_API CloudWithNormalPtr addGaussianNoiseToPointCloud(CloudWithNormalPtr &inputCloud,  float std_dev, unsigned int seedValue);
}