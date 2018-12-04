#pragma once
#include<string>
#include"Datatypes.h"
#include"Config.h"

namespace pct
{
    REG3D_API int LoadPCTFIle(const std::string &fileNameWithLocation, std::vector<UVData<float>> &pixelCoordinates,
        std::vector<float> &intensityValues, std::vector<PointType>&point_coordinate);
    REG3D_API unsigned readLines(const std::string &strFileName);
    REG3D_API int SavePCTFile(const std::string FileName, std::vector<UVData<float>>pixelCoordinates ={ UVData<float>(0,0)}, std::vector<PointType>point_cloud = {},
        std::vector<float>intensityValues = { 100 });
}
