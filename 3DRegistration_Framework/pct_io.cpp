#include"pch.h"
#include<fstream>
#include"pct_io.h"


unsigned pct::readLines(const std::string &strFileName)
{
    unsigned numberOfLines = 0;
    std::string line;
    std::ifstream myfile(strFileName.c_str());

    if (myfile.is_open())
    {
        while (std::getline(myfile, line))
            ++numberOfLines;
        std::cout << "Number of lines in text file: " << numberOfLines << std::endl;
    }
    myfile.close();
    return numberOfLines;
}
int pct :: LoadPCTFIle(const std::string &fileNameWithLocation, std::vector<UVData<float>> &pixelCoordinates, std::vector<float> &intensityValues,
    std::vector<PointType>&point_coordinate)
{
   unsigned noOfLines = readLines(fileNameWithLocation);
   int fileRead = 0;
   FILE *pFile;
   pFile = fopen(fileNameWithLocation.c_str(), "rb");
   pixelCoordinates.clear();
   intensityValues.clear();
   point_coordinate.clear();
   pixelCoordinates.reserve(noOfLines - 1);
   intensityValues.reserve(noOfLines - 1);
   point_coordinate.reserve(noOfLines - 1);
   if (NULL == pFile)
   {
       std::cout << "Failed to read data file " <<  std:: endl;
       abort();
   }
   char szParam1[50], szParam2[50], szParam3[50], szParam4[50], szParam5[50], szParam6[50];
   // fields
   fscanf(pFile, "%s %s %s %s %s %s ", szParam1, szParam2, szParam3, szParam4, szParam5, szParam6);
   for (int i = 1; i < noOfLines; i++)
   {
       fscanf(pFile, "%s %s %s %s %s %s\n", szParam1, szParam2, szParam3, szParam4, szParam5, szParam6);
       PointType point;
       point.x = atof(szParam4);
       point.y = atof(szParam5);
       point.z = atof(szParam3);
       point_coordinate.push_back(point);
       pixelCoordinates.push_back(UVData<float>(atof(szParam1), atof(szParam2)));
       intensityValues.push_back(atof(szParam6));
   }
   fileRead = 1;
   fclose(pFile);
   return fileRead;
}
int pct::SavePCTFile(const std::string FileName, std::vector<UVData<float>>pixelCoordinates, std::vector<PointType>point_cloud,
    std::vector<float>intensityValues)
{
    std::string outputFileName = FileName.c_str();
    intensityValues.resize(point_cloud.size());
    FILE *nFile = fopen(outputFileName.c_str(), "wb");
    if (intensityValues.size() == 1)
    {
        intensityValues.resize(point_cloud.size(), (float)100);
       
    }
    if (pixelCoordinates.size() == 1)
    {
        pixelCoordinates.resize(point_cloud.size(), UVData<float>(0,0));

    }
    if (NULL == nFile)
    {
        return (-1);
    }
    fprintf(nFile, "i\tj\tz\tx\ty\tGreyValue\n");
    for (int i  =  0; i < pixelCoordinates.size(); i++)
    {
        fprintf(nFile, "%d\t%d\t%.6f\t%0.6f\t%0.6f\t%d\r\n", (int)pixelCoordinates[i].u, (int)pixelCoordinates[i].v, 
            point_cloud[i].z, point_cloud[i].x, point_cloud[i].y, intensityValues[i]);
    }
    fclose(nFile);
    return 0;

}
