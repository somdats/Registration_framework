#pragma once

#include"Ifile_io.h"
#include"Config.h"


#define BOOST_IS_NOT_INCLUDED
namespace iospace
{
    REG3D_API int loadPlyFile(const std::string &fileNameWithLocation,  CloudWithoutType &inputCloud) ;
    REG3D_API int writePlyFile(const std::string & outputFileWithLocation, CloudWithoutType &outputCloud);
    REG3D_API CloudWithoutType MergeCloud(CloudWithoutType &CloudA, CloudWithoutType &outputCloudB);
    REG3D_API std::vector<CloudWithNormalPtr> loadPlyFileinBlock (const std::string &fileName, const std::string &filePath);
    REG3D_API std::vector<std::string>readFilesfromList(const std::string &fileList);
    REG3D_API CloudWithoutType Load3DDataFromCSVFile(const std::string &fileNameWithLocation, const int x_id, const int y_id,
        const int z_id, const int k, const std::string s);
    REG3D_API std::vector<std::string> ReadFilesFromDirectory(const std::string &folderName, const std::string fileExtension);
    REG3D_API void WriteCorrespondenceAsPointCloud(const std::string &fileNameWithLocation, std::vector<Eigen::Vector3f>pts, std::vector<Eigen::Vector3f>normals);
    REG3D_API bool FindFilesEndWith(const std::string & path, const std::string & endingString, std::vector<std::string> & foundFiles,
        bool getOnlyFileName = false, const int nMaxItems = 10000000);

    long long LexicalCast(const std::string& s);
    REG3D_API  bool ExistsDirectory(const std::string &directory);
    REG3D_API void CreateDir(const std::string &directory);
    REG3D_API std::string GetPathDir(const std:: string &filename);
    REG3D_API void CreateFullPath(const  std::string &directory);
    REG3D_API std::string GetRelativeName(const  std::string &filename);
    REG3D_API bool ExistsFile(const std:: string &filename_abs);
    template<typename T> REG3D_API void LoadSingleValue(const std::string &absFileName, T &value, const bool binary = false);
    template<typename T> REG3D_API void SaveSingleValue(const std::string &absFileName, T value, const bool binary = false);
   
    //logical sorting (e.g. Windows explorer)
    class  REG3D_API  StringCompare_Smart_Incr
    {
    public:
        inline bool operator() (const std::string& a, const std::string& b) const;
        
    };
   
    
};
