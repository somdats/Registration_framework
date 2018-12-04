#include"pch.h"
#include"ply_file_io.h"
#include"pct_io.h"
#include<Windows.h>
#include <io.h>
//#include<pcl/io/ply_io.h>
//#include<pcl/point_cloud.h>
//#include <pcl/console/print.h>
//#include <pcl/console/parse.h>
//#include <pcl/console/time.h>

using namespace pcl::console;
using namespace std;

namespace
{
    void readFilenames(std::vector<std::string> &filenames, const  std::string &directory)
    {

        HANDLE dir;
        WIN32_FIND_DATA file_data;

        if ((dir = FindFirstFile((directory + "/*").c_str(), &file_data)) == INVALID_HANDLE_VALUE)
            return; /* No files found */

        do {
            const string file_name = file_data.cFileName;
            const string full_file_name = directory + "/" + file_name;
            const bool is_directory = (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;

            if (file_name[0] == '.')
                continue;

            if (is_directory)
                continue;

            filenames.push_back(full_file_name);
        } while (FindNextFile(dir, &file_data));

        FindClose(dir);
    }
}

    std::vector<std::string> iospace::readFilesfromList(const std::string &fileList)
    {
        int noOfFiles = pct::readLines(fileList);
        std::vector<std::string> fileArray;
        //fileArray.resize(noOfFiles);

        FILE *pFile;

        // open file for reading
        pFile = fopen(fileList.c_str(), "rb");
        if (NULL == pFile)
        {
            std::cout << "Failed to read data file " <<  std::endl;
            exit(-1);
        }
        char szParam1[100];

        for (int i = 0; i < noOfFiles; i++)
        {
            // fields
            fscanf(pFile, "%s\n", szParam1);
            fileArray.push_back(szParam1);

        }
        fclose(pFile);
        return fileArray;
    }

int iospace::loadPlyFile(const std::string &fileNameWithLocation,  CloudWithoutType &inputCloud)
{
  
    TicToc tt;
    print_highlight("Loading "); print_value("%s ", fileNameWithLocation.c_str());
    tt.tic();
    if (pcl::io::loadPLYFile(fileNameWithLocation, *inputCloud) < 0)
        return(-1);
    print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms : "); print_value("%d", inputCloud->width * inputCloud->height); print_info(" points]\n");
    print_info("Available dimensions: "); print_value("%s\n", pcl::getFieldsList(*inputCloud).c_str());
    return 1;
    
}
int iospace::writePlyFile(const std::string & outputFileWithLocation, CloudWithoutType &outputCloud)
{
    TicToc tt;
    tt.tic();
    CreateFullPath(GetPathDir(outputFileWithLocation));
    print_highlight("Saving "); print_value("%s ", outputFileWithLocation.c_str());
    int filesaved = pcl::io::savePLYFile(outputFileWithLocation, *outputCloud);
    print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms : "); print_value("%d", outputCloud->width * outputCloud->height); print_info(" points]\n");
    return filesaved;
}
CloudWithoutType  iospace::MergeCloud(CloudWithoutType &CloudA, CloudWithoutType &CloudB)
{
    CloudWithoutType mergedCloud(new pcl::PCLPointCloud2());   //TODO add option for pointwithout normal;
    CloudWithNormalPtr cloudOne(new pcl::PointCloud <PointNormalType>);
    CloudWithNormalPtr CloudTwo(new pcl::PointCloud <PointNormalType>);
    CloudWithNormalPtr CloudThree(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*CloudA, *cloudOne);
    pcl::fromPCLPointCloud2(*CloudB, *CloudTwo);
    CloudThree = cloudOne;
    *CloudThree = *CloudThree + *CloudTwo;
    pcl::toPCLPointCloud2(*CloudThree, *mergedCloud);
    return mergedCloud;
}
std::vector<CloudWithNormalPtr> iospace:: loadPlyFileinBlock(const std::string &fileName, const std::string &filePath)
{
    std::vector<std::string>Cloudname =  readFilesfromList(fileName);
 
    
    std::vector<CloudWithNormalPtr>clouds;
    clouds.reserve(Cloudname.size());
    for (int i = 0; i < Cloudname.size(); i++)
    {
        CloudWithNormalPtr cloudOne(new pcl::PointCloud <PointNormalType>);
        std::string full_file_Name = filePath + "/" + Cloudname[i];
        pcl::io::loadPLYFile(full_file_Name, *cloudOne);
        clouds.push_back(cloudOne);
      
    }
    return clouds;

}
CloudWithoutType iospace:: Load3DDataFromCSVFile(const std::string &fileNameWithLocation, const int x_id, const int y_id,
    const int z_id, const int k, const std::string s)
{
    //open file
    ifstream in(fileNameWithLocation.c_str());
    if (!in.is_open())
    {
        print_error("Error loading file\n");
        exit(1);
    }

    CloudPtr new_cloud(new  pcl ::PointCloud<PointType>);

    int n_lines = 0;

   std:: string line;
   std:: vector<std::string > vec;
    while (in.eof() != 1)
    {
        //Always get the line -> so while advances
        getline(in, line);
        // 		cout << line << endl;

        //skip a number k of rows
        n_lines++;
        if (n_lines <= k)
        {
            continue;
        }

        //void lines could be accientaly present, in case continue
        if (line.size() == 0)
        {
            continue;
        }


        //Tokenize the line
        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        boost::char_separator<char> sep(s.c_str());
        tokenizer tokens(line, sep);

        //Assign tokens to a string vector
        vec.clear();
        vec.assign(tokens.begin(), tokens.end());


        //now check if we have a column for each requested field
        if (vec.size() < x_id)
        {
            print_error("You requested a x field that does not exist in your file! Check separtor too.\n");
            exit(1);
        }

        if (vec.size() < y_id)
        {
            print_error("You requested a y field that does not exist in your file! Check separtor too.\n");
            exit(1);
        }
        if (vec.size() < z_id)
        {
            print_error("You requested a z field that does not exist in your file! Check separtor too.\n");
            exit(1);
        }
       
        //just some verbosity
        if (n_lines % 10000 == 0)
        {
           // print_info("line %i\n", n_lines);
        }

        //Create the point
        PointType point;
        point.x = atof(vec[x_id - 1].c_str());
        point.y = atof(vec[y_id - 1].c_str());
        point.z = atof(vec[z_id - 1].c_str());
       
        //Add the point to the cloud
        new_cloud->push_back(point);
    }
    CloudWithoutType cloud_output(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*new_cloud, *cloud_output);
    in.close();
    return cloud_output;
}
std::vector<std::string> iospace:: ReadFilesFromDirectory(const std::string &folderName, const std::string fileExtension)
{
   
    std::vector<std::string>tempFileNames;
   std:: vector<int> positions;
    std::string sub = "/";
    readFilenames(tempFileNames, folderName);
    std::vector<string>fileNames;
    fileNames.reserve(tempFileNames.size());
    int slashposns = tempFileNames[0].find(sub, 0);
    while (slashposns != std::string::npos)
    {
        positions.push_back(slashposns);
        slashposns = tempFileNames[0].find(sub, slashposns + 1);
    }
  
    for (int j = 0; j < tempFileNames.size(); j++)
    {
        std::string Name, fileCounter, extension;
        

        if (!tempFileNames[j].empty())
        {
            int pos = tempFileNames[j].find(".");
            if (pos != std::string::npos)
            {
                Name = tempFileNames[j].substr(0, pos - 1);
            }
            int endPos = tempFileNames[j].length() - pos;
            extension = tempFileNames[j].substr(pos + 1, endPos);

            if (strcmp(extension.c_str(), fileExtension.c_str()) == 0)
            {
                fileNames.push_back(tempFileNames[j]);
                int newpos = positions[positions.size() - 1] + 1;
                int diffpos = pos - newpos;
                fileCounter = tempFileNames[j].substr(newpos, diffpos);

            }
        }
    }
    return fileNames;
}

 void iospace:: WriteCorrespondenceAsPointCloud(const std::string &fileNameWithLocation, std::vector<Eigen::Vector3f>pts, std::vector<Eigen::Vector3f>normals)
{
     std::string  fileName = fileNameWithLocation + ".ply";
   
    FILE *pFile;
    pFile = fopen(fileName.c_str(), "wb");
    fprintf(pFile, "ply\n");
    fprintf(pFile, "format ascii 1.0\n");
    fprintf(pFile, "element vertex %d\n", static_cast<int>(pts.size()));
    fprintf(pFile, "property float x \n");
    fprintf(pFile, "property float y \n");
    fprintf(pFile, "property float z \n");
    fprintf(pFile, "property float nx \n");
    fprintf(pFile, "property float ny \n");
    fprintf(pFile, "property float nz \n");
    fprintf(pFile, "end_header\n");
    for (int i = 0; i < pts.size(); i++)
    {
        fprintf(pFile, "%f %f %f %f %f %f\r\n", pts[i](0), pts[i](1), pts[i](2), normals[i](0), normals[i](1), normals[i](2));
    }
    fclose(pFile);
}
 bool iospace::FindFilesEndWith(const std::string & path, const std::string & endingString, std::vector<std::string> & foundFiles,
     bool getOnlyFileName, const int nMaxItems)
 {
     foundFiles.clear();

#ifdef _MSC_VER
     // Required structs for searching for files and directories
     WIN32_FIND_DATA FindFileData;
     HANDLE hFind = INVALID_HANDLE_VALUE;

     // Build the file search string...
     char searchDir[2048] = { 0 };
     char fullpath[2048] = { 0 };

     // ...if it already is a path that ends with \ or /, add '*'...
     if (path.at(path.length() - 1) == '\\' || path.at(path.length() - 1) == '/')
     {
         _snprintf(searchDir, 2047, "%s*", path.c_str());
         _snprintf(fullpath, 2047, "%s", path.c_str()); // just copy path
     }
     // ...otherwise, add '\*' to the end of the path.
     else
     {
         _snprintf(searchDir, 2047, "%s/*", path.c_str());
         _snprintf(fullpath, 2047, "%s/", path.c_str()); // copy path and add slash (required when building filenames)
     }

     // Find the first file in the directory.
     hFind = FindFirstFile(searchDir, &FindFileData);

     // If there is no file, return
     if (hFind == INVALID_HANDLE_VALUE)
     {
         return false;
     }

     // loop
     do
     {
         // Skip ".", ".." and all directories
         if (((FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0) || strcmp(FindFileData.cFileName, ".") == 0 || strcmp(FindFileData.cFileName, "..") == 0)
         {
             continue;
         }

         if (endingString.size() > std::string(FindFileData.cFileName).size())
         {
             continue;
         }

         // Store filename into the vector if it starts with the given string
         if (endingString.size() > 0)
         {
             if (std::string(FindFileData.cFileName).rfind(endingString) == (std::string(FindFileData.cFileName).size() - endingString.size()))
             {
                 if (getOnlyFileName)
                 {
                     foundFiles.push_back(FindFileData.cFileName);
                 }
                 else
                 {
                     // File found: create a path to the file
                     char filePath[2048] = { 0 };
                     _snprintf(filePath, 2047, "%s%s", fullpath, FindFileData.cFileName);
                     // Add it to vector of files found
                     foundFiles.push_back(filePath);
                 }
             }
         }
         else // Always store filename if a starting string has not been provided
         {
             if (getOnlyFileName)
             {
                 foundFiles.push_back(FindFileData.cFileName);
             }
             else
             {
                 // File found: create a path to the file
                 char filePath[2048] = { 0 };
                 _snprintf(filePath, 2047, "%s%s", fullpath, FindFileData.cFileName);
                 // Add it to vector of files found
                 foundFiles.push_back(filePath);
             }
         }
     }
     // Loop while we find more files
     while (FindNextFile(hFind, &FindFileData) != 0);

     // Release
     FindClose(hFind);

     sort(foundFiles.begin(), foundFiles.end(), StringCompare_Smart_Incr());

     foundFiles.resize(std::min(nMaxItems, (int)foundFiles.size()));

     return true;
#else // not _MSC_VER
     DIR* directory = opendir(path.c_str());
     if (directory)
     {
         string parent(path);
         if (parent[parent.length() - 1] != '/')
             parent.append("/");

         struct dirent dirEntry;
         struct dirent* res = &dirEntry;
         while ((readdir_r(directory, &dirEntry, &res) == 0) && (res)) // thread-safe
             if ((dirEntry.d_type == DT_REG) &&
                 (strncmp(dirEntry.d_name + (d_namlen - endingString.size()), endingString.c_str(), endingString.length()) == 0) &&
                 (strcmp(dirEntry.d_name, ".") != 0) &&
                 (strcmp(dirEntry.d_name, "..") != 0))
                 if (getOnlyFileName)
                 {
                     foundFiles.push_back(dirEntry.d_name);
                 }
                 else
                 {
                     foundFiles.push_back(parent + dirEntry.d_name);
                 }
         closedir(directory);

         sort(foundFiles.begin(), foundFiles.end(), StringCompare_Smart_Incr());

         foundFiles.resize(Min(nMaxItems, (int)foundFiles.size()));

         return true;
     }

     return false;
#endif // _MSC_VER
 }
 long long  iospace::LexicalCast(const std::string& s)
 {
     std::stringstream ss(s);

     long long result;
     if ((ss >> result).fail() || !(ss >> std::ws).eof())
     {
         //throw std::bad_cast();
         cout << "ERROR:Impossible to cast " << s;
         getchar();
         exit(-1);
     }

     return result;
 }
 bool iospace::ExistsDirectory(const std::string &directory)
 {
     if (_access(directory.c_str(), 0) == 0)
     {
         struct stat status;
         stat(directory.c_str(), &status);

         if (status.st_mode & S_IFDIR)
         {
             return true;
         }

         //The path you entered is a file.
         return false;
     }

     return false;
 }

 void iospace::CreateDir(const std::string &directory)
 {
#ifdef WIN32
     CreateDirectory(directory.c_str(), NULL);
#elif defined (__GNUC__)
     /*  read/write/exe for owner
     read/exe for group owner
     read for other              */
     mkdir(directory.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH);
#elif 
#ifndef (BOOST_IS_NOT_INCLUDED)
     create_directory(path(directory.c_str()));
#endif
#endif
 }
 std::string iospace::GetPathDir(const string &filename)
 {
     string strFinal = filename;
     size_t posSlash = strFinal.rfind('/');
     size_t posBackSlash = strFinal.rfind('\\');
     if ((posSlash == string::npos) && (posBackSlash == string::npos))
     {
         return ".";
     }

     if (posSlash == string::npos)
     {
         return strFinal.substr(0, posBackSlash);
     }

     if (posBackSlash == string::npos)
     {
         return strFinal.substr(0, posSlash);
     }

     return strFinal.substr(0, (posSlash > posBackSlash) ? posSlash : posBackSlash);

 }
 void iospace::CreateFullPath(const std::string &directory)
 {
     string strDirectory = directory;
     if ((strDirectory[strDirectory.size() - 1] != ':') && (!ExistsDirectory(directory)))
     {
         string subDir = GetPathDir(directory);
         CreateFullPath(subDir.c_str());
         CreateDir(directory);
     }
 }
 std::string iospace::GetRelativeName(const  std::string &filename)
 {
     std::string strFinal = filename;
     size_t posSlash = strFinal.rfind('/');
     size_t posBackSlash = strFinal.rfind('\\');
     if ((posSlash == std::string::npos) && (posBackSlash == std::string::npos))
     {
         return strFinal;
     }

     if (posSlash == std::string::npos)
     {
         return strFinal.substr(posBackSlash + 1);
     }

     if (posBackSlash == std::string::npos)
     {
         return strFinal.substr(posSlash + 1);
     }

     return strFinal.substr(std::max(posSlash + 1, posBackSlash + 1));
 }
 bool iospace::ExistsFile(const std:: string &filename_abs)
 {
#ifdef _MSC_VER
     return !(GetFileAttributes(filename_abs.c_str()) == INVALID_FILE_ATTRIBUTES);
#else	
#ifndef BOOST_IS_NOT_INCLUDED
     return boost::filesystem::exists(filename_abs.c_str());
#else		
#ifdef __GNUC__
     //assume that on Linux the compiler is gcc 
     struct stat st;
     return (stat(filename_abs.c_str(), &st) == 0);
#else			
     cout << "ERROR (FileExists): unknown environment";
     getchar();
     exit(-1);

     return true;

#endif

#endif

#endif
 }
 template<typename T>
 void iospace::LoadSingleValue(const std::string &absFileName, T &value, const bool binary)
 {
     if (binary)
     {
        std::fstream binary_file(absFileName, ios::binary | ios::in);

         if (!binary_file.is_open())
         {
             cout << "ERROR: Impossible to open file " << absFileName << "\n";
             getchar();
             return;
         }

         binary_file.read((char*)(&value), sizeof(value));

         binary_file.close();
     }
     else
     {

         ifstream file;
         file.open(absFileName, ios_base::in);

         if (!file.is_open())
         {
             cout << "ERROR: Impossible to open file " << absFileName << "\n";
             getchar();
             return;
         }

         file >> value;

         file.close();
     }
 }
 template<typename T>
 void iospace::SaveSingleValue(const std::string &absFileName, T value, const bool binary )
 {
     if (binary)
     {
         fstream binary_file;

         binary_file.open(absFileName, ios::out | ios::binary); // |ios::binary

         if (!binary_file.is_open())
         {
             cout << "ERROR: Impossible to open file " << absFileName << "\n";
             getchar();
             return;
         }

         binary_file.write((char*)(&value), sizeof(value));
         binary_file.close();

     }
     else
     {

         ofstream file;
         file.open(absFileName, ios_base::out);

         if (!file.is_open())
         {
             cout << "ERROR: Impossible to open file " << absFileName << "\n";
             getchar();
             return;
         }

         file << value;

         file.close();
     }
 }
 bool iospace::StringCompare_Smart_Incr::operator() (const std::string& a, const std::string& b) const
 {

     unsigned posStr = 0;
     while ((posStr < a.size()) && (posStr < b.size()))
     {
         unsigned tkn_idx_a = (unsigned int)a.find_first_of("0123456789", posStr);
         unsigned tkn_idx_b = (unsigned int)b.find_first_of("0123456789", posStr);
         std::string suba = a.substr(posStr, tkn_idx_a - posStr);
         std::string subb = b.substr(posStr, tkn_idx_b - posStr);
         if (suba == subb)
         {
             //same substring

             if (tkn_idx_a == a.size())
             {
                 //end of a and at least of b
                 return true;
             }

             if (tkn_idx_a == a.size())
             {
                 //end of b but not of a
                 return false;
             }

             unsigned numberEnd_a = (unsigned int)a.find_first_not_of("0123456789", tkn_idx_a + 1);
             unsigned numberEnd_b = (unsigned int)b.find_first_not_of("0123456789", tkn_idx_b + 1);
             //check number
             long long number_a = LexicalCast(a.substr(tkn_idx_a, numberEnd_a - tkn_idx_a));
             long long number_b = LexicalCast(b.substr(tkn_idx_b, numberEnd_b - tkn_idx_b));
             //long number_a = std::atol(a.substr(tkn_idx_a).c_str());
             //long number_b = std::atol(b.substr(tkn_idx_b).c_str());
             if (number_a != number_b)
             {
                 return (number_a < number_b);
             }
         }
         else
         {
             //different substring
             return (suba < subb);
         }
         posStr = (unsigned int)a.find_first_not_of("0123456789", tkn_idx_a + 1);
     }

     return (a.size() < b.size());

 }
 //////
 //
 // Explicit template instantiations
 //
 template REG3D_API void iospace::LoadSingleValue<double>(const std::string &absFileName, double &value, const bool binary);
 template REG3D_API void iospace::LoadSingleValue<float>(const std::string &absFileName, float &value, const bool binary);
 template REG3D_API void iospace::LoadSingleValue<int>(const std::string &absFileName, int &value, const bool binary);

 template REG3D_API void iospace::SaveSingleValue<double>(const std::string &absFileName, double value, const bool binary);
 template REG3D_API void iospace::SaveSingleValue<float>(const std::string &absFileName, float value, const bool binary);
 template REG3D_API void iospace::SaveSingleValue<int>(const std::string &absFileName, int value, const bool binary);

