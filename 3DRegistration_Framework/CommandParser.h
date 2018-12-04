#pragma once
#include "Config.h"
#include<pcl/console/parse.h>
#include<pcl/console/print.h>

namespace parser
{
    REG3D_API  std::string source_filename_;  // to be aligned
    REG3D_API std::string target_filename_;  // to align with
    REG3D_API std::string target_pctfile_;  // to align with(pct file)
    REG3D_API bool batch_mode; // batch processing
    REG3D_API std:: string output_dir;
    REG3D_API std::string  output_transformation_matrix;
   

   REG3D_API void showHelp(char *fileName);
   REG3D_API void ParseCommandLine(int argc, char *argv[]);
   REG3D_API int BatchProcess(int argc, char *argv[], std::vector<std::string> plyFiles = { std::string ("ply")},  std::vector<std::string> pctFiles = { std::string("pct")});
   
}
