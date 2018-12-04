#pragma once
#include<string>
#include"Datatypes.h"
#include"Config.h"

namespace IO
{
    class REG3D_API iFileIO
    {
    public:
       virtual int loadFile(const std::string &fileNameWithLocation, CloudWithoutType &inputCloud) = 0;
       virtual int writeFile(const std::string & outputFileWithLocation, CloudWithoutType &outputCloud) = 0;
    };
}
