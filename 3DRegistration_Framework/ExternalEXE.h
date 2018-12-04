#pragma once
#include<string>
#include <iostream> 
#include <sstream>
#include"Config.h"
// WinAPI
#include <windows.h>
#define NOMINMAX 1

#define OSR_BINARY "Z:/staff/SDutta/Online_Surface_Reconstruction.exe"

REG3D_API std::string formatWinErrorMsg(DWORD errorCode);
REG3D_API bool runFGR();
REG3D_API std::string toString(float number);
REG3D_API std::string toString(int number);
REG3D_API std::string toString(unsigned int number);