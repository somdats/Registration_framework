#pragma once
#ifdef REG3DFRAMEWORK_EXPORTS
#define REG3D_API __declspec(dllexport)
#else
#define REG3D_API __declspec(dllimport)
#endif