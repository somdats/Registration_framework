#pragma once
#include"Config.h"
#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#define NOMINMAX
#include "windows.h"
#endif

#ifdef __GNUC__
#include <time.h>
#include <sys/times.h>
#endif // __GNUC__


class REG3D_API CpuTimeProfiler
{
protected:
#if defined (_MSC_VER)
    LARGE_INTEGER m_frequency;

    ULARGE_INTEGER m_actualTime;
    FILETIME m_actualUserTime;
    FILETIME m_actualKernelTime;
    FILETIME m_actualCreateTime;
    FILETIME m_actualExitTime;
#elif defined __GNUC__
    clockid_t m_clk_id;
    timespec m_actualTime;
#endif

public:
#ifdef __GNUC__
    /// Typical choices: CLOCK_MONOTONIC, CLOCK_PROCESS_CPUTIME_ID, CLOCK_THREAD_CPUTIME_ID
    CpuTimeProfiler(clockid_t clk_id = CLOCK_PROCESS_CPUTIME_ID);
#else
    CpuTimeProfiler();
#endif
    double GetElapsedHours();
    double GetElapsedMins();
    double GetElapsedSecs();
    double GetElapsedMilli();
    void GetActualTime();
};
