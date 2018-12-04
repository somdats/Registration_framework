#include"pch.h"
#include"CpuTimeProfiler.h"


#ifdef __GNUC__
Pairwise3DRegistrationEvaluation::CpuTimeProfiler::CpuTimeProfiler(clockid_t clk_id)
    : m_clk_id(clk_id)
#else
CpuTimeProfiler::CpuTimeProfiler()
#endif
{
#if defined (_MSC_VER)
    QueryPerformanceFrequency(&m_frequency);
#endif
    GetActualTime();
}

void CpuTimeProfiler::GetActualTime()
{
#if defined (_MSC_VER)
    GetProcessTimes(GetCurrentProcess(), &m_actualCreateTime, &m_actualExitTime, &m_actualKernelTime, &m_actualUserTime);

    ULARGE_INTEGER actualkernel, actualUser;
    actualkernel.LowPart = m_actualKernelTime.dwLowDateTime;
    actualkernel.HighPart = m_actualKernelTime.dwHighDateTime;

    actualUser.LowPart = m_actualUserTime.dwLowDateTime;
    actualUser.HighPart = m_actualUserTime.dwHighDateTime;

    m_actualTime.QuadPart = actualkernel.QuadPart + actualUser.QuadPart;
#elif defined __GNUC__
    clock_gettime(m_clk_id, &m_actualTime);
#endif
}

double CpuTimeProfiler::GetElapsedHours()
{
    return GetElapsedMins() / 60.0;
}

double CpuTimeProfiler::GetElapsedMins()
{
    return GetElapsedSecs() / 60.0;
}

double CpuTimeProfiler::GetElapsedSecs()
{
#if defined (_MSC_VER)
    FILETIME stopUserTime, stopKernelTime, stopCreateTime, stopExitTime;
    GetProcessTimes(GetCurrentProcess(), &stopCreateTime, &stopExitTime, &stopKernelTime, &stopUserTime);

    ULARGE_INTEGER stopKernel, stopUser, elapsed;
    stopUser.LowPart = stopUserTime.dwLowDateTime;
    stopUser.HighPart = stopUserTime.dwHighDateTime;

    stopKernel.LowPart = stopKernelTime.dwLowDateTime;
    stopKernel.HighPart = stopKernelTime.dwHighDateTime;

    elapsed.QuadPart = stopUser.QuadPart + stopKernel.QuadPart - m_actualTime.QuadPart;

    // the output of GetProcessTimes is expressed in 100-nanosecond time units so we have to divide by 10'000'000 to obtain sec
    return (double)elapsed.QuadPart / 10000000.0;
#elif defined __GNUC__
    return GetElapsedMilli() / 1000;
#else
    return 0;
#endif
}

double CpuTimeProfiler::GetElapsedMilli()
{
#if defined (_MSC_VER)
    FILETIME stopUserTime, stopKernelTime, stopCreateTime, stopExitTime;
    GetProcessTimes(GetCurrentProcess(), &stopCreateTime, &stopExitTime, &stopKernelTime, &stopUserTime);

    ULARGE_INTEGER stopKernel, stopUser, elapsed;
    stopUser.LowPart = stopUserTime.dwLowDateTime;
    stopUser.HighPart = stopUserTime.dwHighDateTime;

    stopKernel.LowPart = stopKernelTime.dwLowDateTime;
    stopKernel.HighPart = stopKernelTime.dwHighDateTime;

    elapsed.QuadPart = stopUser.QuadPart + stopKernel.QuadPart - m_actualTime.QuadPart;

    // the output of GetProcessTimes is expressed in 100-nanosecond time units so we have to divide by 10'000 to obtain ms
    return (double)elapsed.QuadPart / 10000.0;
#elif defined __GNUC__
    timespec stop;
    clock_gettime(m_clk_id, &stop);
    return (difftime(stop.tv_sec, m_actualTime.tv_sec) * 1000.0 + (double)(stop.tv_nsec - m_actualTime.tv_nsec) * 1e-6);
#else
    return 0;
#endif
}