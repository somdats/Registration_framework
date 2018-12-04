#include"pch.h"
#include"ExternalEXE.h"

REG3D_API std::string formatWinErrorMsg(DWORD errorCode)
{
    char msg[1024];
    FormatMessage(
        FORMAT_MESSAGE_FROM_SYSTEM, nullptr, errorCode, NULL, msg, 1024, nullptr
    );
    return msg;
}

REG3D_API bool runFGR()
{
    // WinAPI stuff
    STARTUPINFO si = {
        /* .cb = */              sizeof(STARTUPINFO),
        /* .lpReserved = */      nullptr,
        /* .lpDesktop = */       nullptr,
        /* .lpTitle = */         nullptr,
        /* .dwX = */             NULL,
        /* .dwY = */             NULL,
        /* .dwXSize = */         NULL,
        /* .dwYSize = */         NULL,
        /* .dwXCountChars = */   NULL,
        /* .dwYCountChars = */   NULL,
        /* .dwFillAttribute = */ NULL,
        /* .dwFlags = */         NULL,
        /* .wShowWindow = */     NULL,
        /* .cbReserved2 = */     NULL,
        /* .lpReserved2 = */     nullptr,
        /* .hStdInput = */       nullptr,
        /* .hStdOutput = */      nullptr,
        /* .hStdError = */       nullptr
    }; PROCESS_INFORMATION pi;

    // Try to invoke executable
    if (!CreateProcess(OSR_BINARY, nullptr, nullptr, nullptr,
        false, NULL, nullptr, nullptr, &si, &pi))
    {
        DWORD errCode = GetLastError();
        std::cout << "ERROR [" << errCode << "]:" << std::endl
            << formatWinErrorMsg(errCode) << std::endl;
        return false;
    }
    WaitForSingleObject(pi.hProcess, INFINITE);
    return true;
}

REG3D_API std::string toString(float number)
{
    std::ostringstream strm; strm.precision(3);
    strm << number;
    return std::move(strm.str());
}

REG3D_API std::string toString(int number)
{
    std::ostringstream strm;
    strm << number;
    return std::move(strm.str());
}

REG3D_API std::string toString(unsigned int number)
{
    std::ostringstream strm;
    strm << number;
    return std::move(strm.str());
}