#include"pch.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include "Common.h"

FILE	*g_pLogFile = fopen(LOG_FILE, "ab");

int BeginDataLog ()
{
	if (NULL == g_pLogFile)
	{
		// open log file in append mode (to keep previous run logs)
		g_pLogFile = fopen (LOG_FILE, "ab");

		if (NULL != g_pLogFile)
		{
			// timestamp the beginning of the current run
			time_t ltime; /* calendar time */
			ltime = time (NULL); /* get current cal time */
			fprintf (g_pLogFile, "%s\r\n", asctime (localtime (&ltime)));
		}
	}

	return (NULL == g_pLogFile);
}

void EndDataLog ()
{
	if (NULL != g_pLogFile)
	{
		fclose (g_pLogFile);
		g_pLogFile = NULL;
	}
}

void LogData (const char *szFormat, ...)
{
#ifdef LOGDATA
	if (NULL == g_pLogFile)
	{
		int fResult = BeginDataLog ();
		if (0 != fResult)
			return;
	}
	va_list args;
	va_start(args, szFormat);
	vfprintf(g_pLogFile, szFormat, args);
	va_end(args);
	fflush (g_pLogFile);
#endif
}
