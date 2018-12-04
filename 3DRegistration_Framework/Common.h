#include"Config.h"

#pragma once

/*! used globally for error logging */
REG3D_API int BeginDataLog ();
REG3D_API void EndDataLog ();
REG3D_API void LogData (const char *szFormat, ...);
#define error_log LogData

/*! if set, all data files are output as text, else as binary data */
#define _TEXT_DATA

/*! infinity */
#define INF 1.0E10

/*! infinitesimal */
#define EPSILON	1E-6

// defines
#define MAX_STRLEN			256

#define TEMP_FOLDER			"temp"
#define LOG_FILE			"error.log"    //3DScanner
#define SETTINGS_FILE		"3DScanner.cfg"
#define CALIB_PATTERN_FILE	"Calibpattern.dat"
#define CALIB_PARAMETER_FILE "Calibparam.dat"
#define MST_ADJ_MAT_FILE	"AdjMat.dat"
#define D3D_SETTINGS_FILE	"Stereo_Visualizer.cfg"
#define D3D_DEFAULT_SCENE	"Stereo_Visualizer.scene"

#define SCAN_FILE_SIGNATURE	"scan"
#define SCAN_IMAGE_LIST_EXT	"ilf"
#define	SCAN_PCD_LIST_EXT	"plf"
#define SCAN_MESH_LIST_EXT	"mlf"
#define SCAN_KP_LIST_EXT	"klf"
#define SCAN_IMAGE_FOLDER	"images"
#define	SCAN_PCD_FOLDER		"pcd"
#define SCAN_MESH_FOLDER	"meshes"
#define SCAN_KP_FOLDDER		"keypoints"
#define SCAN_VIEW_FOLDER	"view"
#define SCAN_VIEW_CFG_FILE	"view.cfg"
#define SCAN_VIEW_SHOT_IMG	"viewshot"

#define MAX_SHOTS_PER_VIEW		1000
#define MAX_PARALLEL_THREADS	12
#define LOGDATA                LogData( "outputprints.log")