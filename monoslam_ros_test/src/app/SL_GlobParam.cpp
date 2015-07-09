/*
 * SL_GlobParam.cpp
 *
 *  Created on: 2011-7-26
 *      Author: zou
 */

#include "SL_GlobParam.h"
#include "SL_error.h"
#include <fstream>
#include <iostream>
#include <sstream>

bool SLAMParam::saveImage = false;
bool SLAMParam::measureTimings = false;

int SLAMParam::camNum = 0;
int SLAMParam::frmNumTotal = 0;
int SLAMParam::frmNumForRun = -1;
int SLAMParam::frmNumForSkip = 100;
int SLAMParam::frmNumForInit = 50;
int SLAMParam::frmNumForAddKeyFrm = 15;
int SLAMParam::frmNumNoTracking = 10;
int SLAMParam::frmNumForStatisticFeatNum = 500;
int SLAMParam::frmNumForStatisticKeyFrm = 20;
int SLAMParam::nNearestKeyFrm = 5;
int SLAMParam::frmNumOutlier = 10;
int SLAMParam::frmNumAfterRelocalization = 50;
int SLAMParam::maxRegTryNum = 200;
double SLAMParam::maxCameraBackRatio = 0.2;
double SLAMParam::mapReduceRatio = 0.8;
double SLAMParam::baseLineRatio = 0.5;
double SLAMParam::minMappedRatio = 0.08;
double SLAMParam::badTrackingRatio = 0.5; //0.85;
double SLAMParam::SMALL_REPROJECT_ERR = 5.0;
double SLAMParam::MAX_REPROJECT_ERR = 10.0;
double SLAMParam::POINT_REG_RANGE = 30.0;
double SLAMParam::MAX_EPI_ERR = 6.0;
double SLAMParam::DETECT_ERR_VAR = 2.0; //2 pixels error
double SLAMParam::MIN_TRI_ANGLE = 5.0;
double SLAMParam::MAX_COST_KEY_FRM = 100; // threshold for key frame matching

bool SLAMParam::klt_trackWithGain = true;
int SLAMParam::klt_nLevels = 5;
int SLAMParam::klt_windowWidth = 5;
float SLAMParam::klt_minCornerness = 200;
float SLAMParam::klt_convergeThreshold = 4.0f;
float SLAMParam::klt_SSDThreshold = 8000;

string SLAMParam::videoFilePath;
string SLAMParam::calFilePath;
string SLAMParam::resultPath;
int SLAMParam::videoOffset;

using namespace std;
static void _saveParameters(const char* filepath) {
	ofstream file(filepath);
	if (!file)
		repErr("cannot open '%s' to write!\n", filepath);

	file << "saveImage:" << (SLAMParam::saveImage ? 1 : 0) << endl;
	file << "measureTimings:" << (SLAMParam::measureTimings ? 1 : 0) << endl;
	file << "camNum:" << SLAMParam::camNum << endl;
	file << "frmNumTotal:" << SLAMParam::frmNumTotal << endl;
	file << "frmNumForRun:" << SLAMParam::frmNumForRun << endl;
	file << "frmNumForSkip:" << SLAMParam::frmNumForSkip << endl;
	file << "frmNumForInit:" << SLAMParam::frmNumForInit << endl;
	file << "frmNumForBA:" << SLAMParam::frmNumForAddKeyFrm << endl;

	file << endl;
	file << "MAX_REPROJECT_ERR:" << SLAMParam::MAX_REPROJECT_ERR << endl;
	file << "MAX_EPI_ERR:" << SLAMParam::MAX_EPI_ERR << endl;
	file << "DETECT_ERR_VAR:" << SLAMParam::DETECT_ERR_VAR << endl;
	file << endl;

	file << "klt_trackWithGain:" << (SLAMParam::klt_trackWithGain ? 1 : 0) << endl;
	file << "klt_nLevels:" << SLAMParam::klt_nLevels << endl;
	file << "klt_windowWidth:" << SLAMParam::klt_windowWidth << endl;
	file << "klt_minCornerness:" << SLAMParam::klt_minCornerness << endl;
	file << "klt_convergeThreshold:" << SLAMParam::klt_convergeThreshold << endl;
	file << "klt_SSDThreshold:" << SLAMParam::klt_SSDThreshold << endl;

	file << "input video:" << SLAMParam::videoFilePath << endl;
	file << "calibration file:" << SLAMParam::calFilePath << endl;
	file << "result path:" << SLAMParam::resultPath << endl;
}

std::string getPrenameForSave() {
	string prename;
	if (SLAMParam::videoFilePath != "") {
		size_t pos = SLAMParam::videoFilePath.find_last_of('.');
		if (pos != string::npos)
			prename = SLAMParam::videoFilePath.substr(0, pos);
	}
	if (prename == "") {
#ifdef WIN32
		prename = "c:/slam";
#else
		prename = "~/slam";
#endif
	}
	return prename;
}
void saveParameters() {
	string prename = getPrenameForSave();
	ostringstream oss;
	oss << prename << "_param.txt";
	string filepath = oss.str();
	_saveParameters(filepath.c_str());
}
