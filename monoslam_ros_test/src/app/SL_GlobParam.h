/*
 * SL_GlobParam.h
 *
 *  Created on: 2011-7-26
 *      Author: zou
 */
#ifndef SL_GLOBPARAM_H_
#define SL_GLOBPARAM_H_
#include <string>
#include <vector>

#ifdef WIN32
#include <Windows.h>
#define RESULT_PATH "C:/slam_results"
#else
#define RESULT_PATH "/home/rui/workspace/myROS/monoslam_ros_test/slam_results"
#endif


using namespace std;
class SLAMParam {
public:
	/* save images*/
	static bool saveImage;
	/* measure timings*/
	static bool measureTimings;
	/* number of cameras used*/
	static int camNum;
	/* number of all frames in a video sequence*/
	static int frmNumTotal;
	/* maximum number of video frames*/
	static int frmNumForRun;
	/* number of video frames to be skipped*/
	static int frmNumForSkip;
	/* number of frames used for initialization in single camera model*/
	static int frmNumForInit;
	/* number of frames to call bundle adjustment, if < 0, no bundle adjustment is called*/
	static int frmNumForAddKeyFrm;
	/* number of frames not to track*/
	static int frmNumNoTracking;
	/* number of frames for statics */
	static int frmNumForStatisticFeatNum;
	static int frmNumForStatisticKeyFrm;
	
	/* number of nearest key frames*/
	static int nNearestKeyFrm;
	
	/* number of frames to determine outliers*/
	static int frmNumOutlier;
	static int frmNumAfterRelocalization;

	/* maxinum number of point registration*/
	static int maxRegTryNum;
	/* maximum ratio of points lying at the back of camera*/
	static double maxCameraBackRatio;
	/* map reduce ratio*/
	static double mapReduceRatio;
	
	/* minimum base line width to insert a new key frame*/
	static double baseLineRatio;
	static double minMappedRatio;
	/*ratio to determine whether bad tracking happens*/
	static double badTrackingRatio;
	static double SMALL_REPROJECT_ERR;
	/*maximum re-projection error*/
	static double MAX_REPROJECT_ERR;
	/*range for point regirstration*/
	static double POINT_REG_RANGE;
	/*maximum epipolar error*/
	static double MAX_EPI_ERR;
	/*feature detection variance*/
	static double DETECT_ERR_VAR;
	/*minimum triangulation angle*/
	static double MIN_TRI_ANGLE;
	/*maximum cost for seeking the similar key frame in re-localization*/
	static double MAX_COST_KEY_FRM;

	/* KLT parameters for tracking*/
	static bool klt_trackWithGain;
	static int klt_nLevels;
	static int klt_windowWidth;
	static float klt_minCornerness; //*feature detection
	static float klt_convergeThreshold;
	static float klt_SSDThreshold; //*tracking

	static string videoFilePath;
	static string calFilePath;
	static string resultPath;
	static int videoOffset;
};

std::string getPrenameForSave();
void saveParameters();
#endif /* SL_GLOBPARAM_H_ */
