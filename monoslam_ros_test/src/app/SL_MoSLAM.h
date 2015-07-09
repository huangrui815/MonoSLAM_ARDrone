/*
 * SL_MoSLAM.h
 *
 *  Created on: 2011-1-3
 *      Author: Danping Zou
 */

#ifndef COSLAM_H_
#define COSLAM_H_
#include "SL_CameraTracker.h"
#include "SL_Relocalization.h"
#include "SL_PointRegistration.h"
#include "SL_Bundler.h"
#include "SL_PBA.h"
#include "SL_MapMaker.h"

#include "math/SL_Matrix.h"
#include "slam/SL_Define.h"
#include "slam/SL_MapPoint.h"
#include "slam/SL_KeyFrame.h"

#include "SL_MoSLAMRobustBA.h"
//#include <LinearMath/btTransform.h>
#include <tf2/LinearMath/Transform.h>

#include <deque>
#include <map>
#include <list>
#include <tf/transform_broadcaster.h>

class DroneKalmanFilter;

using namespace std;

#define SLAM_STATE_READY 0
#define SLAM_STATE_MAP_INIT 1
#define SLAM_STATE_NORMAL 2
#define SLAM_STATE_RELOCATE 3
#define SLAM_STATE_LOST -1

class MoSLAM {
public:
	int curFrame; //current video frame
	int state;
	
	/* current received command*/
	int cmd;
	
	MoSLAM();
	~MoSLAM();
	
public:
	/* camera tracker*/
	CameraTracker tracker;
	/* relocalization*/
	Relocalizer relocalizer;
	/* for bundle adjustment*/
	Bundler bundler;
	PBABundler pbaBundler;
	/* for point registration*/
	PointRegister ptRegister;
	/* for generating new 3D points*/
	MapMaker mpMaker;

	/* map points*/
	list<MapPoint*> allMapPts;	//store all map points generated
	list<MapPoint*> actMapPts;	//active map points for registration
	list<MapPoint*> curMapPts;	//store currently tracked map points

	/* camera poses*/
	CamPoseList camPose;
	KeyFrameList keyFrm;

	/* key frames*/
	vector<KeyFrame*> actKeyFrm;

	int frmNumAfterReloc;
	int firstFrmForBA;
	int lastFrmAddKeyFrm;

	// predicted camera pose from the state kept in filter
	tf::Transform _predictedWorldToCamTf;

#ifdef ARDRONE
	std::vector<MyNavData> m_navdata;
#endif

	int nActMapPts;
	//timings
	map<int, double> _frmId2tmPerStep;
	double m_tmPerStep;
	double m_tmReadFrame;
	double m_tmFeatureTracking;
	double m_tmPoseUpdate;
	double m_tmCameraGrouping;
	double m_tmNewMapPoints;
	double m_tmIntraCamMapping;
	double m_tmMapClassify;
	double m_tmCurMapRegister;
	double m_tmActMapRegister;
public:
	//on-line version by Rui Jan 17
	void setCam(const char* calFilePath);

	//for on-line access
	void setVideoReader(VideoReader* vr) {
		tracker.videoReader = vr;
	}
	bool openVideoReader() {
		return tracker.videoReader->open();
	}

	void setVideo(const char* videoFilePath, const char* calFilePath,
			int startFrame = 0);

	void init(bool offline);
	void readFrame();
	void grabReadFrame();

	void releaseAllMapPoints();
	void reset();

	/*map initialization*/
	bool mapInitOffline();
	bool mapInitOnline();

	void refineInitMap(const Mat_d& K, Mat_d& R1, Mat_d& t1, Mat_d& R2,
			Mat_d& t2, Mat_d& pts1, Mat_d& pts2, Mat_d& Ms);

	/*feature tracking*/
	int featureTracking();

	/*pose estimation*/
	bool estimateCamPose();

	CamPoseItem* addCameraPose(int f, const double* R, const double* t);

	void searchNearestKeyFrm(vector<KeyFrame*>& vecKeyFrm, int nKeyFrm);

	void getActiveMapPts(std::list<MapPoint*>& pts);
	void mapStateUpdate();

	/* check if the number of feature points with corresponding map points decrease
	 * significantly from the last key frame.*/
	bool isNumMappedPtsDropBelow(double ratio);
	/* check if the #camId-th camera have already to insert a new key pose*/
	int isReadyForKeyFrame();

	bool addNewKeyFrame();

	/*key frame matching & bundle adjustment*/
	std::deque<RobustBundleRTS*> m_requestBAs;

	int getCurFrmOffline() const {
		return tracker.startFrameInVideo + curFrame + 1;
	}
	
	void startMapInit(tf::Transform& firstPredictedCamPose);
	void startMapInit();

	void endMapInit(tf::Transform& secondPredictedCamPose);
	void endMapInit();

	void startRelocalization();
	bool doRelocalization();
	bool doRelocalization_online();
	void endRelocalization();

	void processOneFrame();
	int processOneFrameNew();
	void setPredictedCamPose(tf::Transform predictedPose);
public:
	//for debug
	void pause();

	//for debug
	void exportResults(const char timeStr[]) const;
	//void saveCurrentFrame(const char timeStr[]) const;
	void saveCurrentImages(const char* dirPath) const;
};

#endif /* COSLAM_H_ */
