/*
 * SL_MoSLAM.cpp
 *
 *  Created on: 2011-1-3
 *      Author: Danping Zou
 */
#include "SL_MoSLAM.h"
#include "SL_GlobParam.h"
#include "SL_Relocalization.h"

#include "slam/SL_MoSLAMHelper.h"

#include "app/APP_MyApp.h"
#include "app/APP_SynObj.h"

#include "tools/SL_Tictoc.h"
#include "tools/GUI_ImageViewer.h"
#include "tools/SL_Print.h"
#include "videoReader/VR_AVIReader.h"

#include "geometry/SL_Triangulate.h"
#include "geometry/SL_Geometry.h"
#include "imgproc/SL_ImageIO.h"

#include "calibration/SL_CalibTwoCam.h"

#include <cassert>
#include <set>

MoSLAM::MoSLAM() :
		curFrame(0), state(SLAM_STATE_READY), tracker(curFrame, camPose,
				keyFrm), relocalizer(*this), bundler(*this), ptRegister(*this), mpMaker(
				*this), frmNumAfterReloc(-1), firstFrmForBA(0), lastFrmAddKeyFrm(
				0) {
	tracker.setSLAM(this);
	pbaBundler.setSLAM(this);
}

MoSLAM::~MoSLAM() {
	//clear the queued BAs
	while (!m_requestBAs.empty()) {
		//start of critical section
		enterBACriticalSection();
		RobustBundleRTS* pBA = m_requestBAs.back();
		m_requestBAs.pop_back();
		leaveBACriticalSection();
		//end of critical section
		delete pBA;
	}
}
//Online version
void MoSLAM::setCam(const char* calFilePath) {
	tracker.calFilePath = calFilePath;
}

//Offline version
void MoSLAM::setVideo(const char* videoFilePath, const char* calFilePath,
		int startFrame) {
	tracker.videoFilePath = videoFilePath;
	tracker.calFilePath = calFilePath;
	tracker.startFrameInVideo = startFrame;
}

void MoSLAM::init(bool offline) {
	int minFrmNum = 0;
	if (offline) {
		//skip frames
		((AVIReader*) tracker.videoReader)->filePath = tracker.videoFilePath;
		tracker.videoReader->open();
//		tracker.videoReader->skip(
//				tracker.startFrameInVideo + SLAMParam::frmNumForSkip);
		if (minFrmNum < tracker.videoReader->getTotalFrame())
			minFrmNum = tracker.videoReader->getTotalFrame();
		tracker.startFrameInVideo += SLAMParam::frmNumForSkip;
	}
	tracker.videoReader->grabFrame();
	tracker.W = tracker.videoReader->_w;
	tracker.H = tracker.videoReader->_h;
	//////////////////////////////////////////////////////////
	tracker.rgb.resize(tracker.videoReader->_w, tracker.videoReader->_h);
	tracker.img.resize(tracker.videoReader->_w, tracker.videoReader->_h);

	tracker.K.resize(3, 3);
	tracker.kDist.resize(5, 1);
	readIntrinDistParam(tracker.calFilePath.c_str(), tracker.K, tracker.kDist);
	tracker.invK.resize(3, 3);
	tracker.kUnDist.resize(7, 1);

	matInv(3, tracker.K.data, tracker.invK.data);
	invDistorParam(tracker.videoReader->_w, tracker.videoReader->_h,
			tracker.invK.data, tracker.kDist.data, tracker.kUnDist.data);
	SLAMParam::frmNumTotal = minFrmNum;

	tracker.initTracker();
}
void MoSLAM::readFrame() {
	tracker.readFirstFrame();
}
void MoSLAM::grabReadFrame() {
	tracker.grabReadFrame();
	curFrame++;
}
void MoSLAM::releaseAllMapPoints() {
	typedef std::list<MapPoint*>::iterator MapPointListIter;
	for (MapPointListIter iter = allMapPts.begin(); iter != allMapPts.end();
			iter++) {
		MapPoint* mpt = *iter;
		delete mpt;
	}
	allMapPts.clear();
}
void MoSLAM::reset() {
	releaseAllMapPoints();
	tracker.reset();
	curMapPts.clear();
	actMapPts.clear();
	camPose.clear();
	keyFrm.clear();
	state = SLAM_STATE_READY;
}
//#define FOR_DEBUG
bool MoSLAM::mapInitOffline() {
	//1.tracking feature points for a set of frames
//	int nFeatPts = tracker.trackFeaturePoints(); // detect the frame skipped number + 1
//	if (nFeatPts <= 0)
//		repErr("not feature point is detected at frame %d.",
//				getCurFrmOffline());

	int nFeatPts = 0;
	curFrame = 0;
	int curFrame0 = curFrame;

	readFrame();
	nFeatPts = tracker.trackFeaturePoints();
	printf("frame %d, nFeatPts %d\n", curFrame, nFeatPts);
	double R0[9], t0[3];
	matEyes(3, R0);
	matZeros(3, 1, t0);
	camPose.add(curFrame0, tracker.timeStamp, R0, t0);
	tracker.updateCamParamForFeatPts(tracker.K, camPose.current());
	tracker.addKeyFrame();

	for (int f = curFrame + 1; f < curFrame0 + SLAMParam::frmNumForInit; f++) {
		tracker.grabReadFrame();
		nFeatPts = tracker.trackFeaturePoints();
		curFrame = f;
		printf("frame %d, nFeatPts %d\n", curFrame, nFeatPts);
//		else if (f == curFrame0 + SLAMParam::frmNumForInit - 1){
//			nFeatPts = tracker.trackFeaturePoints();
//		}
//		curFrame++;

		updateDisplayData();

		if (nFeatPts <= 0)
			repErr("not feature point is detected at frame %d.",
					getCurFrmOffline());
	}

	FILE* pFile = fopen("debug01.txt", "w");
	vector<TrackedFeaturePoint*> vecPts1, vecPts2; // feature points
	for (int i = 0; i < tracker.featTracker.maxTrackNum_; i++) {
		Track2D& tk = tracker.featTracker.tracks[i];
		if (tk.empty() || !tk.valid())
			continue;
		fprintf(pFile, "%d %d\n", tk.f1, tk.f2);
		if (tk.f1 == curFrame0 && tk.f2 == curFrame) {
			vecPts2.push_back(tk.tail);
			vecPts1.push_back(tk.head.next);
		}
	}
	fclose(pFile);
	printf("long tracks size: %d\n", vecPts1.size());
	//2.estimate the initial camera poses
	Mat_d matPts1, matPts2;
	trackedFeatPoint2Mat(vecPts1, matPts1);
	trackedFeatPoint2Mat(vecPts2, matPts2);

	CalibTwoCam calib;
	calib.setIntrinParam(tracker.K.data, tracker.K.data);
	calib.setMatchedPoints(matPts1, matPts2);
//	calib.estimateEMatOld(2.0, CV_FM_RANSAC);
	calib.estimateEMat();

	vector<int> inlierInd;
	calib.getInlierInd(inlierInd);

	Mat_d R1, t1, R2, t2, pts1, pts2;
	calib.outputInlierNormPoints(pts1, pts2);
	calib.outputRTs(R1, t1, R2, t2);

	int npts = pts1.m;

	//3. triangulate initial map points
	Mat_d Ms(npts, 3);
	binTriangulatePoints(R1, t1, R2, t2, npts, pts1.data, pts2.data, Ms.data);

	//4.refine the initialization by bundle adjustment
	calib.outputInlierPoints(pts1, pts2);
	refineInitMap(tracker.K, R1, t1, R2, t2, pts1, pts2, Ms);

	//5. check if there are lots of points at the back of camera, which indicates 
	//bad estimation caused by camera rotation (not enough translation) 
	Mat_uc flag(npts, 1);
	int nCameraBack = 0;
	for (int i = 0; i < npts; i++) {
		if (isAtCameraBack(R1, t1, Ms.data + 3 * i)
				|| isAtCameraBack(R2, t2, Ms.data + 3 * i)) {
			flag[i] = 1;
			nCameraBack++;
		} else
			flag[i] = 0;
	}

	printf("npts: %d, nCameraBack: %d\n", npts, nCameraBack);
	if (nCameraBack > npts * SLAMParam::maxCameraBackRatio) {
		//bad estimation
		reset();
		return false;
	}

	for (int i = 0; i < npts; i++) {
		if (flag[i] > 0)
			continue;
		MapPoint* pM = new MapPoint(Ms.data[3 * i], Ms.data[3 * i + 1],
				Ms.data[3 * i + 2], curFrame);
		curMapPts.push_back(pM);
		allMapPts.push_back(pM);
		pM->state = STATE_MAPPOINT_CURRENT;
		getBinTriangulateCovMat(tracker.K, R1, t1, tracker.K, R2, t2, pM->M,
				pM->cov, SLAMParam::DETECT_ERR_VAR);

		int featId = inlierInd[i];
		assert(vecPts1[featId]->ptInKeyFrm);
		pM->addFeaturePoint(vecPts1[featId]->ptInKeyFrm);
		for (TrackedFeaturePoint* pm = vecPts1[featId]; pm; pm = pm->pre)
			pm->pt.mpt = pM;

		for (TrackedFeaturePoint* pm = vecPts2[featId]; pm; pm = pm->pre)
			pm->pt.mpt = pM;
	}

	//6. add key frame
	camPose.add(curFrame, tracker.timeStamp, R2.data, t2.data);
	tracker.updateCamParamForFeatPts(tracker.K, camPose.current());
	tracker.addKeyFrame();
	lastFrmAddKeyFrm = curFrame;

	state = SLAM_STATE_NORMAL;
	return true;
}
bool MoSLAM::mapInitOnline() {
	//1.tracking feature points for a set of frames
	tracker.grabReadFrame();
	int nFeatPts = tracker.trackFeaturePoints();
	if (nFeatPts <= 0) {
		reset();
		MyApp::bStartInit = true;
		return false;
	}

	int curFrame0 = curFrame;

	//add the start frame as  a key frame
	double R0[9], t0[3];
	matEyes(3, R0);
	matZeros(3, 1, t0);
	addCameraPose(curFrame0, R0, t0);
	tracker.updateCamParamForFeatPts(tracker.K, camPose.current());
	tracker.addKeyFrame();

	for (int f = curFrame0; MyApp::bStartInit != false; f++) {

		tracker.grabReadFrame();
		nFeatPts = tracker.trackFeaturePoints();
		curFrame++;
		updateDisplayData();

		if (nFeatPts <= 0) {
			reset();
			MyApp::bStartInit = true;
			return false;
		}
	}

	vector<TrackedFeaturePoint*> vecPts1, vecPts2; // feature points
	for (int i = 0; i < tracker.featTracker.maxTrackNum_; i++) {
		Track2D& tk = tracker.featTracker.tracks[i];
		if (tk.empty())
			continue;
		if (tk.f1 == curFrame0 && tk.f2 == curFrame) {
			assert(tk.head.next->ptInKeyFrm);
			vecPts1.push_back(tk.head.next);
			vecPts2.push_back(tk.tail);
		}
	}

	if (vecPts1.empty()) {
		reset();
		MyApp::bStartInit = true;
		return false;
	}
	//2.estimate the initial camera poses
	Mat_d matPts1, matPts2;
	trackedFeatPoint2Mat(vecPts1, matPts1);
	trackedFeatPoint2Mat(vecPts2, matPts2);

#ifdef FOR_DEBUG
	imwrite(tracker.m_img, "D:/for_debug/img_02.png");
	writeMat(matPts1, "D:/for_debug/pts1.txt");
	writeMat(matPts2, "D:/for_debug/pts2.txt");
#endif

	CalibTwoCam calib;
	calib.setIntrinParam(tracker.K.data, tracker.K.data);
	calib.setMatchedPoints(matPts1, matPts2);
	calib.estimateEMat(SLAMParam::MAX_EPI_ERR);

	vector<int> inlierInd;
	calib.getInlierInd(inlierInd);

	Mat_d R1, t1, R2, t2, pts1, pts2;
	calib.outputInlierNormPoints(pts1, pts2);
	calib.outputRTs(R1, t1, R2, t2);

	int npts = pts1.m;

	//3. triangulate initial map points
	Mat_d Ms(npts, 3);
	binTriangulatePoints(R1, t1, R2, t2, npts, pts1.data, pts2.data, Ms.data);

	//4.refine the initialization by bundle adjustment
	calib.outputInlierPoints(pts1, pts2);
	refineInitMap(tracker.K, R1, t1, R2, t2, pts1, pts2, Ms);

	//5. check if there are lots of points at the back of camera, which indicates 
	//bad estimation caused by camera rotation (not enough translation) 
	Mat_uc flag(npts, 1);
	int nCameraBack = 0;
	for (int i = 0; i < npts; i++) {
		if (isAtCameraBack(R1, t1, Ms.data + 3 * i)
				|| isAtCameraBack(R2, t2, Ms.data + 3 * i)) {
			flag[i] = 1;
			nCameraBack++;
		} else
			flag[i] = 0;
	}

	if (nCameraBack > npts * SLAMParam::maxCameraBackRatio) {
		//bad estimation
		reset();
		MyApp::bStartInit = true;
		return false;
	}

	for (int i = 0; i < npts; i++) {
		if (flag[i] > 0)
			continue;

		MapPoint* pM = new MapPoint(Ms.data[3 * i], Ms.data[3 * i + 1],
				Ms.data[3 * i + 2], curFrame);

		pM->state = STATE_MAPPOINT_CURRENT;
		curMapPts.push_back(pM);
		allMapPts.push_back(pM);

		getBinTriangulateCovMat(tracker.K, R1, t1, tracker.K, R2, t2, pM->M,
				pM->cov, SLAMParam::DETECT_ERR_VAR);

		int featId = inlierInd[i];
		assert(vecPts1[featId]->ptInKeyFrm);
		pM->addFeaturePoint(vecPts1[featId]->ptInKeyFrm);
		for (TrackedFeaturePoint* pm = vecPts1[featId]; pm; pm = pm->pre)
			pm->pt.mpt = pM;
		for (TrackedFeaturePoint* pm = vecPts2[featId]; pm; pm = pm->pre)
			pm->pt.mpt = pM;
	}

	//6. add key frame
	addCameraPose(curFrame, R2, t2);
	tracker.updateCamParamForFeatPts(tracker.K, camPose.current());
	tracker.addKeyFrame();
	lastFrmAddKeyFrm = curFrame;
	state = SLAM_STATE_NORMAL;

	//test
	cout << "allMapPts.size():" << allMapPts.size() << endl;
	return true;
}

void MoSLAM::refineInitMap(const Mat_d& K, Mat_d& R1, Mat_d& t1, Mat_d& R2,
		Mat_d& t2, Mat_d& pts1, Mat_d& pts2, Mat_d& Ms) {

	assert(K.m == 3 && K.n == 3);
	assert(R1.m == 3 && R1.n == 3 && R2.m == 3 && R2.n == 3);
	assert(t1.m == 3 && t1.n == 1 && t2.m == 3 && t1.n == 1);
	assert(pts1.m == pts2.m && pts1.n == 2 && pts2.n == 2);
	assert(Ms.m == pts1.m && Ms.n == 3);

	print(R1);
	print(t1);

	print(R2);
	print(t2);

	vector<Mat_d> Ks, Rs, ts;
	vector<Point3d> pt3d;
	vector<vector<Meas2D> > meas2d;

	Ks.push_back(K);
	Ks.push_back(K);
	Rs.push_back(R1);
	Rs.push_back(R2);
	ts.push_back(t1);
	ts.push_back(t2);

	int npts = pts1.m;
	pt3d.resize(npts);
	meas2d.resize(npts);
	for (int i = 0; i < npts; ++i) {
		pt3d[i].set(Ms(i, 0), Ms(i, 1), Ms(i, 2));
		meas2d[i].push_back(Meas2D(0, pts1(i, 0), pts1(i, 1)));
		meas2d[i].push_back(Meas2D(1, pts2(i, 0), pts2(i, 1)));
	}

	bundleAdjust(1, Ks, Rs, ts, 0, pt3d, meas2d, 50, 0);

	//write back 
	R1.copyFrom(Rs[0]);
	R2.copyFrom(Rs[1]);
	t1.copyFrom(ts[0]);
	t2.copyFrom(ts[1]);

	print(R1);
	print(t1);

	print(R2);
	print(t2);
	for (int i = 0; i < npts; ++i) {
		Ms(i, 0) = pt3d[i].x;
		Ms(i, 1) = pt3d[i].y;
		Ms(i, 2) = pt3d[i].z;
	}
}

int MoSLAM::featureTracking() {
	TimeMeasurer tm;
	tm.tic();
	int trackRes = tracker.trackFeaturePoints();
	m_tmFeatureTracking = tm.toc();
	printf("Feature tracking time: %f, tracking res: %d\n", m_tmFeatureTracking,
			trackRes);

	return trackRes;
}
bool MoSLAM::estimateCamPose() {
	TimeMeasurer tm;
	tm.tic();

	enterBACriticalSection();
	tracker.propagateFeatureStates();
	double R[9], t[3];
	bool bEstPose = tracker.estimateCamPose(camPose.current()->R,
			camPose.current()->t, R, t);

	if (bEstPose) {
		CamPoseItem* cam = addCameraPose(curFrame, R, t);
		tracker.updateCamParamForFeatPts(tracker.K, cam);
		leaveBACriticalSection();
	}
	else{
		leaveBACriticalSection();
		return false;
	}

//	printf("Current map points number: %d\n", curMapPts.size());
	enterBACriticalSection();
	mapStateUpdate();
	leaveBACriticalSection();

	if (!bEstPose)
		return false;

//	printf("Current map points number: %d\n", curMapPts.size());
	m_tmPoseUpdate = tm.toc();
	enterBACriticalSection();
	getActiveMapPts(actMapPts);
	leaveBACriticalSection();

	return true;
}

CamPoseItem* MoSLAM::addCameraPose(int f, const double* R, const double* t) {
	CamPoseItem* cam = camPose.add(f, tracker.timeStamp, R, t);
	return cam;
}

struct KeyPoseBaseLineWidthItem {
	KeyFrame* keyFrm;
	double bw;

	KeyPoseBaseLineWidthItem(KeyFrame* kf, double w) {
		keyFrm = kf;
		bw = w;
	}
	KeyPoseBaseLineWidthItem(const KeyPoseBaseLineWidthItem& other) {
		keyFrm = other.keyFrm;
		bw = other.bw;
	}
	KeyPoseBaseLineWidthItem& operator =(
			const KeyPoseBaseLineWidthItem& other) {
		if (&other != this) {
			keyFrm = other.keyFrm;
			bw = other.bw;
		}
		return *this;
	}
};

bool KeyFrmBaseLineComp(KeyPoseBaseLineWidthItem item1,
		KeyPoseBaseLineWidthItem item2) {
	return item1.bw < item2.bw;
}

void MoSLAM::searchNearestKeyFrm(vector<KeyFrame*>& vecKeyFrm, int nKeyFrm) {
	CamPoseItem * curPose = camPose.current();

	vector<KeyPoseBaseLineWidthItem> keyFrmBaseLine;
	keyFrmBaseLine.reserve((size_t) keyFrm.num * 2);

	for (KeyFrame* p = keyFrm.head.next; p; p = p->next) {
		CamPoseItem* camPose = p->cam;
		double w = getBaseLineWidth(curPose, camPose);
		keyFrmBaseLine.push_back(KeyPoseBaseLineWidthItem(p, w));
		//reset the flag for displaying
		p->flag = KEY_FRAME_FLAG_NORMAL;
	}

	int nMaxFrm = (int) keyFrmBaseLine.size();
	std::nth_element(keyFrmBaseLine.begin(), keyFrmBaseLine.begin() + (std::min)(nKeyFrm,nMaxFrm-1),
			keyFrmBaseLine.end(), KeyFrmBaseLineComp);

	vecKeyFrm.clear();
	vecKeyFrm.reserve(nKeyFrm);

	nKeyFrm =
			nKeyFrm > (int) keyFrmBaseLine.size() ?
					(int) keyFrmBaseLine.size() : nKeyFrm;
	for (int i = 0; i < nKeyFrm; i++) {
		vecKeyFrm.push_back(keyFrmBaseLine[i].keyFrm);

		//for displaying
		keyFrmBaseLine[i].keyFrm->flag = KEY_FRAME_FLAG_ACTIVE;
	}
}
void MoSLAM::getActiveMapPts(std::list<MapPoint*>& pts) {

	//clear flags
	typedef std::list<MapPoint*>::iterator MapPointListIter;
	for (MapPointListIter iter = pts.begin(); iter != pts.end(); iter++) {
		MapPoint* mpt = *iter;
		mpt->state = STATE_MAPPOINT_INACTIVE;
	}

	pts.clear();

	//use set to avoid repetitive points
	std::set<MapPoint*> ptset;
	for (size_t k = 0; k < actKeyFrm.size(); k++) {
		KeyFrame* kf = actKeyFrm[k];
		for (size_t i = 0; i < kf->featPts.size(); i++) {
			FeaturePoint* fp = kf->featPts[i];
			if (fp->mpt
					&& !fp->mpt->isFalse() && fp->mpt->state != STATE_MAPPOINT_CURRENT) {
				ptset.insert(fp->mpt);
			}
		}
	}

	for (std::set<MapPoint*>::iterator p = ptset.begin(); p != ptset.end();
			p++) {
		(*p)->state = STATE_MAPPOINT_ACTIVE;
		pts.push_back(*p);
	}
}

void MoSLAM::mapStateUpdate() {
	searchNearestKeyFrm(actKeyFrm, SLAMParam::nNearestKeyFrm);
	typedef std::list<MapPoint*>::iterator MapPointListIter;
	if (!curMapPts.empty()) {
		//remove untracked map points
		for (MapPointListIter iter = curMapPts.begin(); iter != curMapPts.end();) {
			MapPoint* p = *iter;
			//printf("lastFrame: %d, curFrame: %d\n", p->lastFrame, curFrame);
			if (p->lastFrame < curFrame) {
				p->state = STATE_MAPPOINT_INACTIVE;
				iter = curMapPts.erase(iter);
			}else
			 	iter++;
		}
	}
}

//check if the number tracked points from the last key frame 
//decreases to the ratio of the number 
//of current detected feature points
bool MoSLAM::isNumMappedPtsDropBelow(double ratio) {

//	enterBACriticalSection();

	int num = 0;
	int totalNum = tracker.curFeatPts.size();
	int lastFrame = keyFrm.tail->frame;

	for (size_t i = 0; i < tracker.curFeatPts.size(); i++) {
		FeaturePoint* fp = &tracker.curFeatPts[i]->pt;
		if (fp->mpt && fp->mpt->firstFrame <= lastFrame && !fp->mpt->isFalse())
			num++;
	}
//	leaveBACriticalSection();

	if (num < totalNum * ratio) {
		return true;
	}
	return false;
}
int MoSLAM::isReadyForKeyFrame() {
//	SLAMParam::frmNumForAddKeyFrm = 1;
//	if (SLAMParam::frmNumForAddKeyFrm > 0
//			&& curFrame - lastFrmAddKeyFrm <= SLAMParam::frmNumForAddKeyFrm)
//		return -1;

//	if (frmNumAfterReloc == 0) {
//		frmNumAfterReloc = -1;
//		return KEY_FRAME_AFTER_RELOC;
//	} else {
//		if (frmNumAfterReloc > 0) {
//			frmNumAfterReloc--;
//			return -1;
//		}
//	}

	if ( curFrame - tracker.keyFrm.tail->frame < 10)
		return -1;

	if ((int) tracker.keyFrmBaseLineWidth.size() >= 0) {
		double minBaseLineWidth = tracker.avgKeyFrmBaseLineWidth
				* SLAMParam::baseLineRatio;

		for (size_t i = 0; i < actKeyFrm.size(); i++) {
			double w = getBaseLineWidth(camPose.current(), actKeyFrm[i]->cam);
			if (w < minBaseLineWidth)
				return -1;
		}
	}

	SLAMParam::mapReduceRatio  = 0.6f;
	if (isNumMappedPtsDropBelow(SLAMParam::mapReduceRatio))
		return KEY_FRAME_MAP_DECREASING;

	if (mpMaker.numNewMapPts > tracker.curMappedFeatPtsNum * 0.3) {
		mpMaker.numNewMapPts = 0;
		return KEY_FRAME_NEW_MAPPTS;
	}

	return -1;
}

bool MoSLAM::addNewKeyFrame() {
	int ready = isReadyForKeyFrame();
	if (ready < 0)
		return false;
//	if (curFrame % 10 != 0)
//		return false;
//
//	//add a new key frame
//	enterBACriticalSection();
	KeyFrame* kf = tracker.addKeyFrame();
	kf->reason = ready;
//	leaveBACriticalSection();
//
	lastFrmAddKeyFrm = curFrame;
////	if (keyFrm.num > 2) {
////		bundler.requestRecent(2, 7);
////	}
	cout << "a new key frame has been add! for reason - " << ready << endl;
	return true;
}

//for debug usage
void MoSLAM::pause() {
	updateDisplayData();
	MyApp::bStop = true;
	while (MyApp::bStop) {
	};
}

void MoSLAM::startMapInit(tf::Transform& firstPredictedCamPose) {
	if (mpMaker.startInit(firstPredictedCamPose))
		state = SLAM_STATE_MAP_INIT;
	else {
		reset();
		state = SLAM_STATE_READY;
	}
}

void MoSLAM::startMapInit() {
	if (mpMaker.startInit())
		state = SLAM_STATE_MAP_INIT;
	else {
		reset();
		state = SLAM_STATE_READY;
	}
}

void MoSLAM::endMapInit(tf::Transform& secondPredictedCamPose) {
	if (mpMaker.endInit(secondPredictedCamPose))
		state = SLAM_STATE_NORMAL;
	else {
		reset();
		state = SLAM_STATE_READY;
	}
}

void MoSLAM::endMapInit() {
	if (mpMaker.endInit())
		state = SLAM_STATE_NORMAL;
	else {
		reset();
		state = SLAM_STATE_READY;
	}
}

void MoSLAM::startRelocalization() {
//	state = SLAM_STATE_RELOCATE;
//	frmNumAfterReloc = -1;
//	relocalizer.reset();
	tracker.featTracker._bLost = true;
}
bool MoSLAM::doRelocalization() {
	bool res = false;
	while(curFrame < SLAMParam::frmNumTotal){
		grabReadFrame();
		tracker.frame++;
		tracker.featTracker.frame_++;

		cout << "frame: " << curFrame << endl;
		MyApp::bStop = true;
//		while (MyApp::bStop) {
//			Sleep(3);
////			cout << "bStop: " << MyApp::bStop << "\n";
//		}
//		featureTracking();
		if (relocalizer.tryToRecover()){
			MyApp::bStop = true;
			while (MyApp::bStop) {
				Sleep(3);
//				cout << "bStop: " << MyApp::bStop << "\n";
			}
			res = true;
			break;
//			return true;
		}
	}
//	return false;
	return res;
//	if (relocalizer.isCameraSteady())
//		return relocalizer.tryToRecover();
//	return false;
}

bool MoSLAM::doRelocalization_online() {
	bool res = false;

	while (1){
		grabReadFrame();
		tracker.frame++;
		tracker.featTracker.frame_++;

		cout << "frame: " << curFrame << endl;

		if (relocalizer.tryToRecover()){
			res = true;
			return res;
	//			return true;
		}
	}
//	return false;
	return res;
//	if (relocalizer.isCameraSteady())
//		return relocalizer.tryToRecover();
//	return false;
}

void MoSLAM::endRelocalization() {
//	state = SLAM_STATE_NORMAL;
//	frmNumAfterReloc = SLAMParam::frmNumAfterRelocalization;
//	cout << "Recovered!" << endl;
}
void MoSLAM::processOneFrame() {
//	grabReadFrame();
	if (state == SLAM_STATE_READY)
		return;
	featureTracking();
	if (state == SLAM_STATE_NORMAL) {
		if (estimateCamPose()) {
			TimeMeasurer tm;
			tm.tic();
			ptRegister.process(SLAMParam::DETECT_ERR_VAR);
			m_tmActMapRegister = tm.toc();

			mpMaker.genNewMapPoints();
			addNewKeyFrame();

		} else
			startRelocalization();
	} else if (state == SLAM_STATE_RELOCATE) {
		ControlCommand cmd(0,0,0,0);
		MyApp::_estimationNode->sendControlToDrone(cmd);
		if (doRelocalization()) {
			endRelocalization();
		}
	}
}

int MoSLAM::processOneFrameNew(){
	if (state == SLAM_STATE_READY)
		return SLAM_STATE_READY;

	TimeMeasurer testTm;
	int trackingRes = featureTracking();

	// When the SLAM in MAP_INIT mode, only do feature tracking and then return
	if (state == SLAM_STATE_MAP_INIT)
		return state;

	if (trackingRes < 0){
		printf("Start relocalisation\n");
		ControlCommand cmd(0,0,0,0);
		MyApp::_estimationNode->sendControlToDrone(cmd);
		if (!doRelocalization_online())
			return SLAM_STATE_LOST;
	}
	else{
		testTm.tic();
		if(!estimateCamPose()){
			printf("Start relocalisation\n");
			ControlCommand cmd(0,0,0,0);
			MyApp::_estimationNode->sendControlToDrone(cmd);
			if (!doRelocalization_online())
				return SLAM_STATE_LOST;
			cout << "Pose estimation: " << testTm.toc() << endl;
		}
	}

	testTm.tic();
	addNewKeyFrame();
	cout << "add new keyframe: " << testTm.toc() << endl;
//////
	testTm.tic();
	int numNewPts = mpMaker.genNewMapPoints();
	cout << "map points generation: " << testTm.toc() << endl;
	if (keyFrm.num > 8) {
		bundler.requestRecent(2, 5);
	}
	return SLAM_STATE_NORMAL;
}

void MoSLAM::setPredictedCamPose(tf::Transform predictedPose){
	_predictedWorldToCamTf = predictedPose;
}

#include <time.h>
void MoSLAM::exportResults(const char timeStr[]) const {
	using namespace std;
	char dirPath[256];
	sprintf(dirPath, "%s/%s", SLAMParam::resultPath.c_str(), timeStr);

#ifdef WIN32
	CreateDirectoryA(dirPath,NULL);
#else
	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

	char filePath[256];

	if (!MyApp::savedFrames.empty()){
		cout << "savedFrames size: " << MyApp::savedFrames.size() << endl;
		cout << MyApp::savedFrames[0].cols << " " << MyApp::savedFrames[0].rows << endl;

		sprintf(filePath, "%s/test.ppm", dirPath);
		cv::imwrite(filePath, MyApp::savedFrames.back());

		cv::VideoWriter vw;
		sprintf(filePath, "%s/cam_view.avi", dirPath);
		vw.open(filePath,CV_FOURCC('M','J','P','G'), 25, cv::Size(MyApp::savedFrames[0].cols,
				MyApp::savedFrames[0].rows), 0);
		for (int i = 0; i < MyApp::savedFrames.size(); i++){
			vw << MyApp::savedFrames[i];
		}
		vw.release();

//		sprintf(filePath, "%s/slam_view.avi", dirPath);;
//		vw.open(filePath,CV_FOURCC('M','J','P','G'), 25, cv::Size(MyApp::savedScene[0].cols,
//				MyApp::savedScene[0].rows), 0);
//		for (int i = 0; i < MyApp::savedScene.size(); i++){
//			vw << MyApp::savedScene[i];
//		}
//		vw.release();
	}




//save the information of the input video sequences
	sprintf(filePath, "%s/input_videos.txt", dirPath);
	ofstream file(filePath);
	if (!file)
		repErr("cannot open file %s to write!\n", filePath);

	file << "video file:" << SLAMParam::videoFilePath << endl;
	file << "calibration file:" << SLAMParam::calFilePath << endl;
	file.close();
	cout << filePath << " has been saved!" << endl;

//save map points and feature points
	sprintf(filePath, "%s/map_points.txt", dirPath);
	file.open(filePath);
	file << allMapPts.size() << endl;
	list<MapPoint*>::const_iterator mpIter = allMapPts.begin();
	for (; mpIter != allMapPts.end(); ++mpIter) {
		//test
		file << (*mpIter)->id << endl;
		file << (*mpIter)->x << " " << (*mpIter)->y << " "
				<< (*mpIter)->z << endl;
	//	for (size_t i = 0; i < 9; i++)
	//		file << mapPoints[i]->cov[i] << " ";
		file << endl;
	}
	file.close();
	cout << filePath << " has been saved!" << endl;

//save key frames
	sprintf(filePath, "%s/keyfrms.txt", dirPath);
	file.open(filePath);
	if (!file)
		repErr("cannot open file %s to write!\n", filePath);

	file << keyFrm.size() << endl;
	for (KeyFrame* kf = keyFrm.head.next; kf; kf = kf->next) {
		CamPoseItem* cam = kf->cam;
		file << cam->f << " " << cam->ts;
		for (int i = 0; i < 9; i++)
			file << " " << cam->R[i];
		file << " " << cam->t[0] << " " << cam->t[1] << " " << cam->t[2]
				<< endl;

		for (int i = 0; i < kf->featPts.size(); i++){
			if (kf->featPts[i]->mpt){
				file << kf->featPts[i]->mpt->id << " " << kf->featPts[i]->xo << " " << kf->featPts[i]->yo << " "
						<< kf->featPts[i]->x << " " << kf->featPts[i]->y << endl;
			}
		}
	}
	file.close();
	cout << filePath << " has been saved!" << endl;

//save camera poses
	sprintf(filePath, "%s/cam_pose.txt", dirPath);
	file.open(filePath);
	if (!file)
		repErr("cannot open file %s to write!\n", filePath);

	for (CamPoseItem* cam = camPose.first(); cam; cam = cam->next) {
		file << cam->f << " " << cam->ts;
		for (int i = 0; i < 9; i++)
			file << " " << cam->R[i];
		file << " " << cam->t[0] << " " << cam->t[1] << " " << cam->t[2]
				<< endl;
	}
	file.close();
	cout << filePath << " has been saved!" << endl;

//save timing information
	sprintf(filePath, "%s/timing.txt", dirPath);
	file.open(filePath);
	if (!file)
		repErr("cannot open file %s to write!\n", filePath);

	for (map<int, double>::const_iterator it = _frmId2tmPerStep.begin();
			it != _frmId2tmPerStep.end(); it++) {
		file << " " << it->first << " " << it->second << endl;
	}
	file.close();
	cout << filePath << " has been saved!" << endl;

//save parameters
	sprintf(filePath, "%s/video_parameters.txt", dirPath);
	file.open(filePath);
	if (!file)
		repErr("cannot open '%s' to write!\n", filePath);

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

	file << "input video:" << SLAMParam::videoFilePath << endl;
	file << "calibration file:" << SLAMParam::calFilePath << endl;
	file << "result path:" << SLAMParam::resultPath << endl;
	file.close();
	cout << filePath << " has been saved!" << endl;

//save navigation data
#ifdef ARDRONE
	sprintf(filePath, "%s/nav_data.txt", dirPath);
	file.open(filePath);
	if (!file)
		repErr("cannot open file %s to write!\n", filePath);

	for (size_t i = 0; i < m_navdata.size(); i++) {
		file << m_navdata[i]._vx << " " << m_navdata[i]._vy << " "
				<< m_navdata[i]._altitude << endl;
	}
	file.close();
#endif
}
void MoSLAM::saveCurrentImages(const char* dirPath) const {
#ifdef WIN32
	CreateDirectoryA(dirPath,NULL);
#else
	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
	char filePath[1024];
	sprintf(filePath, "%s/cam_%d.pgm", dirPath, 0);
	savePGM(tracker.img, filePath);
	logInfo("save images OK!\n");
}
