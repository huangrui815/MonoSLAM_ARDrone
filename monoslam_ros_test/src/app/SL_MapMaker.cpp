/*
 * SL_MapMaker.cpp
 *
 *  Created on: May 9, 2013
 *      Author: Danping Zou
 * 	 	E-mail: Dannis.zou@gmail.com
 */

#include "SL_MapMaker.h"
#include "SL_MoSLAM.h"
#include "SL_GlobParam.h"

#include "geometry/SL_Distortion.h"
#include "geometry/SL_Geometry.h"
#include "geometry/SL_Triangulate.h"

#include "calibration/SL_CalibTwoCam.h"
#include "math/SL_LinAlg.h"
#include "slam/SL_SLAMHelper.h"

#include "APP_MyApp.h"
#include "APP_SynObj.h"

MapMaker::MapMaker(MoSLAM& theSLAM) :
		slam(theSLAM), numNewMapPts(0) {
	doInit = false;
}

MapMaker::~MapMaker() {

}
bool MapMaker::startInit(tf::Transform& firstPredictedCamPose) {
	if (slam.tracker.trackFeaturePoints() <= 0){
		doInit = false;
		return false;
	}

	// save the first predicted cam pose from filter
	_firstPredictedCamPose = firstPredictedCamPose;

	//add the starting frame as  a key frame
	double R0[9], t0[3];
	matEyes(3, R0);
	matZeros(3, 1, t0);
	slam.addCameraPose(slam.curFrame, R0, t0);
	slam.tracker.updateCamParamForFeatPts(slam.tracker.K,
			slam.camPose.current());
	slam.tracker.addKeyFrame();

	doInit = true;
	return true;
}

bool MapMaker::startInit() {
	if (slam.tracker.trackFeaturePoints() <= 0){
		doInit = false;
		return false;
	}

	//add the starting frame as  a key frame
	double R0[9], t0[3];
	matEyes(3, R0);
	matZeros(3, 1, t0);
	slam.addCameraPose(slam.curFrame, R0, t0);
	slam.tracker.updateCamParamForFeatPts(slam.tracker.K,
			slam.camPose.current());
	slam.tracker.addKeyFrame();

	doInit = true;
	return true;
}

bool MapMaker::endInit(tf::Transform& secondPredictedCamPose) {
	//Save the second predicted cam pose from filter
	_secondPredictedCamPose = secondPredictedCamPose;

	//1.get tracked correspondences
	int prevKeyFrm = slam.tracker.lastKF->frame;
	vector<TrackedFeaturePoint*> vecPts1, vecPts2; // feature points
	for (int i = 0; i < slam.tracker.featTracker.maxTrackNum_; i++) {
		Track2D& tk = slam.tracker.featTracker.tracks[i];
		if (tk.empty())
			continue;
		if (tk.f1 == prevKeyFrm && tk.f2 == slam.curFrame) {
			assert(tk.head.next->ptInKeyFrm);
			vecPts1.push_back(tk.head.next);
			vecPts2.push_back(tk.tail);
		}
	}

	if (vecPts1.empty()) {
		ROS_INFO("Map initialization fail: no tracked feature points\n");
		doInit = false;
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
	calib.setIntrinParam(slam.tracker.K.data, slam.tracker.K.data);
	calib.setMatchedPoints(matPts1, matPts2);
	calib.estimateEMat(SLAMParam::MAX_EPI_ERR);

	vector<int> inlierInd;
	calib.getInlierInd(inlierInd);

	Mat_d R1, t1, R2, t2, pts1, pts2;
	calib.outputInlierNormPoints(pts1, pts2);
	calib.outputRTs(R1, t1, R2, t2);

	int npts = pts1.m;

	//3.triangulate initial map points
	Mat_d Ms(npts, 3);
	binTriangulatePoints(R1, t1, R2, t2, npts, pts1.data, pts2.data, Ms.data);

	//4.refine the initialization by bundle adjustment
	calib.outputInlierPoints(pts1, pts2);
	//slam.refineInitMap(slam.tracker.K, R1, t1, R2, t2, pts1, pts2, Ms);


	// 5. Estimate an initial scale
	double org2[3];
	getCameraCenter(R2.data, t2.data, org2);

	tf::Vector3 slamTranslation(org2[0], org2[1], org2[2]);
	tf::Vector3 predictedTranslationInGlobalUnit =
			MyApp::_filter->_secondCamPoseInGlobalUnit.getOrigin() -
			MyApp::_filter->_firstCamPoseInGlobalUnit.getOrigin();
	MyApp::_filter->updateScale(slamTranslation, predictedTranslationInGlobalUnit);

	ROS_INFO("Initial scale: %f\n", MyApp::_filter->_scaleGlobalToSlam);

	//6. Check the initial camera poses
	//   using the predicted camera poses from filter
	// Register the first keyframe pose with the predicted cam pose from the filter (with scale)

	tf::Transform firstKFPose, secondKFPose;
	tf::Transform firstKFPoseTransformed, secondKFPoseTransformed;

	secondKFPose = getTfFromRT(R2,t2); // transform from camera frame to world frame
	secondKFPose = secondKFPose.inverse(); // transform from world frame to camera frame
	tf::Vector3 scaledOrigin(secondKFPose.getOrigin()[0] * MyApp::_filter->_scaleGlobalToSlam,
			secondKFPose.getOrigin()[1] * MyApp::_filter->_scaleGlobalToSlam,
			secondKFPose.getOrigin()[2] * MyApp::_filter->_scaleGlobalToSlam);
	secondKFPose.setOrigin(scaledOrigin);

	firstKFPoseTransformed = _firstPredictedCamPose;
	secondKFPoseTransformed = firstKFPoseTransformed * secondKFPose;

	tf::Vector3 translateVec = _secondPredictedCamPose.getOrigin() - _firstPredictedCamPose.getOrigin();
	tf::Vector3 translateVecTransformed =
			secondKFPoseTransformed.getOrigin() - firstKFPoseTransformed.getOrigin();
	double errorAngle = translateVec.angle(translateVecTransformed);

	ROS_INFO("cam 2: %f %f %f; secondKFPose: %f, %f, %f\n", org2[0], org2[1], org2[2],
			secondKFPose.getOrigin()[0], secondKFPose.getOrigin()[1], secondKFPose.getOrigin()[2]);

	ROS_INFO("Predicted Cam position: 1st (%f %f %f) to 2nd (%f %f %f)\n",
			_firstPredictedCamPose.getOrigin()[0], _firstPredictedCamPose.getOrigin()[1], _firstPredictedCamPose.getOrigin()[2],
			_secondPredictedCamPose.getOrigin()[0], _secondPredictedCamPose.getOrigin()[1], _secondPredictedCamPose.getOrigin()[2]);

	ROS_INFO("Predicted Cam position in global unit: 1st (%f %f %f) to 2nd (%f %f %f)\n",
			MyApp::_filter->_firstCamPoseInGlobalUnit.getOrigin()[0],
			MyApp::_filter->_firstCamPoseInGlobalUnit.getOrigin()[1],
			MyApp::_filter->_firstCamPoseInGlobalUnit.getOrigin()[2],
			MyApp::_filter->_secondCamPoseInGlobalUnit.getOrigin()[0],
			MyApp::_filter->_secondCamPoseInGlobalUnit.getOrigin()[1],
			MyApp::_filter->_secondCamPoseInGlobalUnit.getOrigin()[2]);

	ROS_INFO("Transformed Predicted Cam position: 1st (%f %f %f) to 2nd (%f %f %f)\n",
				firstKFPoseTransformed.getOrigin()[0], firstKFPoseTransformed.getOrigin()[1], firstKFPoseTransformed.getOrigin()[2],
				secondKFPoseTransformed.getOrigin()[0], secondKFPoseTransformed.getOrigin()[1], secondKFPoseTransformed.getOrigin()[2]);

	ROS_INFO("Angle between estimated translation and predicted translation: %f\n", errorAngle * 180 / 3.1415926);

//	tf::Vector3 slamTranslation(org2[0], org2[1], org2[2]);
//	tf::Vector3 predictedTranslationInGlobalUnit =
//			MyApp::_filter->_secondCamPoseInGlobalUnit.getOrigin() -
//			MyApp::_filter->_firstCamPoseInGlobalUnit.getOrigin();
//	MyApp::_filter->updateScale(slamTranslation, predictedTranslationInGlobalUnit);
//
//	ROS_INFO("Initial scale: %f\n", MyApp::_filter->_scaleGlobalToSlam);

	if (slamTranslation == 0)
	{
		ROS_INFO("Initialization fail: estimated translation equal to zero.\n");
		doInit = false;
		return false;
	}
	if (predictedTranslationInGlobalUnit.length() < 0.05)
	{
		ROS_INFO("Initialization fail: predicted translation %f is smaller than 0.05m\n",
				predictedTranslationInGlobalUnit.length());
		doInit = false;
		return false;
	}
	if (errorAngle > 30)
	{
		ROS_INFO("Initialization fail: angle between predicted and estimated translations are larger than 30 degrees\n");
		doInit = false;
		return false;
	}


	//7. check if there are lots of points at the back of camera, which indicates
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
		doInit = false;
		return false;
	}

	for (int i = 0; i < npts; i++) {
		if (flag[i] > 0)
			continue;

		MapPoint* pM = new MapPoint(Ms.data[3 * i], Ms.data[3 * i + 1],
				Ms.data[3 * i + 2], slam.curFrame);

		pM->state = STATE_MAPPOINT_CURRENT;
		slam.curMapPts.push_back(pM);
		slam.allMapPts.push_back(pM);

		getBinTriangulateCovMat(slam.tracker.K, R1, t1, slam.tracker.K, R2, t2,
				pM->M, pM->cov, SLAMParam::DETECT_ERR_VAR);

		int featId = inlierInd[i];
		assert(vecPts1[featId]->ptInKeyFrm);
		pM->addFeaturePoint(vecPts1[featId]->ptInKeyFrm);
		for (TrackedFeaturePoint* pm = vecPts1[featId]; pm; pm = pm->pre)
			pm->pt.mpt = pM;
		for (TrackedFeaturePoint* pm = vecPts2[featId]; pm; pm = pm->pre)
			pm->pt.mpt = pM;
	}

	//7. add key frame
	slam.addCameraPose(slam.curFrame, R2, t2);
	slam.tracker.updateCamParamForFeatPts(slam.tracker.K,
			slam.camPose.current());
	slam.tracker.addKeyFrame();
	slam.lastFrmAddKeyFrm = slam.curFrame;
	slam.state = SLAM_STATE_NORMAL;

	//test
	cout << "allMapPts.size():" << slam.allMapPts.size() << endl;
	doInit = false;
	return true;
}
int MapMaker::triangulateTracks(vector<MapPoint*>& mapPts, double maxEpiErr) {
	std::vector<TrackedFeaturePoint*> vecFeatPts;
	int preKeyFrm = slam.tracker.lastKF->frame;

	slam.tracker.getUnMappedFeaturePointsFrom(vecFeatPts, preKeyFrm);

	mapPts.clear();
	mapPts.reserve(4096);

	double M[3], m1[2], m2[2];

	//reconstruct 3D map points
	int numAll = 0;
	int numRecons = 0;
	for (size_t k = 0; k < vecFeatPts.size(); k++) {
		TrackedFeaturePoint* cur_fp = vecFeatPts[k];
		TrackedFeaturePoint* pre_fp = cur_fp;
		while (pre_fp->pre && pre_fp->pre->pt.cam)
			pre_fp = pre_fp->pre;

		if(pre_fp->pt.id == -1)
			continue;
		if (!pre_fp->pt.cam)
			continue;


		assert(pre_fp->pt.f <= preKeyFrm);

		const double* K = slam.tracker.K.data;
		const double* invK = slam.tracker.invK.data;
		const double* preR = pre_fp->pt.cam->R;
		const double* preT = pre_fp->pt.cam->t;

		const double* curR = cur_fp->pt.cam->R;
		const double* curT = cur_fp->pt.cam->t;

		normPoint(invK, pre_fp->pt.m, m1);
		normPoint(invK, cur_fp->pt.m, m2);

		//triangulate the two feature points to get the 3D point
		binTriangulate(preR, preT, curR, curT, m1, m2, M);

		numAll++;

		if (isAtCameraBack(curR, curT, M))
			continue;

		double cov[9], org[3];
		getBinTriangulateCovMat(K, preR, preT, K, curR, curT, M, cov,
				SLAMParam::DETECT_ERR_VAR);

//		getCameraCenter(curR, curT, org);
//		double s = fabs(cov[0] + cov[4] + cov[8]);
//		if (dist3(org, M) < sqrt(s))
//			continue;

		//check the re-projection error
		double err1 = reprojErrorSingle(K, preR, preT, M, pre_fp->pt.m);
		double err2 = reprojErrorSingle(K, curR, curT, M, cur_fp->pt.m);

		if (err1 < maxEpiErr && err2 < maxEpiErr) {
			//a new map point is generated
			err1 = reprojErrorSingle(K, preR, preT, M, pre_fp->pt.m);
			err2 = reprojErrorSingle(K, curR, curT, M, cur_fp->pt.m);

			if (!isAtCameraBack(cur_fp->pt.cam->R, cur_fp->pt.cam->t, M)
					&& !isAtCameraBack(pre_fp->pt.cam->R, pre_fp->pt.cam->t,
							M)) {
				MapPoint* pM = new MapPoint(M[0], M[1], M[2], pre_fp->pt.f);
				doubleArrCopy(pM->cov, 0, cov, 9);

				if (getViewAngleChange(pM->M, cur_fp->pt.cam, pre_fp->pt.cam)
						< SLAMParam::MIN_TRI_ANGLE)
					continue;

				mapPts.push_back(pM);
				pM->firstFrame = pM->lastFrame = cur_fp->pt.f;

				//establish the 3D-2D correspondence
				for (TrackedFeaturePoint* fp = cur_fp; fp; fp = fp->pre) {
					fp->pt.mpt = pM;
					if (fp->ptInKeyFrm) {
						assert(fp->pt.K && fp->pt.cam);
						fp->ptInKeyFrm->mpt = pM;
						pM->addFeaturePoint(fp->ptInKeyFrm);
					}
				}
				numRecons++;
			}
		}
	}
	return numRecons;
}


bool MapMaker::endInit() {
	//1.get tracked correspondences
	int prevKeyFrm = slam.tracker.lastKF->frame;
	vector<TrackedFeaturePoint*> vecPts1, vecPts2; // feature points

	ROS_INFO("preKeyFrm: %d, slam.curFrame: %d\n",
			prevKeyFrm, slam.curFrame);
	for (int i = 0; i < slam.tracker.featTracker.maxTrackNum_; i++) {
		Track2D& tk = slam.tracker.featTracker.tracks[i];
		if (tk.empty())
			continue;
		if (tk.f1 == prevKeyFrm && tk.f2 == slam.curFrame) {
			assert(tk.head.next->ptInKeyFrm);
			vecPts1.push_back(tk.head.next);
			vecPts2.push_back(tk.tail);
		}
	}

	if (vecPts1.empty()) {
		doInit = false;
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
	calib.setIntrinParam(slam.tracker.K.data, slam.tracker.K.data);
	calib.setMatchedPoints(matPts1, matPts2);
	calib.estimateEMat(SLAMParam::MAX_EPI_ERR);

	vector<int> inlierInd;
	calib.getInlierInd(inlierInd);

	Mat_d R1, t1, R2, t2, pts1, pts2;
	calib.outputInlierNormPoints(pts1, pts2);
	calib.outputRTs(R1, t1, R2, t2);

	int npts = pts1.m;

	//3.triangulate initial map points
	Mat_d Ms(npts, 3);
	binTriangulatePoints(R1, t1, R2, t2, npts, pts1.data, pts2.data, Ms.data);

	//4.refine the initialization by bundle adjustment
	calib.outputInlierPoints(pts1, pts2);
	slam.refineInitMap(slam.tracker.K, R1, t1, R2, t2, pts1, pts2, Ms);

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
		doInit = false;
		return false;
	}

	double aveDepth = 0;
	for (int i = 0; i < npts; i++) {
		if (flag[i] > 0)
			continue;

		MapPoint* pM = new MapPoint(Ms.data[3 * i], Ms.data[3 * i + 1],
				Ms.data[3 * i + 2], slam.curFrame);

		pM->state = STATE_MAPPOINT_CURRENT;
		slam.curMapPts.push_back(pM);
		slam.allMapPts.push_back(pM);

		aveDepth += pM->z;

		getBinTriangulateCovMat(slam.tracker.K, R1, t1, slam.tracker.K, R2, t2,
				pM->M, pM->cov, SLAMParam::DETECT_ERR_VAR);

		int featId = inlierInd[i];
		assert(vecPts1[featId]->ptInKeyFrm);
		pM->addFeaturePoint(vecPts1[featId]->ptInKeyFrm);
		for (TrackedFeaturePoint* pm = vecPts1[featId]; pm; pm = pm->pre)
			pm->pt.mpt = pM;
		for (TrackedFeaturePoint* pm = vecPts2[featId]; pm; pm = pm->pre)
			pm->pt.mpt = pM;
	}
	aveDepth /= slam.allMapPts.size();
	normalizeInitMap(slam.allMapPts, aveDepth);
	t2[0] /= aveDepth;
	t2[1] /= aveDepth;
	t2[2] /= aveDepth;

	//6. add key frame
	slam.addCameraPose(slam.curFrame, R2, t2);
	slam.tracker.updateCamParamForFeatPts(slam.tracker.K,
			slam.camPose.current());
	slam.tracker.addKeyFrame();
	slam.lastFrmAddKeyFrm = slam.curFrame;
	slam.state = SLAM_STATE_NORMAL;

	//test
	cout << "allMapPts.size():" << slam.allMapPts.size() << endl;
	doInit = false;
	return true;
}

double MapMaker::normalizeInitMap(list<MapPoint*>& mapPts, double scale){
	list<MapPoint*>::iterator mapIter = mapPts.begin();

	for (; mapIter != mapPts.end(); ++mapIter){
		(*mapIter)->x = (*mapIter)->x / scale;
		(*mapIter)->x = (*mapIter)->y / scale;
		(*mapIter)->z = (*mapIter)->z / scale;
	}
}

int MapMaker::genNewMapPoints() {
	//check if there is one camera ready for insert a new key frame
	enterBACriticalSection();
	bool trackGood = slam.tracker.isFeatureTrackingGood();
	leaveBACriticalSection();

	if (!trackGood){
		printf("Tracking no good\n");
		return 0;
	}


	if (!slam.isNumMappedPtsDropBelow(SLAMParam::mapReduceRatio))
	{
		printf("Num of mapped pts drop below\n");
		return 0;
	}

	numNewMapPts = 0;
	//triangulate new map points
	enterBACriticalSection();
	vector<MapPoint*> newMapPts;
	numNewMapPts = triangulateTracks(newMapPts, SLAMParam::MAX_EPI_ERR);

	for (size_t k = 0; k < newMapPts.size(); k++) {
		newMapPts[k]->state = STATE_MAPPOINT_CURRENT;
		slam.curMapPts.push_back(newMapPts[k]);
		slam.allMapPts.push_back(newMapPts[k]);
	}

	leaveBACriticalSection();

	return numNewMapPts;
}
double getViewAngleChange(const double center[3], const CamPoseItem* cam1,
		const CamPoseItem* cam2) {
	const double PI = 3.14;
	double org1[3], org2[3];
	getCamCenter(cam1, org1);
	getCamCenter(cam2, org2);

	double r = getAbsRadiansBetween(center, org1, org2);
	return r / PI * 180.0;
}

void refineInitMap(const Mat_d& K, Mat_d& R1, Mat_d& t1, Mat_d& R2, Mat_d& t2,
		Mat_d& pts1, Mat_d& pts2, Mat_d& Ms) {

	assert(K.m == 3 && K.n == 3);
	assert(R1.m == 3 && R1.n == 3 && R2.m == 3 && R2.n == 3);
	assert(t1.m == 3 && t1.n == 1 && t2.m == 3 && t1.n == 1);
	assert(pts1.m == pts2.m && pts1.n == 2 && pts2.n == 2);
	assert(Ms.m == pts1.m && Ms.n == 3);

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

	for (int i = 0; i < npts; ++i) {
		Ms(i, 0) = pt3d[i].x;
		Ms(i, 1) = pt3d[i].y;
		Ms(i, 2) = pt3d[i].z;
	}
}
tf::Transform getTfFromRT(Mat_d& R, Mat_d& t){
	tf::Transform pose;
	tf::Matrix3x3 rot;
	rot.setValue(R.data[0], R.data[1], R.data[2],
			R.data[3],R.data[4],R.data[5],
			R.data[6],R.data[7],R.data[8]);
	pose.setBasis(rot);
	pose.setOrigin(tf::Vector3(t.data[0], t.data[1], t.data[2]));
	return pose;
}
