/*
 * SL_CameraTracker.cpp
 *
 *  Created on: 2011-1-12
 *      Author: Danping Zou
 */

#include "SL_GlobParam.h"
#include "SL_CameraTracker.h"

#include "slam/SL_MapPoint.h"
#include "slam/SL_IntraCamPose.h"
#include "slam/SL_SLAMHelper.h"
#include "slam/SL_MoSLAMHelper.h"
#include "slam/SL_SolvePnP.h"

#include "imgproc/SL_ImageOp.h"
#include "math/SL_LinAlg.h"
#include "matching/SL_GuidedSSDMatcher.h"

#include "geometry/SL_Triangulate.h"
#include "geometry/SL_5point.h"
#include "geometry/SL_Distortion.h"

#include "app/SL_MoSLAM.h"
#include "tools/SL_Tictoc.h"

#include <cfloat>

//#define DEBUG_MODE 1
CameraTracker::CameraTracker(int& curFrame, CamPoseList& cps, KeyFrameList& kps) :
		frame(curFrame), CameraTrackerParam(), camPose(
				cps), keyFrm(kps), W(0), H(0), blkW(0), blkH(0), startFrameInVideo(
				0) {

	smallScale = 0.3;
	maxThumbW = 100;
	_logOpened = false;

	reset();
}
CameraTracker::~CameraTracker() {
}
//reset intermediate results
void CameraTracker::reset() {
	frmNumBackTracking = 0;

	numFeatPts.clear();
	totalNumFeatPts = 0;
	avgNumFeatPts = 0;

	avgKeyFrmBaseLineWidth = 0.0;
	curMappedFeatPtsNum = 0;
	curFeatPtsNum = 0;

	avgRepErr = 0;
	outlierRatio = 0;

	prevKF = 0;
	lastKF = 0;

	featTracker.firstTracking = true;
}

void CameraTracker::setSLAM(MoSLAM* slam){
	_slam = slam;
}

void CameraTracker::propagateFeatureStates() {
//	char filePath[100] = "/home/rui/workspace/myROS/monoslam_ros_test/slam_results/frame_2D3D.txt";
//	FILE* pFile;
//	if (_logOpened)
//		pFile = fopen(filePath, "a+");
//	else{
//		pFile = fopen(filePath, "w");
//		_logOpened = true;
//	}
//
//	fprintf(pFile, "%d\n", frame);
	for (int i = 0; i < featTracker.maxTrackNum_; i++) {
		Track2D& tk = featTracker.tracks[i];
		if (tk.empty())
			continue;
		TrackedFeaturePoint* node = tk.tail;
		if (node->pre) {
			//propagate the type of feature points
			node->pt.type = node->pre->pt.type;
			if (node->pre->pt.mpt) {
				MapPoint* pMapPt = node->pre->pt.mpt;
				if (pMapPt->state != STATE_MAPPOINT_CURRENT)
					continue;
				//map correspondence propagation
				if (!pMapPt->isFalse()) {
					node->pt.mpt = pMapPt;
					pMapPt->lastFrame = frame;
//					fprintf(pFile, "%f %f %f %f %ld %f %f %f\n", node->pt.xo, node->pt.yo, node->pt.x, node->pt.y, pMapPt->id, pMapPt->x,
//							pMapPt->y, pMapPt->z);
				}
			}
		}
	}
//	fprintf(pFile, "\n");
//	fclose(pFile);
}
int CameraTracker::getMappedFeaturePoints(
		std::vector<TrackedFeaturePoint*>& featPts) {
	int k = 0;
	for (int i = 0; i < featTracker.maxTrackNum_; i++) {
		Track2D& tk = featTracker.tracks[i];
		if (tk.empty() || !tk.valid())
			continue;
		TrackedFeaturePoint* node = tk.tail;
//		printf("%d %d\n", node->pt.f, frame);
		assert(node->pt.f == frame);
		if (node->pt.mpt && !node->pt.mpt->isFalse()) {
			featPts.push_back(node);
			k++;
		}
	}
	return k;
}
int CameraTracker::getUnMappedFeaturePointsFrom(
		std::vector<TrackedFeaturePoint*>& nodes, int startFrm) {
	int k = 0;
	for (int i = 0; i < featTracker.maxTrackNum_; i++) {
		Track2D& tk = featTracker.tracks[i];
		if (tk.empty() || tk.f1 > startFrm || !tk.valid())
			continue;
		TrackedFeaturePoint* node = tk.tail;
		if (!node->pt.mpt) {
			nodes.push_back(node);
			k++;
		}
	}
	return k;
}

int CameraTracker::getNumOfMappedFeatPts() {
	int nMapPts = 0;
	for (size_t i = 0; i < curFeatPts.size(); i++) {
		if (curFeatPts[i]->pt.mpt)
			nMapPts++;
	}
	curMappedFeatPtsNum = nMapPts;
	curFeatPtsNum = (int) curFeatPts.size();
	return nMapPts;
}

void CameraTracker::initTracker() {
	assert(videoReader);
	W = videoReader->_w;
	H = videoReader->_h;

	blkW = 20; //W / nColBlk;
	blkH = 20;
	nColBlk = W / blkW;
	nRowBlk = H / blkH;

	rgb.resize(W, H);
	img.resize(W, H);
	smallImg.resize((int) (W * smallScale), (int) (H * smallScale));

	V3D_GPU::KLT_SequenceTrackerConfig cfg;
	cfg.minCornerness = SLAMParam::klt_minCornerness;
	cfg.nLevels = SLAMParam::klt_nLevels;
	cfg.windowWidth = SLAMParam::klt_windowWidth;
	cfg.convergenceThreshold = SLAMParam::klt_convergeThreshold;
	cfg.SSD_Threshold = SLAMParam::klt_SSDThreshold;
	cfg.trackWithGain = SLAMParam::klt_trackWithGain;

	featTracker.openGPU(W, H, &cfg);
	featTracker.setIntrinsicParam(K, invK, kUnDist, kDist.data);
//	featTracker.readFMatrix01("/home/rui/SFM_data/ARDrone/data09/imgs01_FM.txt");
}

void CameraTracker::readFirstFrame() {
	videoReader->readCurFrame(rgb.data, img.data);
	scaleDownAvg(img, smallImg, smallScale);
	timeStamp = videoReader->getTimeStamp();
}
void CameraTracker::grabReadFrame() {
	videoReader->grabFrame();
	videoReader->readCurFrame(rgb.data, img.data);

	cv::Mat cvImg(img.rows, img.cols, CV_8UC1, img.data);
	cv::Mat cvSmallImg(smallImg.rows, smallImg.cols, CV_8UC1, smallImg.data);
	cv::resize(cvImg, cvSmallImg, cv::Size(smallImg.cols, smallImg.rows));
	timeStamp = videoReader->getTimeStamp();
}
int CameraTracker::trackFeaturePoints() {
	int trackRes = 0;
	// get the flags for mapped tracks
	featTracker.getMappedFlags();
	try {
		if (featTracker.firstTracking) {
			featTracker.first(_slam->curFrame, img);
			featTracker.firstTracking = false;
		} else
			trackRes = featTracker.next(img);

		featTracker.getFeatPoints(curFeatPts);
		printf("curFeatPts number: %d\n", curFeatPts.size());
	} catch (SL_Exception& exp) {
		featTracker.detectedFeatureNum_ = 0;
		featTracker.trackedFeatureNum_ = 0;
	}

	int nfeatPts = featTracker.detectedFeatureNum_
			+ featTracker.trackedFeatureNum_;

//	printf("Frame Id: %d, Detected points size %d, tracked points size %d\n",
//			frame, featTracker.detectedFeatureNum_, featTracker.trackedFeatureNum_);

	//for statistics of the number of tracked feature points
	numFeatPts.push_back(nfeatPts);
	totalNumFeatPts += nfeatPts;
	if ((int) numFeatPts.size() > SLAMParam::frmNumForStatisticFeatNum) {
		totalNumFeatPts -= numFeatPts.front();
		numFeatPts.pop_front();
	}

	//record the average number of tracked feature points
	avgNumFeatPts = (int) (1.0 * totalNumFeatPts / numFeatPts.size());

	frame = featTracker.frame_;
	if (trackRes < 0){
		if (keyFrm.tail->frame != frame-1){
			//add the previous frame as a keyframe
			KeyFrame* kp = keyFrm.add(frame-1, camPose.current());
			prevKF = lastKF;
			lastKF = kp;

			kp->featPts.clear();

			cv::Mat& cvImg_prev = featTracker.klt_.prevGray;
			ImgG img_prev(W,H);
			ImgG smallImg_prev((int) W * smallScale, (int)H * smallScale);
			memcpy(img_prev.data, cvImg_prev.data, W * H * sizeof(uchar));
			cv::Mat cvSmallImg(smallImg_prev.rows, smallImg_prev.cols, CV_8UC1, smallImg_prev.data);
			cv::resize(cvImg_prev, cvSmallImg, cv::Size(smallImg.cols, smallImg.rows));
			kp->setSmallImage(smallImg_prev, smallScale);
			kp->setThumbImage(smallImg_prev, maxThumbW);
			kp->setCameraIntrinsic(K.data);
			kp->setImage(img_prev);
			for ( int i =0; i < featTracker.maxTrackNum_; i++){
				if (featTracker.tracks[i].empty())
					continue;
				TrackedFeaturePoint* trackedPt = featTracker.tracks[i].tail;
				if (trackedPt->pt.id != -1 && trackedPt->pt.mpt){
					FeaturePoint* fp = new FeaturePoint(trackedPt->pt);
					fp->keyFrm = kp;
					kp->featPts.push_back(fp);
					trackedPt->ptInKeyFrm = fp;
				}
			}
		}
		return trackRes;
	}


	return nfeatPts;
}

void CameraTracker::updateCamParamForFeatPts(const double* intrin,
		CamPoseItem* camPos) {
	for (size_t i = 0; i < curFeatPts.size(); i++) {
		curFeatPts[i]->pt.setIntrinsic(intrin);
		curFeatPts[i]->pt.setCameraPose(camPos);
	}
}

int CameraTracker::chooseFeaturePoints(vector<FeaturePoint*>& featPts) {
	int len = nRowBlk * nColBlk + 1;

	Track2D** tracks = new Track2D*[len];
	memset(tracks, 0, sizeof(Track2D*) * len);

	for (int i = 0; i < featTracker.maxTrackNum_; i++) {
		Track2D* tk = &featTracker.tracks[i];
		if (tk->empty() || !tk->valid())
			continue;

		FeaturePoint* fp = &tk->tail->pt;
		if (fp->type == TYPE_FEATPOINT_STATIC
				|| (fp->mpt && !fp->mpt->isFalse())) {

			//compute in which block the feature point should lie
			int bx = static_cast<int>((fp->xo) / blkW);
			int by = static_cast<int>((fp->yo) / blkH);
			if (bx >= nColBlk || by >= nRowBlk)
				continue;

			int bi = by * nColBlk + bx;
			assert(bi < len);

			Track2D* tkOld = tracks[bi];
			if (tkOld == 0) {
				tracks[bi] = tk;
			} else {
				FeaturePoint* fpOld = &tkOld->tail->pt;
				if (!fpOld->mpt) {
					if (fp->mpt) {
						tracks[bi] = tk;
					} else {
						if (tkOld->length() < tk->length()) {
							tracks[bi] = tk;
						}
					}
				}
			}
		}
	}

	featPts.reserve(nRowBlk * nColBlk * 2);
	int k = 0;
	for (int i = 0; i < len; i++) {
		if (tracks[i]) {
			featPts.push_back(&tracks[i]->tail->pt);
			k++;
		}
	}
	delete[] tracks;
	return k;
}


int CameraTracker::chooseFeaturePointsNew(vector<FeaturePoint*>& featPts) {
	int len = nRowBlk * nColBlk + 1;

	Track2D** tracks = new Track2D*[len];
	memset(tracks, 0, sizeof(Track2D*) * len);

	for (int i = 0; i < featTracker.maxTrackNum_; i++) {
		Track2D* tk = &featTracker.tracks[i];
		if (tk->empty() || !tk->valid())
			continue;

		FeaturePoint* fp = &tk->tail->pt;
		if (fp->mpt && !fp->mpt->isFalse()) {

			//compute in which block the feature point should lie
			int bx = static_cast<int>((fp->xo) / blkW);
			int by = static_cast<int>((fp->yo) / blkH);
			if (bx >= nColBlk || by >= nRowBlk)
				continue;

			int bi = by * nColBlk + bx;
			assert(bi < len);

			Track2D* tkOld = tracks[bi];
			if (tkOld == 0) {
				tracks[bi] = tk;
			} else {
				FeaturePoint* fpOld = &tkOld->tail->pt;
				if (tkOld->length() < tk->length()) {
					tracks[bi] = tk;
				}
			}
		}
	}

	featPts.reserve(nRowBlk * nColBlk);
	int k = 0;
	for (int i = 0; i < len; i++) {
		if (tracks[i]) {
			featPts.push_back(&tracks[i]->tail->pt);
			k++;
		}
	}
	delete[] tracks;
	return k;
}

bool CameraTracker::estimateCamPose(const double* R0, const double* t0,
		double* R, double *t) {
	//get the feature points corresponding to the map points
	std::vector<TrackedFeaturePoint*> trackedPts;
	int num = getMappedFeaturePoints(trackedPts);
	printf(" get mapped feature points: %d\n", num);
//	int featNum = featTracker.trackedFeatureNum_;
//	if (num < Param::minMappedRatio * featNum)
//	{
//		printf("num of mapped feature points (%d) < ratio (%f) * feature number (%d)\n",
//			num, Param::minMappedRatio, featNum);
//		std::cout << "Number of mapped feature points below a threshold\n";
//		return false;
//	}

	//choose the feature points for pose estimation
	std::vector<FeaturePoint*> featPts;
//	chooseFeaturePoints(featPts);
	chooseFeaturePointsNew(featPts);
	std::vector<FeaturePoint*> mappedFeatPts;
	mappedFeatPts.reserve(nRowBlk * nColBlk);

	for (size_t i = 0; i < featPts.size(); i++)
		if (featPts[i]->mpt)
			mappedFeatPts.push_back(featPts[i]);

	//get the 2D-3D corresponding points
	int n3D2Ds = mappedFeatPts.size();
	Mat_d ms(n3D2Ds, 2), Ms(n3D2Ds, 3), covs(n3D2Ds, 9);
	for (int i = 0; i < n3D2Ds; i++) {
		FeaturePoint* fp = mappedFeatPts[i];
		ms.data[2 * i] = fp->x;
		ms.data[2 * i + 1] = fp->y;

		Ms.data[3 * i] = fp->mpt->x;
		Ms.data[3 * i + 1] = fp->mpt->y;
		Ms.data[3 * i + 2] = fp->mpt->z;

		memcpy(covs.data, fp->mpt->cov, sizeof(double) * 9);
	}

	//Mat_d R(3, 3), t(3, 1);
	double* cR = const_cast<double*>(R0);
	double* cT = const_cast<double*>(t0);

	//estimate the camera poses
	IntraCamPoseOption opt;
	intraCamEstimate(K.data, cR, cT, Ms.rows, 0, Ms.data, ms.data,
			SLAMParam::MAX_REPROJECT_ERR, R, t, &opt);

	//find outliers
	avgRepErr = 0;
	int numOut = 0;
	double rm[2];
	for (int i = 0; i < num; i++) {
		double* pM = trackedPts[i]->pt.mpt->M;
		project(K, R, t, pM, rm);
		double repErr = dist2(rm, trackedPts[i]->pt.m);
		MapPoint* mpt = trackedPts[i]->pt.mpt;
		if (repErr < SLAMParam::SMALL_REPROJECT_ERR)
			mpt->nFrmSmallRepErr++;
		else
			numOut++;

		if (repErr > SLAMParam::MAX_REPROJECT_ERR) {
			if (mpt->nFrmSmallRepErr < SLAMParam::frmNumOutlier) {
				//remove outliers
				trackedPts[i]->pt.mpt->setFalse();
			} else
				trackedPts[i]->pt.mpt = 0;
		} else
			avgRepErr += repErr;
	}

//	if (repErr > Param::MAX_REPROJECT_ERR)
//		trackedPts[i]->pt.mpt->setFalse();
//	}

//	avgRepErr /= (num - numOut);
	outlierRatio = numOut * 1.0 / num;

	if (outlierRatio > 0.6){
		printf("Camera estimation falied: outlier ratio %f\n", outlierRatio);
		return false;
	}

	return true;
}

double getBaseLineWidth(const CamPoseItem* cam1, const CamPoseItem* cam2) {
	double c1[3], c2[3];
	getCamCenter(cam1, c1);
	getCamCenter(cam2, c2);
	return dist3(c1, c2);
}

KeyFrame* CameraTracker::addKeyFrame() {
	KeyFrame* kp = keyFrm.add(frame, camPose.current());

	kp->setNumMappedPoints(curMappedFeatPtsNum);
	kp->setImage(img);
	kp->setSmallImage(smallImg, smallScale);
	kp->setThumbImage(smallImg, maxThumbW);
	kp->setCameraIntrinsic(K.data);

	prevKF = lastKF;
	lastKF = kp;

	kp->featPts.clear();

	for (size_t i = 0; i < curFeatPts.size(); i++) {
		if (curFeatPts[i]->pt.id == -1)
			continue;

		FeaturePoint* fp = new FeaturePoint(curFeatPts[i]->pt);
		fp->keyFrm = kp;
		kp->featPts.push_back(fp);
		curFeatPts[i]->ptInKeyFrm = fp;
		if (fp->mpt && !fp->mpt->isFalse()) {
			fp->mpt->addFeaturePoint(fp);
			updateStaticPointPosition(fp->mpt, SLAMParam::DETECT_ERR_VAR, true);
		}
	}

	//for statistic
	if (prevKF && lastKF != prevKF) {
		double w = getBaseLineWidth(prevKF->cam, lastKF->cam);
		keyFrmBaseLineWidth.push_back(w);
		if ((int) keyFrmBaseLineWidth.size() > SLAMParam::frmNumForStatisticKeyFrm)
			keyFrmBaseLineWidth.pop_front();

		avgKeyFrmBaseLineWidth = 0.0;
		for (size_t k = 0; k < keyFrmBaseLineWidth.size(); k++) {
			avgKeyFrmBaseLineWidth += keyFrmBaseLineWidth[k];
		}
		avgKeyFrmBaseLineWidth /= keyFrmBaseLineWidth.size();
	}
	return kp;
}
void trackedFeatPoint2Mat(const vector<TrackedFeaturePoint*>& pTrackNodes,
		Mat_d& matPts) {
	size_t num = pTrackNodes.size();
	matPts.resize(num, 2);
	for (size_t i = 0; i < num; i++) {
		matPts.data[2 * i] = pTrackNodes[i]->pt.x;
		matPts.data[2 * i + 1] = pTrackNodes[i]->pt.y;
	}
}
bool CameraTracker::isFeatureTrackingGood() {
	getNumOfMappedFeatPts();
//	if (featTracker.trackRatio < Param::badTrackingRatio) {
//		frmNumBackTracking = Param::frmNumNoTracking;
//		printf("Bad feature tracking\n");
//		return false;
//	}
//	//tracking is good only the counter 'm_numBadTracking' reduced to zero
//	if (frmNumBackTracking == 0)
//		return true;
//
//	frmNumBackTracking--;
//	return false;
	if(featTracker.trackRatio < SLAMParam::badTrackingRatio){
		return false;
	}
	else
		return true;
}
void CameraTracker::getDistortedCoord(const Mat_d& pts_in, Mat_d& pts_out) {
	assert(pts_in.m > 0);
	pts_out.resize(pts_in.m, 2);

	for (int i = 0; i < pts_in.m; i++) {
		double nm[2];
		normPoint(invK, pts_in.data + 2 * i, nm);
		imagePoint(K, nm, pts_out.data + 2 * i);
	}
}
void CameraTracker::undistortFeaturePts(const Mat_d& pts_in, Mat_d& pts_out){
	assert(pts_in.m > 0);
	pts_out.resize(pts_in.m, 2);

	for (int i = 0; i < pts_in.m; i++) {
		double in[2], out[2];
		in[0] = pts_in.data[2 * i];
		in[1] = pts_in.data[2 * i + 1];
		featTracker.undistorPointNew(K.data, kDist.data, in, out);
		pts_out.data[2 * i] = out[0];
		pts_out.data[2 * i + 1] = out[1];
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//for debug
void CameraTracker::saveCamPoses(const char* filePath, int startFrame) {
	FILE* fp = fopen(filePath, "w");
	if (!fp)
		repErr("SingleSLAM::saveCamPoses - cannot open '%s'!", filePath);

	for (CamPoseItem* cam = camPose.current(); cam && cam->f >= startFrame;
			cam = cam->pre) {
		fprintf(fp, "%d\t", cam->f);
		for (int i = 0; i < 9; i++)
			fprintf(fp, "%g ", cam->R[i]);
		fprintf(fp, "%g %g %g\n", cam->t[0], cam->t[1], cam->t[2]);
	}
	fclose(fp);
}
