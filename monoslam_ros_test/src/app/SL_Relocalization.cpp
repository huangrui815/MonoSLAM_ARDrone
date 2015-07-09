#include "SL_Relocalization.h"
#include "SL_MoSLAM.h"
#include "SL_GlobParam.h"
#include "slam/SL_MapPoint.h"
#include "slam/SL_MoSLAMHelper.h"

#include "geometry/SL_Triangulate.h"

#include "tools/SL_TypeConversion.h"
#include "tools/SL_WriteRead.h"
#include "tools/SL_Print.h"

#include "imgproc/SL_ImageIO.h"
#include "APP_SynObj.h"

#include "calibration/SL_CalibTwoCam.h"

#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include "geometry/epnp.h"
#include "geometry/SL_Geometry.h"

#include <numeric>

cv::Mat Relocalizer::genPointMask(vector<FeaturePoint*>& keyPts){
	int nrow = slam.tracker.H;
	int ncol = slam.tracker.W;
	cv::Mat cvMask(nrow, ncol, CV_8UC1, 255);

	int margin = 20;
	for (int x = 0; x < nrow; x++){
		for (int y = 0; y < ncol; y++){
			int id = y + x * ncol;
			if (x < margin || x > nrow - margin || y < margin || y > ncol - margin){
				cvMask.data[id] = 0;
			}
			else
				cvMask.data[id] = 255;
		}
	}

	for (int i = 0; i < keyPts.size(); i++) {
			cv::circle(cvMask,
					cv::Point2f(keyPts[i]->xo, keyPts[i]->yo),
					20, cv::Scalar(0), -1);
	}
	return cvMask;
}

bool Relocalizer::registerToKeyFrame(const KeyFrame* keyFrame, const ImgG& curImg,
		vector<MapPoint*>& matchedMapPts, Mat_d& matched2DPts, vector<cv::Point2f>& newPointsPair) {

	assert(keyFrame && !keyFrame->img.empty());
	assert(!curImg.empty());

	vector<FeaturePoint*> keyPts;
	for (int ii = 0; ii < keyFrame->featPts.size(); ii++){
		FeaturePoint* fp = keyFrame->featPts[ii];
		if( fp->mpt && !fp->mpt->isFalse()){
			keyPts.push_back(fp);
		}
	}
	if( keyPts.size()  == 0){
		printf("Relocalisation failed: reason #1\n");
		return false;
	}

	// detect some new points for triangulation
	  cv::Mat mask = genPointMask(keyPts);
	  vector<cv::Point2f> newPoints;
	  cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
	  cv::Size subPixWinSize(10,10), winSize(31,31);
	  cv::Mat cvI1(keyFrame->img.m, keyFrame->img.n, CV_8UC1, keyFrame->img.data);
	  cv::goodFeaturesToTrack(cvI1, newPoints, 128, 0.01, 10, mask, 3, 0, 0.04);
	  cornerSubPix(cvI1, newPoints, subPixWinSize, cv::Size(-1,-1), termcrit);


	//2.find the correspondences of the projected feature points in the current image
	Mat_d curPts;
	Mat_uc flag;

	int nMatched = frameRegister(keyFrame->img, curImg, keyPts, newPoints, curPts, flag);

	if (nMatched < 30 /*0.6 * keyPts.size()*/){
		printf("Relocalisation failed: reason #2(nMatched %d, keyPts.m %d\n)",
				nMatched, keyPts.size());
		return false;
	}
	else
		printf("Matched points found: %d\n", nMatched);

	matched2DPts.resize(nMatched, 2);
	int k = 0;
	for (int i = 0; i < keyPts.size(); i++) {
		if (flag[i] > 0) {
			matched2DPts[2 * k] = curPts[2 * i];
			matched2DPts[2 * k + 1] = curPts[2 * i + 1];
			matchedMapPts.push_back(keyPts[i]->mpt);
			k++;
		}
	}
	for ( int j = 0; j < newPoints.size(); j++){
		int id = keyPts.size() + j;
		if (flag[id] > 0){
			newPointsPair.push_back(newPoints[j]);
			newPointsPair.push_back(cv::Point2f(curPts[2 * id], curPts[2 * id + 1]));
		}
	}
	return true;
}

Relocalizer::Relocalizer(MoSLAM& slam_) :
		slam(slam_) {
}
void Relocalizer::reset() {
	numTrackedFeatPts.clear();
}
bool Relocalizer::isNumChangeLittle(const deque<int>& nums) {
	float sum = accumulate(nums.begin(), nums.end(), 0.0f);
	float mean = sum / nums.size();

	float std = 0;
	for (size_t i = 0; i < nums.size(); i++) {
		float d = nums[i] - mean;
		std += d * d;
	}

	std = sqrt(std / (nums.size() - 1));

	if (std < RELOC_MAX_NUM_STD)
		return true;
	return false;
}

bool Relocalizer::isCameraSteady() {
	FeatureTracker& featTracker = slam.tracker.featTracker;

	if ((int) numTrackedFeatPts.size() < RELOC_FRM_NUM) {
		numTrackedFeatPts.push_back(featTracker.trackedFeatureNum_);
		return false;
	} else {
		numTrackedFeatPts.pop_front();
		numTrackedFeatPts.push_back(featTracker.trackedFeatureNum_);

		if (isNumChangeLittle(numTrackedFeatPts))
			return true;
	}
	return false;
}

bool Relocalizer::tryToRecover() {
	CameraTracker& tracker = slam.tracker;

	enterBACriticalSection();
	SLAMParam::MAX_COST_KEY_FRM = 55;
	KeyFrame* kf = searchKeyPosebyThumbImage(SLAMParam::MAX_COST_KEY_FRM);
	leaveBACriticalSection();

	if (!kf){
		cout << " cannot find matched keyframe\n";
		return false;
	}

	printf("key frame (id: %d) matched!\n", kf->frame);

	vector<MapPoint*> matchedMapPts;
	Mat_d matched2DPts, matched2DPts_undist;
	vector<cv::Point2f> newPointsPair;

	bool bReg = registerToKeyFrame(kf, tracker.img, matchedMapPts,
			matched2DPts, newPointsPair);
	if (!bReg)
		return false;
	cout << " new points size: " << newPointsPair.size() << endl;

	cout << "success in registering to the key pose!" << endl;

//	tracker.getDistortedCoord(matched2DPts, matched2DPts_dist);
	tracker.undistortFeaturePts(matched2DPts, matched2DPts_undist);

	//add to feature point list
	/*==============(critical section)=================*/
	enterBACriticalSection();
	vector<FeaturePoint> vecFeatPts;
	for (int i = 0; i < matched2DPts.m; i++) {
		FeaturePoint fp(slam.curFrame, matched2DPts[2 * i],
				matched2DPts[2 * i + 1], matched2DPts_undist[2 * i],
				matched2DPts_undist[2 * i + 1]);
		fp.mpt = matchedMapPts[i];
		fp.mpt->lastFrame = slam.curFrame;
		fp.mpt->state = STATE_MAPPOINT_CURRENT;
		slam.curMapPts.push_back(fp.mpt);
		vecFeatPts.push_back(fp);
	}
	leaveBACriticalSection();
//	tracker.curFeatPts.clear();
//	tracker.featTracker.reset(tracker.img, vecFeatPts);
//	tracker.featTracker.getFeatPoints(tracker.curFeatPts);

	double R[9], t[3];
	////////////try pnp////////////////////////////////////
/*	epnp pnp;
	double R_pnp[3][3], t_pnp[3];
	pnp.set_internal_parameters(tracker.K(0,2), tracker.K(1,2), tracker.K(0,0),
			tracker.K(1,1));
	pnp.set_maximum_number_of_correspondences(vecFeatPts.size());
	for (int ii = 0; ii < vecFeatPts.size(); ii++){
		MapPoint* mpt = vecFeatPts[ii].mpt;
		pnp.add_correspondence(mpt->x, mpt->y, mpt->z, vecFeatPts[ii].x,
				vecFeatPts[ii].y);
	}
	pnp.compute_pose(R_pnp,t_pnp);
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			R[i*3 + j] = R_pnp[i][j];
			cout << R_pnp[i][j] << " ";
		}
		cout << endl;
	}*/
	std::vector<cv::Point3f> objPts;
	std::vector<cv::Point2f> imgPts;
	for (int i = 0; i < matched2DPts.m; i++){
		MapPoint* mpt = vecFeatPts[i].mpt;
		objPts.push_back(cv::Point3f(mpt->x, mpt->y, mpt->z));
		imgPts.push_back(cv::Point2f(vecFeatPts[i].x, vecFeatPts[i].y));
	}
	cv::Mat cvK(3,3,CV_64F), rvec, rmat, tvec(3,1,CV_64F,cv::Scalar(0)), inliers;
	Mat_d R_kf(3,3), R_init_final(3,3);
	memcpy(R_kf.data, kf->cam->R, 9 * sizeof(double));

	mat33ATB(R2_init, R_kf, R_init_final);
	cout << "R_init_final" << endl;
	print(R_init_final);

	cv::Mat rmat_init(3,3,CV_64F, R_init_final.data);
	cv::Rodrigues(rmat_init, rvec);

	memcpy(cvK.data, tracker.K.data, 9 * sizeof(double));
	cv::solvePnPRansac(objPts, imgPts, cvK, cv::Mat(),rvec, tvec, true, 100, 5.0, 100,
			inliers, CV_ITERATIVE);
	cv::Rodrigues(rvec, rmat);

	cout << rvec << endl;
	cout << rmat << endl;
	cout << tvec << endl;
	cout << inliers.rows << endl;
	memcpy(R, rmat.data, 9 * sizeof(double));
	memcpy(t, tvec.data, 3 * sizeof(double));
	std::vector<cv::Point2f> imgPts_reproj;
	cv::projectPoints(objPts, rvec, tvec, cvK, cv::Mat(), imgPts_reproj);

	///////////////// Triangulate new points/////////////////////////////
	int inlier = 0;
	for (int j =0 ; j < newPointsPair.size(); j = j + 2){
		double mo1[2], mo2[2], m1[2], m2[2], M[3], mo1_undist[2], mo2_undist[2];
		double* K = slam.tracker.K.data;
		const double* invK = slam.tracker.invK.data;
		const double* preR = kf->cam->R;
		const double* preT = kf->cam->t;

		const double* curR = R;
		const double* curT = t;

		mo1[0] = newPointsPair[j].x;
		mo1[1] = newPointsPair[j].y;
		slam.tracker.featTracker.undistorPointNew(K,slam.tracker.featTracker._kc, mo1, mo1_undist);
		normPoint(invK, mo1_undist, m1);
		mo2[0] = newPointsPair[j+1].x;
		mo2[1] = newPointsPair[j+1].y;
		slam.tracker.featTracker.undistorPointNew(K,slam.tracker.featTracker._kc, mo2, mo2_undist);
		normPoint(invK, mo2_undist, m2);

		//triangulate the two feature points to get the 3D point
		binTriangulate(preR, preT, curR, curT, m1, m2, M);
		double err1 = reprojErrorSingle(K, preR, preT, M, mo1_undist);
		double err2 = reprojErrorSingle(K, curR, curT, M, mo2_undist);
		if (err1 < 3 && err2 < 3){
			FeaturePoint fp(slam.curFrame, mo2[0], mo2[1], mo2_undist[0], mo2_undist[1]);
			fp.mpt = new MapPoint(M[0], M[1], M[2], slam.curFrame);
			fp.mpt->lastFrame = slam.curFrame;
			fp.mpt->state = STATE_MAPPOINT_CURRENT;
			slam.curMapPts.push_back(fp.mpt);
			vecFeatPts.push_back(fp);
			inlier++;
		}

	}
	cout << "inlier " << inlier << endl;

	// Reset the tracker
	tracker.curFeatPts.clear();
	tracker.featTracker.reset(tracker.img, vecFeatPts);
	tracker.featTracker.getFeatPoints(tracker.curFeatPts);

	/////////////////////////////////////////////////////////////////////

	int numOut = 0;
		double m[2], rm[2];
		for (int i = 0; i < imgPts.size(); i++) {
//			double* pM = vecFeatPts[i].mpt->M;
//			project(tracker.K, R, t, pM, rm);
			m[0] = imgPts[i].x; m[1] = imgPts[i].y; rm[0] = imgPts_reproj[i].x;
			rm[1] = imgPts_reproj[i].y;
			double repErr = dist2(m, rm);
			if (repErr <= 5.0)
				continue;
			else{
				tracker.curFeatPts[i]->pt.mpt = 0;
				numOut++;
			}
	}
	printf("Outlier ratio (pnp): %f\n", numOut * 1.0 / vecFeatPts.size());
	///////////////////////////////////////////////////////
	CamPoseItem* cam = slam.addCameraPose(slam.curFrame, R, t);
	tracker.updateCamParamForFeatPts(tracker.K, cam);
	return true;


//	if (tracker.estimateCamPose(kf->cam->R, kf->cam->t, R, t)) {
//		CamPoseItem* cam = slam.addCameraPose(slam.curFrame, R, t);
//		tracker.updateCamParamForFeatPts(tracker.K, cam);
//		return true;
//	}
//	return false;
}
KeyFrame* Relocalizer::searchKeyPosebyThumbImage(double maxcost) {
	//get the thumbnail image of the current frame
	ImgG imgThumb;
	getThumbImage(slam.tracker.smallImg, imgThumb, slam.tracker.maxThumbW);

	KeyFrame* p = 0;
	KeyFrame* min_kp = 0;
	double min_cost = DBL_MAX;
	for (p = slam.keyFrm.current(); p; p = p->prev) {
		assert(!p->imgThumb.empty());
		double cost = compareThumbImage(imgThumb, p->imgThumb);
		if (cost < min_cost) {
			min_cost = cost;
			min_kp = p;
		}
	}
	cout << "min cost:" << min_cost << endl;
	if (min_cost < maxcost)
		return min_kp;
	return 0;
}

int Relocalizer::frameRegister
(const ImgG& I1, const ImgG& I2, const vector<FeaturePoint*>& pts1, vector<cv::Point2f>& newPoints, Mat_d& pts2, Mat_uc& flag){
	std::vector<cv::KeyPoint> keypoints_1, keypoints_2, temp;

	cv::Mat cvI1(I1.m, I1.n, CV_8UC1, I1.data);
	cv::Mat cvI2(I2.m, I2.n, CV_8UC1, I2.data);
	FILE* pFile = fopen("points0.txt", "w");
	for (int i = 0; i < pts1.size(); i++){
		  cv::KeyPoint pt(pts1[i]->xo, pts1[i]->yo, 6.0);
		  keypoints_1.push_back(pt);
		  fprintf(pFile, "%f %f\n", pts1[i]->xo, pts1[i]->yo);
	}
	for (int i = 0; i < newPoints.size(); i++){
		  cv::KeyPoint pt(newPoints[i].x, newPoints[i].y, 6.0);
		  keypoints_1.push_back(pt);
		  fprintf(pFile, "%f %f\n", newPoints[i].x, newPoints[i].y);
	}
	fclose(pFile);

	cv::FastFeatureDetector detector(20);
	detector.detect( cvI2, temp);
//	std::vector<cv::Point2f> temp_pts;
//	const int MAX_COUNT = 512;
//	cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
//	cv::Size subPixWinSize(10,10), winSize(31,31);
//	cv::goodFeaturesToTrack(cvI2, temp_pts, MAX_COUNT, 0.01, 10);
//	cornerSubPix(cvI2, temp_pts, subPixWinSize, cv::Size(-1,-1), termcrit);


	for (int i = 0; i < temp.size(); i++){
	  cv::KeyPoint key(temp[i].pt.x, temp[i].pt.y, 6.0);
//		cv::KeyPoint key(temp_pts[i].x, temp_pts[i].y, 6.0);
		keypoints_2.push_back(key);
	}
	cv::SiftDescriptorExtractor extractor;
	cv::Mat descriptors_1, descriptors_2;

	extractor.compute( cvI1, keypoints_1, descriptors_1 );
	extractor.compute( cvI2, keypoints_2, descriptors_2 );
	cv::FlannBasedMatcher matcher;
	std::vector< cv::DMatch > matches;
	matcher.match( descriptors_1, descriptors_2, matches );

	std::vector<cv::Point2f> cvPts1;
	std::vector<cv::Point2f> cvPts2;

	Mat_d matPts1, matPts2;
	matPts1.resize(matches.size(), 2);
	matPts2.resize(matches.size(), 2);
	for( int i = 0; i < matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		cvPts1.push_back( keypoints_1[ matches[i].queryIdx ].pt );
		cvPts2.push_back( keypoints_2[ matches[i].trainIdx ].pt );
		matPts1[2* i] = keypoints_1[ matches[i].queryIdx ].pt.x;
		matPts1[2* i + 1] = keypoints_1[ matches[i].queryIdx ].pt.y;
		matPts2[2* i] = keypoints_2[ matches[i].trainIdx ].pt.x;
		matPts2[2* i + 1] = keypoints_2[ matches[i].trainIdx ].pt.y;
	}

	CalibTwoCam calib;
	calib.setIntrinParam(slam.tracker.K.data, slam.tracker.K.data);
	calib.setMatchedPoints(matPts1, matPts2);
//	calib.estimateEMatOld(2.0, CV_FM_RANSAC);
	calib.estimateEMat(3.0);
	Mat_d R1, t1, R2, t2;
	calib.outputRTs(R1, t1, R2, t2);
	print(R2);
	R2_init.cloneFrom(R2);

	vector<uchar> status;
	status.resize(calib.inlierFlag.m);
	for (int i = 0; i < calib.inlierFlag.m; i++){
		if (calib.inlierFlag[i] > 0)
			status[i] = 1;
		else
			status[i] = 0;
	}

//	vector<uchar> status;
//	//	cv::findHomography(pts1, pts2, CV_RANSAC, 15, status);
//	cv::findFundamentalMat(cvPts1, cvPts2, cv::FM_RANSAC, 5, 0.99, status);

	///////////////////////////////////////////////////////////////////////////////////////////////

	cv::imwrite("prevGray.jpg", cvI1);
	cv::imwrite("gray.jpg", cvI2);

	int ii = 0;
	vector<cv::DMatch> goodMatches;
	for (vector<cv::DMatch>::iterator matchIter = matches.begin(); matchIter != matches.end();){
		  if (status[ii] > 0)
			  goodMatches.push_back(*matchIter);

		  matchIter++;
		  ii = ii + 1;
	}

	printf("good match number: %d\n", goodMatches.size());



	  //-- Draw only "good" matches
	cv::Mat img_matches;
	cv::drawMatches( cvI1, keypoints_1, cvI2, keypoints_2,
				   goodMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
				   vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	  //-- Show detected matches
//	imshow( "Good Matches", img_matches );
	cv::imwrite("matches.jpg", img_matches);
	//  for( int i = 0; i < (int)good_matches.size(); i++ )
	//  { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

//	cv::waitKey(-1);

	///////////////////////////////////////////////////////////////////////////////////////////////

	int nTracked = 0;
	flag.resize(pts1.size() + newPoints.size(), 1);
	pts2.resize(pts1.size() + newPoints.size(), 2);
	flag.fill(0);
	pts2.fill(-1);

	for (size_t i = 0; i < matches.size(); i++){
		int pts1_Id = matches[i].queryIdx;

		if (status[i] == 0){
			flag[pts1_Id] = 0;
		}
		else if (status[i] > 0){
			pts2.data[2 * pts1_Id] = cvPts2[i].x;
			pts2.data[2 * pts1_Id + 1] = cvPts2[i].y;
			flag[pts1_Id] = 1;
			if (pts1_Id < pts1.size())
				nTracked++;
		}
	}
	return nTracked;
}

//#include "tools/GUI_ImageViewer.h"
//#include "tools/SL_DrawCorners.h"
//int klt_main() {
//
//	ImgG I1, I2;
//
//	imread(I1, "/home/tsou/I1.png");
//	imread(I2, "/home/tsou/I2.png");
//
//	Mat_d pts;
//	readMat(pts, "/home/tsou/keypts.txt");
//
//	Mat_d ptsnew;
//	Mat_uc flag;
//	int nMatch = trackFeatureKLT(I1, I2, pts, ptsnew, flag);
//	
//	ImgRGB outImg;
//	drawMatching(I1,pts,I2,ptsnew,outImg);
//	
//	imshow("outImg", outImg);
//	cout << "nMatch:" << nMatch << endl;
//	cv::waitKey(-1);
//	return 0;
//}
