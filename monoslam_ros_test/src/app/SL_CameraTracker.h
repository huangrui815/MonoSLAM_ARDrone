/*
 * SL_CameraTracker.h
 *
 *  Created on: 2011-1-12
 *      Author: Danping Zou
 */

#ifndef SL_SINGLESLAM_H_
#define SL_SINGLESLAM_H_
#include "SL_PtrVec.h"
#include "tracking/SL_FeatureTracker.h"
#include "videoReader/VR_VideoReader.h"
#include "math/SL_Matrix.h"

#include "slam/SL_CameraPose.h"
#include "slam/SL_NCCBlock.h"
#include "slam/SL_MapPoint.h"
#include "slam/SL_KeyFrame.h"

#include "imgproc/SL_Image.h"
#include "tracking/SL_Track2D.h"

#include <string>
#include <vector>
class MoSLAM;

class CameraTrackerParam {
public:
	//divide the image plane into nRowBlk x nColBlk blocks
	//For each block, only one feature points is selected for pose estimation  
	int nRowBlk;
	int nColBlk;
	CameraTrackerParam() {
		//640x360 divide
		nRowBlk = 18;
		nColBlk = 32;

		//640x480
//		nRowBlk = 12;
//		nColBlk = 16;
	}
};
class CameraTracker: public CameraTrackerParam {
public:
	int frame;
	CamPoseList& camPose;
	KeyFrameList& keyFrm;

	KeyFrame* prevKF;
	KeyFrame* lastKF;

	int W, H; //image size
	int blkW, blkH; //block size

	VideoReader* videoReader;

	//for off-line usage
	std::string videoFilePath;
	std::string calFilePath;
	int startFrameInVideo;

	FeatureTracker featTracker; //feature point tracker;

	Mat_d K, invK; 	 //intrinsic matrix and its inverse
	Mat_d kDist, kUnDist; //distortion and its inverse

	ImgRGB rgb;				//color image
	ImgG img; 				//original image
	ImgG smallImg;			//small image
	double smallScale;		//scale for the small image
	int maxThumbW;			//maximum width of the thumbnail image
	double timeStamp;

	vector<TrackedFeaturePoint*> curFeatPts;//feature points detected in the current frame;
	MoSLAM* _slam;
public:
	//intermediate variables
	int frmNumBackTracking;
	deque<int> numFeatPts;
	int totalNumFeatPts;
	int avgNumFeatPts;

	deque<double> keyFrmBaseLineWidth;
	double avgKeyFrmBaseLineWidth;

	int curMappedFeatPtsNum;
	int curFeatPtsNum;

	double avgRepErr;
	double outlierRatio;

	bool _logOpened;

public:
	CameraTracker(int& curFrame, CamPoseList& cps, KeyFrameList& kps);
	~CameraTracker();
public:
	void reset();
	void propagateFeatureStates();
public:
	int getMappedFeaturePoints(std::vector<TrackedFeaturePoint*>& featPts);
	int getUnMappedFeaturePointsFrom(std::vector<TrackedFeaturePoint*>& featPts,
			int startFrm);
	int getNumOfMappedFeatPts();

	void setSLAM(MoSLAM*);
public:
	void initTracker();

	/*read video frames */
	void readFirstFrame();
	void grabReadFrame();

	/*feature point tracking */
	int trackFeaturePoints();

	void updateCamParamForFeatPts(const double* intrin, CamPoseItem* camPos);
	/**
	 * choose feature points for pose estimation
	 */
	int chooseFeaturePoints(vector<FeaturePoint*>& featPts);
	int chooseFeaturePointsNew(vector<FeaturePoint*>& featPts);

	/**
	 * compute the current camera pose from the 3D - 2D point pairs
	 */
	bool estimateCamPose(const double* R0, const double* t0, double* R,
			double* t);
	/**
	 * add a key pose for this camera 
	 */
	KeyFrame* addKeyFrame();

	/* refine the map point*/
	void refineTriangulation(const FeaturePoint* fp, double M[3],
			double cov[9]);

	bool isFeatureTrackingGood();

	void getDistortedCoord(const Mat_d& pts_in, Mat_d& pts_out);
	void undistortFeaturePts(const Mat_d& pts_in, Mat_d& pts_out);
public:
	//debug
	void saveCamPoses(const char* filePath, int startFrame = -1);
};

void trackedFeatPoint2Mat(const vector<TrackedFeaturePoint*>& pTrackNodes,
		Mat_d& matPts);
double getBaseLineWidth(const CamPoseItem* cam1, const CamPoseItem* cam2);
#endif /* SL_SINGLESLAM_H_ */
