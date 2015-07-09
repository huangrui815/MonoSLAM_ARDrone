/*
 * SL_KeyFrmLst.h
 *
 *  Created on: 2010-11-25
 *      Author: Danping Zou
 */

#ifndef SL_KEYFRMLST_H_
#define SL_KEYFRMLST_H_
#include "SL_Define.h"

#include "imgproc/SL_Image.h"
#include "imgproc/SL_ImageOp.h"

#include "slam/SL_CameraPose.h"
#include "slam/SL_FeaturePoint.h"

#define KEY_FRAME_FLAG_NORMAL 0
#define KEY_FRAME_FLAG_RELOC  1
#define KEY_FRAME_FLAG_ACTIVE 2

#define KEY_FRAME_NEW_MAPPTS 0
#define KEY_FRAME_MAP_DECREASING 1
#define KEY_FRAME_AFTER_RELOC 2

class KeyFrame {
public:
	int flag;			//for debug
	int reason;			//reason to insert this key frame			
	int camId;			//camera ID
	int frame; 			//current frame
	CamPoseItem* cam; 	//camera pose
	double K[9]; 		//camera intrinsic parameters

	//pointers to the feature points at the current frame
	vector<FeaturePoint*> featPts;

	int nMappedPts;

	KeyFrame* prev;
	KeyFrame* next;

	ImgG img;		//current image
	ImgG imgSmall;  //small image
	double imgScale;

	ImgG imgThumb;	//thumbnail image with the maximum size no larger than 100
public:
	KeyFrame() :
			flag(0), reason(0), camId(0), frame(0), cam(0), nMappedPts(0), prev(
					0), next(0), imgScale(0.3) {
	}
	KeyFrame(int f, CamPoseItem* cam_) :
			flag(0), reason(0), camId(0), frame(f), cam(cam_), nMappedPts(0), prev(
					0), next(0), imgScale(0.3) {
	}
	~KeyFrame() {
		releaseFeatPts();
	}
	void setNumMappedPoints(int num) {
		nMappedPts = num;
	}
	void setAllImages(const ImgG& imgGray, const ImgG& imgGraySmall,
			double scale) {
		cloneImg(imgGray, img);
		cloneImg(imgGraySmall, imgSmall);
		imgScale = scale;
	}
	void setImage(const ImgG& imgGray) {
		cloneImg(imgGray, img);
	}
	void setSmallImage(const ImgG& imgGraySmall, double scale) {
		cloneImg(imgGraySmall, imgSmall);
		imgScale = scale;
	}
	void setThumbImage(const ImgG& imgGraySmall, int maxW) {
		int nw = maxW;
		double ratio = (nw * 1.0) / imgGraySmall.w;
		int nh = (int) (ratio * imgGraySmall.h + 0.5);
		imresize(imgGraySmall, imgThumb, nw, nh);
	}
	void setCameraIntrinsic(const double* intrin) {
		memcpy(K, intrin, sizeof(double) * 9);
	}
	void releaseFeatPts();
	void getMapPoints(Mat_d& pts3d) const;
	void getMapPoints(vector<MapPoint*>& mappts) const;
public:
	//debug
	void print() {
		logInfo("----------++++++++++\n");
		logInfo("frame:%d\n", frame);
		logInfo("nMappedPts:%d\n", nMappedPts);
		logInfo("++++++++++----------\n");
	}
};
class KeyFrameList {
public:
	int num;
	KeyFrame head;
	KeyFrame* tail;
public:
	KeyFrameList();
	~KeyFrameList();
public:
	void clear();
	KeyFrame* add(int f, CamPoseItem* cam_);
	const int size() const {
		return num;
	}
	KeyFrame* current() const {
		return tail;
	}
	KeyFrame* first() const {
		return head.next;
	}
};
#endif /* SL_KEYFRMLST_H_ */
