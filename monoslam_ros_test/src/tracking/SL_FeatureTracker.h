/*
 * GPUKLT.h
 *
 *  Created on: Mar 24, 2011
 *      Author: Danping Zou
 */

#ifndef GPUKLT_H_
#define GPUKLT_H_


#include "SL_GPUKLTTracker.h"
#include "SL_Track2D.h"
#include "CVKLTTracker.h"

//class Point2D{
//public:
//	Point2D():x(0),y(0),id(0),frmId(0){};
//	Point2D(int x0, int y0, int id0, int frmId0):x(x0), y(y0), id(id0), frmId(frmId0){};
//	float x, y;
//	int id;
//	int frmId;
//};
class PointsInImage{
public:
	PointsInImage(){};
	void clear(){ptMap.clear();};
	void add(int id, FeaturePoint* pt){
		if(ptMap.count(id) == 0)
			ptMap[id] = pt;
	}
	map<int, FeaturePoint*> ptMap;
	vector<FeaturePoint*> pts;
	vector<int> ptsId;
};

class ImagePair{
public:
	ImagePair():frmId0(0), frmId1(0){};
	void clear(){pointPair.clear();pointPair_reverse.clear();frmId0 = 0; frmId1 = 0;};
	void add(int id0, int id1){
		pointPair[id0] = id1;
		pointPair_reverse[id1] = id0;
	}

	map<int, int> pointPair, pointPair_reverse;
	int frmId0, frmId1;
};


class FeatureTracker {
public:
	int frame_;
	int W_, H_;
	int maxTrackNum_;
	int trackedFeatureNum_;
	int detectedFeatureNum_;
	double trackRatio;	//# of tracked features / # of feature points in the previous frame
	//intrinsic parameters
	Mat_d K_, invK_, kUnDist_;
	double _kc[5];
	Track2D *tracks;
//	GPUKLTTracker klt_;
	CVKLTTracker klt_;
	bool _bLost;

	vector<ImagePair> imgPairs;
	vector<PointsInImage> ptsInImage;
	bool firstTracking;

	float *_corners;
	ImgG oldImg;
	ImgG bufferImg;
	bool _bRetrack;
public:
	void updateTracks(const Mat_d& pts, const Mat_i& flag);
	void updateTracks01(const Mat_d& pts, const Mat_i& flag, const ImgG& img);
public:
	FeatureTracker();
	virtual ~FeatureTracker();
	void setIntrinsicParam(const double* K, const double* invK,
			const double* k_ud, double* kc) {
		K_.cloneFrom(K, 3, 3);
		invK_.cloneFrom(invK, 3, 3);
		kUnDist_.cloneFrom(k_ud, 7, 1);
		memcpy(_kc, kc, 5 * sizeof(double));
	}
public:
	//routines for KLT tracking
	void openGPU(int W, int H,	V3D_GPU::KLT_SequenceTrackerConfig* pCfg);
	
	//detect corners at the first frame to start KLT tracking
	//  frame : start frame
	//	img : 8-bit gray image
	void first(int frame, const ImgG& img);
	int next(const ImgG& img);
	void rollBack();

	void getFeatPoints(vector<TrackedFeaturePoint*>& featPts);
	void getCurrFeatures();
	void getMappedFlags();
	void provideCurrFeatures();
	void reset(const ImgG& img, const vector<FeaturePoint>& vecPts);
	void undistorPointNew(double* K, double* kc, double* x, double* x_undist);
	
	int getNumOfFeatPts(){
		return trackedFeatureNum_ + detectedFeatureNum_;
	}
	bool readFMatrix(string path);
	bool readFMatrix01(string path);
};
#endif /* GPUKLT_H_ */
