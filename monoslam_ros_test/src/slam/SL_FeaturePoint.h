/*
 * SL_ImgPoint.h
 *
 *  Created on: 2010-11-21
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#ifndef SL_FEATUREPOINT_H_
#define SL_FEATUREPOINT_H_
#include <map>
#include <vector>

#include "math/SL_Matrix.h"
#include "geometry/SL_Point.h"

#include "SL_CameraPose.h"
#include "SL_Define.h"

#define TYPE_FEATPOINT_STATIC  0
#define TYPE_FEATPOINT_DYNAMIC 1

class MapPoint;
class KeyFrame;
class FeaturePoint{
public:
	union{
		struct{
			double xo, yo;	//original coordinates in the image
		};
		double mo[2];
	};
	union{
		struct{
			double x, y;	//undistorted coordinates
		};
		double m[2];
	};

	int f; //frame
	int type; 
	int id;
	bool tracked;
	int virtualNum;

	//pointer to map point
	MapPoint* mpt;
	//pointer to the intrinsic matrix
	const double* K;
	//pointer to the camera pose
	CamPoseItem* cam;
	KeyFrame* keyFrm;
public:
	FeaturePoint();
	~FeaturePoint();

	FeaturePoint(int f1, double x1, double y1);
	FeaturePoint(int f1, double x1, double y1, double xu, double yu);
	FeaturePoint(const FeaturePoint& other);
	FeaturePoint& operator=(const FeaturePoint& other);

	void setIntrinsic(const double* intrn) {
		K = intrn;
	}
	void setCameraPose(CamPoseItem* camPos) {
		cam = camPos;
	}
};
template<class FEAT_PT> 
void vecFeatPt2Mat(const std::vector<FEAT_PT*>& featPts, Mat_d& fpts, bool origin = false){
	int nfpts = (int)featPts.size();
	fpts.resize(nfpts, 2);
	if( !origin){
		for (int i = 0; i < nfpts; i++) {
			fpts.data[2 * i] = featPts[i]->x;
			fpts.data[2 * i + 1] = featPts[i]->y;
		}
	}else{
		for (int i = 0; i < nfpts; i++) {
			fpts.data[2 * i] = featPts[i]->xo;
			fpts.data[2 * i + 1] = featPts[i]->yo;
		}
	}
}
#endif /* SL_IMGPOINT_H_ */
