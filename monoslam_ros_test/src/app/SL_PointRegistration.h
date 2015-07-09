/*
 * SL_PointRegistration.h
 *
 *  Created on: May 9, 2013
 *      Author: Danping Zou
 * 	 	E-mail: Dannis.zou@gmail.com
 */

#ifndef SL_POINTREGISTRATION_H_
#define SL_POINTREGISTRATION_H_

#include "math/SL_Matrix.h"
#include "slam/SL_FeaturePoint.h"
#include "slam/SL_MapPoint.h"
#include "tracking/SL_Track2D.h"
#include <vector>
#include <cassert>

using namespace std;

class MoSLAM;
class PointRegister {
public:
	MoSLAM& slam;
	
	double scaleRatio;
	Mat_i mappedRegion;
	vector<TrackedFeaturePoint*> unMappedFeatPts;
	
	int nRegTried;
	int nRegDeepTried;
public:
	PointRegister(MoSLAM& theSLAM);
	virtual ~PointRegister();

	void genMappedMask();
	void getUnMappedFeatPoints();
	
	bool regOnePoint(MapPoint* mp, double pixelErrVar);
	int process(double pixelErrVar);
};

/* functions for mapPointsRegister */
bool compareFeaturePt(const ImgG& scaledImg1, double imgScale1,
		const ImgG& scaledImg2, double imgScale2, const FeaturePoint* pt1,
		const FeaturePoint* pt2);

FeaturePoint* searchNearestFeatPt(const vector<TrackedFeaturePoint*>& featPts,
		double m[2], double maxDist);

bool isRegistable(MapPoint* mp, FeaturePoint* fp, double pixelVar);
void updatePointUncertainty(MapPoint* mp, FeaturePoint* fp, double pixelVar);
#endif /* SL_POINTREGISTRATION_H_ */
