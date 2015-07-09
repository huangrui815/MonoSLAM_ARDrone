/*
 * SL_MapMaker.h
 *
 *  Created on: May 9, 2013
 *      Author: Danping Zou
 * 	 	E-mail: Dannis.zou@gmail.com
 */

#ifndef SL_MAPMAKER_H_
#define SL_MAPMAKER_H_
#include "slam/SL_MapPoint.h"
#include "slam/SL_CameraPose.h"

#include "math/SL_Matrix.h"
#include "tf/transform_datatypes.h"
#include <vector>
#include <list>

class DroneKalmanFilter;

using namespace std;

class MoSLAM;
class MapMaker {
public:
	bool doInit;
	MoSLAM& slam;
	int numNewMapPts;

	tf::Transform _firstPredictedCamPose;
	tf::Transform _secondPredictedCamPose;
	float _initialScaleToFilter;
//	DroneKalmanFilter* _filter;

public:
	MapMaker(MoSLAM& theSLAM);
	virtual ~MapMaker();

	bool startInit(tf::Transform& firstPredictedCamPose);
	bool startInit();
	bool endInit(tf::Transform& secondPredictedCamPose);
	bool endInit();
	
	int triangulateTracks(vector<MapPoint*>& mapPts, double maxEpiErr);
	int genNewMapPoints();

	double normalizeInitMap(std::list<MapPoint*>& mapPts, double scale);
};
double getViewAngleChange(const double center[3], const CamPoseItem* cam1,
		const CamPoseItem* cam2);

void refineInitMap(const Mat_d& K, Mat_d& R1, Mat_d& t1, Mat_d& R2,
		Mat_d& t2, Mat_d& pts1, Mat_d& pts2, Mat_d& Ms);

tf::Transform getTfFromRT(Mat_d& R, Mat_d& t);
#endif /* SL_MAPMAKER_H_ */
