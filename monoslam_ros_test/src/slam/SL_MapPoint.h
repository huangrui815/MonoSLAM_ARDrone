/*
 * SL_Pt3D.h
 *
 *  Created on: 2010-11-19
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#ifndef SL_MAPPOINT_H_
#define SL_MAPPOINT_H_

#include "SL_Define.h"
#include "math/SL_Matrix.h"
#include "slam/SL_NCCBlock.h"

#define FLAG_MAPPOINT_NORMAL 0
#define FLAG_MAPPOINT_TEST1 1
#define FLAG_MAPPOINT_TEST2 2
#define FLAG_MAPPOINT_TEST3 3
#define FLAG_MAPPOINT_TEST4 4

#define TYPE_MAP_FALSE	  -2
#define TYPE_MAP_STATIC  0

#define STATE_MAPPOINT_CURRENT 0
#define STATE_MAPPOINT_ACTIVE 1
#define STATE_MAPPOINT_INACTIVE -1

#include <fstream>
#include "geometry/SL_Point.h"

class FeaturePoint;
class MapPoint: public Point3dId {
public:
	//3x3 covariance matrix to describe position uncertainty
	double cov[9];
	uchar color[3];

	//source frame
	int firstFrame;	//the frame when the map point is reconstructed
	int lastFrame;	//the last frame when the map point can been seen
	int nFrmSmallRepErr;	//number for frames with small re-projection error

	//corresponding	feature points in each key frame
	vector<FeaturePoint*> featPts;

	//state of map point : current frame, active, inactive
	int state;
	//type of map point : static, dynamic, uncertain, and false
	int type; //'static', or 'false'

	//for debug usage
	int flag;

	MapPoint* pre;
	MapPoint* next;
public:
	MapPoint();
	MapPoint(double a, double b, double c);
	MapPoint(double a, double b, double c, int frame);
	MapPoint(const MapPoint& other);
	~MapPoint();
public:
	void setFalse(); /*false map point caused by incorrect matching*/
	bool isFalse() const {
		return type == TYPE_MAP_FALSE;
	}
	void setColor(uchar r, uchar g, uchar b);
	void setColor(uchar rgb[3]);
	void addFeaturePoint(FeaturePoint* fp);
public:
	//for debug	
	void print();
};

template<class MAP_POINT>
void vecMapPt2Mat(const vector<MAP_POINT*> & mptPts, Mat_d& matPts) {
	int npts = (int) mptPts.size();
	matPts.resize(npts, 3);
	for (int i = 0; i < npts; ++i) {
		matPts.data[3 * i] = mptPts[i]->x;
		matPts.data[3 * i + 1] = mptPts[i]->y;
		matPts.data[3 * i + 2] = mptPts[i]->z;
	}
}
#endif /* SL_MAPPOINT_H_ */
