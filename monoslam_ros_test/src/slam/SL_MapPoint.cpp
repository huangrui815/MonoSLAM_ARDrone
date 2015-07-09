/*
 * SL_MapPoint.cpp
 *
 *  Created on: 2010-11-19
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#include "SL_MapPoint.h"
#include "SL_FeaturePoint.h"
#include "SL_error.h"
#include <cassert>
/////////////////////////////////////////////////////////////////////////////////////////
//MapPoint
MapPoint::MapPoint() :
		Point3dId(), firstFrame(-1), lastFrame(-1), nFrmSmallRepErr(0), state(
				STATE_MAPPOINT_CURRENT), type(0), flag(FLAG_MAPPOINT_NORMAL), pre(0), next(0) {
	id = (longInt) this;
	fill_n(cov, 9, 0);
	fill_n(color, 3, 0);
}
MapPoint::MapPoint(double a, double b, double c) :
		Point3dId(a, b, c, -1), firstFrame(-1), lastFrame(-1), nFrmSmallRepErr(0), state(
				STATE_MAPPOINT_CURRENT), type(0), flag(FLAG_MAPPOINT_NORMAL), pre(0), next(0) {
	id = (longInt) this;
	fill_n(cov, 9, 0);
	fill_n(color, 3, 0);
}
MapPoint::MapPoint(double a, double b, double c, int frame) :
		Point3dId(a, b, c, -1), firstFrame(frame), lastFrame(frame), nFrmSmallRepErr(0), 
			state(STATE_MAPPOINT_CURRENT), type(0),flag(FLAG_MAPPOINT_NORMAL), pre(0), next(0) {
	id = (longInt) this;
	fill_n(cov, 9, 0);
	fill_n(color, 3, 0);
}
MapPoint::MapPoint(const MapPoint& other) :
		Point3dId(other), firstFrame(other.firstFrame), lastFrame(
				other.lastFrame), nFrmSmallRepErr(other.nFrmSmallRepErr),
				state(other.state), type(other.type), flag(other.flag), pre(0), next(0) {
	memcpy(cov, other.cov, sizeof(double) * 9);
}
MapPoint::~MapPoint() {

}
void MapPoint::setFalse() {
	type = TYPE_MAP_FALSE;
}

void MapPoint::setColor(uchar r, uchar g, uchar b) {
	color[0] = r;
	color[1] = g;
	color[2] = b;
}
void MapPoint::setColor(uchar rgb[3]) {
	color[0] = rgb[0];
	color[1] = rgb[1];
	color[2] = rgb[2];
}
void MapPoint::addFeaturePoint(FeaturePoint* fp){
	featPts.push_back(fp);
}
void MapPoint::print() {
	logInfo("------------------\n");
	logInfo("%d:\n", id);

	if (isFalse())
		logInfo("type : false\n");
	else {
		logInfo("local type:'%d'\n", type);
	}

	logInfo("first frame:%d , last frame:%d\n", firstFrame, lastFrame);
	logInfo("==================\n");
}