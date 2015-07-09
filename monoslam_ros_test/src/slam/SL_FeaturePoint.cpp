/*
 * SL_ImgPoint.cpp
 *
 *  Created on: 2010-11-21
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#include "SL_FeaturePoint.h"
#include "SL_MapPoint.h"
#include "SL_error.h"
#include <cstddef>
#include <stdint.h>

FeaturePoint::FeaturePoint() :
		xo(-1), yo(-1), x(-1), y(-1), f(-1), type(0), mpt(0),K(0), cam(0), keyFrm(0), id(0) {
	tracked = false;
	virtualNum = 0;
}
FeaturePoint::FeaturePoint(int f1, double x1, double y1) :
		xo(x1), yo(y1), x(x1), y(y1), f(f1), type(0), mpt(0), K(0),cam(0), keyFrm(0), id(0) {
	tracked = false;
	virtualNum = 0;
}
FeaturePoint::FeaturePoint(int f1, double x1, double y1, double xu, double yu) :
		xo(x1), yo(y1), x(xu), y(yu), f(f1), type(0), mpt(0), K(0), cam(0), keyFrm(0), id(0) {
	tracked = false;
	virtualNum = 0;
}
FeaturePoint::FeaturePoint(const FeaturePoint& other) {
	operator=(other);
}
FeaturePoint& FeaturePoint::operator=(const FeaturePoint& other) {
	if (&other != this) {
		xo = other.xo;
		yo = other.yo;
		x = other.x;
		y = other.y;
		f = other.f;
		type = other.type;
		mpt = other.mpt;
		K = other.K;
		cam = other.cam;
		keyFrm = other.keyFrm;
		id = other.id;
		tracked = other.tracked;
		virtualNum = other.virtualNum;
	}
	return *this;
}
FeaturePoint::~FeaturePoint() {
}

