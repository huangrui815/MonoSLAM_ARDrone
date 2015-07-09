
/*
 * SL_CoSLAMHelper.h
 *
 *  Created on: 2011-2-20
 *      Author: Danping Zou
 */

#ifndef SL_COSLAMHELPER_H_
#define SL_COSLAMHELPER_H_
#include "SL_KeyFrame.h"
#include "SL_SLAMHelper.h"

#include "tools/SL_WriteRead.h"
#include "imgproc/SL_ImageOp.h"

/*
 * get the convex hull of a set of 2D points
 */
void get2DConvexHull(const std::vector<double>& pts, std::vector<double>& cxh);

/**
 * update position for static points
 */
void updateStaticPointPosition(MapPoint* mp, double pixelVar,
		bool updateCov = true);

void updatePointUncertainty(MapPoint* mp, double pixelVar);
/**
 * get the area of polygon
 */
double getPolyArea(const std::vector<double>& poly);

void getThumbImage(const ImgG& img, ImgG& thumbImg, int maxw);

double compareThumbImage(const ImgG& img0, const ImgG& img1);

/************************************************************************
* get the position of the feature points pts1 in I1 in the next image (I2)
* pts2 : position in I2
* flag : 1 - success tracking
/************************************************************************/
int trackFeatureKLT( const ImgG& I1, const ImgG& I2, const Mat_d& pts1, Mat_d& pts2, Mat_uc& flag,
	int windowWidth = 12, int pyLevel = 10, double stopEPS = 1);
#endif /* SL_COSLAMHELPER_H_ */