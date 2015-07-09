/*
 * SL_CoSLAMHelper.cpp
 *
 *  Created on: 2011-2-20
 *      Author: Danping Zou
 */

#include "SL_MoSLAMHelper.h"
#include "SL_MapPoint.h"

#include "math/SL_LinAlg.h"
#include "geometry/SL_Geometry.h"
#include "geometry/SL_Distortion.h"
#include "geometry/SL_ConvexHull2D.h"
#include "geometry/SL_Triangulate.h"
#include "tools/SL_TypeConversion.h"
#include <algorithm>
/**
 * update position for static points
 */
void updateStaticPointPosition(MapPoint* mp, double pixelVar, bool updateCov) {
	size_t nview = mp->featPts.size();
	if (nview < 2)
		return;

	Mat_d Ks(nview, 9), Rs(nview, 9), ts(nview, 3), ms(nview, 2), nms(nview, 2);

	for (size_t c = 0; c < nview; c++) {
		const FeaturePoint* fp = mp->featPts[c];
		assert(fp->K && fp->cam);
		const double* K = fp->K;
		const double* R = fp->cam->R;
		const double* t = fp->cam->t;
		const double* m = fp->m;

		doubleArrCopy(Ks, c, K, 9);
		doubleArrCopy(Rs, c, R, 9);
		doubleArrCopy(ts, c, t, 3);
		doubleArrCopy(ms, c, m, 2);

		double iK[9];
		getInvK(K, iK);
		normPoint(iK, m, nms + 2 * c);
	}
	triangulateMultiView((int) nview, Rs, ts, nms, mp->M);
	if (updateCov)
		getTriangulateCovMat((int) nview, Ks, Rs, ts, mp->M, mp->cov, pixelVar);
}


void updatePointUncertainty(MapPoint* mp, double pixelVar) {
	if (mp->featPts.size() < 2)
		return;

	size_t nview = mp->featPts.size();
	Mat_d Ks(nview, 9), Rs(nview, 9), ts(nview, 3), ms(nview, 2), nms(nview, 2);

	for (size_t c = 0; c < nview; c++) {
		const FeaturePoint* fp = mp->featPts[c];
		assert(fp->K && fp->cam);
		const double* K = fp->K;
		const double* R = fp->cam->R;
		const double* t = fp->cam->t;
		const double* m = fp->m;

		doubleArrCopy(Ks, c, K, 9);
		doubleArrCopy(Rs, c, R, 9);
		doubleArrCopy(ts, c, t, 3);
		doubleArrCopy(ms, c, m, 2);

		double iK[9];
		getInvK(K, iK);
		normPoint(iK, m, nms + 2 * c);
	}
	getTriangulateCovMat((int) nview, Ks, Rs, ts, mp->M, mp->cov, pixelVar);
}



void getThumbImage(const ImgG& img, ImgG& thumbImg, int maxw) {
	assert(maxw > 0);

	int nw = maxw;
	double ratio = (nw * 1.0) / img.w;
	int nh = static_cast<int>(img.h * ratio + 0.5);
	imresize(img, thumbImg, nw, nh);
}

double compareThumbImage(const ImgG& img0, const ImgG& img1){
	assert(!img0.empty() && img0.w == img1.w && img0.h == img1.h);
	double avg_I0 = 0, avg_I1 = 0;
	int len = img0.w * img0.h;
	for( int i = 0; i < len; i++){
		avg_I0 += img0.data[i];
		avg_I1 += img1.data[i];
	}
	avg_I0 /= len;
	avg_I1 /= len;
	
	double diff = 0;
	for( int i = 0; i < len; i++){ 
		diff += fabs((img0.data[i] - avg_I0) - (img1.data[i] - avg_I1));
//		diff += fabs((1.0*img0.data[i] - 1.0*img1.data[i]));
	}
	return diff/len;
}

int trackFeatureKLT( const ImgG& I1, const ImgG& I2, const Mat_d& pts1, Mat_d& pts2, Mat_uc& flag,
	int windowWidth, int pyLevel, double stopEPS){
	cv::Mat cvI1(I1.m, I1.n, CV_8UC1, I1.data);
	cv::Mat cvI2(I2.m, I2.n, CV_8UC1, I2.data);

	pts2.cloneFrom(pts1);

	Mat_f fpts1, fpts2;
	matDouble2Float(pts1, fpts1);
	matDouble2Float(pts2, fpts2);

	cv::Mat cvPts1(fpts1.m, fpts1.n, CV_32FC1, fpts1.data);
	cv::Mat cvPts2(fpts2.m, fpts2.n, CV_32FC1, fpts2.data);
	cv::Mat cvStatus(fpts1.m, 1, CV_8UC1);
	cv::Mat cvErr(fpts1.m, 1, CV_8UC1);

	cv::calcOpticalFlowPyrLK(cvI1, cvI2, cvPts1, cvPts2, cvStatus, cvErr,
			cv::Size(windowWidth, windowWidth), pyLevel,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
					20, stopEPS), cv::OPTFLOW_USE_INITIAL_FLOW);

	matFloat2Double(fpts2, pts2);

	int nMatched = 0;
	uchar* pstatus = (uchar*) cvStatus.data;
	flag.resize(fpts1.m, 1);
	for (int i = 0; i < fpts1.m; i++) {
		if (pstatus[i] > 0) {
			flag.data[i] = 1;	//matched
			nMatched++;
		} else
			flag.data[i] = 0;	//unmatched
	}
	return nMatched;
}
