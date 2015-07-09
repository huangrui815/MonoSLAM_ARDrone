/*
 * SL_PointRegistration.cpp
 *
 *  Created on: May 9, 2013
 *      Author: Danping Zou
 * 	 	E-mail: Dannis.zou@gmail.com
 */

#include "SL_PointRegistration.h"
#include "SL_GlobParam.h"
#include "geometry/SL_Triangulate.h"
#include "geometry/SL_Geometry.h"
#include "geometry/SL_Distortion.h"

#include "math/SL_LinAlg.h"

#include "SL_MoSLAM.h"
#include "APP_SynObj.h"

PointRegister::PointRegister(MoSLAM& theSLAM) :
		slam(theSLAM) {
	scaleRatio = 1.0 / 10.0;
}

PointRegister::~PointRegister() {
}

bool compareFeaturePt(const ImgG& scaledImg1, double imgScale1,
		const ImgG& scaledImg2, double imgScale2, const FeaturePoint* pt1,
		const FeaturePoint* pt2) {

	NCCBlock nccblk1, nccblk2;
	getScaledNCCBlock(scaledImg1, imgScale1, pt1->xo, pt1->yo, nccblk1);
	getScaledNCCBlock(scaledImg2, imgScale2, pt2->xo, pt2->yo, nccblk2);

	double ncc = matchNCCBlock(&nccblk1, &nccblk2);
	if (ncc >= 0.6)
		return true;

	return false;
}

void PointRegister::genMappedMask() {
	int sw = (int) slam.tracker.img.w * scaleRatio;
	int sh = (int) slam.tracker.img.h * scaleRatio;

	mappedRegion.resize(sh, sw);
	mappedRegion.fill(0);

	//compute the mask spanned by the mapped feature points
	for (size_t i = 0; i < slam.tracker.curFeatPts.size(); i++) {
		if (slam.tracker.curFeatPts[i]->pt.mpt) {
			double x = slam.tracker.curFeatPts[i]->pt.x * scaleRatio;
			double y = slam.tracker.curFeatPts[i]->pt.y * scaleRatio;
			int ix = (int) (x + 0.5);
			int iy = (int) (y + 0.5);

			if (ix < 0 || ix >= sw || iy < 0 || iy >= sh)
				continue;
			mappedRegion(iy, ix) = 1;
		}
	}
}

void PointRegister::getUnMappedFeatPoints() {
	int sw = (int) slam.tracker.img.w * scaleRatio;
	int sh = (int) slam.tracker.img.h * scaleRatio;

	unMappedFeatPts.clear();
	for (size_t i = 0; i < slam.tracker.curFeatPts.size(); i++) {
		if (slam.tracker.curFeatPts[i]->pt.mpt == 0) {
			double x = slam.tracker.curFeatPts[i]->pt.x * scaleRatio;
			double y = slam.tracker.curFeatPts[i]->pt.y * scaleRatio;
			int ix = (int) (x + 0.5);
			int iy = (int) (y + 0.5);

			if (ix < 0 || ix >= sw || iy < 0 || iy >= sh || mappedRegion(iy, ix) == 0)
				unMappedFeatPts.push_back(slam.tracker.curFeatPts[i]);
		}
	}
}

bool PointRegister::regOnePoint(MapPoint* mp, double pixelErrVar) {
	if (nRegTried > SLAMParam::maxRegTryNum)
		return false;
	assert(mp);
	double m[2];

	double* R_cur = slam.camPose.current()->R;
	double* t_cur = slam.camPose.current()->t;

	if (isAtCameraBack(R_cur, t_cur, mp->M))
		return false;

	project(slam.tracker.K.data, R_cur, t_cur, mp->M, m);
	
	if (m[0] < 0 || m[0] >= slam.tracker.videoReader->_w || m[1] < 0
			|| m[1] >= slam.tracker.videoReader->_h)
		return false;
	
	int ix = (int)(m[0]*scaleRatio+0.5);
	int iy = (int)(m[1]*scaleRatio+0.5);
	
	if( mappedRegion(iy, ix) != 0)
		return false;

	FeaturePoint* pFeat = searchNearestFeatPt(unMappedFeatPts,
			m, SLAMParam::POINT_REG_RANGE);

	if (pFeat > 0) {
		nRegTried++;
		if (pFeat->mpt == 0) {
			nRegDeepTried++;
			//find a feature point to be registered
			//compared with the corresponding feature point in the nearest frame
			FeaturePoint* fp_min = mp->featPts.back();
			if (compareFeaturePt(slam.tracker.smallImg, slam.tracker.smallScale,
					fp_min->keyFrm->imgSmall, fp_min->keyFrm->imgScale, pFeat,
					fp_min)) {

				if (isRegistable(mp, pFeat, SLAMParam::MAX_REPROJECT_ERR)) {
					updatePointUncertainty(mp, pFeat, pixelErrVar);
					pFeat->mpt = mp;
					return true;
				}
			}
		}
	}
	return false;

}
int PointRegister::process(double pixelErrVar) {
	/*==============(critical section)=================*/
	enterBACriticalSection();
	bool empty = slam.actMapPts.empty();
	leaveBACriticalSection();
	if (empty)
		return 0;
	/*------------------------------------------------*/
	vector<MapPoint*> vecActMapPts;
	vecActMapPts.reserve(slam.actMapPts.size() * 2);

	/*==============(critical section)=================*/
	enterBACriticalSection();
	typedef std::list<MapPoint*>::iterator MapPointListIter;
	for (MapPointListIter iter = slam.actMapPts.begin();
			iter != slam.actMapPts.end(); iter++) {
		MapPoint* p = *iter;
		vecActMapPts.push_back(p);
	}
	slam.actMapPts.clear();
	slam.nActMapPts = (int) vecActMapPts.size();
	leaveBACriticalSection();
	/*------------------------------------------------*/

	/*get the mask and unmapped feature points for registration*/
	genMappedMask();
	getUnMappedFeatPoints();

	vector<bool> flagReged;
	flagReged.resize((size_t) slam.nActMapPts, false);
	/*==============(critical section)=================*/
	enterBACriticalSection();
	nRegTried = 0;
	nRegDeepTried = 0;
	for (int i = 0; i < slam.nActMapPts; i++) {
		MapPoint* p = vecActMapPts[i];
		if (!p->isFalse()) {
			flagReged[i] = regOnePoint(p, pixelErrVar);
		}
	}
	leaveBACriticalSection();
	/*------------------------------------------------*/

	/*==============(critical section)=================*/
	enterBACriticalSection();
	int nReg = 0;
	for (size_t i = 0; i < slam.nActMapPts; i++) {
		MapPoint* p = vecActMapPts[i];
		if (flagReged[i] == false) {
			slam.actMapPts.push_back(p);
		} else {
			p->lastFrame = slam.curFrame;
			p->state = STATE_MAPPOINT_CURRENT;
			slam.curMapPts.push_back(p);
			nReg++;
		}
	}
	leaveBACriticalSection();
	return nReg;
}

FeaturePoint* searchNearestFeatPt(const vector<TrackedFeaturePoint*>& featPts,
		double m[2], double maxDist) {
	double dMin = (std::numeric_limits<double>::max)();
	FeaturePoint* pMin = 0;
	for (size_t i = 0; i < featPts.size(); i++) {
		FeaturePoint* p = &featPts[i]->pt;
		double d = dist2(m, p->m);
		if (d < maxDist && d < dMin) {
			dMin = d;
			pMin = p;
		}
	}
	return pMin;
}
bool isRegistable(MapPoint* mp, FeaturePoint* new_fp, double pixelVar) {
	if (mp->featPts.size() == 0)
		return false;

	size_t nview = mp->featPts.size();
	Mat_d Ks(nview + 1, 9), Rs(nview + 1, 9), ts(nview + 1, 3), ms(nview + 1,
			2), nms(nview + 1, 2);

	double iK[9];
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

		getInvK(K, iK);
		normPoint(iK, m, nms + 2 * c);
	}

	doubleArrCopy(Ks, nview, new_fp->K, 9);
	doubleArrCopy(Rs, nview, new_fp->cam->R, 9);
	doubleArrCopy(ts, nview, new_fp->cam->t, 3);
	doubleArrCopy(ms, nview, new_fp->m, 2);
	normPoint(iK, new_fp->m, nms + 2 * nview);

	double M[3], m[2];
	triangulateMultiView((int) (nview + 1), Rs, ts, nms, M);

	//check the re-projection error
	for (size_t c = 0; c < nview; c++) {
		const FeaturePoint* fp = mp->featPts[c];
		assert(fp->K && fp->cam);
		project(fp->K, fp->cam->R, fp->cam->t, M, m);
		if (dist2(m, fp->m) > pixelVar)
			return false;
	}

	project(new_fp->K, new_fp->cam->R, new_fp->cam->t, M, m);
	if (dist2(m, new_fp->m) > pixelVar)
		return false;
	return true;
}

void updatePointUncertainty(MapPoint* mp, FeaturePoint* new_fp,
		double pixelVar) {
	if (mp->featPts.size() == 0)
		return;

	size_t nview = mp->featPts.size();
	Mat_d Ks(nview + 1, 9), Rs(nview + 1, 9), ts(nview + 1, 3), ms(nview + 1,
			2), nms(nview + 1, 2);

	double iK[9];
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

		getInvK(K, iK);
		normPoint(iK, m, nms + 2 * c);
	}

	doubleArrCopy(Ks, nview, new_fp->K, 9);
	doubleArrCopy(Rs, nview, new_fp->cam->R, 9);
	doubleArrCopy(ts, nview, new_fp->cam->t, 3);
	doubleArrCopy(ms, nview, new_fp->m, 2);
	normPoint(iK, new_fp->m, nms + 2 * nview);

	getTriangulateCovMat((int) (nview + 1), Ks, Rs, ts, mp->M, mp->cov,
			pixelVar);
}

