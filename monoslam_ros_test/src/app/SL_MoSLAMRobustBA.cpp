/*
 * SL_RobustBA.cpp
 *
 *  Created on: 2011-7-24
 *      Author: zou
 */

#include "SL_error.h"
#include "SL_MoSLAM.h"
#include "SL_MoSLAMRobustBA.h"
#include "SL_GlobParam.h"

#include "slam/SL_SLAMHelper.h"
#include "slam/SL_MoSLAMHelper.h"
#include "math/SL_LinAlg.h"
#include "geometry/SL_Quaternion.h"
#include "geometry/SL_RigidTransform.h"
#include "geometry/SL_Triangulate.h"
#include "geometry/SL_Geometry.h"
#include <cassert>
#include <algorithm>
#include <utility>

RobustBundleRTS::RobustBundleRTS(MoSLAM& moslam) :
		processed(false), slam(moslam), firstKeyFrame(0), lastKeyFrame(0) {
}
RobustBundleRTS::~RobustBundleRTS() {
}
void RobustBundleRTS::setKeyFrms(vector<KeyFrame*>& frms) {
	map<MapPoint*, vector<FeaturePoint*> > point;

	int minFrm = frms[0]->frame;
	int maxFrm = frms[0]->frame;

	firstKeyFrame = frms[0];
	lastKeyFrame = firstKeyFrame;

	for (size_t i = 0; i < frms.size(); i++) {
		KeyFrame* kf = frms[i];
		addOneKeyFrm(kf);

		if (minFrm > kf->frame) {
			minFrm = kf->frame;
			firstKeyFrame = kf;
		}
		if (maxFrm < kf->frame) {
			maxFrm = kf->frame;
			lastKeyFrame = kf;
		}
	}

	for (size_t i = 0; i < frms.size(); i++) {
		KeyFrame* kf = frms[i];
		for (size_t k = 0; k < kf->featPts.size(); k++) {
			FeaturePoint* fp = kf->featPts[k];
			if (fp->mpt && !fp->mpt->isFalse()) {
				point[fp->mpt].push_back(fp);
			}
		}
	}
	m_nPtsCon = 0;
	typedef map<MapPoint*, vector<FeaturePoint*> >::iterator MPIter;
	for (MPIter iter = point.begin(); iter != point.end(); iter++) {
		MapPoint* mpt = iter->first;
		if (mpt->firstFrame < firstKeyFrame->frame) {
			addOneMapPoint(iter->first, iter->second);
			m_nPtsCon++;
		}
	}

	for (MPIter iter = point.begin(); iter != point.end(); iter++) {
		MapPoint* mpt = iter->first;
		if (mpt->firstFrame >= firstKeyFrame->frame
				&& iter->second.size() > 1) {
			addOneMapPoint(iter->first, iter->second);
		}
	}
}

void RobustBundleRTS::addOneKeyFrm(KeyFrame* kf) {
	frm2ViewId[kf] = keyCamPoses.size();
	Ks.push_back(Mat_d(3, 3, kf->K));
	Rs.push_back(Mat_d(3, 3, kf->cam->R));
	Ts.push_back(Mat_d(3, 1, kf->cam->t));
	keyCamPoses.push_back(kf->cam);
}
void RobustBundleRTS::addOneMapPoint(MapPoint* mp,
		vector<FeaturePoint*>& ftPts) {
	mapPoints.push_back(mp);
	pt3Ds.push_back(Point3d(mp->M[0], mp->M[1], mp->M[2]));
	meas2Ds.push_back(vector<Meas2D>());

	map<int, FeaturePoint*> keyFrm2FeatPts;
	for (size_t i = 0; i < ftPts.size(); i++) {
		FeaturePoint* fp = ftPts[i];
		assert(frm2ViewId.count(fp->keyFrm) > 0);
		int viewId = frm2ViewId[fp->keyFrm];

		if (keyFrm2FeatPts.count(viewId) > 0) {
			FeaturePoint* oldfp = keyFrm2FeatPts[viewId];
			if (fp->f > oldfp->f)
				keyFrm2FeatPts[viewId] = fp;
		} else
			keyFrm2FeatPts[viewId] = fp;
	}

	for (map<int, FeaturePoint*>::iterator iter = keyFrm2FeatPts.begin();
			iter != keyFrm2FeatPts.end(); iter++) {
		meas2Ds.back().push_back(
				Meas2D(iter->first, iter->second->x, iter->second->y));
	}

}
#include "tools/SL_Tictoc.h"
#include <iostream>
void RobustBundleRTS::run() {
	TimeMeasurer tm;
	tm.tic();
	cout << "#points:" << pt3Ds.size() << " #fixed:" << m_nPtsCon << " #views:"
			<< Ks.size() << " #fixed:" << m_nCamsCon << endl;
	
	bundleAdjustRobust(m_nCamsCon, Ks, Rs, Ts, m_nPtsCon, pt3Ds, meas2Ds,
			SLAMParam::MAX_REPROJECT_ERR, m_nMaxIter, m_nInnerMaxIter);
	
	double dt = tm.toc();

	cout << "time:" << dt << endl;
}

void RobustBundleRTS::constructCameraGraphs() {
	assert(firstKeyFrame && lastKeyFrame);
	camGraphs.clear();
	nonKeyCamNodeId.clear();
	nonKeyCamPoses.clear();
	int nTotalNode = 0;
	for (CamPoseItem* cam = firstKeyFrame->cam; cam; cam = cam->next)
		nTotalNode++;

	camGraphs.reserve(nTotalNode, nTotalNode);

	//add nodes
	for (CamPoseItem* cam = firstKeyFrame->cam; cam; cam = cam->next) {
		CamPoseNode* node = camGraphs.newNode();
		node->set(cam->f, cam->R, cam->t);

		//record the pointers
		nonKeyCamNodeId[cam] = node->id;
		nonKeyCamPoses.push_back(cam);
	}

	//set fixed nodes on key frames
	for (KeyFrame* kp = firstKeyFrame; kp && kp->frame <= lastKeyFrame->frame;
			kp = kp->next) {
		CamPoseItem* cam = kp->cam;
		int id = nonKeyCamNodeId[cam];
		CamPoseNode* fixedNode = &camGraphs.poseNodes[id];
		fixedNode->fixed = true;
	}

	//add edges
	CamPoseItem* cam0 = firstKeyFrame->cam->next;
	assert(cam0);
	for (CamPoseItem* cam = cam0; cam; cam = cam->next) {
		int id1 = nonKeyCamNodeId[cam->pre];
		int id2 = nonKeyCamNodeId[cam];
		CamPoseEdge* edge = camGraphs.addEdge();

		double R[9], t[3];
		getRigidTransFromTo(cam->pre->R, cam->pre->t, cam->R, cam->t, R, t);
		edge->set(id1, id2, R, t);
	}
}
void RobustBundleRTS::updateNonKeyCameraPoses() {
	camGraphs.computeNewCameraRotations();
	camGraphs.computeNewCameraTranslations();
	//update the camera poses;
	for (int i = 0; i < camGraphs.nNodes; i++) {
		CamPoseItem* cam = nonKeyCamPoses[i];
		if (!camGraphs.poseNodes[i].fixed) {
			memcpy(cam->R, camGraphs.poseNodes[i].newR, sizeof(double) * 9);
			memcpy(cam->t, camGraphs.poseNodes[i].newt, sizeof(double) * 3);
		}
	}
}
void RobustBundleRTS::updateNewPosesPoints() {
	//update the points
	typedef std::list<MapPoint*>::iterator MapPointListIter;
	for (MapPointListIter iter = slam.curMapPts.begin();
			iter != slam.curMapPts.end(); iter++) {
		MapPoint* mpt = *iter;
		if (mpt->lastFrame < firstKeyFrame->frame)
			continue;
		if (!mpt->isFalse())
			updateStaticPointPosition(mpt, SLAMParam::DETECT_ERR_VAR, true);
	}
}

void RobustBundleRTS::output(bool updateNonKeyCameras) {
	//for updating the camera poses of non-key frames
	if (updateNonKeyCameras) {
		constructCameraGraphs();
	}
	//update the camera positions
	assert(Rs.size() == Ts.size());
	for (size_t c = (size_t) m_nCamsCon; c < Rs.size(); c++) {
		memcpy(keyCamPoses[c]->R, Rs[c].data, sizeof(double) * 9);
		memcpy(keyCamPoses[c]->t, Ts[c].data, sizeof(double) * 3);

		//update the camera poses of fixed nodes in camera graphs ( for updating the non-key frames)
		if (updateNonKeyCameras) {
			int nodeId = nonKeyCamNodeId[keyCamPoses[c]];
			memcpy(camGraphs.poseNodes[nodeId].R, keyCamPoses[c]->R,
					sizeof(double) * 9);
			memcpy(camGraphs.poseNodes[nodeId].t, keyCamPoses[c]->t,
					sizeof(double) * 3);
		}
	}
	//update the map points
	for (size_t i = (size_t) m_nPtsCon; i < pt3Ds.size(); i++) {
		memcpy(mapPoints[i]->M, pt3Ds[i].M, sizeof(double) * 3);
		bool outlier = false;
		for (size_t j = 0; j < meas2Ds[i].size(); j++) {
			if (meas2Ds[i][j].outlier > 0) {
				outlier = true;
				break;
			}
		}
		if (outlier)
			mapPoints[i]->setFalse();
	}
	//update the camera poses of non-key frames
	if (updateNonKeyCameras) {
		updateNonKeyCameraPoses();
		updateNewPosesPoints();
	}
}
