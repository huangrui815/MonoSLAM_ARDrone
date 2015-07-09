/*
 * SL_RobustBA.h
 *
 *  Created on: 2011-7-24
 *      Author: zou
 */

#ifndef SL_ROBUSTBA_H_
#define SL_ROBUSTBA_H_

/*
 * SL_Bundle.h
 *
 *  Created on: 2011-1-20
 *      Author: Danping Zou
 */

#include "math/SL_Matrix.h"
#include "slam/SL_CameraPose.h"
#include "slam/SL_MapPoint.h"
#include "slam/SL_FeaturePoint.h"
#include "slam/SL_KeyFrame.h"
#include "slam/SL_GlobalPoseEstimation.h"
#include "geometry/SL_BundleAdjust.h"
#include <vector>

using namespace std;
/*
 * use bundle adjustment to refine the camera poses and 3D points simultaneously
 */
class MoSLAM;
class RobustBundleRTSParameter {
public:
	//parameters
	int m_nPtsCon, m_nCamsCon, m_nMaxIter, m_nInnerMaxIter;
public:
	RobustBundleRTSParameter() :
			m_nPtsCon(0), m_nCamsCon(0), m_nMaxIter(2), m_nInnerMaxIter(30) {
	}
	void setParameters(int nCamsCon, int nMaxIter, int nInnerMaxIter) {
		m_nCamsCon = nCamsCon;
		m_nMaxIter = nMaxIter;
		m_nInnerMaxIter = nInnerMaxIter;
	}
};
class RobustBundleRTS: public RobustBundleRTSParameter {
public:
	bool processed;
	MoSLAM& slam;

	KeyFrame* firstKeyFrame;
	KeyFrame* lastKeyFrame;

	vector<CamPoseItem*> keyCamPoses;
	vector<MapPoint*> mapPoints;

	map<KeyFrame*, int> frm2ViewId;

	/* for correcting the camera poses of non-key frames*/
	GlobalPoseGraph camGraphs;
	map<CamPoseItem*, int> nonKeyCamNodeId;
	vector<CamPoseItem*> nonKeyCamPoses;

	/* temporary variables*/
	vector<Mat_d> Ks, Rs, Ts;
	vector<Point3d> pt3Ds;
	vector<vector<Meas2D> > meas2Ds;
public:
	RobustBundleRTS(MoSLAM& moslam);
	virtual ~RobustBundleRTS();
protected:
	bool checkReprojError(
			map<MapPoint*, std::vector<const FeaturePoint*> >::iterator& iter,
			double errMax);
public:
	//////////////////////////////////////////////////////////////////////////////////////////////
	/* the following is for bundle adjustment*/
	void setKeyFrms(vector<KeyFrame*>& adjFrms);

	void addOneKeyFrm(KeyFrame*);
	void addOneMapPoint(MapPoint* mpt, vector<FeaturePoint*>& ftPts);
	//////////////////////////////////////////////////////////////////////////////////////////////
	/* the following is for updating the camera poses of non-key frames*/
	void constructCameraGraphs();
	void updateNonKeyCameraPoses();
public:
	void run();
	void output(bool updateNonKeyPoses = false);
	/************************************************************************/
	/* update the value of new camera poses and new map points generated during
	 /* calling bundle adjustment
	 /************************************************************************/
	void updateNewPosesPoints();
};
#endif /* SL_ROBUSTBA_H_ */
