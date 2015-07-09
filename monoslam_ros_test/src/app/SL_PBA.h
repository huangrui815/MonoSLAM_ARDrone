
#ifndef SL_PBA_BUNDER_H_
#define SL_PBA_BUNDER_H_
#include <string.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include "pba/pba.h"
#include "slam/SL_KeyFrame.h"

using namespace std;

class MoSLAM;
class PBABundler {
public:
	bool busy;
	int lastKeyFrmReq;
protected:
	pthread_mutex_t mutexBundler;
	pthread_cond_t condBundler;

	bool running;
	bool toQuit;
	pthread_t bundleThreadId;
	MoSLAM* _slam;
	ParallelBA* _pba;
//	deque<RobustBundleRTS*> req;

	deque<vector<KeyFrame*> > _kfReqVec;

	vector<KeyFrame*>      _kfVec;
	vector<MapPoint*>      _mptVec;
    vector<CameraT>        _camera_data;    //camera (input/ouput)
    vector<Point3D>        _point_data;     //3D point(iput/output)
    vector<Point2D>        _measurements;   //measurment/projection vector
    vector<int>            _camidx, _ptidx;  //index of camera/point for each projection
public:
	PBABundler();
	~PBABundler();
	void setSLAM(MoSLAM* slam);
	bool setRecentKeyFrms(int nRecentFrm = 5);
	bool setKeyframeVec(vector<KeyFrame*>& kfVec);
	void run();
	void update();
	void reset();

	void start();
	void loop();
	void end();

	/* use the recent key frames for bundle adjustment*/
	bool requestRecent(int nRecentKeyFrm = 5);
	/* use the nearest key frames for bundle adjustment*/
	void requestNearest(int nNearFrm);
	/* use both the recent and nearest key frames for bundle adjustment*/
	void requestCombined(int nRecentKeyFrm, int nNearFrm);

	void bundleAdjustLocal();
	void bundleAdjustFull();

	static void* bundlerProc(void* param);
};
#endif /* SL_PBA_BUNDER_H_ */
