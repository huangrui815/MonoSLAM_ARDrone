/*
 * SL_Bundler.h
 *
 *  Created on: May 7, 2013
 *      Author: Danping Zou
 */

#ifndef SL_BUNDER_H_
#define SL_BUNDER_H_

#include "slam/SL_KeyFrame.h"
#include "SL_MoSLAMRobustBA.h"

#include <deque>
#include <set>
#include <pthread.h>

using namespace std;

class MoSLAM;
class Bundler {
public:
	bool busy;
	int lastKeyFrmReq;
protected:
	pthread_mutex_t mutexBundler;
	pthread_cond_t condBundler;

	bool running;
	bool toQuit;
	pthread_t bundleThreadId;
	MoSLAM& slam;	
	deque<RobustBundleRTS*> req;
public:
	Bundler(MoSLAM& slam_);
	~Bundler();

	void start();
	void loop();
	void end();

	/* use the recent key frames for bundle adjustment*/
	void requestRecent(int nFixFrm, int nRecentKeyFrm);
	/* use the nearest key frames for bundle adjustment*/
	void requestNearest(int nNearFrm);
	/* use both the recent and nearest key frames for bundle adjustment*/
	void requestCombined(int nRecentKeyFrm, int nNearFrm);

	void bundleAdjustLocal();
	void bundleAdjustFull();

	static void* bundlerProc(void* param);
	bool finished(){
		return req.empty();
	}
};
#endif /* SL_BUNDER_H_ */
