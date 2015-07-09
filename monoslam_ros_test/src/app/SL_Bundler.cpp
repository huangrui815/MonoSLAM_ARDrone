/*
 * SL_Bundler.cpp
 *
 *  Created on: May 7, 2013
 *      Author: Danping Zou
 */

#include "SL_Bundler.h"
#include "SL_MoSLAM.h"
#include <iostream>
#include "APP_MyApp.h"
#include "APP_SynObj.h"

using namespace std;

#define CHECK_QUIT if(bQuit) {break;};

Bundler::Bundler(MoSLAM& slam_) :
		slam(slam_) {

	pthread_mutex_init(&mutexBundler, 0);
	pthread_cond_init(&condBundler, 0);

	running = false;
	toQuit = false;
	busy = false;
	lastKeyFrmReq = 0;
#ifdef WIN32
	bundleThreadId = pthread_self();
#else
	bundleThreadId = 0;
#endif
	start();
}

Bundler::~Bundler() {
	end();
}
void Bundler::start() {
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&bundleThreadId, &attr, bundlerProc, this);
}

void Bundler::loop() {
	running = true;
	while (!toQuit) {
		bundleAdjustLocal();
	}
	running = false;
}

void Bundler::end() {
	if (running) {
		toQuit = true;
		pthread_mutex_lock(&mutexBundler);
		pthread_cond_signal(&condBundler);
		pthread_mutex_unlock(&mutexBundler);
		cout << "waiting the bundler thread to quit..." << endl;
		pthread_join(bundleThreadId, 0);
		cout << "bundler thread quit!" << endl;
	}
	//release the unhanlded requests;
	for (size_t i = 0; i < req.size(); i++) {
		delete req[i];
	}
}

void Bundler::requestRecent(int nFixFrm, int nRecentFrm) {
	vector<KeyFrame*> frms;
	int num = 0;
	for (KeyFrame* kf = slam.keyFrm.current();
			kf && kf->frame >= lastKeyFrmReq && num < nRecentFrm; kf =
					kf->prev) {
		frms.push_back(kf);
		num++;
	}

	if (num < nRecentFrm)
		return;

	num = 0;
	for (KeyFrame* kf = frms.back()->prev; kf && num < nFixFrm; kf = kf->prev) {
		frms.push_back(kf);
		num++;
	}

	lastKeyFrmReq = frms[0]->frame;
	RobustBundleRTS* pBA = new RobustBundleRTS(slam);
	pBA->setKeyFrms(frms);
	pBA->setParameters(num < 2 ? 2:num, 2, 30);

	pthread_mutex_lock(&mutexBundler);
	req.push_back(pBA);
	pthread_cond_signal(&condBundler);
	pthread_mutex_unlock(&mutexBundler);
}
void Bundler::requestNearest(int nNearFrm) {
	vector<KeyFrame*> frms;
	slam.searchNearestKeyFrm(frms, nNearFrm);

	RobustBundleRTS* pBA = new RobustBundleRTS(slam);
	pBA->setKeyFrms(frms);
	pBA->setParameters(2, 2, 30);

	pthread_mutex_lock(&mutexBundler);
	req.push_back(pBA);
	pthread_cond_signal(&condBundler);
	pthread_mutex_unlock(&mutexBundler);

}
void Bundler::requestCombined(int nRecentFrm, int nNearFrm) {
	vector<KeyFrame*> frms;
	slam.searchNearestKeyFrm(frms, nNearFrm);
	int num = 0;
	for (KeyFrame* kf = slam.keyFrm.tail;
			kf != &slam.keyFrm.head && num < nRecentFrm; kf = kf->prev) {
		frms.push_back(kf);
		num++;
	}

	RobustBundleRTS* pBA = new RobustBundleRTS(slam);
	pBA->setKeyFrms(frms);
	pBA->setParameters(2, 2, 30);

	pthread_mutex_lock(&mutexBundler);
	req.push_back(pBA);
	pthread_cond_signal(&condBundler);
	pthread_mutex_unlock(&mutexBundler);
}

void Bundler::bundleAdjustLocal() {
	pthread_mutex_lock(&mutexBundler);
	RobustBundleRTS* pBA = req.empty() ? 0 : req.front();
	if (!pBA)
		pthread_cond_wait(&condBundler, &mutexBundler);
	pthread_mutex_unlock(&mutexBundler);

	if (pBA) {
		busy = true;
		req.pop_front();
		pBA->run();
		enterBACriticalSection();
		pBA->output();
		delete pBA;
		leaveBACriticalSection();
		busy = false;
	}
}

void Bundler::bundleAdjustFull() {
	vector<KeyFrame*> frms;
	int num = 0;
	for (KeyFrame* kf = slam.keyFrm.current();
			kf; kf = kf->prev) {
		frms.push_back(kf);
		num++;
	}

//	num = 0;
//	for (KeyFrame* kf = frms.back()->prev; kf && num < nFixFrm; kf = kf->prev) {
//		frms.push_back(kf);
//		num++;
//	}

	KeyFrame* kf = frms.front();
	frms.push_back(kf);

//	lastKeyFrmReq = frms[0]->frame;
	RobustBundleRTS* pBA = new RobustBundleRTS(slam);
	pBA->setKeyFrms(frms);
	pBA->setParameters(1, 2, 30);

	pthread_mutex_lock(&mutexBundler);
	req.push_back(pBA);
	pthread_cond_signal(&condBundler);
	pthread_mutex_unlock(&mutexBundler);
}

void* Bundler::bundlerProc(void* param) {
	Bundler* bundler = (Bundler*) param;
	bundler->loop();
	return 0;
}
