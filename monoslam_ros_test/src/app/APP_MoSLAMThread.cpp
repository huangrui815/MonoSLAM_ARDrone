/*
 * MoSLAMThread.cpp
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */
#include "SL_GlobParam.h"
#include "SL_MoSLAM.h"
#include "APP_MyApp.h"
#include "APP_SynObj.h"

#include "tools/SL_Tictoc.h"
#include "tools/SL_Timing.h"
#include "slam/SL_SLAMHelper.h"

DEFINE_EVENT_TYPE(EventUpdateViews);
MoSLAMThread::MoSLAMThread() {
}
MoSLAMThread::~MoSLAMThread() {
}
void updateDisplayData() {
//	lockDrawing();
	if (!MyApp::bBusyDrawingVideo && MyApp::videoWnd)
		MyApp::videoWnd->copyDataForDisplay();
	if (!MyApp::bBusyDrawingModel){
	if (MyApp::modelWnd1)
		MyApp::modelWnd1->copyDispData();
	if (MyApp::modelWnd2)
		MyApp::modelWnd2->copyDispData();
	if (MyApp::mapViewWnd)
		MyApp::mapViewWnd->copyDispData();
	}

	char filePath[256];
	if (SLAMParam::saveImage)
		MyApp::videoWnd->save(filePath, MyApp::savedFrames);
//	unlockDrawing();
}

void redrawAllViews() {
	MyApp::redrawViews();
}

MoSLAMThread::ExitCode MoSLAMThread::Entry() {
	if (MyApp::runMode == RUN_MODE_OFFLINE) {
		if (offlineMain()) {
			MyApp::exitProgram();
		}
	} else if (MyApp::runMode == RUN_MODE_ONLINE_USB) {
		if (usbCamMain()) {
			MyApp::exitProgram();
		}
	} else if (MyApp::runMode == RUN_MODE_ROS_ARDRONE) {
		if (ardroneROSMain()) {
			MyApp::exitProgram();
		}
	}
	return 0;
}
