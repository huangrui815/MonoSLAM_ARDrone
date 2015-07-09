/*
 * MyApp.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#ifndef MYAPP_H_
#define MYAPP_H_

#include "APP_EstimationNode.h"
#include "stateestimation/Drone_Filter.h"
#include "app/SL_MoSLAM.h"
#include "app/SL_GlobParam.h"

#include "videoReader/VR_AVIReader.h"
#include "videoReader/VR_USBCamReader.h"
#include "videoReader/VR_DroneCamReader.h"

#include "pthread.h"

#include "gui/GLImageWnd.h"
#include "gui/GLSceneWnd.h"
#include "stateestimation/Drone_MapViewWnd.h"
#include "gui/MainDialog.h"
#include "APP_MoSLAMThread.h"
#include "tf/transform_datatypes.h"

#include "wx/wx.h"


DECLARE_EVENT_TYPE(CloseApp, -1);

#define RUN_MODE_OFFLINE 0
#define RUN_MODE_ONLINE_ARDRONE 1
#define RUN_MODE_ONLINE_USB 2
#define RUN_MODE_ONLINE_MINI_CAM 3
#define RUN_MODE_ROS_ARDRONE 4

#define TIMER_ID wxID_HIGHEST + 1000

#define PI_MATH 3.14159268
#define RADIAN_PER_DEGREE 3.14159268 / 180
#define DEGREE_PER_RADIAN 180 / 3.14159268

class MyApp: public wxApp {
protected:
	void initSyncVars();
	virtual int OnExit();
	virtual bool OnInit();

	bool initOffline();
	bool initUSBCam();
	bool initARDrone();
	bool initMiniCam();
	bool initRosARDrone();

	int readScript(string& filePath);
	vector<int> mStartFrame;
	string mModeStr;

public:
//	static AVIReader aviReader;
//	static USBCamReader usbcamReader;
//	static DroneCamReader droneCamReader;

	static GLImageWnd* videoWnd;
	static GLSceneWnd* modelWnd1;
	static GLSceneWnd* modelWnd2;
	static MapViewWnd* mapViewWnd;

	static MainDlg* mainDlg;
	static double videoWndScale;

	static MoSLAM* moSLAM;

	static bool bInitSucc;
	static bool bBusyDrawingModel;
	static bool bBusyDrawingVideo;

	/*flag for busy running bundle adjustment*/
	static MyApp* app;

	static MoSLAMThread *coSlamThread;
	static pthread_t _mainThread;
	static pthread_t _navdataThread;

	bool render_loop_on;
	static EstimationNode* _estimationNode;
	static DroneKalmanFilter* _filter;
	static std::deque<ardrone_autonomy::Navdata> _navInfoQueue;
	static pthread_mutex_t _mutexNavInfoQueue;
	static pthread_mutex_t _mutexScaleFilterUpdate;
	static pthread_cond_t _condScaleFilterUpdate;

	static vector<cv::Mat> savedFrames;
	static vector<cv::Mat> savedScene;


	DECLARE_EVENT_TABLE();
public:
	static void redrawViews();
	static void exitProgram();
	void onUpdateViews(wxCommandEvent& evt);
	void onClose(wxCommandEvent& evt);
	void onIdle(wxIdleEvent& event);
	void activateRenderLoop(bool on);
public:
	static int runMode;
	static bool bStartInit;
	static bool bExit;
	static bool bStop;
	static bool bSingleStep;
	static bool bConstScale;
	static bool bRunning;
	static char timeStr[256];

	// For debug
public:
	static std::vector<tf::Transform> log_camPoseFromFilterVec;
	static std::vector<tf::Transform> log_slamRes;
	static std::vector<tf::Transform> log_droneState;
	static std::deque<ardrone_autonomy::Navdata> log_navdata;

	static std::vector<double> log_stateBeforeSlam;
	static std::vector<double> log_currSlamRes;
	static std::vector<double> log_stateAfterSlam;
	static std::vector<double> log_scale;
	static std::vector<ScaleEstimator> log_scalePairs;
};
void getCurTimeString(char* timeStr);
#endif /* MYAPP_H_ */
