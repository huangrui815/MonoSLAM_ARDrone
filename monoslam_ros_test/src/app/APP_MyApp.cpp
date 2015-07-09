/*
 * main.cpp
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */
#include "APP_EstimationNode.h"
#include "APP_MyApp.h"
#include "APP_SynObj.h"
#include "APP_MoSLAMThread.h"

#include "tracking/CGKLT/v3d_gpuklt.h"

#include "tools/SL_Timing.h"
#include "tools/SL_Tictoc.h"
#include "tools/GUI_ImageViewer.h"

#include "slam/SL_SLAMHelper.h"

//USBCamReader MyApp::usbcamReader;
//AVIReader MyApp::aviReader;
//DroneCamReader MyApp::droneCamReader;

GLImageWnd* MyApp::videoWnd;
GLSceneWnd* MyApp::modelWnd1 = 0;
MapViewWnd* MyApp::mapViewWnd = 0;
GLSceneWnd* MyApp::modelWnd2 = 0;
MainDlg* MyApp::mainDlg;

double MyApp::videoWndScale = 1.0;
MoSLAM* MyApp::moSLAM;

bool MyApp::bInitSucc = false;
bool MyApp::bBusyDrawingModel = false;
bool MyApp::bBusyDrawingVideo = false;

MyApp* MyApp::app = 0;

int MyApp::runMode = RUN_MODE_OFFLINE;

bool MyApp::bStartInit = false;
bool MyApp::bExit = false;
bool MyApp::bStop = false;
bool MyApp::bSingleStep = false;
bool MyApp::bConstScale = false;
bool MyApp::bRunning = true;

vector<cv::Mat> MyApp::savedFrames;
vector<cv::Mat> MyApp::savedScene;

char MyApp::timeStr[256];

MoSLAMThread* MyApp::coSlamThread;
pthread_t MyApp::_mainThread;
pthread_t MyApp::_navdataThread;

EstimationNode* MyApp::_estimationNode = 0;
DroneKalmanFilter* MyApp::_filter = 0;
std::deque<ardrone_autonomy::Navdata> MyApp::_navInfoQueue;
pthread_mutex_t MyApp::_mutexNavInfoQueue = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t MyApp::_mutexScaleFilterUpdate = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t MyApp::_condScaleFilterUpdate = PTHREAD_COND_INITIALIZER;

// Log data
std::vector<tf::Transform> MyApp::log_camPoseFromFilterVec;
std::vector<tf::Transform> MyApp::log_slamRes;
std::vector<tf::Transform> MyApp::log_droneState;
std::deque<ardrone_autonomy::Navdata> MyApp::log_navdata;

std::vector<double> MyApp::log_stateBeforeSlam;
std::vector<double> MyApp::log_stateAfterSlam;
std::vector<double> MyApp::log_currSlamRes;
std::vector<double> MyApp::log_scale;
std::vector<ScaleEstimator> MyApp::log_scalePairs;

DEFINE_EVENT_TYPE(CloseApp);
BEGIN_EVENT_TABLE(MyApp, wxApp)

EVT_COMMAND(wxID_ANY, CloseApp, MyApp::onClose)

END_EVENT_TABLE()

void MyApp::initSyncVars() {
	initSynObjs();
}
int MyApp::OnExit() {
	cout << "exit program\n";
	destroySynObjs();
	delete moSLAM;
	delete _estimationNode;
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////
void MyApp::exitProgram() {
	wxCommandEvent evt(CloseApp, wxID_ANY);
	evt.SetInt(0);
	MyApp::app->AddPendingEvent(evt);
}

void MyApp::redrawViews() {
	wxCommandEvent evt(EventUpdateViews, wxID_ANY);
	evt.SetInt(0);
	MyApp::app->AddPendingEvent(evt);
}

void MyApp::onUpdateViews(wxCommandEvent& evt) {
	char filePath[256];

	videoWnd->redraw();
	if (SLAMParam::saveImage)
		videoWnd->save(filePath, MyApp::savedFrames);

	if (modelWnd1){
		modelWnd1->redraw();
	}
	if (modelWnd2){
		modelWnd2->redraw();
		if (SLAMParam::saveImage){
			modelWnd2->save(filePath, MyApp::savedScene);
		}
	}
	if (mapViewWnd)
		mapViewWnd->redraw();


//	if (SLAMParam::saveImage) {
//		static bool bFirst = true;
//		char dirPath[1024];
//		sprintf(dirPath, "/home/tsou/slam_results_video/%s", MyApp::timeStr);
//#ifdef WIN32
//		mkdir(dirPath);
//#else
//		mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
//#endif
//
//		if (moSLAM->curFrame > 0) {
//			static int frame = moSLAM->curFrame;
//			char filePath[256];
//			sprintf(filePath, "%s/%dcam_%04d.ppm", dirPath, 0, frame);
//			videoWnd->save(filePath);
//			sprintf(filePath, "%s/1map_%04d.ppm", dirPath, frame);
//			modelWnd1->save(filePath);
//			sprintf(filePath, "%s/2map_%04d.ppm", dirPath, frame);
//			modelWnd2->save(filePath);
//			frame++;
//		}


//	}
}
void MyApp::onClose(wxCommandEvent& evt) {
	cout << "close app\n";
//	activateRenderLoop(false);
	evt.Skip(); // don't stop event, we still want window to close
//	FILE* pFile;
//	pFile = fopen("/home/rui/Record/state.txt","w");
//	if (!pFile)
//		cout << "Cannot open file state.txt\n";
//	std::vector<movisslam_ros_test::filter_state>::iterator it;
//	for (it = MyApp::_estimationNode->stateVec.begin(); it != MyApp::_estimationNode->stateVec.end(); it++){
//		fprintf(pFile, "%f %f %f %f %f %f\n", it->x, it->y, it->z, it->dx, it->dy, it->dz);
//	}
	destroySynObjs();

	delete moSLAM;
	delete _estimationNode;
	exit(0);
}
void getCurTimeString(char* timeStr) {
	time_t rawtime;
	time(&rawtime);
	struct tm * timeinfo;
	timeinfo = localtime(&rawtime);
	strftime(timeStr, 256, "%y-%m-%d=%H-%M", timeinfo);
}

int MyApp::readScript(string& filePath){
	FILE* fid = fopen(filePath.c_str(), "r");
	if (fid == NULL){
		printf("cannot open the file\n");
		return -1;
	}

	char* fgetsFlag = 0;
	char str[100];
	string prevLine;
	string modeStr("mode");
	string videoStr("video path");
	string calibrationStr("calibration files");
	string skippedStr("skipped frames");
	string initStr("init frames");

	while (1){
		fgetsFlag = fgets(str, 100, fid);
		if (fgetsFlag == NULL){
			break;
		}
		string s = str;
		if (s[s.size()-1] == '\n')
			s.erase(s.size()-1,1);

		if (str[0] == '#'){
			prevLine = s;
			prevLine.erase(0,1);
			printf("%s\n", prevLine.c_str());
		}
		else if (str[0] == '$'){
			printf("%s\n", s.c_str());
		}
		else{
			if (prevLine.compare(modeStr) == 0 && s.size() > 0){
				mModeStr = s;
				printf("%s\n", mModeStr.c_str());
			}
			else if (prevLine.compare(videoStr) == 0 && s.size() >0){
				SLAMParam::videoFilePath = s;
				printf("%s\n", SLAMParam::videoFilePath.c_str());
			}
			else if (prevLine.compare(calibrationStr) == 0 && s.size() > 0){
				SLAMParam::calFilePath = s;
				printf("%s\n", SLAMParam::calFilePath.c_str());
			}
			else if (prevLine.compare(skippedStr) == 0 && s.size() >> 0){
				SLAMParam::frmNumForSkip = (atoi(s.c_str()));
				printf("%d\n", SLAMParam::frmNumForSkip);
			}
			else if (prevLine.compare(initStr) == 0 && s.size() >> 0){
				SLAMParam::frmNumForInit = (atoi(s.c_str()));
				printf("%d\n", SLAMParam::frmNumForInit);
			}
		}
	}
	return 0;
}

bool MyApp::initOffline() {
	try {
		initSyncVars();
		glutInit(&argc, argv);
		glutInitDisplayMode(
				GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
		glEnable(GL_MULTISAMPLE);
		app = this;
		MyApp::runMode = RUN_MODE_OFFLINE;

		//      mainDlg = new MainDlg(0);
		//      if (mainDlg->ShowModal() != wxID_OK)
		//              return false;
		getCurTimeString(timeStr);

//		SLAMParam::videoFilePath =
//				//"/home/rui/workspace/Drone_Data/test_video/test02.avi";
//				//"/home/rui/workspace/Data/MVI_1916.MOV";
//				//"/home/rui/SFM_data/00091_out.mp4";
////				"/home/rui/SFM_data/ARDrone/data21/imgs01.avi";
////				"/home/rui/SFM_data/ARDrone/data01/imgs03.avi";
//				//"/home/rui/SFM_data/TsingHua Video/seqce.avi";
//				//"/media/rui/CE989020989008DB/SFM_data/ARDrone/data58/video.avi";
////				"/media/rui/Data/SFM_data/ARDrone/Handheld/data03/imgs02.avi";
////				"/media/rui/Data/Chatterbox_Data/ros_data/test3/video0.avi";
////				"/media/rui/Data/SFM_data/ARDrone/flight_data_pc/data60/video.avi";
////				"/media/rui/Data/Chatterbox_Data/test2/v1.mp4";
////				"/media/rui/Data/Chatterbox_Data/test3/v1.mp4";
////				"/media/rui/Data/SFM_data/demo_videos/square_front_cam_video.avi";
////				"/media/rui/Data/SFM_data/ARDrone/flight_data_pc/data60/video.avi";
//				//"/media/rui/Data/SFM_data/ARDrone/flight_data_pc/data63_hover/video.avi";
//				"/media/rui/Data/SFM_data/ARDrone/flight_data_pc/data71/video.avi";
//		//				"/home/rui/SFM_data/TsingHua Video/seq3.avi";
//		//"/media/WIND_DATA/SLAM_Benchmark/rgbd_dataset_freiburg2_desk_with_person-rgb.avi";
//		//"D:/ARDrone_Data/test_ardrone_11_35.avi";
//		SLAMParam::calFilePath =
//		//"D:/UAV_project/cal/mini_cam_on_UAV/mini_cam.txt";
//		//"/media/WIND_DATA/ARDrone_Data/AR_Drone_calib/ARDrone_Cam.txt";
//		//"/media/WIND_DATA/ARDrone_Data/AR_Drone_calib/ARDrone_Cam_Our_new.txt";
//				//"/media/WIND_DATA/ARDrone_Data/Calibration/fr3.txt";
//				//"/media/WIND_DATA/ARDrone_Data/Noisy_Video/mini_cam.txt";
//				//"D:/UAV_Project/Test_Video/mini_cam.txt";
//				//"/home/rui/workspace/Drone_Data/test_video/mini_cam.txt";
//				"/home/rui/workspace/calibrations/ardrone_cam3.txt";
////				"/home/rui/workspace/calibrations/nexus5.txt";
//				//"/home/rui/SFM_data/TsingHua Video/tsinghua_cam.txt";
////				"/home/rui/workspace/calibrations/tsingHua_cam.txt";
//				//"/home/rui/workspace/calibrations/0578_cal_new.txt";
////				"/home/rui/workspace/calibrations/odroid.txt";
//		SLAMParam::resultPath =
//				//"/media/rui/Data/SFM_data/ARDrone/Handheld/data03/res";
////				"/media/rui/Data/Chatterbox_Data/test3";
////				"/media/rui/Data/SFM_data/ARDrone/flight_data_pc/data60/res";
//				"/media/rui/Data/SFM_data/demo_videos";
//
//		//"D:/ARDRone_Data/iphone4_video_cal.txt";
//		///media/WIND_DATA/ARDrone_Data/iphone4_video_cal.txt";
//
//		SLAMParam::frmNumForSkip = 180; //skip frames
//		SLAMParam::frmNumForInit = 120;
		SLAMParam::frmNumForRun = 3300;
		SLAMParam::saveImage = true;

		//tracking parameters
		SLAMParam::klt_SSDThreshold = 25000; //larger stable
		SLAMParam::klt_minCornerness = 200;
		SLAMParam::camNum = (int) SLAMParam::videoFilePath.size();
//		saveParameters();
		preWaitCreateGUI();

		//create main thread
		coSlamThread = new MoSLAMThread();
		coSlamThread->Create();
		coSlamThread->Run();

		waitCreateGUI();

		if (!bInitSucc) {
			wxMessageBox("Initialization failed!");
			return false;
		}
// 
 		modelWnd1 = new GLSceneWnd("model window1", 0, 0, 600, 600);
 		modelWnd1->glPane->m_autoScale = true;
 		modelWnd1->glPane->m_pointSize = 0.5;
// 		modelWnd1->Show();

		modelWnd2 = new GLSceneWnd("model window2", 600, 0, 600, 600);
		modelWnd2->glPane->m_autoScale = false;
		modelWnd2->glPane->m_camView = true;
		modelWnd2->glPane->m_followCamId = 0;
//		modelWnd2->Show();

//		MyApp::controlwnd = new ControlWnd("control window", 500, 0, 700, 700);
//		controlwnd->Show(true);

		char wndName[256];
		int W = moSLAM->tracker.videoReader->_w;
		int H = moSLAM->tracker.videoReader->_h;
		int Ws = W * videoWndScale;
		int Hs = H * videoWndScale;
		sprintf(wndName, "video");
		videoWnd = new GLImageWnd(wndName, 300, 600, Ws, Hs);
		videoWnd->initVideoFrame(W, H);
//		videoWnd->Show();

		//finish creating GUIs, broadcast the CoSLAMThread to start SLAM
		broadcastCreateGUI();
		waitCreateGUI();
		modelWnd1->Show();
		modelWnd2->Show();
		videoWnd->Show();
//		videoWnd->SetFocus();
		modelWnd1->SetFocus();
	} catch (SL_Exception& e) {
		logInfo("%s", e.what());
		wxMessageBox(e.what(), "Error!");
		exit(0);
	}
	return true;
}
bool MyApp::initUSBCam() {
	initSyncVars();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glEnable(GL_MULTISAMPLE);

	app = this;
	MyApp::runMode = RUN_MODE_ONLINE_USB;
	MyApp::moSLAM = new MoSLAM();

	getCurTimeString(timeStr);
	//Param::calFilePath="C:/SDKs/calibration/USB_Camera.txt";
#ifdef WIN32
	Param::calFilePath = //"D:/ARDrone_Data/Calibration/USB_Camera.txt";
		"D:/UAV_Project/cal/USB_Camera.txt";
#else
	SLAMParam::calFilePath =
			//"/media/WIND_DATA/ARDrone_Data/Calibration/USB_Camera.txt";
			"/home/rui/workspace/calibrations/logiCam.txt";
#endif
	SLAMParam::klt_SSDThreshold = 20000;
	SLAMParam::klt_minCornerness = 200; //550; //0;

	try {
		//open usb cameras
		moSLAM->setCam(SLAMParam::calFilePath.c_str());
//		MyApp::usbcamReader.camid = 1;
//		moSLAM->setVideoReader(&MyApp::usbcamReader);
		moSLAM->tracker.videoReader = new USBCamReader(1);
		//moSLAM->openVideoReader();
		std::cout << " Openning Video...\n";
		while(!moSLAM->openVideoReader()){Sleep(10);};
		std::cout << " Video opened\n";
		bInitSucc = true;
	} catch (SL_Exception& e) {
		wxMessageBox(e.what(), "Error");
		Exit();
	}
	preWaitCreateGUI();

	//create main thread
	coSlamThread = new MoSLAMThread();
	coSlamThread->Create();
	coSlamThread->Run();

	waitCreateGUI();

	if (!bInitSucc) {
		wxMessageBox("Initialization failed!");
		return false;
	}

//	modelWnd1 = new GLSceneWnd("model window1", 0, 0, 600, 600);
//	modelWnd1->glPane->m_autoScale = true;
//	modelWnd1->glPane->m_pointSize = 0.5;
//	modelWnd1->Show();

	modelWnd2 = new GLSceneWnd("model window2", 600, 0, 600, 600);
	modelWnd2->glPane->m_autoScale = false;
	modelWnd2->glPane->m_camView = true;
	modelWnd2->glPane->m_followCamId = 0;
	modelWnd2->Show();

	char wndName[256];
	int W = moSLAM->tracker.videoReader->_w;
	int H = moSLAM->tracker.videoReader->_h;
	int Ws = W * videoWndScale;
	int Hs = H * videoWndScale;
	sprintf(wndName, "video");
	videoWnd = new GLImageWnd(wndName, 300, 600, Ws, Hs);
	videoWnd->initVideoFrame(W, H);
	videoWnd->Show();

	//finish creating GUIs, broadcast the CoSLAMThread to start SLAM
	broadcastCreateGUI();
	videoWnd->SetFocus();

	return true;
}

bool MyApp::initRosARDrone() {
	initSyncVars();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glEnable(GL_MULTISAMPLE);

	app = this;
	MyApp::runMode = RUN_MODE_ROS_ARDRONE;
	MyApp::moSLAM = new MoSLAM();

	_filter = _estimationNode->_droneFilter;

	getCurTimeString(timeStr);
	//Param::calFilePath="C:/SDKs/calibration/USB_Camera.txt";
#ifdef WIN32
	Param::calFilePath = //"D:/ARDrone_Data/Calibration/USB_Camera.txt";
		"D:/UAV_Project/cal/USB_Camera.txt";
#else
	SLAMParam::calFilePath =
			//"/media/WIND_DATA/ARDrone_Data/Calibration/USB_Camera.txt";
			//"/home/rui/workspace/calibrations/ARDrone_Cam.txt";
			"/home/rui/workspace/calibrations/ardrone_cam3.txt";

	SLAMParam::resultPath =
//			"/home/rui/SFM_data/ARDrone/online_test";
			"/media/rui/Data/SFM_data/ARDrone";

#endif
	SLAMParam::klt_SSDThreshold = 25000;
	SLAMParam::klt_minCornerness = 200; //0;

	try {
		//open video reader
		moSLAM->setCam(SLAMParam::calFilePath.c_str());
		moSLAM->tracker.videoReader = _estimationNode->_droneCamReader;

		std::cout << " Openning Video...\n";
		while(!moSLAM->openVideoReader()){
			//std::cout << " Open video reader\n";
			Sleep(100);};
		std::cout << " Video opened\n";
		bInitSucc = true;
	} catch (SL_Exception& e) {
		wxMessageBox(e.what(), "Error");
		Exit();
	}
	preWaitCreateGUI();

	//create main thread
	coSlamThread = new MoSLAMThread();
	coSlamThread->Create();
	coSlamThread->Run();

	waitCreateGUI();

	if (!bInitSucc) {
		wxMessageBox("Initialization failed!");
		return false;
	}

	modelWnd1 = new GLSceneWnd("model window1", 0, 0, 600, 600);
	modelWnd1->glPane->m_autoScale = true;
	modelWnd1->glPane->m_pointSize = 0.5;
//	modelWnd1->Show();

//	mapViewWnd = new MapViewWnd("Map View Window", 0, 0, 600, 600);
//	mapViewWnd->glPane->m_autoScale = true;
//	mapViewWnd->glPane->m_pointSize = 0.5;
//	mapViewWnd->Show();

//	modelWnd2 = new GLSceneWnd("model window2", 600, 0, 600, 600);
//	modelWnd2->glPane->m_autoScale = false;
//	modelWnd2->glPane->m_camView = true;
//	modelWnd2->glPane->m_followCamId = 0;
//	modelWnd2->Show();

	char wndName[256];
	int W = moSLAM->tracker.videoReader->_w;
	int H = moSLAM->tracker.videoReader->_h;
	std::cout << "Img Size W x H " << W << " x " << H << "\n";
	int Ws = W * videoWndScale;
	int Hs = H * videoWndScale;
	sprintf(wndName, "video");
	videoWnd = new GLImageWnd(wndName, 300, 600, Ws, Hs);
	videoWnd->initVideoFrame(W, H);
//	videoWnd->Show();

	//finish creating GUIs, broadcast the CoSLAMThread to start SLAM
	broadcastCreateGUI();
	waitCreateGUI();
	modelWnd1->Show();
	videoWnd->Show();
//	videoWnd->SetFocus();
	modelWnd1->SetFocus();

	return true;
}
void MyApp::onIdle(wxIdleEvent& event) {
	if (render_loop_on) {
		if (MyApp::videoWnd)
			MyApp::videoWnd->redraw();
		if (MyApp::modelWnd1)
			MyApp::modelWnd1->redraw();
		if (MyApp::modelWnd2)
			MyApp::modelWnd2->redraw();
		if (MyApp::mapViewWnd)
			MyApp::mapViewWnd->redraw();

		event.RequestMore(); // render continuously, not only once on idle
	}
}

void MyApp::activateRenderLoop(bool on) {
	if (on && !render_loop_on) {
		Connect(wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(MyApp::onIdle));
		render_loop_on = true;
	} else if (!on && render_loop_on) {
		Disconnect(wxEVT_IDLE, wxIdleEventHandler(MyApp::onIdle));
		render_loop_on = false;
	}
}

bool MyApp::OnInit() {
	render_loop_on = false;
	//For displaying icon
//	wxPNGHandler* png = new wxPNGHandler;
//	wxImage::AddHandler(png);

	string scriptPath("/home/rui/rosbuild_ws/myROS/monoslam_ros_test/src/script.txt");
	readScript(scriptPath);

	if (mModeStr.compare("offline") == 0){
		if (!initOffline ())
			return false;
	}
	else if(mModeStr.compare("usbCam") == 0){
		if (!initUSBCam())
	  		return false;
	}
	else if(mModeStr.compare("rosARDrone") == 0){
	  	if (!initRosARDrone())
	  	  	return false;
	}

	activateRenderLoop(true);
	return true;
}

//IMPLEMENT_APP(MyApp)


int checkDroneId(int argc, char** argv){
	char *drone_ip_address = NULL;

	argc--; argv++;
	while( argc && *argv[0] == '-' )
	{
		if( !strcmp(*argv, "-ip") && ( argc > 1 ) )
		{
			drone_ip_address = *(argv+1);
			printf("Using custom ip address %s\n",drone_ip_address);
			argc--; argv++;
		}
		argc--; argv++;
	}
	if(drone_ip_address)
   {
	 printf("===================+> %s\n", drone_ip_address);
	 std::string ipStr(drone_ip_address);
	 std::string droneIdStr;
	 std::string ipTemplate("192.168.1.");
	 std::size_t found = ipStr.find(ipTemplate);
	 if (found != std::string::npos){
		std::cout << "Find IP template: " << found << '\n';
		droneIdStr.assign(ipStr, ipTemplate.length(), std::string::npos);
		std::cout << "Find Drone ID: " << droneIdStr << '\n';
	 }
	 int ardrone_id = atoi(droneIdStr.c_str());
	 return ardrone_id;
   }
	else
		return 1;
}

int main(int argc, char** argv) {
	ros::init(argc,argv, "drone_stateestimation", ros::init_options::AnonymousName);
	ROS_INFO("Drone state estimation node started.\n");
	MyApp::_estimationNode = new EstimationNode(checkDroneId(argc, argv));

	MyApp::_estimationNode->start();
	MyApp* pApp = new MyApp();

	wxApp::SetInstance(pApp);
	wxEntry(argc, argv);
	return 0;
}
