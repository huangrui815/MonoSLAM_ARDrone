#include "APP_MoSLAMThread.h"
#include "app/SL_GlobParam.h"
#include "app/SL_MoSLAM.h"
#include "app/APP_MyApp.h"
#include "app/APP_SynObj.h"

#include "tools/SL_Tictoc.h"
#include "tools/SL_Timing.h"
#include "slam/SL_SLAMHelper.h"

#include "calibration/SL_CalibTwoCam.h"
#include "tools/SL_Print.h"

void readFeatPts(string filename, vector<Point2f>& pts){
	FILE* pFile = fopen(filename.c_str(), "r");
	float x, y;
	while(fscanf(pFile, "%f %f\n", &x, &y) != EOF)
	{
		Point2f pt(x,y);
		pts.push_back(pt);
	}
}

void RotationDecomp(Mat_d R, double& phi, double& theta, double& psi)
{
    //Converting from Camera frame to NED frame
    double temp_phi, temp_theta, temp_psi;

    // Camera Frame
//    temp_phi = atan2(R(2,1), R(2,2));
//    double temp = sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2));
//    temp_theta = atan2(-R(2,0), temp);
//    temp_psi = atan2(R(1,0), R(0,0));

	double temp = sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
	temp_theta = atan2(-R(2,0), temp);
	temp_phi = atan2(R(1,0) / cos(temp_theta), R(0,0)/ cos(temp_theta));
	temp_psi = atan2(R(2,1) / cos(temp_theta), R(2,2)/ cos(temp_theta));

    // NED Frame
    phi = temp_psi;
    theta = temp_phi;
    psi = temp_theta;

}

bool offlineMain() {
	/////////////////////////1.GPU initilization/////////////////////////
	//initialization for CG;
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(" ");
	glutHideWindow();
	
	// initialize glew
	GLenum err = glewInit();
	if (GLEW_OK != err)
		cout << "ERROR: could not initialize GLEW!" << endl;
	
	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "OpenGL vendor: " << glGetString(GL_VENDOR) << endl;
	cout << "OpenGL renderer: " << glGetString(GL_RENDERER) << endl << endl;

	V3D_GPU::Cg_ProgramBase::initializeCg();

	//////////////////////////2.read video information//////////////////
	MyApp::moSLAM = new MoSLAM();
	MoSLAM* moSLAM = MyApp::moSLAM;
	try {
		//set VideoReaders
		moSLAM->setVideo(SLAMParam::videoFilePath.c_str(),
			SLAMParam::calFilePath.c_str());
		moSLAM->tracker.videoReader = new AVIReader(); //&MyApp::aviReader;
		moSLAM->init(true);
		MyApp::bInitSucc = true;
		logInfo("Loading video sequences.. OK!\n");
	} catch (SL_Exception& e) {
		logInfo(e.what());
#ifdef WIN32
		wxMessageBox(e.what());
#endif
		return true;
	}

	//notify the GUI thread to create GUIs
	broadcastCreateGUI();

	//wait for the accomplishment of creating GUIs
	waitCreateGUI();

	if (MyApp::videoWnd)
		MyApp::videoWnd->setSLAMData(moSLAM);
	if( MyApp::modelWnd1)
		MyApp::modelWnd1->setSLAMData(moSLAM);
	if( MyApp::modelWnd2)
		MyApp::modelWnd2->setSLAMData(moSLAM);

	broadcastCreateGUI();

	//for measuring the timings 
	Timings timingsPerStep;
	Timings timingsReadFrame;
	Timings timingsNewMapPonits;

	//////////////////////////////////////////////////////////////////////////////
		// Try essential matrix
		/*vector<Point2f> pts[2];
		readFeatPts("/home/rui/SFM_data/Kevin data/FeaturePts/first.txt", pts[0]);
		readFeatPts("/home/rui/SFM_data/Kevin data/FeaturePts/second.txt", pts[1]);
		cout << "pts[0] size" << pts[0].size() << endl;
		cout << "pts[1] size" << pts[1].size() << endl;

		bool remove = false;
		for (int i = 0; i < pts[0].size(); ){
			if (pts[0][i].x <= 0 || pts[0][i].y <= 0 || pts[0][i].x >= 640 || pts[0][i].y >= 480)
				remove = true;
			if (pts[1][i].x <= 0 || pts[1][i].y <= 0 || pts[1][i].x >= 640 || pts[1][i].y >= 480)
				remove = true;
			if (remove){
				pts[0].erase(pts[0].begin()+i);
				pts[1].erase(pts[1].begin()+i);
			}
			else
				i++;
			remove = false;
		}

		cout << "pts[0] size" << pts[0].size() << endl;
		cout << "pts[1] size" << pts[1].size() << endl;
		Mat_d matPts1, matPts2;
		matPts1.resize(pts[0].size(), 2);
		matPts2.resize(pts[1].size(), 2);
		for( int i = 0; i < pts[0].size(); i++ )
		{
			matPts1[2* i] = pts[0][i].x;
			matPts1[2* i + 1] = pts[0][i].y;
			matPts2[2* i] = pts[1][i].x;
			matPts2[2* i + 1] = pts[1][i].y;
		}

		CalibTwoCam calib;
		Mat_d K;
		K.resize(3,3);
		K(0,0) = 522.7 522.64089; K(0,1) = 0.0; K(0,2) = 320 320.73712;
		K(1,0) = 0; K(1,1) = 522.7 522.84655; K(1,2) = 240 222.17035;
		K(2,0) = 0; K(2,1) = 0; K(2,2) = 1;
 		calib.setIntrinParam(K.data, K.data);
		calib.setMatchedPoints(matPts1, matPts2);
	//	calib.estimateEMatOld(2.0, CV_FM_RANSAC);
		calib.estimateEMat(1.0);
		vector<int> inlierInd;
		calib.getInlierInd(inlierInd);

		Mat_d R1, t1, R2, t2, pts1, pts2;
		calib.outputInlierNormPoints(pts1, pts2);
		calib.outputRTs(R1, t1, R2, t2);
		print(R1);
		print(t1);
		print(R2);
		print(t2);

		double phi, theta, psi;
		RotationDecomp(R2, phi, theta, psi);
		cout << "phi, theta, psi " << phi << " " << theta << " " << psi << endl;


		MyApp::bStop = true;
		while (MyApp::bStop) {
			Sleep(3);
			//cout << "bStop: " << MyApp::bStop << "\n";
		}*/

	////////////////////////////////////////////////////////////////////////////

	/* start the SLAM process*/
	try {


		//moSLAM->readFrame();
		//copy the data to buffers for display
		updateDisplayData();
		//initialize the map points
		int curFrame = 0;
		MyApp::bStop = true;
		while (MyApp::bStop) {
			Sleep(3);
			//cout << "bStop: " << MyApp::bStop << "\n";
		}
		while(1){
			if (curFrame < SLAMParam::frmNumForSkip){
				moSLAM->grabReadFrame();
				updateDisplayData();
				redrawAllViews();
				Sleep(30);
				curFrame++;
			}
			else
				break;
		}
		while(!moSLAM->mapInitOffline()){printf("Map init failed\n");};

		updateDisplayData();

//		MyApp::bStop = true;
//		while (MyApp::bStop) {
//			Sleep(3);
//			//cout << "bStop: " << MyApp::bStop << "\n";
//		}

		SLAMParam::frmNumTotal = SLAMParam::frmNumTotal - SLAMParam::frmNumForSkip - 10;
		int endFrame = SLAMParam::frmNumTotal - SLAMParam::frmNumForInit;
//		endFrame = 900;

//		if (SLAMParam::frmNumForRun > 0 && endFrame > SLAMParam::frmNumForRun)
//			endFrame = SLAMParam::frmNumForRun;

		updateDisplayData();
//		MyApp::bStop = true;
//		while (MyApp::bStop) {
//			Sleep(3);
//			//cout << "bStop: " << MyApp::bStop << "\n";
//		}
		for (int i = 0; i < endFrame && !MyApp::bExit; i++) {
//			MyApp::bStop = true;
			while (MyApp::bStop) {
				Sleep(3);
//				cout << "bStop: " << MyApp::bStop << "\n";
			}
			//moSLAM->pause();
			TimeMeasurer tm;
			tm.tic();
			moSLAM->grabReadFrame();


			TimeMeasurer testTm;
			if (moSLAM->featureTracking() < 0){
				printf("Start relocalisation\n");
				if (!moSLAM->doRelocalization())
					break;
//				MyApp::bStop = true;
//				while (MyApp::bStop) {
//					Sleep(3);
//	//				cout << "bStop: " << MyApp::bStop << "\n";
//				}
			}
			else{
	//			printf("Current map points number: %d\n", moSLAM->curMapPts.size());

				testTm.tic();
				if(!moSLAM->estimateCamPose()){
					printf("Start relocalisation\n");
					if (!moSLAM->doRelocalization())
						break;
//					MyApp::bStop = true;
//					while (MyApp::bStop) {
//						Sleep(3);
//		//				cout << "bStop: " << MyApp::bStop << "\n";
//					}
					cout << "Pose estimation: " << testTm.toc() << endl;
				}
			}

//			if(!moSLAM->estimateCamPose())
//			{
//				break;
//				//pause();
//				moSLAM->startRelocalization();
//				//enter re-localization loop
//				do
//				{
//					while(MyApp::bStop){/*stop*/}
//					moSLAM->grabReadFrame();
//					moSLAM->featureTracking();
//					updateDisplayData();
//				} while (!MyApp::bExit && !moSLAM->doRelocalization());
//				moSLAM->endRelocalization();
//				updateDisplayData();
//				//MyApp::bStop = true;
//				while(MyApp::bStop){/*stop*/}
//				if(MyApp::bExit)
//					return true;
//			}
//			printf("Current map points number: %d\n", moSLAM->curMapPts.size());
//			/* point registration for active points*/
//			moSLAM->ptRegister.process(SLAMParam::DETECT_ERR_VAR);

//
			testTm.tic();
			moSLAM->addNewKeyFrame();
			cout << "add new keyframe: " << testTm.toc() << endl;
//////
			testTm.tic();
			int numNewPts = moSLAM->mpMaker.genNewMapPoints();
			cout << "map points generation: " << testTm.toc() << endl;
			if (moSLAM->keyFrm.num > 8) {
				moSLAM->bundler.requestRecent(2, 5);
			}
//			while(moSLAM->bundler.busy){
//				Sleep(3);
//			}

			// PBA
//			moSLAM->pbaBundler.requestRecent(5);
//
//			printf("Number of new generated map points: %d\n", numNewPts);
//			tm.toc();
//			moSLAM->_frmId2tmPerStep[moSLAM->curFrame] = tm.get_pass_time();
//			printf("One step time consumed: %f\n", tm.get_pass_time());

			updateDisplayData();
			redrawAllViews();
			// show the video
//			cv::Mat video(moSLAM->tracker.img.rows, moSLAM->tracker.img.cols, CV_8UC1, moSLAM->tracker.img.data);
//			for (int i = 0; i < moSLAM->tracker.curFeatPts.size(); i++){
//				double x = moSLAM->tracker.curFeatPts[i]->pt.x;
//				double y = moSLAM->tracker.curFeatPts[i]->pt.y;
//				cv::circle(video, cv::Point2f(x,y),
//						3, cv::Scalar(255,0,0),2,CV_AA);
//			}
//			cv::imshow("video", video);

			cout << endl;
//			Sleep(30);

//			if (moSLAM->curFrame > 650 - 360){
//				MyApp::bStop = true;
//				while (MyApp::bStop) {
//					Sleep(3);
//					//cout << "bStop: " << MyApp::bStop << "\n";
//				}
//			}
//			cv::waitKey(-1);
//			if (MyApp::bExit)
//				return true;
		}
//		moSLAM->bundler.requestRecent(2, moSLAM->keyFrm.size());
		while(moSLAM->bundler.busy){
			Sleep(3);
		}



		cout << " the result is saved at " << MyApp::timeStr << endl;
		moSLAM->exportResults(MyApp::timeStr);
		logInfo("slam finished\n");
	} catch (SL_Exception& e) {
		logInfo(e.what());
	} catch (std::exception& e) {

#ifdef WIN32
		wxMessageBox(e.what());
#endif
		logInfo("%s\n", e.what());
		logInfo("slam failed!\n");
#ifdef WIN32
		wxMessageBox(e.what());
#endif
	}

	/*save the timing result*/
	if (SLAMParam::measureTimings) {

		std::string prename = getPrenameForSave();
		std::stringstream ossPerStep, ossReadFrm, ossNewMapPts;

		ossPerStep << prename << "_timingsPerStep_" << MyApp::timeStr << ".txt";
		ossReadFrm << prename << "_timingsReadFrm_" << MyApp::timeStr << ".txt";
		ossNewMapPts << prename << "_timingsNewPts_" << MyApp::timeStr
			<< ".txt";

		timingsPerStep.save(ossPerStep.str().c_str());
		timingsReadFrame.save(ossReadFrm.str().c_str());
		timingsNewMapPonits.save(ossNewMapPts.str().c_str());
	}
	MyApp::bRunning = false;
	logInfo("slam stopped!\n");
	return true;
}
