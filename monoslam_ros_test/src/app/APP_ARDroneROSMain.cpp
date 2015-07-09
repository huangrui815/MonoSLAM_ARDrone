#include "APP_MoSLAMThread.h"
#include "SL_GlobParam.h"
#include "SL_MoSLAM.h"
#include "APP_MyApp.h"
#include "APP_SynObj.h"

#include "tools/SL_Tictoc.h"
#include "tools/SL_Timing.h"
#include "slam/SL_SLAMHelper.h"


tf::Transform preGetSlamObs(int frameTS, int videoDelay, int imuDelay){
	// Get the filter state before estimate the camera pose
	pthread_mutex_lock(&MyApp::_filter->s_mutexFilterState);
	double filterPose[10];
	MyApp::_filter->getPoseAtAsVec(frameTS - videoDelay, true, filterPose);
	ROS_INFO("filter state: %f %f %f %f %f %f\n", filterPose[0], filterPose[1], filterPose[2],
			filterPose[3], filterPose[4], filterPose[5]);
	pthread_mutex_unlock(&MyApp::_filter->s_mutexFilterState);

	// Transform filter state to cam pose in cam CS
	double camPoseFromFilter[6];
	MyApp::_filter->backTransformPTAMObservation(filterPose,camPoseFromFilter);
	ROS_INFO("transformed filter state: %f, %f, %f, %f, %f, %f\n", camPoseFromFilter[0], camPoseFromFilter[1],
			camPoseFromFilter[2], camPoseFromFilter[3], camPoseFromFilter[4],camPoseFromFilter[5]);

	tf::Transform globalPoseTf;
	globalPoseTf.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion rot;
	rot.setRPY(camPoseFromFilter[3] * RADIAN_PER_DEGREE,
			camPoseFromFilter[4] * RADIAN_PER_DEGREE,
			camPoseFromFilter[5] * RADIAN_PER_DEGREE);

	globalPoseTf.setRotation(rot);
	globalPoseTf = EstimationNode::_globalToDrone * globalPoseTf * EstimationNode::_droneToGlobal;
	globalPoseTf.setOrigin(tf::Vector3(camPoseFromFilter[0], camPoseFromFilter[1], camPoseFromFilter[2]));

	if (!MyApp::_filter->_bFirstCamPoseInGlobalUnit)
	{
		MyApp::_filter->_firstCamPoseInGlobalUnit = globalPoseTf;
		MyApp::_filter->_bFirstCamPoseInGlobalUnit = true;
	}

	ROS_INFO("global pose tf: %f, %f, %f\n", globalPoseTf.getOrigin()[0], globalPoseTf.getOrigin()[1],
			globalPoseTf.getOrigin()[2]);

	tf::Transform worldToCamTf, camToWorldTf;
	worldToCamTf = EstimationNode::_camToGlobalRotOnly * globalPoseTf * EstimationNode::_globalToCamRotOnly;
//	camToWorldTf = worldToCamTf.inverse();
//	ROS_INFO("worldToCamTf origin: %f, %f, %f\n", camToWorldTf.getOrigin()[0],camToWorldTf.getOrigin()[1],
//			camToWorldTf.getOrigin()[2]);
	return worldToCamTf;
	//MyApp::log_camPoseFromFilterVec.push_back(camPoseFromFilterGlobal);
}

tf::Transform preGetSlamObsNew(int frameTS, int videoDelay, int imuDelay){
	// Get the filter state before estimate the camera pose
	pthread_mutex_lock(&MyApp::_filter->s_mutexFilterState);
	double filterPose[10];
	MyApp::_filter->getPoseAtAsVec(frameTS - videoDelay, true, filterPose);
//	ROS_INFO("filter state: %f %f %f %f %f %f\n", filterPose[0], filterPose[1], filterPose[2],
//			filterPose[3], filterPose[4], filterPose[5]);
	pthread_mutex_unlock(&MyApp::_filter->s_mutexFilterState);

	// obtain global transformation: convert RPY representation to rotation
	tf::Transform globalPoseTf;
	globalPoseTf.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion rot;
	rot.setRPY(filterPose[3] * RADIAN_PER_DEGREE,
			filterPose[4] * RADIAN_PER_DEGREE,
			filterPose[5] * RADIAN_PER_DEGREE);

	globalPoseTf.setRotation(rot);
	globalPoseTf = EstimationNode::_globalToDrone * globalPoseTf * EstimationNode::_droneToGlobal;
	globalPoseTf.setOrigin(tf::Vector3(filterPose[0], filterPose[1], filterPose[2]));

//	ROS_INFO("global pose tf: %f, %f, %f\n", globalPoseTf.getOrigin()[0], globalPoseTf.getOrigin()[1],
//			globalPoseTf.getOrigin()[2]);

	// transform into camera CS
	tf::Transform worldToCamTf;
	worldToCamTf = EstimationNode::_camToGlobal * globalPoseTf * EstimationNode::_globalToCam;
//	ROS_INFO("cam pose tf: %f, %f, %f\n", worldToCamTf.getOrigin()[0], worldToCamTf.getOrigin()[1],
//			worldToCamTf.getOrigin()[2]);

	if (!MyApp::_filter->_bFirstCamPoseInGlobalUnit)
	{
		MyApp::_filter->_firstCamPoseInGlobalUnit = worldToCamTf;
		MyApp::_filter->_bFirstCamPoseInGlobalUnit = true;
	}
	else if (MyApp::_filter->_bFirstCamPoseInGlobalUnit && !MyApp::_filter->_bSecondCamPoseInGlobalUnit)
	{
		MyApp::_filter->_secondCamPoseInGlobalUnit = worldToCamTf;
		MyApp::_filter->_bSecondCamPoseInGlobalUnit = true;
	}

	// scale with the scale factors
	worldToCamTf = MyApp::_filter->scalePredictedCamPoseToSlamUnit(worldToCamTf);
//	camToWorldTf = worldToCamTf.inverse();
	return worldToCamTf;
}

void postGetSlamObs(MoSLAM* moSLAM, int frameTS, int videoDelay, int imuDelay){
	int lastScaleTS = 0;
	double lastSlamPosForScale[3];

	CamPose* cam = moSLAM->camPose.current();
	tf::Matrix3x3 slamRot(cam->R[0],cam->R[1],cam->R[2],
	                    cam->R[3],cam->R[4],cam->R[5],
	                    cam->R[6],cam->R[7],cam->R[8]);

	tf::Transform camToWorld, worldToCam;
	camToWorld.setBasis(slamRot);
	camToWorld.setOrigin(tf::Vector3(cam->t[0],cam->t[1],cam->t[2]));
	worldToCam = camToWorld.inverse();
	//MyApp::log_slamRes.push_back(slamRes);

	// convert to global CS
	tf::Transform worldToCamInGlobalCS = EstimationNode::_globalToCamRotOnly * worldToCam * EstimationNode::_camToGlobalRotOnly;

	double slamObsVec[6], slamObsVecScaled[6];
	slamObsVec[0] = worldToCamInGlobalCS.getOrigin().getX();
	slamObsVec[1] = worldToCamInGlobalCS.getOrigin().getY();
	slamObsVec[2] = worldToCamInGlobalCS.getOrigin().getZ();

	// Transform the rotation to roll-pitch-yaw CS
	tf::Transform globalRot;
	globalRot.setRotation(worldToCamInGlobalCS.getRotation());
	globalRot.setOrigin(tf::Vector3(0,0,0));
	globalRot = EstimationNode::_droneToGlobal * globalRot * EstimationNode::_globalToDrone;
	globalRot.getBasis().getRPY(slamObsVec[3], slamObsVec[4], slamObsVec[5]);

	slamObsVec[3] = slamObsVec[3] * DEGREE_PER_RADIAN;
	slamObsVec[4] = slamObsVec[4] * DEGREE_PER_RADIAN;
	slamObsVec[5] = slamObsVec[5] * DEGREE_PER_RADIAN;

//	MyApp::_filter->transformPTAMObservation(slamObsVec, slamObsVecScaled);

	double filterPosePostSlam[10], filterPosePostSlamTransformed[6];
	// add slam observation
	pthread_mutex_lock(&MyApp::_filter->s_mutexFilterState);
	MyApp::_filter->addPTAMObservation(slamObsVec, frameTS - videoDelay);
	MyApp::_filter->getCurrentPoseSpeedAsVec(filterPosePostSlam);
	pthread_mutex_unlock(&MyApp::_filter->s_mutexFilterState);
//	for (int i = 0; i < 10; i++)
//		MyApp::log_filterState.push_back(filterPosePostSlam[i]);

	MyApp::_filter->backTransformPTAMObservation(filterPosePostSlam,filterPosePostSlamTransformed);

	// if the time interval > 2000, add a scale pair for scale estimation
	if (lastScaleTS == 0){
		lastScaleTS = frameTS;
		lastSlamPosForScale[0] = filterPosePostSlamTransformed[0];
		lastSlamPosForScale[1] = filterPosePostSlamTransformed[1];
		lastSlamPosForScale[2] = filterPosePostSlamTransformed[2];
	}
	else{
		if (frameTS - lastScaleTS > 2000){
			double slamDiff[3] = {filterPosePostSlamTransformed[0] - lastSlamPosForScale[0],
					filterPosePostSlamTransformed[1] - lastSlamPosForScale[1],
					filterPosePostSlamTransformed[2] - lastSlamPosForScale[2]
			};
			double imuDiff[3];
			MyApp::_filter->getImuDiff(lastScaleTS - videoDelay + imuDelay, frameTS - videoDelay + imuDelay);
			lastScaleTS = frameTS;
			double slamObsVecXYZ[3] = {slamObsVec[0], slamObsVec[1], slamObsVec[2]};
			MyApp::_filter->updateScaleXYZ(slamDiff, imuDiff, slamObsVecXYZ);
		}
	}
}

void postGetSlamObsNew(MoSLAM* moSLAM, int frameTS, int videoDelay, int imuDelay){

	CamPose* cam = moSLAM->camPose.current();
	tf::Matrix3x3 slamRot(cam->R[0],cam->R[1],cam->R[2],
	                    cam->R[3],cam->R[4],cam->R[5],
	                    cam->R[6],cam->R[7],cam->R[8]);

	tf::Transform camToWorld, worldToCam;
	camToWorld.setBasis(slamRot);
	camToWorld.setOrigin(tf::Vector3(cam->t[0],cam->t[1],cam->t[2]));
	worldToCam = camToWorld.inverse();

	// scale camera position to global unit and convert to global CS
	tf::Transform worldToCamInGlobalCS;
	worldToCam = MyApp::_filter->scaleCamPoseToGlobalUnit(worldToCam);
	worldToCamInGlobalCS = EstimationNode::_globalToCam * worldToCam * EstimationNode::_camToGlobal;

	// transform with the first worldToCamInGlobalCS ( saved with the first Keyframe )
	tf::Transform currCamInGlobalCS = MyApp::_filter->_firstCamPoseInGlobalUnit * worldToCamInGlobalCS;

	// convert the Tf to (x,y,z,roll,pitch,yaw) representation
	double slamObsVec[6];
	slamObsVec[0] = currCamInGlobalCS.getOrigin().getX();
	slamObsVec[1] = currCamInGlobalCS.getOrigin().getY();
	slamObsVec[2] = currCamInGlobalCS.getOrigin().getZ();

	// Transform the rotation to roll-pitch-yaw CS
	tf::Transform globalRot;
	globalRot.setRotation(currCamInGlobalCS.getRotation());
	globalRot.setOrigin(tf::Vector3(0,0,0));
	globalRot = EstimationNode::_droneToGlobal * globalRot * EstimationNode::_globalToDrone;
	globalRot.getBasis().getRPY(slamObsVec[3], slamObsVec[4], slamObsVec[5]);

	slamObsVec[3] = slamObsVec[3] * DEGREE_PER_RADIAN;
	slamObsVec[4] = slamObsVec[4] * DEGREE_PER_RADIAN;
	slamObsVec[5] = slamObsVec[5] * DEGREE_PER_RADIAN;

	// add slam observation
	pthread_mutex_lock(&MyApp::_filter->s_mutexFilterState);
	MyApp::_filter->addSlamObservation(slamObsVec, frameTS - videoDelay);
	pthread_mutex_unlock(&MyApp::_filter->s_mutexFilterState);

	tf::Transform filterPosePostAddSlamObs;
	filterPosePostAddSlamObs = MyApp::_filter->getCurrentPoseAsTf();
	filterPosePostAddSlamObs = MyApp::_filter->scalePredictedCamPoseToSlamUnit(filterPosePostAddSlamObs);

	// Update the estimated scale every 2s
	// add a scale pair for scale estimation
	if (MyApp::_filter->_scaleStop){
		MyApp::log_scale.push_back(MyApp::_filter->_scaleGlobalToSlam);
		return;
	}

	if ( !MyApp::_filter->_bStartAddImuObs ){
		// Save the SLAM observation at the starting point
		MyApp::_filter->_lastAddSlamObsTS = frameTS;
		MyApp::_filter->_lastSlamObsForScale = filterPosePostAddSlamObs.getOrigin();
		// Save the IMU observations also
		MyApp::_filter->_bStartAddImuObs = true;
		MyApp::_filter->_lastAddImuTS = frameTS;
		MyApp::_filter->_imuDiff.setZero();
	}
	else if ( MyApp::_filter->_bStartAddImuObs &&
			(frameTS - MyApp::_filter->_lastAddSlamObsTS <= 2000))
	{
		// Integrate the IMU observations within 2 seconds
		MyApp::_filter->_imuDiff += MyApp::_filter->getImuDiff(MyApp::_filter->_lastAddImuTS
					- videoDelay + imuDelay, frameTS - videoDelay + imuDelay);
		MyApp::_filter->_lastAddImuTS = frameTS;
	}
	else if (MyApp::_filter->_bStartAddImuObs &&
			(frameTS - MyApp::_filter->_lastAddSlamObsTS > 2000))
	{
		tf::Vector3 slamDiff = filterPosePostAddSlamObs.getOrigin() - MyApp::_filter->_lastSlamObsForScale;
		MyApp::_filter->updateScale(slamDiff, MyApp::_filter->_imuDiff);
		MyApp::_filter->_bStartAddImuObs = false;

		double ptamDiff[3] = {slamDiff.getX(), slamDiff.getY(), slamDiff.getZ()};
		double imuDiff[3] = {MyApp::_filter->_imuDiff.getX(),
				MyApp::_filter->_imuDiff.getY(), MyApp::_filter->_imuDiff.getZ()};
		ScaleEstimator newMeasPair = ScaleEstimator(ptamDiff, imuDiff);
		ROS_INFO("Add a new pair: SLAM(%f %f %f) IMU(%f %f %f) %f\n", newMeasPair._ptamMeas[0], newMeasPair._ptamMeas[1], newMeasPair._ptamMeas[2],
				newMeasPair._imuMeas[0], newMeasPair._imuMeas[1], newMeasPair._imuMeas[2], newMeasPair._singlePairEstimate);
		//MyApp::log_scalePairs.push_back(newMeasPair);
	}
	MyApp::log_scale.push_back(MyApp::_filter->_scaleGlobalToSlam);
}

bool ardroneROSMain() {
	/////////////////////////1.GPU initilization/////////////////////////
	//initialization for CG;
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(" ");
	glutHideWindow();
	GLenum err = glewInit();
	if (GLEW_OK != err)
		cout << "ERROR: could not initialize GLEW!" << endl;

	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "OpenGL vendor: " << glGetString(GL_VENDOR) << endl;
	cout << "OpenGL renderer: " << glGetString(GL_RENDERER) << endl << endl;

	V3D_GPU::Cg_ProgramBase::initializeCg();

	MoSLAM* moSLAM = MyApp::moSLAM;
	bool visionUpdateEnable = true;

	try {
		while (!MyApp::bExit) {

			//initialization
			moSLAM->init(false);
			printf("MoSLAM init finished\n");

				//notify the GUI thread to create GUIs
				broadcastCreateGUI();
				//wait for the accomplishment of creating GUIs
				waitCreateGUI();

			//update the data associated with the GUIs
			MyApp::videoWnd->setSLAMData(moSLAM);

			if (MyApp::mapViewWnd){
				MyApp::mapViewWnd->setSLAMData(moSLAM);
				MyApp::mapViewWnd->setFilterPointer(MyApp::_filter);
				MyApp::mapViewWnd->setNodePointer(MyApp::_estimationNode);
			}

			if (MyApp::modelWnd1)
				MyApp::modelWnd1->setSLAMData(moSLAM);
			if (MyApp::modelWnd2)
				MyApp::modelWnd2->setSLAMData(moSLAM);
			printf("MoSLAM pointer has been set\n");
			broadcastCreateGUI();

			int videoDelay = MyApp::_estimationNode->_droneFilter->delayVideo;
			int imuDelay = MyApp::_estimationNode->_droneFilter->delayXYZ;

			//start online SLAM process
			int i = 0;
			while (!MyApp::bExit) {

				TimeMeasurer tm;
				tm.tic();
//				printf("Grad new frame\n");
				moSLAM->grabReadFrame();
				/*int frameTS = MyApp::_estimationNode->_droneCamReader->_currImgTS;

				// get the predicted world to camera transformation from filter state
				if(moSLAM->state == SLAM_STATE_NORMAL)
					moSLAM->_predictedWorldToCamTf = preGetSlamObsNew(frameTS,videoDelay,imuDelay);
				*/

				// get slam results
				int state = moSLAM->processOneFrameNew();

				if( MyApp::bStartInit && !moSLAM->mpMaker.doInit){
//					tf::Transform firstPredictedCamPose = preGetSlamObsNew(frameTS,videoDelay,imuDelay);
//					ROS_INFO("firstPredictedCamPose: %f, %f, %f\n", firstPredictedCamPose.getOrigin()[0],
//							firstPredictedCamPose.getOrigin()[1], firstPredictedCamPose.getOrigin()[2]);
//					moSLAM->startMapInit(firstPredictedCamPose);
					moSLAM->startMapInit();
					MyApp::_estimationNode->_navdata_first_set = true;
				}
				if( !MyApp::bStartInit && moSLAM->mpMaker.doInit){
//					tf::Transform secondPredictedCamPose = preGetSlamObsNew(frameTS,videoDelay,imuDelay);
//					ROS_INFO("secondPredictedCamPose: %f, %f, %f\n", secondPredictedCamPose.getOrigin()[0],
//							secondPredictedCamPose.getOrigin()[1], secondPredictedCamPose.getOrigin()[2]);
//					moSLAM->endMapInit(secondPredictedCamPose);
					moSLAM->endMapInit();
				}

				int range = 30;

				if (state == SLAM_STATE_NORMAL && moSLAM->camPose.size() > range){
					printf("camPose ts: %lf, ts_last_update_nav: %lf\n",
							moSLAM->camPose.current()->ts, MyApp::_estimationNode->getLastUpdateTs());
				}

				if (visionUpdateEnable && state == SLAM_STATE_NORMAL && moSLAM->camPose.size() > range){
					double cam[3];
					getCamCenter(moSLAM->camPose.current(), cam);
					tf2::Vector3 cam_center(cam[0], cam[1], cam[2]);
					tf2::Vector3 cam_center_NED = MyApp::_estimationNode->_body2cam_rot * cam_center;
					double imgTs = moSLAM->camPose.current()->ts;
					// get the k-range camera pose
					int i = 0;
					CamPoseItem* camPose;
					for (camPose = moSLAM->camPose.current(); camPose && i < range;){
						i++;
						camPose = camPose->pre;
					}
					getCamCenter(camPose, cam);
					tf2::Vector3 cam_center_prev(cam[0], cam[1], cam[2]);
					tf2::Vector3 cam_center_NED_prev = MyApp::_estimationNode->_body2cam_rot * cam_center_prev;
					double imgTs_prev = camPose->ts;
					tf2::Vector3 cam_vel = (cam_center_NED - cam_center_NED_prev) / (imgTs - imgTs_prev);
					MyApp::_estimationNode->updateFilterSLAM(imgTs, cam_center_NED, cam_vel);
				}

				if (state == SLAM_STATE_NORMAL && moSLAM->camPose.size() > range){
					printf("camPose ts: %lf, ts_last_update_nav: %lf\n",
							moSLAM->camPose.current()->ts, MyApp::_estimationNode->getLastUpdateTs());
				}

				// Add SLAM observations to update filter state
				// then update the scale
				/*
				if(moSLAM->state == SLAM_STATE_NORMAL)
					postGetSlamObsNew(moSLAM, frameTS, videoDelay, imuDelay);
				else
				{
					MyApp::_filter->addFakePTAMObservation(frameTS - videoDelay);
					//std::cout << " addFakePTAMObservation at " << frameTS - videoDelay << endl;
				}*/

//				moSLAM->m_tmPerStep = tm.toc();
				updateDisplayData();
				i++;
			}
		}
		while(moSLAM->bundler.busy){
			Sleep(3);
		}
		cout << " the result is saved at " << MyApp::timeStr << endl;
		moSLAM->exportResults(MyApp::timeStr);
		logInfo("slam finished\n");
	} catch (SL_Exception& e) {
		logInfo(e.what());
#ifdef WIN32
		wxMessageBox(e.what());
#endif
	} catch (std::exception& e) {
		logInfo("%s\n", e.what());
		logInfo("slam failed!\n");
#ifdef WIN32
		wxMessageBox(e.what());
#endif
	}
	MyApp::bRunning = false;
	logInfo("slam exits\n");
	return true;
}
