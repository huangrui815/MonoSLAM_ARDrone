/*
 * GLScenePane.cpp
 *
 *  Created on: 2011-7-15
 *      Author: zou
 */
#include "Drone_Filter.h"
#include "Drone_MapViewPane.h"
#include "gui/GLScenePaneHelper.h"
#include "app/APP_MyApp.h"
#include "app/APP_SynObj.h"
#include "slam/SL_SLAMHelper.h"
#include "tools/SL_Print.h"
#include "gui/GLHelper.h"
#include "app/APP_EstimationNode.h"

#define PI 3.1415926

float MapViewPane::CAMERA_COLORS[15] = { 1.0f, 0.0f, 0.0f, 0.2f, 0.8f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.7f, 0.7f, 0.0f, 0.0f, 1.0f, 1.0f };

BEGIN_EVENT_TABLE(MapViewPane, GLTrackballPane)
EVT_SIZE(MapViewPane::resized)
EVT_RIGHT_DOWN(MapViewPane::rightClick)
EVT_KEY_DOWN(MapViewPane::keyPressed)
EVT_CHAR(MapViewPane::charPressed)
END_EVENT_TABLE()

pthread_mutex_t MapViewPane::_mutexDronePoseVec = PTHREAD_MUTEX_INITIALIZER;

void MapViewPane::copyDispData() {
	curMapPoints.clear();
	actMapPoints.clear();
	iactMapPoints.clear();

	typedef std::list<MapPoint*>::const_iterator MapPointListConstIter;

	for (MapPointListConstIter iter = m_pSLAM->allMapPts.begin();
			iter != m_pSLAM->allMapPts.end(); iter++) {
		MapPoint* p = *iter;
		if (p->isFalse())
			continue;

		if (p->state == STATE_MAPPOINT_CURRENT)
			curMapPoints.push_back(p);
		else if (p->state == STATE_MAPPOINT_ACTIVE)
			actMapPoints.push_back(p);
		else if (p->state == STATE_MAPPOINT_INACTIVE)
			iactMapPoints.push_back(p);

	}

	camPoses.clear();
	keyPoses.clear();

	for (CamPoseItem* cam = m_pSLAM->camPose.first(); cam; cam = cam->next)
		camPoses.push_back(cam);

	for (KeyFrame* pose = m_pSLAM->keyFrm.first(); pose; pose = pose->next) {
		keyPoses.push_back(pose);
	}

	if (m_autoScale)
		getSceneScale();
}
void MapViewPane::getSceneScale() {
	m_center[0] = 0;
	m_center[1] = 0;
	m_center[2] = 0;
	int num = 0;

	double org[3], axisX[3], axisY[3], axisZ[3];
	const CamPoseItem* cam = m_pSLAM->camPose.first();
	while (cam) {
		getCamCoords(cam, org, axisX, axisY, axisZ);
		m_center[0] += org[0];
		m_center[1] += org[1];
		m_center[2] += org[2];
		num++;
		cam = cam->next;
	}
	m_center[0] /= num;
	m_center[1] /= num;
	m_center[2] /= num;

	m_scale = 0;
	cam = m_pSLAM->camPose.first();
	while (cam) {
		getCamCoords(cam, org, axisX, axisY, axisZ);
		double dx = m_center[0] - org[0];
		double dy = m_center[1] - org[1];
		double dz = m_center[2] - org[2];
		m_scale += sqrt(dx * dx + dy * dy + dz * dz);
		cam = cam->next;
	}
	m_scale /= num;
	m_scale *= 0.5;
}
void MapViewPane::drawPoints() {
	//draw current map points
	for (size_t i = 0; i < curMapPoints.size(); i++) {
		const MapPoint* p = curMapPoints[i];
		drawCurMapPoint(p, m_scale, m_pointSize, true); //!MyApp::bStop);
	}
	glPointSize(3.0 * m_pointSize);
	//draw active map points
	glBegin(GL_POINTS);
	for (size_t i = 0; i < actMapPoints.size(); i++) {
		const MapPoint* p = actMapPoints[i];
		if (!p->isFalse()) {
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3d(p->x, p->y, p->z);
		}
	}
	glEnd();
	glPointSize(2.0 * m_pointSize);
	//draw inactive map points
	glBegin(GL_POINTS);
	for (size_t i = 0; i < iactMapPoints.size(); i++) {
		const MapPoint* p = iactMapPoints[i];
		if (!p->isFalse()) {
			glColor3f(0.0f, 0.0f, 0.0f);
			glVertex3d(p->x, p->y, p->z);
		}
	}
	glEnd();
}
void MapViewPane::addDronePose() {
	// get new pose.
	pthread_mutex_lock(&_droneFilter->s_mutexFilterState);
	_droneFilter->getCurrentPoseSpeedAsVec(_lastFilterState);	// Note: this is maybe an old pose, but max. one frame old = 50ms = not noticable.
	pthread_mutex_unlock(&_droneFilter->s_mutexFilterState);

	DronePose lastPose;
	lastPose._x = _lastFilterState[0];
	lastPose._y = _lastFilterState[1];
	lastPose._z = _lastFilterState[2];
	lastPose._roll = _lastFilterState[3] * PI / 180;
	lastPose._pitch = _lastFilterState[4] * PI / 180;
	lastPose._yaw = _lastFilterState[5] * PI / 180;
//	ROS_INFO("x y z roll pitch yaw: %f, %f, %f, %f, %f, %f\n", _lastFilterState[0], _lastFilterState[1], _lastFilterState[2],
//			_lastFilterState[3], _lastFilterState[4], _lastFilterState[5]);
	pthread_mutex_lock(&_mutexDronePoseVec);
	currDronePose.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion rot;
	rot.setRPY(lastPose._roll, lastPose._pitch, lastPose._yaw);
	currDronePose.setRotation(rot);
	currDronePose = EstimationNode::_globalToDrone * currDronePose * EstimationNode::_droneToGlobal;
	currDronePose.setOrigin(tf::Vector3(lastPose._x, lastPose._y, lastPose._z));
	pthread_mutex_unlock(&_mutexDronePoseVec);

	if (_toClearTrail){
		_dronePoseVec->clear();
		_toClearTrail = false;
	}

	int trajSize = _dronePoseVec->size();

	bool toAddNewPose = false;
	if (_dronePoseVec->size() ==0 )
		toAddNewPose = true;
	else{
		double xDiff = (*_dronePoseVec)[trajSize - 1].getOrigin()[0] - lastPose._x;
		double yDiff = (*_dronePoseVec)[trajSize - 1].getOrigin()[1] - lastPose._y;
		double zDiff = (*_dronePoseVec)[trajSize - 1].getOrigin()[2] - lastPose._z;
		double diff = pow(xDiff,2) + pow(yDiff,2) + pow(zDiff,3);
		if (diff > 0.1 * 0.1)
			toAddNewPose = true;
	}

	if (toAddNewPose){
		pthread_mutex_lock(&_mutexDronePoseVec);
		_dronePoseVec->push_back(currDronePose);
		pthread_mutex_unlock(&_mutexDronePoseVec);
	}

	MyApp::log_droneState.push_back(currDronePose);
}

void MapViewPane::drawDronePose(){

	addDronePose();

	//draw drone trajectory
//	glLineWidth(2.0f);
//	glPointSize(2.0f);
	//glColor3f(0.5f, 0.5f, 0.5f);
	std::vector<monoslam_ros_test::filter_state>& stateVec =
			_node->stateVec;
	std::vector<monoslam_ros_test::filter_state>::iterator stateIter =
			stateVec.begin();
	monoslam_ros_test::filter_state currState = stateVec.back();
	currDronePose.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion rot;
	rot.setRPY(currState.roll * RADIAN_PER_DEGREE,
			currState.pitch * RADIAN_PER_DEGREE,
			currState.yaw * RADIAN_PER_DEGREE);

	currDronePose.setRotation(rot);
	currDronePose = EstimationNode::_globalToDrone * currDronePose * EstimationNode::_droneToGlobal;
	currDronePose.setOrigin(tf::Vector3(currState.x, currState.y, currState.z));
	glBegin(GL_LINE_STRIP);
	pthread_mutex_lock(&_mutexDronePoseVec);
	for (; stateIter != stateVec.end(); stateIter++) {
		double x = (*stateIter).x;
		double y = (*stateIter).y;
		double z = (*stateIter).z;
		glVertex3d(x, y, z);
	}
	pthread_mutex_unlock(&_mutexDronePoseVec);
	drawTransform(currDronePose);
	glEnd();
}

void MapViewPane::drawTransform(tf::Transform& trans){
	tf::Vector3 org = trans.getOrigin();
	tf::Matrix3x3 rot = trans.getBasis();
	glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(org[0], org[1], org[2]);
	glVertex3f(org[0] + rot.getColumn(0)[0], org[1] + rot.getColumn(0)[1], org[2] + rot.getColumn(0)[2]);
	glColor3f(0,1,0);
	glVertex3f(org[0], org[1], org[2]);
	glVertex3f(org[0] + rot.getColumn(1)[0], org[1] + rot.getColumn(1)[1], org[2] + rot.getColumn(1)[2]);
	glColor3f(0,0,1);
	glVertex3f(org[0], org[1], org[2]);
	glVertex3f(org[0] + rot.getColumn(2)[0], org[1] + rot.getColumn(2)[1], org[2] + rot.getColumn(2)[2]);

	glEnd();
}
void MapViewPane::drawAxis(){
	//draw axes
		glLineWidth(5.0f);
		glBegin(GL_LINES);
		glColor3f(1, 0, 0);	// x
		glVertex3f(0,0,0);
		glVertex3f(1,0,0);
		glColor3f(0, 1, 0);	// y
		glVertex3f(0,0,0);
		glVertex3f(0,1,0);
		glColor3f(0, 0, 1);	//z
		glVertex3f(0,0,0);
		glVertex3f(0,0,1);

//		glColor3f(0.7, 0, 0);	// x
//		glVertex3f(0,0,-1);
//		glVertex3f(0,2,-1);
//		glColor3f(0, 0.7, 0);	// y
//		glVertex3f(0,0,-1);
//		glVertex3f(2,0,-1);
//		glColor3f(0, 0, 0.7);	//z
//		glVertex3f(0,0,-1);
//		glVertex3f(0,0,-2-1);
		//glBegin(GL_LINES);
		//glColor3f(color[0],color[1],color[2]);
		//glquiver3(p0, xp, scale);
		//glColor3f(color[0],color[1],color[2]);
		//glquiver3(p0, yp, scale);
		//glColor3f(color[0],color[1],color[2]);
		//glquiver3(p0, zp, scale);
		glEnd();
}

void MapViewPane::drawRPY(){
	tf::Transform dronePose;
	ardrone_autonomy::Navdata currNav = _droneFilter->navdataQueue->back();
	dronePose.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion rot;
	rot.setRPY(currNav.rotX / 180 * PI, currNav.rotY / 180 * PI, currNav.rotZ / 180 * PI);
	dronePose.setRotation(rot);
	tf::Matrix3x3 rotMat = dronePose.getBasis();

	glLineWidth(3.0f);
	glBegin(GL_LINES);
	glColor3f(1, 0, 0);	// x
	glVertex3f(0,0,0);
	glVertex3f(rotMat.getColumn(0)[0],rotMat.getColumn(0)[1],rotMat.getColumn(0)[2]);
	glColor3f(0, 1, 0);	// y
	glVertex3f(0,0,0);
	glVertex3f(rotMat.getColumn(1)[0],rotMat.getColumn(1)[1],rotMat.getColumn(1)[2]);
	glColor3f(0, 0, 1);	//z
	glVertex3f(0,0,0);
	glVertex3f(rotMat.getColumn(2)[0],rotMat.getColumn(2)[1],rotMat.getColumn(2)[2]);

	tf::Transform globalPose;
	globalPose = EstimationNode::_globalToDrone * dronePose;
	rotMat = globalPose.getBasis();
	//rotMat = EstimationNode::_globalToDrone.getBasis();
	//tf::Vector3 droneOrgInGlobal = EstimationNode::_globalToDrone.getOrigin();
	tf::Vector3 droneOrgInGlobal = globalPose.getOrigin();

	printf("%f, %f, %f\n", droneOrgInGlobal[0],droneOrgInGlobal[1],droneOrgInGlobal[2]);
	double scale = 1;
	glLineWidth(4.0f);
	glBegin(GL_LINES);
	glColor3f(0.7, 0, 0);	// x
	glVertex3f(droneOrgInGlobal[0],droneOrgInGlobal[1],droneOrgInGlobal[2]);
	glVertex3f(rotMat.getColumn(0)[0],rotMat.getColumn(0)[1],rotMat.getColumn(0)[2] + droneOrgInGlobal[2]);
	printf("%f, %f, %f\n", rotMat.getColumn(0)[0],rotMat.getColumn(0)[1],rotMat.getColumn(0)[2] );
	glColor3f(0, 0.7, 0);	// y
	glVertex3f(droneOrgInGlobal[0],droneOrgInGlobal[1],droneOrgInGlobal[2]);
	glVertex3f(rotMat.getColumn(1)[0],rotMat.getColumn(1)[1],rotMat.getColumn(1)[2] + droneOrgInGlobal[2]);
	printf("%f, %f, %f\n", rotMat.getColumn(1)[0],rotMat.getColumn(1)[1],rotMat.getColumn(1)[2] );
	glColor3f(0, 0, 0.7);	//z
	glVertex3f(droneOrgInGlobal[0],droneOrgInGlobal[1],droneOrgInGlobal[2]);
	glVertex3f(rotMat.getColumn(2)[0],rotMat.getColumn(2)[1],rotMat.getColumn(2)[2] + droneOrgInGlobal[2]);
	printf("%f, %f, %f\n", rotMat.getColumn(2)[0],rotMat.getColumn(2)[1],rotMat.getColumn(2)[2]);
	//glBegin(GL_LINES);
	//glColor3f(color[0],color[1],color[2]);
	//glquiver3(p0, xp, scale);
	//glColor3f(color[0],color[1],color[2]);
	//glquiver3(p0, yp, scale);
	//glColor3f(color[0],color[1],color[2]);
	//glquiver3(p0, zp, scale);
	glEnd();
}
void MapViewPane::drawCameras() {
	//draw camera trajectory
	glLineWidth(2.0f);
	glPointSize(2.0f);
	glColor3f(0.5f, 0.5f, 0.5f);

	glBegin(GL_POINTS);
	for (size_t i = 0; i < camPoses.size(); i++) {
		CamPoseItem* cam = camPoses[i];
		double org[3];
		getCamCenter(cam, org);
		glVertex3d(org[0], org[1], org[2]);
	}
	glEnd();

	//draw current camera poses
	drawCamera(m_pSLAM->tracker.videoReader->_w,
			m_pSLAM->tracker.videoReader->_h, m_pSLAM->tracker.K.data,
			camPoses.back(), m_camSize, CAMERA_COLORS, 1000);

	//draw key camera poses
	for (size_t i = 0; i < keyPoses.size(); i++) {
		KeyFrame* kp = keyPoses[i];
		if (kp->flag == KEY_FRAME_FLAG_NORMAL)
			drawCameraPose(kp->cam, m_camSize * 0.3, CAMERA_COLORS, 4);
		else if (kp->flag == KEY_FRAME_FLAG_ACTIVE)
			drawCameraPose(kp->cam, m_camSize * 2, CAMERA_COLORS + 3, 5);
	}

}
void MapViewPane::drawInfo() {
	int topleft_x = 0;
	int topleft_y = 0;
	int bottomright_x = GetSize().x;
	int bottomright_y = GetSize().y;

	glViewport(topleft_x, topleft_y, bottomright_x - topleft_x,
			bottomright_y - topleft_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(topleft_x, bottomright_x, bottomright_y, topleft_y);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (MyApp::bStop || m_pSLAM->state != SLAM_STATE_NORMAL)
		glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
	else
		glColor4f(0.5f, 0.5f, 0.5f, 0.5f);

	glTranslatef(topleft_x, topleft_y, 0);
	glBegin(GL_QUADS);
	glVertex2f(0, 0);
	glVertex2f(GetSize().x, 0);
	glVertex2f(GetSize().x, 60);
	glVertex2f(0, 60);
	glEnd();

	char buf[256];

	if (m_pSLAM) {
		sprintf(buf,
				//"frame:%d, mapped feature points:%d, act map points:%d\n feature points:%d, bundle adjustment (%s) %d\n",
				//m_pSLAM->curFrame, m_pSLAM->tracker.curMappedFeatPtsNum,
				"tried: %d, deep tried:%d, act map points:%d\n feature points:%d, bundle adjustment (%s) %d\n",
				m_pSLAM->ptRegister.nRegTried, m_pSLAM->ptRegister.nRegDeepTried,
				m_pSLAM->nActMapPts,
				m_pSLAM->tracker.featTracker.trackedFeatureNum_,
				m_pSLAM->bundler.busy ? "busy" : "idle",
				m_pSLAM->bundler.lastKeyFrmReq);
		glutPrint2D(20, 15, buf, 1.0f, 0.0f, 0.0f, 0.5f, false);

		sprintf(buf,
				"time:%3.2lf, time reg:%3.2lf, average re-projection error:%lf, outlier ratio:%lf\n",
				m_pSLAM->m_tmPerStep, m_pSLAM->m_tmActMapRegister,
				m_pSLAM->tracker.avgRepErr, m_pSLAM->tracker.outlierRatio);
		glutPrint2D(20, 35, buf, 0.0f, 0.0f, 1.0f, 0.5f, false);
	}
//	if (m_pSLAM) {
//		sprintf(buf, "frame:%d", m_pSLAM->curFrame);
//		glutPrint2D(20, 15, buf, 1.0f, 0.0f, 0.0f, 0.5f, false);
//	}
}
void MapViewPane::drawGLObjs() {
	glClearColor(1.0, 1.0, 1.0, 1.0);
	if (m_followCamId < 0) {
		//glScaled(1. / m_scale, 1. / m_scale, 1. / m_scale);
		//glTranslated(-m_center[0], -m_center[1], -m_center[2]);
		if (m_pSLAM->state == SLAM_STATE_NORMAL)
		{
			glScaled(1. / m_scale, 1. / m_scale, 1. / m_scale);
			glTranslated(-m_center[0], -m_center[1], -m_center[2]);
		}
		else
		{
			glScaled(1. , 1. , 1. );
			glTranslated(0, 0, 0);
		}
	} else {
		if (m_pSLAM) {
			const CamPoseItem* cam = m_pSLAM->camPose.current();
			if (cam) {
				double mat[16];
				glGetDoublev(GL_MODELVIEW_MATRIX, mat);
				glMatrixMode(GL_MODELVIEW);
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						mat[j * 4 + i] = cam->R[i * 3 + j];
						mat[j * 4 + 3] = 0;
					}
				}
				mat[12] = cam->t[0];
				mat[13] = cam->t[1];
				mat[14] = cam->t[2];
				mat[15] = 1.0;
				glMultMatrixd(mat);
			}
		}
	}

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	glLineWidth(2.0f);
	glPointSize(2.0f);
	glColor3f(0.5f, 0.5f, 0.5f);

	drawDronePose();
	drawAxis();
	drawGrids();
	//drawRPY();
	if (m_pSLAM && m_pSLAM->curMapPts.size() > 0) {
		//drawPoints();
		//drawCameras();
		//drawDronePose();
		//drawAxis();
		//drawRPY();
		drawInfo();
	}
}

//draw grids on the xy plane
void MapViewPane::drawGrids(){
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	int mapDim = 20;
	float gridDim = 1;
	if (mapDim % 2 == 1)
		mapDim++;

	//gridDim = gridDim * 1000 / ControlData::s_scale;

	float top_left_x = mapDim / 2 * gridDim;
	float top_left_y = mapDim / 2 * gridDim;
	int bottom_right_x = top_left_x - mapDim * gridDim;
	int bottom_right_y = top_left_y - mapDim * gridDim;

	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for(int i=0;i<=mapDim;i++) {
		if (i==0 || i == mapDim) { glColor4f(1.0,0.0,1.0,0.5);}
		else { glColor4f(.25,.25,.25, .5); };
		glVertex3f(bottom_right_x + i * gridDim, bottom_right_y,0);
		glVertex3f(bottom_right_x + i * gridDim, top_left_y,0);
		if (i==0 || i==mapDim) { glColor4f(1.0,0.0,1.0,0.5); } else { glColor4f(.25,.25,.25, 0.5); };
		glVertex3f(bottom_right_x, bottom_right_y + i * gridDim, 0);
		glVertex3f(top_left_x, bottom_right_y + i * gridDim, 0);
	};
	glEnd();
}

void MapViewPane::draw(bool inPaintEvts) {
	SetCurrent(*m_context);
	if (inPaintEvts)
		wxPaintDC(this);
	else
		wxClientDC(this);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	prepareViewport();

//	lockDrawing();
	glPushMatrix();
	drawGLObjs();
	glPopMatrix();
	glFlush();
	SwapBuffers();
//	unlockDrawing();
}
#include "opencv2/opencv.hpp"
void MapViewPane::saveScreen(const char* filePath) {
	wxGLCanvas::SetCurrent(*m_context);
	wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
	int w = GetParent()->GetScreenRect().width;
	int h = GetSize().y;

	unsigned char* imgData = new unsigned char[w * h * 3];
	glReadPixels(-1, 0, w - 1, h, GL_BGR, GL_UNSIGNED_BYTE, imgData);
	cv::Mat img(h, w, CV_8UC3, imgData);
	cv::line(img, cv::Point2i(0, 0), cv::Point2i(0, h - 1), cv::Scalar(0, 0, 0),
			1, CV_AA, 0);
	cv::line(img, cv::Point2i(0, h - 1), cv::Point2i(w - 1, h - 1),
			cv::Scalar(0, 0, 0), 1, CV_AA, 0);
	cv::line(img, cv::Point2i(w - 1, h - 1), cv::Point2i(w - 1, 0),
			cv::Scalar(0, 0, 0), 1, CV_AA, 0);
	cv::line(img, cv::Point2i(w - 1, 0), cv::Point2i(0, 0), cv::Scalar(0, 0, 0),
			1, CV_AA, 0);
	CvMat cvImg = img;
	cvFlip(&cvImg, 0);
	cv::imwrite(filePath, img);
	delete[] imgData;
}

void MapViewPane::resized(wxSizeEvent& evt) {
	Refresh();
}
void MapViewPane::rightClick(wxMouseEvent& event) {
	MyApp::bStop = true;
	logInfo("save result OK!\n");
}

void MapViewPane::keyPressed(wxKeyEvent& event) {
	if (event.GetKeyCode() == 'K') {
		MyApp::bStartInit = !MyApp::bStartInit;
	} else if (event.GetKeyCode() == 'A') {
		m_autoScale = !m_autoScale;
	} else if (event.GetKeyCode() == 'Q') {
		MyApp::bExit = !MyApp::bExit;
	} else if (event.GetKeyCode() == 'O') {
		m_camSize *= 1.5;
	} else if (event.GetKeyCode() == 'P') {
		m_camSize /= 1.5;
	} else if (event.GetKeyCode() == 'T') {
		MyApp::bSingleStep = !MyApp::bSingleStep;
	} else {
		MyApp::bStop = !MyApp::bStop;
	}
	//Refresh();
}
void MapViewPane::charPressed(wxKeyEvent& event) {
}
