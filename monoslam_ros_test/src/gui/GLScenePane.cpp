/*
 * GLScenePane.cpp
 *
 *  Created on: 2011-7-15
 *      Author: zou
 */
#include "GLScenePane.h"
#include "GLHelper.h"
#include "GLScenePaneHelper.h"
#include "app/APP_MyApp.h"
#include "app/APP_SynObj.h"
#include "slam/SL_SLAMHelper.h"
#include "tools/SL_Print.h"
BEGIN_EVENT_TABLE(GLScenePane, GLTrackballPane) EVT_SIZE(GLScenePane::resized)
EVT_RIGHT_DOWN(GLScenePane::rightClick)
EVT_KEY_DOWN(GLScenePane::keyPressed)
EVT_CHAR(GLScenePane::charPressed)
END_EVENT_TABLE()
///////////////////////////////////////////////////
void getDynTracks(const vector<vector<Point3dId> >& dynMapPts,
		vector<vector<Point3dId> >& dynTracks, int trjLen) {
	map<size_t, vector<Point3dId> > tracks;

	dynTracks.clear();

	if (dynMapPts.empty())
		return;

	int l = 0;
	for (vector<vector<Point3dId> >::const_reverse_iterator iter =
			dynMapPts.rbegin(); iter != dynMapPts.rend() && l < trjLen;
			iter++, l++) {
		const vector<Point3dId>& pts = *iter;
		if (l == 0) {
			for (size_t n = 0; n < pts.size(); n++) {
				size_t id = pts[n].id;
				tracks[id].push_back(pts[n]);
			}
		} else {
			for (size_t n = 0; n < pts.size(); n++) {
				size_t id = pts[n].id;
				if (tracks.count(id) == 0)
					continue;
				else
					tracks[id].push_back(pts[n]);
			}
		}
	}

	for (map<size_t, vector<Point3dId> >::iterator iter = tracks.begin();
			iter != tracks.end(); iter++) {
		dynTracks.push_back(iter->second);
	}
}
void GLScenePane::copyDispData() {
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
void GLScenePane::getSceneScale() {
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
void GLScenePane::drawPoints() {
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
float GLScenePane::CAMERA_COLORS[15] = { 1.0f, 0.0f, 0.0f, 0.2f, 0.8f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.7f, 0.7f, 0.0f, 0.0f, 1.0f, 1.0f };

void GLScenePane::drawCameras() {
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
void GLScenePane::drawInfo() {
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

//		sprintf(buf,
//				"time:%3.2lf, time reg:%3.2lf, average re-projection error:%lf, outlier ratio:%lf\n",
//				m_pSLAM->m_tmPerStep, m_pSLAM->m_tmActMapRegister,
//				m_pSLAM->tracker.avgRepErr, m_pSLAM->tracker.outlierRatio);
		glutPrint2D(20, 35, buf, 0.0f, 0.0f, 1.0f, 0.5f, false);
	}
//	if (m_pSLAM) {
//		sprintf(buf, "frame:%d", m_pSLAM->curFrame);
//		glutPrint2D(20, 15, buf, 1.0f, 0.0f, 0.0f, 0.5f, false);
//	}
}
void GLScenePane::drawGLObjs() {
	glClearColor(1.0, 1.0, 1.0, 1.0);
	if (m_followCamId < 0) {
		glScaled(1. / m_scale, 1. / m_scale, 1. / m_scale);
		glTranslated(-m_center[0], -m_center[1], -m_center[2]);
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

	if (m_pSLAM && m_pSLAM->curMapPts.size() > 0) {
		MyApp::bBusyDrawingModel = true;
		drawPoints();
		drawCameras();
//		drawInfo();
		MyApp::bBusyDrawingModel = false;
	}
}

void GLScenePane::draw(bool inPaintEvts) {
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
//	if (!m_pSLAM->bundler.busy)
		drawGLObjs();
	glPopMatrix();
	glFlush();
	SwapBuffers();
//	unlockDrawing();
}
#include "opencv2/opencv.hpp"
void GLScenePane::saveScreen(const char* filePath, vector<cv::Mat>& frames) {
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
//	CvMat cvImg = img;
//	cvFlip(&cvImg, 0);
//	cv::imwrite(filePath, img);
	cv::Mat flipImg;
	cv::flip(img, flipImg, 0);
	frames.push_back(flipImg);
	delete[] imgData;
}

void GLScenePane::resized(wxSizeEvent& evt) {
	Refresh();
}
void GLScenePane::rightClick(wxMouseEvent& event) {
	MyApp::bStop = true;
	logInfo("save result OK!\n");
}

void GLScenePane::keyPressed(wxKeyEvent& event) {
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
	Refresh();
}
void GLScenePane::charPressed(wxKeyEvent& event) {
}
