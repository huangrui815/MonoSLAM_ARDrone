/*
 * GLImagePane.cpp
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */
#include "GLImagePane.h"
#include "GLHelper.h"
#include "app/SL_GlobParam.h"
#include "app/APP_SynObj.h"
#include "app/APP_MyApp.h"
#include "math/SL_LinAlg.h"
#include "geometry/SL_Triangulate.h"
#include "geometry/SL_Geometry.h"
#include "tools/SL_Print.h"
#include "tools/GUI_ImageViewer.h"
#include "slam/SL_MoSLAMHelper.h"
#include "GLScenePane.h"
#include <cfloat>
BEGIN_EVENT_TABLE(GLImagePane, wxGLCanvas) EVT_MOTION(GLImagePane::mouseMoved) EVT_LEFT_DOWN(GLImagePane::mouseDown)
EVT_LEFT_UP(GLImagePane::mouseReleased)
EVT_RIGHT_DOWN(GLImagePane::rightClick)
EVT_LEAVE_WINDOW(GLImagePane::mouseLeftWindow)
EVT_SIZE(GLImagePane::resized)
EVT_KEY_DOWN(GLImagePane::keyPressed)
EVT_KEY_UP(GLImagePane::keyReleased)
EVT_MOUSEWHEEL(GLImagePane::mouseWheelMoved)
EVT_PAINT(GLImagePane::render)
END_EVENT_TABLE()

// some useful events to use
void GLImagePane::mouseMoved(wxMouseEvent& event) {
}

void GLImagePane::mouseDown(wxMouseEvent& event) {
}
void GLImagePane::mouseWheelMoved(wxMouseEvent& event) {
}
void GLImagePane::mouseReleased(wxMouseEvent& event) {
}
void GLImagePane::rightClick(wxMouseEvent& event) {
}
void GLImagePane::mouseLeftWindow(wxMouseEvent& event) {
}

void GLImagePane::keyPressed(wxKeyEvent& event) {
	if (MyApp::runMode == RUN_MODE_OFFLINE && event.GetKeyCode() == WXK_SPACE)
		MyApp::bStop = !MyApp::bStop;
	else if (event.GetKeyCode() == 'R'){
		if (MyApp::_estimationNode->_recording){
			MyApp::_estimationNode->_recording = false;
			printf("Disable data recording\n");
		}
		else{
			MyApp::_estimationNode->_recording = true;
			printf("Enable data recording\n");
		}
	}
	else if (event.GetKeyCode() == 'N') {
			MyApp::_estimationNode->_navdata_first_set = true;
			//MyApp::_estimationNode->sendLand();
	}
	else if (event.GetKeyCode() == 'P') {
		MyApp::bStartInit = !MyApp::bStartInit;
		//MyApp::_estimationNode->sendLand();
	} else if (event.GetKeyCode() == 'Q') {
		MyApp::bExit = !MyApp::bExit;
		MyApp::_estimationNode->sendLand();
	}else if (event.GetKeyCode() == WXK_RETURN){
		MyApp::_estimationNode->sendTakeoff();
	}else if (event.GetKeyCode() == WXK_ESCAPE){
		MyApp::_estimationNode->sendLand();
	}
	// negative (positive) pitch lower (raise) the nose
	else if (event.GetKeyCode() == 'W'){
		ControlCommand cmd(0,-0.2,0,0);
		MyApp::_estimationNode->sendControlToDrone(cmd);
	}else if (event.GetKeyCode() == 'S'){
			ControlCommand cmd(0,0.2,0,0);
			MyApp::_estimationNode->sendControlToDrone(cmd);
	}
	// negative (positive) roll makes the drone tilt to its left (right)
	else if (event.GetKeyCode() == 'A'){
		ControlCommand cmd(-0.2,0,0,0);
		MyApp::_estimationNode->sendControlToDrone(cmd);
	}else if (event.GetKeyCode() == 'D'){
		ControlCommand cmd(0.2,0,0,0);
		MyApp::_estimationNode->sendControlToDrone(cmd);
	}
	//positive z-vel makes drone rise
	else if (event.GetKeyCode() == WXK_UP){
		ControlCommand cmd(0,0,0,0.2);
		MyApp::_estimationNode->sendControlToDrone(cmd);
	}else if (event.GetKeyCode() == WXK_DOWN){
		ControlCommand cmd(0,0,0,-0.2);
		MyApp::_estimationNode->sendControlToDrone(cmd);
	}
	//positve yaw makes drone rotate right
	else if (event.GetKeyCode() == WXK_LEFT){
			ControlCommand cmd(0,0,-0.2,0);
			MyApp::_estimationNode->sendControlToDrone(cmd);
	}else if (event.GetKeyCode() == WXK_RIGHT){
		ControlCommand cmd(0,0,0.2,0);
		MyApp::_estimationNode->sendControlToDrone(cmd);
	}
	else if (event.GetKeyCode() == 'Z'){
		MyApp::_estimationNode->setRefPos();
		ROS_INFO("Reference Position set.\n");
	}
	else if (event.GetKeyCode() == 'X'){
		MyApp::_estimationNode->_pathPlanner.reset();
		MyApp::_estimationNode->_pathPlanner.preloadPath("/home/rui/workspace/Vicon for Test PCTx/BsplineRefgen/ardrone_diag_25.txt");
		MyApp::_estimationNode->enablePath();
		ROS_INFO("Path enabled.\n");
	}
	else if (event.GetKeyCode() == 'C'){
		MyApp::_estimationNode->_pathPlanner.reset();
		MyApp::_estimationNode->_pathPlanner.preloadPath("/home/rui/workspace/Vicon for Test PCTx/BsplineRefgen/ardrone_circle_more_25.txt");
		MyApp::_estimationNode->enablePath();
		ROS_INFO("Path enabled.\n");
	}
	else if (event.GetKeyCode() == 'V'){
		MyApp::_estimationNode->_pathPlanner.reset();
		MyApp::_estimationNode->_pathPlanner.preloadPath("/home/rui/workspace/Vicon for Test PCTx/BsplineRefgen/ardrone_square_25.txt");
		MyApp::_estimationNode->enablePath();
		ROS_INFO("Path enabled.\n");
	}
	else if (event.GetKeyCode() == 'B'){
		MyApp::_estimationNode->_pathPlanner.reset();
		MyApp::_estimationNode->_pathPlanner.preloadPath("/home/rui/workspace/Vicon for Test PCTx/BsplineRefgen/ardrone_left_right_50.txt");
		MyApp::_estimationNode->enablePath();
		ROS_INFO("Path enabled.\n");
	}
	else if ( event.GetKeyCode() == WXK_NUMPAD0){
		printf("send c autoinit\n");
		MyApp::_estimationNode->publishCommand("c autoinit");
	}
	else if ( event.GetKeyCode() == WXK_NUMPAD1){
		MyApp::_estimationNode->publishCommand("c scaleInit");
	}
	else if ( event.GetKeyCode() == WXK_NUMPAD2){
		MyApp::_estimationNode->publishCommand("c moveByRel 0 0 0 0");
	}
	else if ( event.GetKeyCode() == WXK_NUMPAD3){
		MyApp::_estimationNode->publishCommand("c setReference $POSE$");
		MyApp::_estimationNode->publishCommand("c setInitialReachDist 0.2");
		MyApp::_estimationNode->publishCommand("c setStayWithinDist 0.3");
		MyApp::_estimationNode->publishCommand("c setStayTime 3");
		MyApp::_estimationNode->publishCommand("c setMaxControl 0.2");
		MyApp::_estimationNode->publishCommand("c goto -0.5 0 0 0");
		MyApp::_estimationNode->publishCommand("c goto 0 0 0 0");
		MyApp::_estimationNode->publishCommand("c goto 0.5 0 0 0");
//		MyApp::_estimationNode->publishCommand("c goto -0.5 -0.5 0 0");
//		MyApp::_estimationNode->publishCommand("c goto -0.5 0.5 0 0");
//		MyApp::_estimationNode->publishCommand("c goto 0.5 0.5 0 0");
//		MyApp::_estimationNode->publishCommand("c goto 0.5 -0.5 0 0");
//		MyApp::_estimationNode->publishCommand("c goto 0 0 0 0");
	}

	//	else if( event.GetKeyCode() == 'W'){
	// 		b_drawActMapPts = !b_drawActMapPts;
	// 	}
}
void GLImagePane::keyReleased(wxKeyEvent& event) {
	ControlCommand cmd(0,0,0,0);
	MyApp::_estimationNode->sendControlToDrone(cmd);
}

GLImagePane::GLImagePane(wxFrame* parent, int* args) :
		wxGLCanvas(parent, wxID_ANY, args, wxDefaultPosition, wxDefaultSize,
				wxFULL_REPAINT_ON_RESIZE) {
	m_context = new wxGLContext(this);
	imgData = 0;
	imgWidth = 0;
	imgHeight = 0;

	pslam_ = 0;
	imgTexture_ = 0;
	b_haveImageData = false;
	b_firstRun = true;
	b_drawReprojectionError = true;
	b_drawActMapPts = false;
}

GLImagePane::~GLImagePane() {
	delete m_context;
	if (!imgData)
		delete[] imgData;
}

void GLImagePane::resized(wxSizeEvent& evt) {
	Refresh();
}

/** Inits the OpenGL viewport for drawing in 2D. */
void GLImagePane::prepare2DViewport(int topleft_x, int topleft_y,
		int bottomrigth_x, int bottomrigth_y) {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black Background
	glEnable(GL_TEXTURE_2D); // textures
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glViewport(topleft_x, topleft_y, bottomrigth_x - topleft_x,
			bottomrigth_y - topleft_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(topleft_x, bottomrigth_x, bottomrigth_y, topleft_y);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_LINE_SMOOTH);
}

int GLImagePane::getWidth() {
	return GetSize().x;
}
int GLImagePane::getHeight() {
	return GetSize().y;
}
void GLImagePane::_draw() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// ------------- draw some 2D ----------------
	prepare2DViewport(0, 0, getWidth(), getHeight());
	glLoadIdentity();

	//draw video frame
	if (imgData) {
		glEnable(GL_TEXTURE_2D);
		if (b_firstRun) {
			glGenTextures(1, &imgTexture_); // generate OpenGL texture object
			glBindTexture(GL_TEXTURE_2D, imgTexture_); // use previously created texture object and set options
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			b_firstRun = false;
		}

		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgWidth, imgHeight, 0,
				GL_LUMINANCE, GL_UNSIGNED_BYTE, imgData);

		glBegin(GL_QUADS);
		glTexCoord2f(0, 0);
		glVertex3f(0, 0, 0);
		glTexCoord2f(1, 0);
		glVertex3f(getWidth(), 0, 0);
		glTexCoord2f(1, 1);
		glVertex3f(getWidth(), getHeight(), 0);
		glTexCoord2f(0, 1);
		glVertex3f(0, getHeight(), 0);
		glEnd();
		glDisable(GL_TEXTURE_2D);

		drawInfo(0, 0, getWidth(), getHeight());
	} else {
		// white background
		glBegin(GL_QUADS);
		glVertex3f(0, 0, 0);
		glVertex3f(getWidth(), 0, 0);
		glVertex3f(getWidth(), getHeight(), 0);
		glVertex3f(0, getHeight(), 0);
		glEnd();
	}
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	if (b_drawActMapPts) {
		drawActMapPoints();
	}
	drawFeaturePoints();
	SwapBuffers();
}

void GLImagePane::redraw(bool inPaintEvts = true) {
	wxGLCanvas::SetCurrent(*m_context);
	lockDrawing();
	MyApp::bBusyDrawingVideo = true;
	_draw();
	MyApp::bBusyDrawingVideo = false;
	unlockDrawing();
}
void GLImagePane::render(wxPaintEvent& evt) {
	if (!IsShown())
		return;
	redraw(true);
}
void GLImagePane::convert2ImageCoords(float x0, float y0, float& x, float& y) {
	x = x0 / imgWidth * getWidth();
	y = y0 / imgHeight * getHeight();
}
void GLImagePane::drawActMapPoints() {
	for (size_t i = 0; i < pActMapPts_.size(); i++) {
		double m[2];
		project(pslam_->tracker.K, pCamPos_->R, pCamPos_->t, pActMapPts_[i]->M,
				m);
		if (m[0] < 0 || m[0] >= pslam_->tracker.videoReader->_w || m[1] < 0
				|| m[1] >= pslam_->tracker.videoReader->_h)
			continue;

		float x, y;
		//convert to image coordinate
		convert2ImageCoords(m[0], m[1], x, y);
		//draw projections without corresponding map points
		glColor3f(0.0f, 0.0f, 1.0f);
		drawCircle(x, y, 4.0f, 5.0f);
	}
}
void GLImagePane::drawFeaturePoints() {
	const float COLOR_CORNER_ERR[3] = { 1.0f, 0.0f, 0.0f };
	const float COLOR_CORNER[3] = { 1.0f, 1.0f, 1.0f };

	//glLineWidth(3.0f);
	glLineWidth(2.0f);
	float radius = 1;
	for (size_t k = 0; k < featPts_.size(); k++) {
		FeaturePoint& p = featPts_[k];
		if (p.id == -1)
			continue;

		float x, y;
		//convert to image coordinate
		convert2ImageCoords(p.xo, p.yo, x, y);
		//draw projections without corresponding map points

		glColor3f(COLOR_CORNER[0], COLOR_CORNER[1], COLOR_CORNER[2]);
		drawCircle(x, y, radius, 5);

		if (p.mpt) {
			//draw projections with corresponding map points
			//			if (p->mpt->flag == FLAG_MAPPOINT_NORMAL) {
			if (!p.mpt->isFalse())
				glColor3f(0.0f, 1.0f, 0.0f);
			else
				glColor3f(1.0f, 0.0f, 0.0f);

			//test
			if (p.mpt->flag >= FLAG_MAPPOINT_TEST1) {
				glColor3f(1.0f, 0.0f, 0.0f);
				GLUquadric* quad = gluNewQuadric();
				glTranslatef(x, y, 0);
				gluDisk(quad, 0, 1.5 * radius, 30, 10);
				glTranslatef(-x, -y, 0);
				gluDeleteQuadric(quad);
			}

			glLineWidth(2.0f);

			drawCircle(x, y, 1.5 * radius, 10);

			//drawCircle(x, y, radius, 10);

			if (b_drawReprojectionError=true) {
				//draw re-projection error
				double* M = p.mpt->M;
				double m[2];
				project(pslam_->tracker.K, pCamPos_->R, pCamPos_->t, M, m);
				float x01 = float(m[0]);
				float y01 = float(m[1]);
				float x1, y1;
				convert2ImageCoords(x01, y01, x1, y1);
				glLineWidth(2);
				glColor3f(COLOR_CORNER_ERR[0], COLOR_CORNER_ERR[1],
						COLOR_CORNER_ERR[2]);

				//convert to image coordinate
				float x2, y2;
				convert2ImageCoords(p.x, p.y, x2, y2);

				double dx = x1 - x2;
				double dy = y1 - y2;
				drawLine(x, y, x + dx, y + dy);
				glLineWidth(1);
			}
		}
	}
}

#include "geometry/SL_5point.h"
#include "geometry/SL_FundamentalMatrix.h"
#include "tools/SL_DrawCorners.h"
#include "tools/SL_Print.h"
void GLImagePane::initImageData(int w, int h) {
	if (imgData)
		delete[] imgData;
	imgWidth = w;
	imgHeight = h;
	imgData = new unsigned char[w * h];
}
void GLImagePane::copyDisplayData() {
	enterBACriticalSection();
	//copy the image
	memcpy(imgData, pslam_->tracker.img.data,
			imgWidth * imgHeight * sizeof(unsigned char));

	b_haveImageData = true;
	featPts_.clear();

	for (size_t i = 0; i < pslam_->tracker.curFeatPts.size(); i++) {
		if (pslam_->tracker.curFeatPts[i]->pt.f != pslam_->curFrame){
			//printf("In GLImagePane::copyDisplayData pt.f: %d, pslam->curFrame: %d\n",
			//	pslam_->tracker.curFeatPts[i]->pt.f, pslam_->curFrame);
			continue;
		}
		assert(pslam_->tracker.curFeatPts[i]->pt.f == pslam_->curFrame);
		featPts_.push_back(pslam_->tracker.curFeatPts[i]->pt);
	}

	pCamPos_ = pslam_->camPose.current();
	leaveBACriticalSection();

}
void GLImagePane::drawInfo(int topleft_x, int topleft_y, int bottomrigth_x,
		int bottomrigth_y) {
	glViewport(topleft_x, topleft_y, bottomrigth_x - topleft_x,
			bottomrigth_y - topleft_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(topleft_x, bottomrigth_x, bottomrigth_y, topleft_y);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//glutPrint3D(0, 0, 0, "0,0,0", 255, 0, 0, 0.5);
	if (pslam_->state == SLAM_STATE_RELOCATE) {
		glColor4f(1.0f, 0.0f, 0.0f, 0.3f);
		glBegin(GL_QUADS);
		glVertex2f(0, 0);
		glVertex2f(getWidth(), 0);
		glVertex2f(getWidth(), getHeight());
		glVertex2f(0, getHeight());
		glEnd();
	}

	double camColor[3] = { GLScenePane::CAMERA_COLORS[0],
			GLScenePane::CAMERA_COLORS[1], GLScenePane::CAMERA_COLORS[2] };
	glColor3f(camColor[0], camColor[1], camColor[2]);
	glBegin(GL_QUADS);
	glVertex2f(5, 5);
	glVertex2f(30, 5);
	glVertex2f(30, 30);
	glVertex2f(5, 30);
	glEnd();
	if (pslam_) {
		char buf[256];
		sprintf(buf, "frame:%d, tracking ratio:%lf, bad tracking:%d",
				pslam_->curFrame, pslam_->tracker.featTracker.trackRatio,
				pslam_->tracker.frmNumBackTracking);
		glutPrint2D(40, 20, buf, camColor[0], camColor[1], camColor[2], 1.0f,
				false);
	}
}
void GLImagePane::saveScreen(const char* filePath, vector<cv::Mat>& frames) {
//	wxGLCanvas::SetCurrent(*m_context);
//	wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
	int w = GetParent()->GetSize().x;
	int h = GetParent()->GetSize().y;
//	cout << "w h " << w << " " << h <<endl;

	ImgRGB imgData(w, h);
	glReadPixels(0, 0, w, h, GL_BGR, GL_UNSIGNED_BYTE, imgData.data);
	cv::Mat img(h, w, CV_8UC3, imgData);
	cv::line(img, cv::Point2i(0, 0), cv::Point2i(0, h - 1), cv::Scalar(0, 0, 0),
			2, CV_AA, 0);
	cv::line(img, cv::Point2i(0, h - 1), cv::Point2i(w - 1, h - 1),
			cv::Scalar(0, 0, 0), 2, CV_AA, 0);
	cv::line(img, cv::Point2i(w - 1, h - 1), cv::Point2i(w - 1, 0),
			cv::Scalar(0, 0, 0), 2, CV_AA, 0);
	cv::line(img, cv::Point2i(w - 1, 0), cv::Point2i(0, 0), cv::Scalar(0, 0, 0),
			2, CV_AA, 0);
//	CvMat cvImg = img;
//	cvFlip(&cvImg, 0);
	cv::Mat flipImg;
	cv::flip(img, flipImg, 0);
//	cv::imwrite(filePath, img);
	frames.push_back(flipImg);
}
