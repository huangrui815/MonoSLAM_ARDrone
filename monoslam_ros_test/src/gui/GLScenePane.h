/*
 * GLScenePane.h
 *
 *  Created on: 2011-7-15
 *      Author: zou
 */

#ifndef GLSCENEPANE_H_
#define GLSCENEPANE_H_
#include "GL/glew.h"
#include "GLTrackballPane.h"
#include "app/SL_MoSLAM.h"

using namespace std;
class GLScenePane: public GLTrackballPane {
public:
	GLScenePane(wxWindow * parent, wxWindowID id, int args[]) :
			GLTrackballPane(parent, id, args) {
		memset(m_center, 0, sizeof(double) * 3);
		m_pSLAM = 0;
		m_scale = 1.0;
		m_camSize = 0.05;
		m_autoScale = true;
		m_followCamId = -1;
		m_camView = false;
		m_pointSize = 1.0;
	}
	virtual ~GLScenePane() {
	}
public:
	double m_pointSize;
	int m_followCamId;
	bool m_camView;
	bool m_autoScale;
	double m_camSize;
	
	/* slam data*/
	const MoSLAM* m_pSLAM;
	std::vector<const MapPoint*> curMapPoints;
	std::vector<const MapPoint*> actMapPoints;
	std::vector<const MapPoint*> iactMapPoints;
	std::vector<CamPoseItem*> camPoses;
	std::vector<KeyFrame*> keyPoses;
	
	/* copy SLAM data*/
	void setSLAMData(const MoSLAM* pSLAM) {
		m_pSLAM = pSLAM;
	}
	void copyDispData();
	void getSceneScale();

	static float CAMERA_COLORS[15];
public:
	double m_center[3];
	double m_scale;
	void drawPoints();
	void drawCameras();
	void drawInfo();
	virtual void drawGLObjs();
	virtual void draw(bool inPaintEvts);
	void saveScreen(const char* filePath, vector<cv::Mat>& frames);
public:
	void resized(wxSizeEvent& evt);
	void rightClick(wxMouseEvent& event);
	void keyPressed(wxKeyEvent& event);
	void charPressed(wxKeyEvent& event);DECLARE_EVENT_TABLE()
};

#endif /* GLSCENEPANE_H_ */
