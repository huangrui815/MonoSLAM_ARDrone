/*
 * GLScenePane.h
 *
 *  Created on: 2011-7-15
 *      Author: zou
 */

#ifndef _MAPVIEWPANE_H_
#define _MAPVIEWPANE_H_
#include "GL/glew.h"
#include "gui/GLTrackballPane.h"
#include "app/SL_MoSLAM.h"

class EstimationNode;

class DroneKalmanFilter;

using namespace std;

struct DronePose{
	double _x;
	double _y;
	double _z;
	double _roll;
	double _pitch;
	double _yaw;
};

class MapViewPane: public GLTrackballPane {
public:
	MapViewPane(wxWindow * parent, wxWindowID id, int args[]) :
			GLTrackballPane(parent, id, args) {
		memset(m_center, 0, sizeof(double) * 3);
		m_pSLAM = 0;
		m_scale = 1.0;
		m_camSize = 0.05;
		m_autoScale = true;
		m_followCamId = -1;
		m_camView = false;
		m_pointSize = 1.0;

		_lastFilterState = new double[10];
		_toClearTrail = false;
		_dronePoseVec = new vector<tf::Transform>;
	}
	virtual ~MapViewPane() {
		delete _lastFilterState;
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

	DroneKalmanFilter* _droneFilter;
	EstimationNode* _node;

	double* _lastFilterState;
	bool _toClearTrail;
	tf::Transform currDronePose;
	vector<tf::Transform> *_dronePoseVec;
	static pthread_mutex_t _mutexDronePoseVec;

	/* copy SLAM data*/
	void setSLAMData(const MoSLAM* pSLAM) {
		m_pSLAM = pSLAM;
	}

	void setFilterPointer(DroneKalmanFilter* droneFilter) {
		_droneFilter = droneFilter;
	}

	void setNodePointer(EstimationNode* node){
		_node = node;
	}

	void copyDispData();
	void getSceneScale();

	static float CAMERA_COLORS[15];
public:
	double m_center[3];
	double m_scale;
	void drawPoints();
	void drawCameras();
	void addDronePose();
	void drawDronePose();
	void drawAxis();
	void drawRPY();
	void drawTransform(tf::Transform& trans);
	void drawInfo();
	void drawGrids(); // draw grid on xy plane
	virtual void drawGLObjs();
	virtual void draw(bool inPaintEvts);
	void saveScreen(const char* filePath);
public:
	void resized(wxSizeEvent& evt);
	void rightClick(wxMouseEvent& event);
	void keyPressed(wxKeyEvent& event);
	void charPressed(wxKeyEvent& event);DECLARE_EVENT_TABLE()
};

#endif /* GLSCENEPANE_H_ */
