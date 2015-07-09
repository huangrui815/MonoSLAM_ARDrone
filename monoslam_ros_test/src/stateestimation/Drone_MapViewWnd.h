
#ifndef _DRONE_MAPVIEWWND_H_
#define _DRONE_MAPVIEWWND_H_
#include "Drone_MapViewPane.h"
#include "wx/event.h"
#include "wx/wx.h"
class MapViewWnd: public wxFrame {
public:
	MapViewPane* glPane;
	virtual ~MapViewWnd();
	MapViewWnd(const char* wndName, int x0, int y0, int W, int H);
	void setSLAMData(MoSLAM* pSLAM) {
		glPane->setSLAMData(pSLAM);
	}
	void setFilterPointer(DroneKalmanFilter* filter){
		glPane->setFilterPointer(filter);
	}
	void setNodePointer(EstimationNode* node){
		glPane->setNodePointer(node);
	}
	void copyDispData() {
		glPane->copyDispData();
	}
	void redraw() {
		glPane->Refresh();
	}
	void save(const char* filePath) {
		glPane->saveScreen(filePath);
	}
};

#endif
