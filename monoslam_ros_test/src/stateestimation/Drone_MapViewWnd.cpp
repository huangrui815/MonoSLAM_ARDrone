#include "Drone_MapViewWnd.h"

MapViewWnd::~MapViewWnd() {

}
MapViewWnd::MapViewWnd(const char* wndName, int x0, int y0, int W, int H) :
		wxFrame((wxFrame *) NULL, -1, wndName, wxPoint(x0, y0), wxSize(W, H)) {
	wxIcon icon;
//	if(icon.LoadFile("MoSLAM.png", wxBITMAP_TYPE_PNG))
//		SetIcon(icon);

	int args[] = { WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, WX_GL_SAMPLES, 4, WX_GL_SAMPLE_BUFFERS, 1, 16, 0 };
	glPane = new MapViewPane((wxFrame*) this, wxID_ANY, args);
}
