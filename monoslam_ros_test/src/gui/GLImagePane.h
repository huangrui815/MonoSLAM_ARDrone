/*
 * GLImagePane.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#ifndef GLIMAGEPANE_H_
#define GLIMAGEPANE_H_

#include <GL/glew.h> // should be included before "wx/glcanvas.h"
#include "wx/wx.h"
#include "wx/glcanvas.h"

#include "app/SL_MoSLAM.h"
class GLImagePane: public wxGLCanvas {
	wxGLContext* m_context;
	unsigned char* imgData;
	int imgWidth, imgHeight;
	MoSLAM* pslam_;

	//store pointers to the feature points
	std::vector<FeaturePoint> featPts_;

	//store pointers to the active map points
	std::vector<MapPoint*> pActMapPts_;

	//store current camera pose
	CamPoseItem* pCamPos_;

	GLuint imgTexture_;
	bool b_drawActMapPts;
	bool b_haveImageData;
	bool b_firstRun;
	bool b_drawReprojectionError;

public:
	GLImagePane(wxFrame* parent, int* args);
	virtual ~GLImagePane();

	void resized(wxSizeEvent& evt);

	int getWidth();
	int getHeight();

	void _draw();
	void redraw(bool inPaintEvts);
	void render(wxPaintEvent& evt);
	void prepare2DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y);

	// events
	DECLARE_EVENT_TABLE()
	void mouseMoved(wxMouseEvent& event);
	void mouseDown(wxMouseEvent& event);
	void mouseWheelMoved(wxMouseEvent& event);
	void mouseReleased(wxMouseEvent& event);
	void rightClick(wxMouseEvent& event);
	void mouseLeftWindow(wxMouseEvent& event);
	void keyPressed(wxKeyEvent& event);
	void keyReleased(wxKeyEvent& event);
public:
	void convert2ImageCoords(float x0, float y0, float& x, float& y);
	void drawActMapPoints();
	void drawFeaturePoints();
	void drawClickedPoint(ImgRGB& img);
	void drawBoundBox();
	void initImageData(int w, int h);
	void copyDisplayData();
	void setSLAMData(MoSLAM* pSLAM) {
		pslam_ = pSLAM;
	}
	void saveScreen(const char* filePath, vector<cv::Mat>& frames);
	void drawInfo(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y);
};
#endif /* GLIMAGEPANE_H_ */
