/*
 * GLImageWnd.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#ifndef GLIMAGEWND_H_
#define GLIMAGEWND_H_
#include "GLImagePane.h"
class GLImageWnd : public wxFrame {
protected:
	GLImagePane* glPane;
public:
	virtual ~GLImageWnd();
	GLImageWnd(const char* wndName , int x0 , int y0 , int W , int H);
public:
	void initVideoFrame(int W , int H) {
		glPane->initImageData(W, H);
	}
	void copyDataForDisplay() {
		glPane->copyDisplayData();
	}
	void setSLAMData(MoSLAM* pSLAM) {
		glPane->setSLAMData(pSLAM);
	}
	void redraw() {
		//glPane->draw();
		//glPane->redraw(false);
		glPane->Refresh();
	}
	void save(const char* filePath, vector<cv::Mat>& frames) {
		glPane->saveScreen(filePath, frames);
	}
};

#endif /* GLIMAGEWND_H_ */
