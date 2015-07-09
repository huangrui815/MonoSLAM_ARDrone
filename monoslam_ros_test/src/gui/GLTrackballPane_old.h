/*
 * GLTrackballPane.h
 *
 *  Created on: 2011-6-10
 *      Author: Danping Zou
 */

#ifndef GLTRACKBALL_H_
#define GLTRACKBALL_H_
#include "wx/wx.h"
#include "wx/glcanvas.h"
#include <GL/gl.h>
#include <GL/glut.h>
class GLTrackballPane: public wxGLCanvas {
public:
	GLTrackballPane(wxWindow * parent, wxWindowID id, int args[]) :
			wxGLCanvas(parent, id, args, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE) {
		m_context = new wxGLContext(this);
	}
	virtual ~GLTrackballPane();
	virtual void drawGLObjs();
	virtual void draw(bool inPaintEvts);
	virtual void render(wxPaintEvent& evt);
public:
	wxGLContext* m_context;
	// events
	void prepareViewport();
	void mouseMoved(wxMouseEvent& event);
	void leftMouseDown(wxMouseEvent& event);
	void leftMouseUp(wxMouseEvent& event);

	void midMouseDown(wxMouseEvent& event);
	void midMouseUp(wxMouseEvent& event);

	void mouseWheelMoved(wxMouseEvent& event);
	void mouseReleased(wxMouseEvent& event);
	void rightClick(wxMouseEvent& event);
	void mouseLeftWindow(wxMouseEvent& event);
	void keyPressed(wxKeyEvent& event);
	void keyReleased(wxKeyEvent& event);
	void resized(wxSizeEvent& evt);DECLARE_EVENT_TABLE()
	;
};

#endif /* GLPANE_H_ */
