/*
 * MoSLAMThread.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */
#ifndef MoSLAMTHREAD_H_
#define MoSLAMTHREAD_H_

#include "wx/wx.h"
#include "wx/event.h"
#include "wx/thread.h"

#ifdef WIN32
#include <io.h>  
#include <process.h>  
#else
#include <unistd.h>
#define Sleep(tm) {usleep(tm*1000);}
#endif

DECLARE_EVENT_TYPE(EventUpdateViews,-1)
;

class MoSLAMThread: public wxThread {
public:
	MoSLAMThread();
	~MoSLAMThread();
protected:
	virtual ExitCode Entry();
};
void updateDisplayData();
void redrawAllViews();

bool offlineMain(); 
bool usbCamMain();
bool ardroneMain();
bool ardroneROSMain();
#endif /* MoSLAMTHREAD_H_ */
