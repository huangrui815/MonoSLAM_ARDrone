#ifndef _SYNOBJ_H
#define _SYNOBJ_H
#include "pthread.h"
/* objects for synchronization*/
class SynObj{
public:
	/* for synchronization of reading navigation data and reading image frames*/
	static bool bReadNavData;
	static bool bBusyBAing;
	static pthread_mutex_t s_muteReadNavdata;
	static pthread_cond_t s_condReadNavdata;
	static pthread_mutex_t s_muteWriteNavdata;
	/* end of synchronization*/

	//for creating GUIs
	static pthread_mutex_t s_mutexCreateGUI;
	//for bundle adjustments
	static pthread_mutex_t s_mutexBA;

	static pthread_mutex_t s_mutexDrawing;
	static pthread_cond_t s_condCreateGUI;

	//for reading image frames from the drone
	static pthread_mutex_t s_mutexReadVideo; 
	static pthread_cond_t s_condReadVideo;

	//for accessing the  reference points
	static pthread_mutex_t s_mutexReadRefPoints;

	//for reading video from ardrone_driver
	static pthread_mutex_t s_mutexReadFrameQueue;
	static pthread_cond_t s_condReadFrameQueue;
};

void initSynObjs();
void destroySynObjs();

void enterBACriticalSection();
void leaveBACriticalSection();

void preWaitCreateGUI();
void waitCreateGUI();
void broadcastCreateGUI();

void lockDrawing();
void unlockDrawing();

#endif
