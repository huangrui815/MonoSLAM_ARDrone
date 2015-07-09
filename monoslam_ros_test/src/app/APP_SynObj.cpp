#include "APP_SynObj.h"

bool SynObj::bReadNavData = false;
bool SynObj::bBusyBAing = false;

pthread_mutex_t SynObj::s_muteReadNavdata;
pthread_cond_t SynObj::s_condReadNavdata;
pthread_mutex_t SynObj::s_muteWriteNavdata;

pthread_mutex_t SynObj::s_mutexCreateGUI;
pthread_mutex_t SynObj::s_mutexBA;
pthread_mutex_t SynObj::s_mutexDrawing; 
pthread_cond_t SynObj::s_condCreateGUI;

pthread_mutex_t SynObj::s_mutexReadVideo;
pthread_cond_t SynObj::s_condReadVideo; 

pthread_mutex_t SynObj::s_mutexReadRefPoints;

pthread_mutex_t SynObj::s_mutexReadFrameQueue;
pthread_cond_t SynObj::s_condReadFrameQueue;

void initSynObjs(){
	pthread_mutex_init(&SynObj::s_muteReadNavdata, 0);
	pthread_cond_init(&SynObj::s_condReadNavdata, 0);
	pthread_mutex_init(&SynObj::s_muteWriteNavdata, 0);

	pthread_mutex_init(&SynObj::s_mutexCreateGUI, 0);
	pthread_mutex_init(&SynObj::s_mutexBA, 0);
	pthread_mutex_init(&SynObj::s_mutexDrawing, 0);
	pthread_cond_init(&SynObj::s_condCreateGUI, 0);

	pthread_mutex_init(&SynObj::s_mutexReadVideo, 0);
	pthread_cond_init(&SynObj::s_condReadVideo, 0);

	pthread_mutex_init(&SynObj::s_mutexReadRefPoints, 0);

	pthread_mutex_init(&SynObj::s_mutexReadFrameQueue, 0);
	pthread_cond_init(&SynObj::s_condReadFrameQueue, 0);
}
void destroySynObjs(){
	pthread_mutex_destroy(&SynObj::s_muteReadNavdata);
	pthread_cond_destroy(&SynObj::s_condReadNavdata);
	pthread_mutex_destroy(&SynObj::s_muteWriteNavdata);

	pthread_mutex_destroy(&SynObj::s_mutexCreateGUI);
	pthread_mutex_destroy(&SynObj::s_mutexBA);
	pthread_mutex_destroy(&SynObj::s_mutexDrawing);
	pthread_cond_destroy(&SynObj::s_condCreateGUI);

	pthread_mutex_destroy(&SynObj::s_mutexReadVideo);
	pthread_cond_destroy(&SynObj::s_condReadVideo);
	pthread_mutex_destroy(&SynObj::s_mutexReadRefPoints);

	pthread_mutex_destroy(&SynObj::s_mutexReadFrameQueue);
	pthread_cond_destroy(&SynObj::s_condReadFrameQueue);
}

void enterBACriticalSection() {
	pthread_mutex_lock(&SynObj::s_mutexBA);
}
void leaveBACriticalSection() {
	pthread_mutex_unlock(&SynObj::s_mutexBA);
}

void preWaitCreateGUI() {
	pthread_mutex_lock(&SynObj::s_mutexCreateGUI);
}
void waitCreateGUI() {
	pthread_cond_wait(&SynObj::s_condCreateGUI, &SynObj::s_mutexCreateGUI);
#ifndef WIN32
	pthread_mutex_unlock(&SynObj::s_mutexCreateGUI);
#endif
}
void broadcastCreateGUI() {
#ifndef WIN32
	pthread_mutex_lock(&SynObj::s_mutexCreateGUI);
#endif
	pthread_cond_signal(&SynObj::s_condCreateGUI);
#ifndef WIN32
	pthread_mutex_unlock(&SynObj::s_mutexCreateGUI);
#endif
}

void lockDrawing(){
	pthread_mutex_lock(&SynObj::s_mutexDrawing);
}
void unlockDrawing(){
	pthread_mutex_unlock(&SynObj::s_mutexDrawing);
}
