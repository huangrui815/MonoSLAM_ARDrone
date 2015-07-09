#ifndef _DRONECAMREADER_H
#define _DRONECAMREADER_H
#include "VR_VideoReader.h"
#include "imgproc/SL_Image.h"
#include <fstream>
#include <deque>
using namespace std;

class DroneCamReader:public VideoReader {
public:
	DroneCamReader();
	virtual ~DroneCamReader();
	virtual bool open();
	virtual void grabFrame();
	virtual void readCurFrameRGB(unsigned char* imgdata);
	virtual void readCurFrameGray(unsigned char* grayImgData);
	virtual void readCurFrame(unsigned char* rgbdata, unsigned char* graydata);
	virtual double getTimeStamp();

	void addBack(double tm, const cv::Mat& img);
	bool popFront(double& tm, ImgRGB& img);
	ImgRGB& getCurImg(){return _curImg;}
	void enableRecording();
	void lock();
	void unlock();
	void setlogFilePath(const char* tsPath, const char* videoPath);

public:
	ImgRGB _curImg;
	std::deque<double> timeStamp;
	double _currImgTS;
	std::deque<cv::Mat> vecImg;
	bool _bOpened;
	bool _bRecording;
	bool _bVideoOpened;
	cv::VideoWriter _videoWriter;
	ofstream _tsLog;
	int _frameId;
	const char* _tsFilePath;
	const char* _videoFilePath;
};
#endif
