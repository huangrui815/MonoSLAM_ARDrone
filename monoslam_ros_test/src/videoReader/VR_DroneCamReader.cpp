#include "VR_DroneCamReader.h"
#include "app/APP_SynObj.h"
#include "imgproc/SL_ImageOp.h"
#include <iomanip>

DroneCamReader::DroneCamReader(){
		avi = false;
		_bOpened = false;
		_bRecording = false;
		_bVideoOpened = false;
		_frameId = 0;
}

DroneCamReader::~DroneCamReader(){

}

bool DroneCamReader::open(){
	lock();
	//while(true){
		if (!_curImg.empty()){
			_w = _curImg.w;
			_h = _curImg.h;
			unlock();
			return true;
			//break;
		}
//		else
//			//cout << "Openning video\n";
//			sleep(30/1000);
	//}
		unlock();
		return false;

}

void DroneCamReader::grabFrame(){

}

void DroneCamReader::readCurFrameRGB(unsigned char* imgdata){
	popFront(_currImgTS,_curImg);
	if (!_curImg.empty())
		memcpy(imgdata, _curImg.data, _curImg.m * _curImg.n * 3);
}

void DroneCamReader::readCurFrameGray(unsigned char* grayImgData) {
	popFront(_currImgTS, _curImg);
	if (!_curImg.empty()){
		cv::Mat cvImg(_curImg.m, _curImg.n, CV_8UC3, _curImg.data);
		cv::Mat videoFrame(_curImg.m, _curImg.n, CV_8UC1, grayImgData);
		cv::cvtColor(cvImg, videoFrame, CV_RGB2GRAY);
	}
}

void DroneCamReader::readCurFrame(unsigned char* rgbdata,
	unsigned char* graydata) {
	popFront(_currImgTS,_curImg);
	if (!_curImg.empty()){
		memcpy(rgbdata, _curImg.data, _curImg.m * _curImg.n * 3);
		cv::Mat cvImg(_curImg.m, _curImg.n, CV_8UC3, _curImg.data);
		cv::Mat videoFrame(_curImg.m, _curImg.n, CV_8UC1, graydata);
		cv::cvtColor(cvImg, videoFrame, CV_RGB2GRAY);
	}
}

double DroneCamReader::getTimeStamp(){
	return _currImgTS;
}

void DroneCamReader::enableRecording(){
	_bRecording = true;
}

void DroneCamReader::setlogFilePath(const char* tsPath, const char* videoPath){
	_tsFilePath = tsPath;
	_videoFilePath = videoPath;
}

void DroneCamReader::addBack(double tm, const cv::Mat& imgMat){
	lock();
//	ImgRGB* pImg = new ImgRGB(imgMat.cols, imgMat.rows);
//	memcpy(pImg->data, imgMat.data, imgMat.rows * imgMat.cols * 3);
	timeStamp.push_back(tm);
//	bgr2rgb(*pImg);
	vecImg.push_back(imgMat);
//	_currImgTS = tm;
//	cloneImg(*pImg, _curImg);
//	bgr2rgb(_curImg);
	if (!_bOpened){
//		cloneImg(*pImg, _curImg);
		_curImg.resize(imgMat.cols, imgMat.rows);
		memcpy(_curImg.data, imgMat.data, imgMat.rows * imgMat.cols * 3);
		bgr2rgb(_curImg);
		_bOpened = true;
	}

	pthread_cond_signal(&SynObj::s_condReadFrameQueue);
	unlock();
}

bool DroneCamReader::popFront(double& tm, ImgRGB& img) {
	bool res = false;
	lock();
	while (vecImg.empty()){
//		printf("vecImg Empty\n");
		pthread_cond_wait(&SynObj::s_condReadFrameQueue,
						&SynObj::s_mutexReadFrameQueue);
	}

	if(!vecImg.empty())
	{
		tm = timeStamp.front();
		cv::Mat img_rgb;
		cv::cvtColor(vecImg.front(), img_rgb, CV_BGR2RGB);
		memcpy(img.data, vecImg.front().data,
				vecImg.front().rows * vecImg.front().cols * 3);

		if (_bRecording){
			if (!_bVideoOpened){
				if (!_videoWriter.open(_videoFilePath,
						CV_FOURCC('D', 'I', 'V', 'X'), 30, img_rgb.size(),true))
					printf("cannot open '%s' to write!", _videoFilePath);

				_tsLog.open(_tsFilePath);
				if (!_tsLog)
					printf("cannot open '%s' to write!", _tsFilePath);

				_bVideoOpened = true;
			}
			_tsLog << _frameId++ << " " << setprecision(16) << tm << endl;
			_videoWriter << vecImg.front();
		}
		timeStamp.pop_front();
		vecImg.pop_front();
		res = true;
	}
	unlock();
	return res;
}

void DroneCamReader::lock(){
	pthread_mutex_lock(&SynObj::s_mutexReadFrameQueue);
}

void DroneCamReader::unlock(){
	pthread_mutex_unlock(&SynObj::s_mutexReadFrameQueue);
}
