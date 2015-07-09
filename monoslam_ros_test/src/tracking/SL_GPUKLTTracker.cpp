#include "SL_GPUKLTTracker.h"

GPUKLTTracker::GPUKLTTracker():_W(0),_H(0), _tracker(0), _feature(0){

}
GPUKLTTracker::~GPUKLTTracker(){
	close();
}
void GPUKLTTracker::close(){
	if(_tracker){
		_tracker->deallocate();
		delete _tracker;
	}
	if( _feature){
		delete _feature;
	}
	_tracker = 0;
	_feature = 0;
}
void GPUKLTTracker::open( int W, int H, V3D_GPU::KLT_SequenceTrackerConfig* pCfg){
	close();
	assert(pCfg);
	_W = W;
	_H = H;

	_tracker = new V3D_GPU::KLT_SequenceTracker(*pCfg);
	_tracker->allocate(W, H, pCfg->nLevels, GPUKLT_TRACK_WIDTH, GPUKLT_TRACK_HEIGHT);
	_nMaxTrack = GPUKLT_TRACK_WIDTH * GPUKLT_TRACK_HEIGHT;
	_feature = new V3D_GPU::KLT_TrackedFeature[_nMaxTrack];
}
void GPUKLTTracker::detect(const ImgG& img){
	assert(img.w == _W && img.h == _H);
	int nDetected = 0;
	_tracker->detect(img.data, nDetected, _feature);
	if( nDetected == 0)
		repErr("no feature point is detected!");
	_tracker->advanceFrame();
}
int GPUKLTTracker::track(const ImgG& img){
	assert(img.w == _W && img.h == _H);
	int nTracked = 0;
	_tracker->track(img.data, nTracked, _feature);
	_tracker->advanceFrame();
	return nTracked;
}
int GPUKLTTracker::trackRedetect(const ImgG& img){
	assert(img.w == _W && img.h == _H);
	int nDetected = 0;
	_tracker->redetect(img.data, nDetected, _feature);
	_tracker->advanceFrame();
	return nDetected;
}

void GPUKLTTracker::reset(const ImgG& img, const Mat_f& featPts){
	assert(img.w == _W && img.h == _H);
	assert(featPts.m > 0 && featPts.n == 2);
	_tracker->_pyrCreator1->buildPyramidForGrayscaleImage(img.data);
	
	Mat_f corners(_nMaxTrack, 3);
	for( int i = 0; i < _nMaxTrack; i++){
		if( i < featPts.m){
			_feature[i].status = 1;
			_feature[i].pos[0] = featPts[2*i]/_W;
			_feature[i].pos[1] = featPts[2*i+1]/_H;
			if( _tracker->_trackWithGain)
				_feature[i].gain = 1.0f;

			corners[3*i] = _feature[i].pos[0];
			corners[3*i+1] = _feature[i].pos[1];
			corners[3*i+2] = 1.0f;
		}else{
			_feature[i].status = -1;
			_feature[i].pos[0] = -1;
			_feature[i].pos[1] = -1;
			if( _tracker->_trackWithGain)
				_feature[i].gain = 1.0f;

			corners[3*i] = -1.0f;
			corners[3*i+1] = -1.0f;
			corners[3*i+2] = 1.0f;
		}
	}

	if( _tracker->_trackWithGain)
		_tracker->_trackerWithGain->provideFeaturesAndGain(corners.data);
	else
		_tracker->_tracker->provideFeatures(corners.data);
	_tracker->advanceFrame();
}


void GPUKLTTracker::provideCurrFeatures(const ImgG& img, float* corners){
	_tracker->_pyrCreator1->buildPyramidForGrayscaleImage(img.data);
	for( int i = 0; i < _nMaxTrack; i++){
		if (corners[3 * i] >= 0){
			_feature[i].status = 1;
			_feature[i].pos[0] = corners[3*i]/_W;
			_feature[i].pos[1] = corners[3*i+1]/_H;
			if( _tracker->_trackWithGain)
				_feature[i].gain = 1.0f;
			corners[3*i] = _feature[i].pos[0];
			corners[3*i+1] = _feature[i].pos[1];
			corners[3*i+2] = 1.0f;
		}
		else{
			_feature[i].status = -1;
			_feature[i].pos[0] = -1;
			_feature[i].pos[1] = -1;
			if( _tracker->_trackWithGain)
				_feature[i].gain = 1.0f;

			corners[3*i] = -1.0f;
			corners[3*i+1] = -1.0f;
			corners[3*i+2] = 1.0f;
		}
	}
	if( _tracker->_trackWithGain)
		_tracker->_trackerWithGain->provideFeaturesAndGain(corners);
	else
		_tracker->_tracker->provideFeatures(corners);
	_tracker->advanceFrame();
}

void GPUKLTTracker::readFeatPoints(Mat_d& featPts, Mat_i& flag){
	featPts.resize(_nMaxTrack, 2);
	flag.resize(_nMaxTrack,1);

	for( int i = 0; i < _nMaxTrack; i++){
		flag[i] =_feature[i].status;
		if( _feature[i].status >= 0){
			featPts[2*i] = _feature[i].pos[0] *_W;
			featPts[2*i+1] = _feature[i].pos[1] *_H;
		}else{
			featPts[2*i] = -1;
			featPts[2*i+1] = -1;
		}
	}
}
