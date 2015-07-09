#ifndef GPUKLTTRACKER_H_
#define GPUKLTTRACKER_H_

#include "SL_BaseKLTTracker.h"
#include "CGKLT/v3d_gpupyramid.h"
#include "CGKLT/v3d_gpuklt.h"

#define GPUKLT_TRACK_WIDTH 32
#define GPUKLT_TRACK_HEIGHT 32

class GPUKLTTracker:public BaseKLTTracker{
protected:
	V3D_GPU::KLT_SequenceTracker* _tracker;
	V3D_GPU::KLT_TrackedFeature* _feature;
public:
	int _W, _H;
	int _nMaxTrack;

	GPUKLTTracker();
	virtual ~GPUKLTTracker();

	void close();
	void open(int W, int H, V3D_GPU::KLT_SequenceTrackerConfig* pCfg);
	
	virtual void detect(const ImgG& img);
	virtual int track(const ImgG& img);
	virtual int trackRedetect(const ImgG& img);
	
	virtual void reset(const ImgG& img, const Mat_f& featPts);
	void provideCurrFeatures(const ImgG& img, float* corners);
	virtual void readFeatPoints(Mat_d& featPts, Mat_i& flag);
};
#endif
