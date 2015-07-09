#ifndef RELOCALIZATION_H
#define RELOCALIZATION_H
#include "slam/SL_KeyFrame.h"
#include <deque>
using namespace std;

#define RELOC_MAX_NUM_STD 30.0
#define RELOC_FRM_NUM 10

class MoSLAM;
class Relocalizer {
protected:
	MoSLAM& slam;
	deque<int> numTrackedFeatPts;

	Mat_d R2_init;
public:
	Relocalizer(MoSLAM& slam_);

	void reset();
	bool isNumChangeLittle(const deque<int>& nums);
	bool isCameraSteady();
	bool tryToRecover();
	/* register the current image to the given key frame
	 * output: 1.camera pose of the current image
	 *         2.corresponding 2D feature points of the 3D map points in the key frame
	 */
	bool registerToKeyFrame(const KeyFrame* keyFrame, const ImgG& curImg,
			vector<MapPoint*>& matchedMapPts, Mat_d& matched2DPts, vector<cv::Point2f>& newPointsPair);

	int frameRegister(const ImgG& I1, const ImgG& I2, const vector<FeaturePoint*>& pts1, vector<cv::Point2f>& newPoints, Mat_d& pts2, Mat_uc& flag);

	KeyFrame* searchKeyPosebyThumbImage(double maxcost);

	cv::Mat genPointMask(vector<FeaturePoint*>& keyPts);
};
#endif
