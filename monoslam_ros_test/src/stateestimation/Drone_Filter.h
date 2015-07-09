#ifndef _DRONE_FILTER_H_
#define _DRONE_FILTER_H_
#include "Drone_FilterHelper.h"
#include "Drone_ScaleEstimator.h"
#include <deque>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include "monoslam_ros_test/filter_state.h"
#include "tf/transform_datatypes.h"

class EstimationNode;

class DroneKalmanFilter
{
private:
	// filters
	PVFilter _xFilter;
	PVFilter _yFilter;
	PVFilter _zFilter;
	PFilter _rollFilter;
	PFilter _pitchFilter;
	PVFilter _yawFilter;

	double x_offset, y_offset, z_offset;
	double xy_scale, z_scale;
	double scale_from_z;
	double scale_from_xy;
	double roll_offset, pitch_offset, yaw_offset;
	bool offsets_xyz_initialized;
	bool scale_xyz_initialized;

	double _sumII;
	double _sumPP;
	double _sumPI;

	// intermediate values for re-estimation of relation parameters
	double xyz_sum_IMUxIMU;
	double xyz_sum_PTAMxPTAM;
	double xyz_sum_PTAMxIMU;
	double rp_offset_framesContributed;

	// parameters used for height and yaw differentiation
	double last_yaw_IMU;
	double last_z_IMU;
	long last_yaw_droneTime;
	long last_z_droneTime;
	long last_z_packageID;

	// contains the last slam-pose added (raw slam-data).
	double last_slam_pose[6];

	// contains the pose directly after the last slam-fuse.
	double last_fused_pose[3];
	bool _lastPosesValid;


	// statistics parameters
	int numGoodIMUObservations;
	int numGoodPTAMObservations;

	// parameters used for adding / timing
	long lastIMU_XYZ_dronetime;
	long lastIMU_RPY_dronetime;
	long lastIMU_dronetime;
	int lastIMU_XYZ_ID;
	int lastIMU_RPY_ID;
	double lastPredictedRoll;
	double lastPredictedPitch;
	double initialScaleSet;

	// internal add functions
	void predictInternal(geometry_msgs::Twist activeControlInfo, int timeSpanMicros, bool useControlGains = true);
	void observeIMU_XYZ(const ardrone_autonomy::Navdata* nav);
	void observeIMU_RPY(const ardrone_autonomy::Navdata* nav);
	void observePTAM(double pose[6]);
	void observeSlam(double pose[6]);


	// internal sync functions. called on ptam-add.
	void sync_xyz(double x_global, double y_global, double z_global);
	void sync_rpy(double roll_global, double pitch_global, double yaw_global);

	// ms state of filter
	int predictedUpToTotal;
	long predictedUpToDroneTime;

	// logging
	double lastdYaw, lastdZ;
	double baselineZ_IMU;
	double baselineZ_Filter;
	double last_z_heightDiff;
	double baselineY_IMU;
	double baselineY_Filter;
	bool baselinesYValid;
	int timestampYawBaselineFrom;
	double lastVXGain;
	double lastVYGain;

	// estimation node
	EstimationNode* _estimateNode;

public:
	DroneKalmanFilter(EstimationNode* node);
	~DroneKalmanFilter(void);
	void releaseMemory();

	static int delayRPY;	// assumed 0 (fastest data).
	static int delayXYZ;	// assumed 70 (gets here 70ms later than rpy)
	static int delayVideo;	// assumed 120 (gets here 120ms later than rpy)
	static int delayControl;	// assumed 120 (gets here 120ms later than rpy)

	static const int base_delayXYZ;
	static const int base_delayVideo;
	static const int base_delayControl;

	std::deque<ardrone_autonomy::Navdata>* navdataQueue;	// new navdata messages
	std::deque<geometry_msgs::TwistStamped>* velQueue;		// new velocity messages
	static pthread_mutex_t s_mutexFilterState;


	int predictdUpToTimestamp;
	int scalePairsIn, scalePairsOut;

	// For scale estimation
	std::vector<ScaleEstimator>* scalePairs;
	tf::Vector3 _lastSlamObsForScale;
	int _lastAddSlamObsTS;
	bool _bStartAddImuObs;
	int _lastAddImuTS;
	tf::Vector3 _imuDiff;
	bool _scaleInitialed;
	bool _scaleStop;

	tf::Transform _firstCamPoseInGlobalUnit, _secondCamPoseInGlobalUnit;
	bool _bFirstCamPoseInGlobalUnit, _bSecondCamPoseInGlobalUnit;


	// resets everything to zero.
	void reset();


	// resets everything to do with PTAM to zero (call if tracking continues, but PTAM tracking is reset)
	void clearPTAM();

	// predicts up to a specified time in ms, using all available data.
	// if consume=false, does not delete anything from queues.
	void predictUpTo(int timestamp, bool consume = true, bool useControlGains = true);

	void setPing(unsigned int navPing, unsigned int vidPing);

	// gets current pose and variances (up to where predictUpTo has been called)
	void getCurrentPose(double currentPose[6]);
	monoslam_ros_test::filter_state getCurrentPoseSpeed();
	void getCurrentPoseSpeedAsVec(double currentPoseSpeedVec[10]);
	tf::Transform getCurrentPoseAsTf();
	void getCurrentPoseSpeedVariances(double currentPoseSpeedVariance[10]);
	void getCurrentPoseVariances(double currentPoseVariance[6]);
	void getCurrentOffsets(double currentOffsets[6]);
	void getCurrentScales(double currentScales[3]);
	void getCurrentScalesForLog(double currentScalesForLog[3]);
	float getScaleAccuracy();
	void setCurrentScales(double scales);

	// adds a PTAM observation. automatically predicts up to timestamp.
	// P_global = P_slam * _scaleIMUToSlam + _offsets;
	double _scaleGlobalToSlam;
	tf::Vector3 _offsets;
	tf::Vector3 getImuDiff(int startTS, int endTS);
	void updateScaleXYZ(double ptamDiff[3], double imuDiff[3], double OrgPtamPose[3]);
	void updateScale(tf::Vector3 slamDiff, tf::Vector3 imuDiff);

	// does not actually change the state of the filter.
	// makes a copy of it, flushes all queued navdata into it, then predicts up to timestamp.
	// relatively costly (!)

	// transforms a PTAM observation.
	// translates from front to center, scales and adds offsets.
	void transformPTAMObservation(double x,double y,double z, double yaw, double transformedObs[3]);
	void transformPTAMObservation(double obs[6], double transformedObs[6]);
	void backTransformPTAMObservation(double obs[10], double transformedObs[6]);
	tf::Transform scalePredictedCamPoseToSlamUnit(tf::Transform worldToCamTf);
	tf::Transform scaleCamPoseToGlobalUnit(tf::Transform camPoseFromSlam);
	inline int getNumGoodPTAMObservations() {return numGoodPTAMObservations;}

	// when scaling factors are updated, exacly one point stays the same.
	// if useScalingFixpoint, this point is the current PTAM pose, otherwise it is sclingFixpoint (which is a PTAM-coordinate(!))
	double scalingFixpoint[3];	// in PTAM's system (!)
	bool useScalingFixpoint;

	//
	void flushScalePairs();

	// locking
	bool allSyncLocked;
	bool useControl;
	bool useNavdata;
	bool usePTAM;

	// motion model parameters
	float c1;
	float c2;
	float c3;
	float c4;
	float c5;
	float c6;
	float c7;
	float c8;

	// new ROS interface functions
	void addPTAMObservation(double trans[6], int time);
	void addSlamObservation(double trans[6], int time);
	void addFakePTAMObservation(int time);
	monoslam_ros_test::filter_state getPoseAt(ros::Time t, bool useControlGains = true);
	void getPoseAtAsVec(int timestamp, bool useControlGains, double pose[10]);

};
#endif
