#include "Drone_Param.h"
#include "app/APP_EstimationNode.h"
#include "Drone_Filter.h"
#include "app/APP_MyApp.h"

// constants (assumed delays in ms).
// default ping values: nav=25, vid=50
int DroneKalmanFilter::delayRPY = 0;		// always zero
int DroneKalmanFilter::delayXYZ = 40;		// base_delayXYZ, but at most delayVideo
int DroneKalmanFilter::delayVideo = 75;		// base_delayVideo + delayVid - delayNav
int DroneKalmanFilter::delayControl = 100;	// base_delayControl + 2*delayNav

const int DroneKalmanFilter::base_delayXYZ = 40;		// t_xyz - t_rpy = base_delayXYZ
const int DroneKalmanFilter::base_delayVideo = 50;		// t_cam - t_rpy = base_delayVideo + delayVid - delayNav
const int DroneKalmanFilter::base_delayControl = 50;	// t_control + t_rpy - 2*delayNav

pthread_mutex_t DroneKalmanFilter::s_mutexFilterState = PTHREAD_MUTEX_INITIALIZER;

DroneKalmanFilter::DroneKalmanFilter(EstimationNode* node){
	scalePairs = new std::vector<ScaleEstimator>();
	navdataQueue = new std::deque<ardrone_autonomy::Navdata>();
	velQueue = new std::deque<geometry_msgs::TwistStamped>();

	useScalingFixpoint = false;
	_bFirstCamPoseInGlobalUnit = false;
	_bSecondCamPoseInGlobalUnit = false;

	_bStartAddImuObs = false;
	_lastAddSlamObsTS = 0;
	_lastAddImuTS = 0;
	_imuDiff.setZero();
	_scaleGlobalToSlam = 1;
	_scaleInitialed = false;
	_scaleStop = false;
	_offsets.setZero();

	_sumII = _sumPP = _sumPI = 0;
	_estimateNode = node;

	pthread_mutex_lock( &s_mutexFilterState );
	reset();
	pthread_mutex_unlock( &s_mutexFilterState );
}


DroneKalmanFilter::~DroneKalmanFilter(void)
{
	// dont delete nothing here, as this is also called for shallow copy.
}


void DroneKalmanFilter::releaseMemory()
{
	delete scalePairs;
	delete navdataQueue;
	delete velQueue;
}


void DroneKalmanFilter::setPing(unsigned int navPing, unsigned int vidPing)
{
	// add a constant of 20ms // 40ms to accound for delay due to ros.
	// very, very rough approximation.
	navPing += 20;
	vidPing += 40;

	int new_delayXYZ = base_delayXYZ;
	int new_delayVideo = base_delayVideo + vidPing/(int)2 - navPing/(int)2;
	int new_delayControl = base_delayControl + navPing;

	delayXYZ = std::min(500,std::max(40,std::min(new_delayVideo,new_delayXYZ)));
	delayVideo = std::min(500,std::max(40,new_delayVideo));
	delayControl = std::min(200,std::max(50,new_delayControl));

	std::cout << "new delasXYZ: " << delayXYZ << ", delayVideo: " << delayVideo << ", delayControl: " << delayControl << std::endl;
}


void DroneKalmanFilter::reset()
{
	// init filter with pose 0 (var 0) and speed 0 (var large).
	_xFilter = _yFilter = _zFilter = _yawFilter = PVFilter(0);
	_rollFilter = _pitchFilter = PFilter(0);
	lastIMU_XYZ_ID = lastIMU_RPY_ID = -1;
	predictedUpToDroneTime = 0;
	last_z_heightDiff = 0;
	scalePairsIn = scalePairsOut = 0;

	// set statistic parameters to zero
	numGoodIMUObservations = 0;

	// set last times to 0, indicating that there was no prev. package.
	lastIMU_XYZ_dronetime = lastIMU_RPY_dronetime = 0;
	lastIMU_dronetime = 0;

	// clear PTAM
	clearPTAM();

	// clear IMU-queus
	navdataQueue->clear();
	velQueue->clear();

	predictdUpToTimestamp = _estimateNode->getRelativeTime(ros::Time::now());
	predictedUpToTotal = -1;

	baselineZ_Filter = baselineZ_IMU = -999999;
	baselinesYValid = false;


	_estimateNode->publishCommand("u l EKF has been reset to zero.");
}


void DroneKalmanFilter::clearPTAM()
{
	// offsets and scales are not initialized.
	offsets_xyz_initialized = scale_xyz_initialized = false;
	xy_scale = z_scale = scale_from_xy = scale_from_z = 1;
	roll_offset = pitch_offset = yaw_offset = x_offset = y_offset = z_offset = 0;

	xyz_sum_IMUxIMU = 0.1;
	xyz_sum_PTAMxPTAM = 0.1;
	xyz_sum_PTAMxIMU = 0.1;
	scalePairs->clear();
	scalePairsIn = 1;
	scalePairsOut = 0;

	rp_offset_framesContributed = 0;

	// set statistic parameters to zero
	numGoodPTAMObservations = 0;

	// indicates that last PTAM frame was not valid
	_lastPosesValid = false;
}


// this function does the actual work, predicting one time step ahead.
void DroneKalmanFilter::predictInternal
(geometry_msgs::Twist activeControlInfo, int timeSpanMicros, bool useControlGains)
{
	if(timeSpanMicros <= 0) return;
	//std::cout << "predict internal\n";

	useControlGains = useControlGains && this->useControl;

	bool controlValid = !(activeControlInfo.linear.z > 1.01 || activeControlInfo.linear.z < -1.01 ||
			activeControlInfo.linear.x > 1.01 || activeControlInfo.linear.x < -1.01 ||
			activeControlInfo.linear.y > 1.01 || activeControlInfo.linear.y < -1.01 ||
			activeControlInfo.angular.z > 1.01 || activeControlInfo.angular.z < -1.01);


	double tsMillis = timeSpanMicros / 1000.0;	// in milliseconds
	double tsSeconds = tsMillis / 1000.0;	// in seconds


	// predict roll, pitch, yaw
	float rollControlGain = tsSeconds*c3*(c4 * max(-0.5, min(0.5, (double)activeControlInfo.linear.y)) - _rollFilter._state);
	float pitchControlGain = tsSeconds*c3*(c4 * max(-0.5, min(0.5, (double)activeControlInfo.linear.x)) - _pitchFilter._state);
	float yawSpeedControlGain = tsSeconds*c5*(c6 * activeControlInfo.angular.z - _yawFilter._state(1,0));	// at adaption to ros, this has to be reverted for some reason....



	double yawRad = _yawFilter._state(0,0) * 3.14159268 / 180;
	double rollRad = _rollFilter._state * 3.14159268 / 180;
	double pitchRad = _pitchFilter._state * 3.14159268 / 180;
	double forceX = cos(yawRad) * sin(rollRad) * cos(pitchRad) - sin(yawRad) * sin(pitchRad);
	double forceY = - sin(yawRad) * sin(rollRad) * cos(pitchRad) - cos(yawRad) * sin(pitchRad);


	double vx_gain = tsSeconds * c1 * (c2*forceX - _xFilter._state(1,0));
	double vy_gain = tsSeconds * c1 * (c2*forceY - _yFilter._state(1,0));
	double vz_gain = tsSeconds * c7 * (c8*activeControlInfo.linear.z*(activeControlInfo.linear.z < 0 ? 2 : 1) - _zFilter._state(1,0));

	lastVXGain = vx_gain;
	lastVYGain = vy_gain;
	lastPredictedRoll = _rollFilter._state;
	lastPredictedPitch = _pitchFilter._state;

	if(!useControlGains || !controlValid)
	{
		vx_gain = vy_gain = vz_gain = 0;
		rollControlGain = pitchControlGain = yawSpeedControlGain = 0;
	}

	//update roll pitch yaw filter
	_rollFilter.predict(tsMillis,varSpeedError_rp, rollControlGain);
	_pitchFilter.predict(tsMillis,varSpeedError_rp, pitchControlGain);

	_yawFilter._state(0,0) =  angleFromTo(_yawFilter._state(0,0),-180,180);

	double yawControlGain[2] = {tsSeconds*yawSpeedControlGain/2,yawSpeedControlGain};
	_yawFilter.predict(tsMillis,varAccelerationError_yaw,yawControlGain,1,5*5);
	_yawFilter._state(0,0) =  angleFromTo(_yawFilter._state(0,0),-180,180);
	//_estimateNode->dataVec.push_back(_yawFilter._state.data[0]);
	//_estimateNode->dataVec.push_back(_rollFilter._state);
	//cout << _rollFilter._state << endl;

	//update x y z filters
	double xControlGain[2] = {tsSeconds*vx_gain/2,vx_gain};
	_xFilter.predict(tsMillis,varAccelerationError_xy,xControlGain,0.0001);

	double yControlGain[2] = {tsSeconds*vy_gain/2,vy_gain};
	_yFilter.predict(tsMillis,varAccelerationError_xy,yControlGain,0.0001);

	double zVars[3] = {tsSeconds*tsSeconds*tsSeconds*tsSeconds, 9*tsSeconds,tsSeconds*tsSeconds*tsSeconds*3};
	double zControlGain[2] = {tsSeconds*vz_gain/2,vz_gain};
	_zFilter.predict(tsMillis, zVars, zControlGain);
}


void DroneKalmanFilter::observeIMU_XYZ(const ardrone_autonomy::Navdata* nav)
{
	//std::cout << "observeIMU_XYZ\n";

	// --------------- now: update  ---------------------------
	// transform to global CS, using current yaw
	double yawRad = _yawFilter._state[0] * 3.14159268 / 180;
	double vx_global = (sin(yawRad) * nav->vx + cos(yawRad) * nav->vy) / 1000.0;
	double vy_global = (cos(yawRad) * nav->vx - sin(yawRad) * nav->vy) / 1000.0;

	//ROS_INFO("vx_global: %f, vy_global: %f\n", vx_global, vy_global);
	// update x,y:
	// if PTAM isGood, assume "normal" accuracy. if not, assume very accurate speeds
	// to simulate "integrating up".
	double lastX = _xFilter._state.data[0];
	double lastY = _yFilter._state.data[0];
	if(_lastPosesValid)
	{
		_xFilter.observeSpeed(vx_global,varSpeedObservation_xy*50);
		_yFilter.observeSpeed(vy_global,varSpeedObservation_xy*50);
	}
	else
	{
		_xFilter.observeSpeed(vx_global,varSpeedObservation_xy);
		_yFilter.observeSpeed(vy_global,varSpeedObservation_xy);
	}

	if(abs(lastX-_xFilter._state.data[0]) > 0.2 && lastX != 0)
	{
		// this happens if there was no navdata for a long time -> EKF variances got big -> new update leads to large jump in pos.
		ROS_WARN("detected large x jump. removing. should not happen usually (only e.g. if no navdata for a long time, or agressive re-scaling)");
		_xFilter._state.data[0] = lastX;
	}
	if(abs(lastY-_yFilter._state.data[0]) > 0.2 && lastY != 0)
	{
		ROS_WARN("detected large y jump. removing. should not happen usually (only e.g. if no navdata for a long time, or agressive re-scaling)");
		_yFilter._state.data[0] = lastY;
	}

	// Update Z
	// height is a bit more complicated....
	// only update every 8 packages, or if changed.
	if(last_z_IMU != nav->altd || nav->header.seq - last_z_packageID > 8)
	{
		if(baselineZ_Filter < -100000)	// only for initialization.
		{
			baselineZ_IMU = nav->altd;
			baselineZ_Filter = _zFilter._state.data[0];
		}

		// Update
		if(_lastPosesValid)
		{

			double imuHeightDiff = (nav->altd - baselineZ_IMU )*0.001;	// TODO negative heights??
			double observedHeight = baselineZ_Filter + 0.5*(imuHeightDiff + last_z_heightDiff);
			last_z_heightDiff = imuHeightDiff;

			baselineZ_IMU = nav->altd;
			baselineZ_Filter = _zFilter._state.data[0];

			if((abs(imuHeightDiff) < 0.150 && abs(last_z_heightDiff) < 0.150))	// jumps of more than 150mm in 40ms are ignored
			{
				_zFilter.observePose(observedHeight,varPoseObservation_z_IMU);
				lastdZ = observedHeight;
			}
		}
		else
		{
			double imuHeightDiff = (nav->altd - baselineZ_IMU )*0.001;
			double observedHeight = baselineZ_Filter + imuHeightDiff;
//			std::cout << "observe Height: " << observedHeight << endl;

			if(abs(imuHeightDiff) < 0.110)	// jumps of more than 150mm in 40ms are ignored
			{
				_zFilter.observePose(observedHeight,varPoseObservation_z_IMU_NO_PTAM);
				lastdZ = observedHeight;
			}
			else	// there was a jump: dont observe anything, but set new baselines.
			{
				if(baselineZ_IMU == 0 || nav->altd == 0)
				{
					_zFilter.observePose(observedHeight,0);
					_zFilter.observeSpeed(0,0);
				}

				baselineZ_IMU = nav->altd;
				baselineZ_Filter = _zFilter._state.data[0];
			}
		}

		last_z_IMU = nav->altd;
		last_z_packageID = nav->header.seq;
	}

}


void DroneKalmanFilter::observeIMU_RPY(const ardrone_autonomy::Navdata* nav)
{
	//std::cout << "Observe IMU_RPY\n";
	_rollFilter.observe(nav->rotX,varPoseObservation_rp_IMU);
	_pitchFilter.observe(nav->rotY,varPoseObservation_rp_IMU);


	if(!baselinesYValid)	// only for initialization.
	{
		baselineY_IMU = nav->rotZ;
		baselineY_Filter = _yawFilter._state(0,0);
		baselinesYValid = true;
		timestampYawBaselineFrom = _estimateNode->getRelativeTime(nav->header.stamp);
	}

	double imuYawDiff = (nav->rotZ - baselineY_IMU );
	double observedYaw = baselineY_Filter + imuYawDiff;

	_yawFilter._state.data[0] =  angleFromTo(_yawFilter._state.data[0],-180,180);

	if(_yawFilter._state.data[0] < -90)
		observedYaw = angleFromTo(observedYaw,-360,0);
	else if(_yawFilter._state.data[0] > 90)
		observedYaw = angleFromTo(observedYaw,0,360);
	else
		observedYaw = angleFromTo(observedYaw,-180,180);


	if(_lastPosesValid)
	{

		baselineY_IMU = nav->rotZ;
		baselineY_Filter = _yawFilter._state.data[0];
		timestampYawBaselineFrom = _estimateNode->getRelativeTime(nav->header.stamp);


		if(abs(observedYaw - _yawFilter._state.data[0]) < 10)
		{
			_yawFilter.observePose(observedYaw,2*2);
			lastdYaw = observedYaw;
		}
	}
	else
		if(abs(observedYaw - _yawFilter._state.data[0]) < 10)
		{
			_yawFilter.observePose(observedYaw,1*1);
			lastdYaw = observedYaw;
		}
		else
		{
			baselineY_IMU = nav->rotZ;
			baselineY_Filter = _yawFilter._state.data[0];
			timestampYawBaselineFrom = _estimateNode->getRelativeTime(nav->header.stamp);
		}

	last_yaw_IMU = nav->rotZ;
	_yawFilter._state.data[0] =  angleFromTo(_yawFilter._state.data[0],-180,180);
	_estimateNode->dataVec.push_back(observedYaw);
}


void DroneKalmanFilter::observePTAM(double pose[6])
{

	// ----------------- observe xyz -----------------------------
	// sync!
	if(!allSyncLocked)
	{
		sync_xyz(pose[0], pose[1], pose[2]);
		sync_rpy(pose[3], pose[4], pose[5]);
	}

	for (int i = 0; i < 6; i++)
		last_slam_pose[i] = pose[i];

	double tempPose[6];
	transformPTAMObservation(pose[0], pose[1], pose[2], _yawFilter._state(0,0), tempPose);

	if(offsets_xyz_initialized)
	{
		_xFilter.observePose(tempPose[0],varPoseObservation_xy);
		_yFilter.observePose(tempPose[1],varPoseObservation_xy);
	}

	// observe z
	if(offsets_xyz_initialized)
	{
		_zFilter.observePose(tempPose[2], varPoseObservation_z_PTAM);
	}

	// observe!
	if(rp_offset_framesContributed > 1)
	{
		_rollFilter.observe(pose[3]+roll_offset,varPoseObservation_rp_PTAM);
		_pitchFilter.observe(pose[4]+pitch_offset,varPoseObservation_rp_PTAM);

		_yawFilter._state.data[0] =  angleFromTo(_yawFilter._state.data[0],-180,180);
		double yawPoseState = _yawFilter._state.data[0];

		double observedYaw = pose[5]+yaw_offset;

		if(yawPoseState < -90)
			observedYaw = angleFromTo(observedYaw,-360,0);
		else if(yawPoseState > 90)
			observedYaw = angleFromTo(observedYaw,0,360);
		else
			observedYaw = angleFromTo(observedYaw,-180,180);

		_yawFilter.observePose(observedYaw,varPoseObservation_yaw);
		_yawFilter._state.data[0] =  angleFromTo(yawPoseState,-180,180);
	}

	last_fused_pose[0] = _xFilter._state.data[0];
	last_fused_pose[1] = _yFilter._state.data[0];
	last_fused_pose[2] = _zFilter._state.data[0];
	_lastPosesValid = true;
}


void DroneKalmanFilter::observeSlam(double pose[6])
{

	if (!_scaleInitialed)
	{
		ROS_INFO("Scale has not been initialized\n");
		return;
	}
	// ----------------- observe xyz -----------------------------

	_xFilter.observePose(pose[0],varPoseObservation_xy);
	_yFilter.observePose(pose[1],varPoseObservation_xy);
	_zFilter.observePose(pose[2], varPoseObservation_z_PTAM);

	_rollFilter.observe(pose[3],varPoseObservation_rp_PTAM);
	_pitchFilter.observe(pose[4],varPoseObservation_rp_PTAM);

	_yawFilter._state.data[0] =  angleFromTo(_yawFilter._state.data[0],-180,180);
	double yawPoseState = _yawFilter._state.data[0];
	double observedYaw = pose[5];

	if(yawPoseState < -90)
		observedYaw = angleFromTo(observedYaw,-360,0);
	else if(yawPoseState > 90)
		observedYaw = angleFromTo(observedYaw,0,360);
	else
		observedYaw = angleFromTo(observedYaw,-180,180);

	_yawFilter.observePose(observedYaw,varPoseObservation_yaw);
	_yawFilter._state.data[0] =  angleFromTo(yawPoseState,-180,180);
}


void DroneKalmanFilter::sync_rpy(double roll_global, double pitch_global, double yaw_global)
{
	if(allSyncLocked) return;
	// set yaw on first call
	if(rp_offset_framesContributed < 1)
		yaw_offset = _yawFilter._state.data[0] - yaw_global;

	// update roll and pitch offset continuously as normal average.
	if(rp_offset_framesContributed < 100)
	{
		roll_offset = rp_offset_framesContributed * roll_offset + _rollFilter._state - roll_global;
		pitch_offset = rp_offset_framesContributed * pitch_offset + _pitchFilter._state - pitch_global;
	}
	rp_offset_framesContributed++;

	roll_offset /= rp_offset_framesContributed;
	pitch_offset /= rp_offset_framesContributed;
}

void DroneKalmanFilter::sync_xyz(double x_global, double y_global, double z_global)
{
	if(allSyncLocked) return;
	// ----------- offset: just take first available ---------
	if(!offsets_xyz_initialized)
	{
		x_offset = _xFilter._state.data[0] - x_global*xy_scale;
		y_offset = _yFilter._state.data[0] - y_global*xy_scale;
		z_offset = _zFilter._state.data[0] - z_global*z_scale;
		offsets_xyz_initialized = true;
	}
}

#include <fstream>
void DroneKalmanFilter::flushScalePairs()
{
	std::ofstream* fle = new std::ofstream();
	fle->open ("scalePairs.txt");
	for(unsigned int i=0;i<scalePairs->size();i++)
		(*fle) << (*scalePairs)[i]._ptamMeas[0] << " " <<(*scalePairs)[i]._ptamMeas[1] << " " <<(*scalePairs)[i]._ptamMeas[2] << " " <<
			(*scalePairs)[i]._imuMeas[0] << " " << (*scalePairs)[i]._imuMeas[1] << " " << (*scalePairs)[i]._imuMeas[2] << std::endl;
	fle->flush();
	fle->close();
	delete fle;
}

void DroneKalmanFilter::updateScaleXYZ(double ptamDiff[3], double imuDiff[3], double OrgPtamPose[3])
{
	if(allSyncLocked) return;

	ScaleEstimator newMeasPair = ScaleEstimator(ptamDiff, imuDiff);

	// dont add samples that are way to small...
	if(newMeasPair._imuNorm < 0.05 || newMeasPair._ptamNorm < 0.05) return;

	// update running sums
	(*scalePairs).push_back(newMeasPair);

	double xyz_scale_old = xy_scale;


	// find median.
	std::sort((*scalePairs).begin(), (*scalePairs).end());
	double median = (*scalePairs)[((*scalePairs).size()+1)/2]._singlePairEstimate;

	// hack: if we have only few samples, median is unreliable (maybe 2 out of 3 are completely wrong.
	// so take first scale pair in this case (i.e. the initial scale)
	if((*scalePairs).size() < 5)
		median = initialScaleSet;

	// find sums and median.
	// do separately for xy and z and xyz-all and xyz-filtered
	double sumII = 0;
	double sumPP = 0;
	double sumPI = 0;
	double totSumII = 0;
	double totSumPP = 0;
	double totSumPI = 0;

	double sumIIxy = 0;
	double sumPPxy = 0;
	double sumPIxy = 0;
	double sumIIz = 0;
	double sumPPz = 0;
	double sumPIz = 0;

	int numInliers = 0;
	int numOutliers = 0;
	for(unsigned int i=0;i<(*scalePairs).size();i++)
	{
		if((*scalePairs).size() < 5 || ((*scalePairs)[i]._singlePairEstimate > median * 0.2 && (*scalePairs)[i]._singlePairEstimate < median / 0.2))
		{
			sumII += (*scalePairs)[i]._ii;
			sumPP += (*scalePairs)[i]._pp;
			sumPI += (*scalePairs)[i]._pi;

			sumIIxy += pow((*scalePairs)[i]._imuMeas[0],2) + pow((*scalePairs)[i]._imuMeas[1],2);
			sumPPxy += pow((*scalePairs)[i]._ptamMeas[0],2) + pow((*scalePairs)[i]._ptamMeas[1],2);
			sumPIxy += (*scalePairs)[i]._ptamMeas[0]*(*scalePairs)[i]._imuMeas[0]
			            + (*scalePairs)[i]._ptamMeas[1]*(*scalePairs)[i]._imuMeas[1];

			sumIIz += pow((*scalePairs)[i]._imuMeas[2],2);
			sumPPz += pow((*scalePairs)[i]._ptamMeas[2],2);
			sumPIz += pow((*scalePairs)[i]._ptamMeas[2],2);

			numInliers++;
		}
		else
		{
			totSumII += (*scalePairs)[i]._ii;
			totSumPP += (*scalePairs)[i]._pp;
			totSumPI += (*scalePairs)[i]._pi;
			numOutliers++;
		}
	}
	xyz_sum_IMUxIMU = sumII;
	xyz_sum_PTAMxPTAM = sumPP;
	xyz_sum_PTAMxIMU = sumPI;

	double scale_Filtered = (*scalePairs)[0].computeEstimator(sumPP,sumII,sumPI,0.2,0.01);
	double scale_Unfiltered = (*scalePairs)[0].computeEstimator(sumPP+totSumPP,sumII+totSumII,sumPI+totSumPI,0.2,0.01);
	double scale_PTAMSmallVar = (*scalePairs)[0].computeEstimator(sumPP+totSumPP,sumII+totSumII,sumPI+totSumPI,0.00001,1);
	double scale_IMUSmallVar = (*scalePairs)[0].computeEstimator(sumPP+totSumPP,sumII+totSumII,sumPI+totSumPI,1,0.00001);


	double scale_Filtered_xy = (*scalePairs)[0].computeEstimator(sumPPxy,sumIIxy,sumPIxy,0.2,0.01);
	double scale_Filtered_z = (*scalePairs)[0].computeEstimator(sumPPz,sumIIz,sumPIz,0.2,0.01);


	scalePairsIn = numInliers;
	scalePairsOut = numOutliers;

	printf("scale: in: %i; out: %i, filt: %.3f; xyz: %.1f < %.1f < %.1f; xy: %.1f < %.1f < %.1f; z: %.1f < %.1f < %.1f;\n",
		numInliers, numOutliers, scale_Filtered,
		scale_PTAMSmallVar, scale_Unfiltered, scale_IMUSmallVar,
		(*scalePairs)[0].computeEstimator(sumPPxy,sumIIxy,sumPIxy,0.00001,1),
		scale_Filtered_xy,
		(*scalePairs)[0].computeEstimator(sumPPxy,sumIIxy,sumPIxy,1,0.00001),
		(*scalePairs)[0].computeEstimator(sumPPz,sumIIz,sumPIz,0.00001,1),
		scale_Filtered_z,
		(*scalePairs)[0].computeEstimator(sumPPz,sumIIz,sumPIz,1,0.00001)
		);


	if(scale_Filtered > 0.1)
		z_scale = xy_scale = scale_Filtered;
	else
		ROS_WARN("calculated scale is too small %.3f, disallowing!",scale_Filtered);


	scale_from_xy = scale_Filtered_xy;
	scale_from_z = scale_Filtered_z;
	// update offsets such that no position change occurs (X = x_global*xy_scale_old + offset = x_global*xy_scale_new + new_offset)
	if(useScalingFixpoint)
	{
		// fix at fixpoint
		x_offset += (xyz_scale_old - xy_scale)*scalingFixpoint[0];
		y_offset += (xyz_scale_old - xy_scale)*scalingFixpoint[1];
		z_offset += (xyz_scale_old - z_scale)*scalingFixpoint[2];
	}
	else
	{
		// fix at current pos.
		x_offset += (xyz_scale_old - xy_scale)*OrgPtamPose[0];
		y_offset += (xyz_scale_old - xy_scale)*OrgPtamPose[1];
		z_offset += (xyz_scale_old - z_scale)*OrgPtamPose[2];
	}
	scale_xyz_initialized = true;
}


void DroneKalmanFilter::updateScale
(tf::Vector3 ptamDiffVec, tf::Vector3 imuDiffVec)
{
	if(allSyncLocked){
		ROS_INFO("Scale estimation stopped\n");
		return;
	}

	double ptamDiff[3] = {ptamDiffVec.getX(), ptamDiffVec.getY(), ptamDiffVec.getZ()};
	double imuDiff[3] = {imuDiffVec.getX(), imuDiffVec.getY(), imuDiffVec.getZ()};
	ScaleEstimator newMeasPair = ScaleEstimator(ptamDiff, imuDiff);

	// do not add a sample if the movement is too small
	if(newMeasPair._imuNorm < 0.05){
		ROS_INFO("updateScale: translation measured by IMU is too small, return\n");
		return;
	}

	// update running sums
	(*scalePairs).push_back(newMeasPair);

	if (!_scaleInitialed)
		_scaleGlobalToSlam = newMeasPair._singlePairEstimate;


	double median = 0;

	if((*scalePairs).size() < 5){
		median = _scaleGlobalToSlam;
	}
	else
	{
		// find median with enough number of samples
		std::sort((*scalePairs).begin(), (*scalePairs).end());
		median = (*scalePairs)[((*scalePairs).size()+1)/2]._singlePairEstimate;
	}

	if(newMeasPair._singlePairEstimate > median * 0.2
			&& newMeasPair._singlePairEstimate < median / 0.2)
	{
		_sumII += newMeasPair._ii;
		_sumPP += newMeasPair._pp;
		_sumPI += newMeasPair._pi;
		_scaleGlobalToSlam = newMeasPair.computeEstimator(_sumPP, _sumII, _sumPI, 0.2, 0.01);
		ROS_INFO("updateScale: scale (global to slam) updated to %f\n", _scaleGlobalToSlam);
		if (!_scaleInitialed)
			_scaleInitialed = true;
	}
}

tf::Vector3 DroneKalmanFilter::getImuDiff(int startTS, int endTS){
	float firstZ = 0;
	float lastAddedTS = 0;

	tf::Vector3 imuDiff;
	imuDiff.setZero();

	pthread_mutex_lock( &MyApp::_mutexNavInfoQueue );
	for(std::deque<ardrone_autonomy::Navdata>::iterator cur = MyApp::_navInfoQueue.begin();
				cur != MyApp::_navInfoQueue.end(); cur++){
		int curTS = _estimateNode->getRelativeTime(cur->header.stamp);
		if (curTS >= startTS && curTS <= endTS){
			if (lastAddedTS == 0)
			{
				lastAddedTS = curTS;
				firstZ = cur->altd * 0.001;
			}
			double timespan =curTS - lastAddedTS;	// in miliseconds
			lastAddedTS = curTS;
			if(timespan > 50 || timespan < 0)
				timespan = std::max(0.0,std::min(50.0,timespan));	// clamp strange values

			double dxDrone = cur->vx * timespan / 1000000;	// in meters
			double dyDrone = cur->vy * timespan / 1000000;	// in meters

			double yawRad = cur->rotZ * RADIAN_PER_DEGREE;
			imuDiff.setX(imuDiff.getX() + sin(yawRad)*dxDrone+cos(yawRad)*dyDrone);
			imuDiff.setY(imuDiff.getY() + cos(yawRad)*dxDrone-sin(yawRad)*dyDrone);
			imuDiff.setZ(cur->altd * 0.001 - firstZ);
		}
	}
	pthread_mutex_unlock( &MyApp::_mutexNavInfoQueue );
	return imuDiff;
}

float DroneKalmanFilter::getScaleAccuracy()
{
	return 0.5 + 0.5*std::min(1.0,std::max(0.0,xyz_sum_PTAMxIMU * xy_scale/4));	// scale-corrected PTAM x IMU
}


void DroneKalmanFilter::predictUpTo(int timestampToBePredictedTo, bool toRemovePreviousData, bool toUseControlGains)
{
	//toRemovePreviousData = true;
	if(predictdUpToTimestamp == timestampToBePredictedTo) return;

	//std::cout << (toRemovePreviousData ? "per " : "tmp ") << " pred @ " << this << ": " << predictdUpToTimestamp << " to " << timestampToBePredictedTo << std::endl;

	// at this point:
	// - velQueue contains controls, timestamped with time at which they were sent.
	// - navQueue contains navdata, timestamped with time at which they were received.
	// - timestamp is the time up to which we want to predict, i.e. maybe a little bit into the feature

	// start at [predictdUpToTimestamp]. predict step-by-step observing at [currentTimestamp] the
	// - rpy timestamped with [currentTimestamp + delayRPY]
	// - xyz timestamped with [currentTimestamp + delayXYZ]
	// using
	// - control timestamped with [currentTimestamp - delayControl]

	// fast forward until first package that will be used.
	// for controlIterator, this is the last package with a stamp smaller/equal than what it should be.
	// for both others, this is the first package with a stamp bigger than what it should be.
	// if consume, delete everything before permanently.
	std::deque<geometry_msgs::TwistStamped>::iterator controlIterator = velQueue->begin();
	while(controlIterator != velQueue->end() &&
			controlIterator+1 != velQueue->end() &&
			_estimateNode->getRelativeTime((controlIterator+1)->header.stamp) + delayControl <= predictdUpToTimestamp)
		if(toRemovePreviousData)
		{
			velQueue->pop_front();
			controlIterator = velQueue->begin();
		}
		else
			controlIterator++;
	if(velQueue->size() == 0) toUseControlGains = false;

	// dont delete here, it will be deleted if respective rpy data is consumed.
	std::deque<ardrone_autonomy::Navdata>::iterator xyzIterator = navdataQueue->begin();
	while(xyzIterator != navdataQueue->end() &&
			_estimateNode->getRelativeTime(xyzIterator->header.stamp) - delayXYZ <= predictdUpToTimestamp){
		//std::cout << " xyzIterator - header: " << _estimateNode->getRelativeTime(xyzIterator->header.stamp) - delayXYZ
		//		 << " predictedUpToTimestamp: " << predictdUpToTimestamp << endl;
		xyzIterator++;
	}

	std::deque<ardrone_autonomy::Navdata>::iterator rpyIterator = navdataQueue->begin();
	while(rpyIterator != navdataQueue->end() &&
			(_estimateNode->getRelativeTime(rpyIterator->header.stamp) - delayRPY <= predictdUpToTimestamp))
		if(toRemovePreviousData)
		{
			navdataQueue->pop_front();
			rpyIterator = navdataQueue->begin();
		}
		else{
			rpyIterator++;
		}
	//std::cout << "xyzIterator == navdataQueue->end(): " << (xyzIterator == navdataQueue->end()) << endl;
	//std::cout << "rpyIterator == navdataQueue->end(): " << (rpyIterator == navdataQueue->end()) << endl;

	//printf("search forward to navdataTS %10f, rpyTS %10f\n", xyzIterator->header.stamp.toSec(), rpyIterator->header.stamp.toSec());

	// now, each iterator points to the first element in queue that is to be integrated.
	// start predicting,
	while(true)
	{
		// predict ahead to [timestampToBePredicted]
		int predictTo = timestampToBePredictedTo;
		//cout << "timestamp to be predicted " << predictTo;
		// but a maximum of 10ms per prediction step, to guarantee nonlinearities.
		predictTo = min(predictTo, predictdUpToTimestamp+10);
		//cout << " real timestamp to be predicted " << predictTo << endl;

		// get three queues to the right point in time by rolling forward in them.
		// for xyz this is the first point at which its obs-time is bigger than or equal to [predictdUpToTimestamp]
		while(xyzIterator != navdataQueue->end() &&
				_estimateNode->getRelativeTime(xyzIterator->header.stamp) - delayXYZ < predictdUpToTimestamp)
			xyzIterator++;
		while(rpyIterator != navdataQueue->end() &&
				_estimateNode->getRelativeTime(rpyIterator->header.stamp) - delayRPY < predictdUpToTimestamp)
			rpyIterator++;
		// for control that is last message with stamp <= predictdUpToTimestamp - delayControl.
		while(controlIterator != velQueue->end() &&
				controlIterator+1 != velQueue->end() &&
				_estimateNode->getRelativeTime((controlIterator+1)->header.stamp) + delayControl <= predictdUpToTimestamp)
			controlIterator++;

//		printf("search forward to navdataTS %u, rpyTS %u\n", _estimateNode->getRelativeTime(xyzIterator->header.stamp),
//				_estimateNode->getRelativeTime(rpyIterator->header.stamp));


		// predict not further than the point in time where the next observation needs to be added.
		if(rpyIterator != navdataQueue->end() )
			predictTo = min(predictTo, (int)_estimateNode->getRelativeTime(rpyIterator->header.stamp)- delayRPY);
		if(xyzIterator != navdataQueue->end() )
			predictTo = min(predictTo, (int)_estimateNode->getRelativeTime(xyzIterator->header.stamp)- delayXYZ);
		//cout << " predicted from " << predictdUpToTimestamp << " to " << predictTo << endl;


		// control max. 200ms old.
		predictInternal(toUseControlGains ? controlIterator->twist : geometry_msgs::Twist(),
				(predictTo - predictdUpToTimestamp)*1000,
				toUseControlGains &&
				_estimateNode->getRelativeTime(controlIterator->header.stamp) + delayControl + 200 > predictdUpToTimestamp);

		//cout << " " << (predictTo - predictdUpToTimestamp);
		//cout << " predicted from " << predictdUpToTimestamp << " to " << predictTo
		//     << " until " << timestampToBePredictedTo << " ";
		//printf("roll state: %f, pitch state: %f, yaw state: %f\n", _rollFilter._state, _pitchFilter._state, _yawFilter._state[0]);

		// if an observation needs to be added, it HAS to have a stamp equal to [predictTo],
		// as we just set [predictTo] to that timestamp.
		bool observedXYZ = false, observedRPY=false;
		if(rpyIterator != navdataQueue->end() && _estimateNode->getRelativeTime(rpyIterator->header.stamp)-delayRPY == predictTo)
		{
			if(this->useNavdata)
				observeIMU_RPY(&(*rpyIterator));

			observedRPY = true;
			//cout << "a\n";
		}

		if(xyzIterator != navdataQueue->end())
		//if(xyzIterator != navdataQueue->end() && _estimateNode->getRelativeTime(xyzIterator->header.stamp)-delayXYZ == predictTo)
		{
			if(this->useNavdata)
				observeIMU_XYZ(&(*xyzIterator));

			observedXYZ = true;
			//cout << "p\n";
		}


		predictdUpToTimestamp = predictTo;
//		cout <<" predicted up to time stamp changed to " << predictdUpToTimestamp << endl;
//		if(consume)
//		{
//			if(node->logfileFilter != NULL)
//			{
//				pthread_mutex_lock(&(node->logFilter_CS));
//				(*(node->logfileFilter)) << predictdUpToTimestamp << " " << 0 << " " << 0 << " " << 0 << " " <<
//					0 << " " << 0 << " " << 0 << " " <<
//					controlIterator->twist.linear.y << " " << controlIterator->twist.linear.x << " " << controlIterator->twist.linear.z << " " << controlIterator->twist.angular.z << " " <<
//					(observedRPY ? rpyIterator->rotX : -1) << " " << (observedRPY ? rpyIterator->rotY : -1) << " " << (observedRPY ? lastdYaw : -1) << " " <<
//					(observedXYZ ? xyzIterator->vx : -1) << " " << (observedXYZ ? xyzIterator->vy : -1) << " " << (observedXYZ ? lastdZ : -1) << " " <<
//					x.state[0] << " " << y.state[0] << " " << z.state[0] << " " << roll.state << " " << pitch.state << " " << yaw.state[0] << " " << x.state[1] << " " << y.state[1] << " " << z.state[1] << " " << yaw.state[1] << " " <<
//					lastVXGain << " " << lastVYGain << " " << "\n";
//				pthread_mutex_unlock(&(node->logFilter_CS));
//			}
//		}

		if(observedRPY) rpyIterator++;
		if(observedXYZ) xyzIterator++;


		// if this is where we wanna get, quit.
		if(predictTo == timestampToBePredictedTo)
			break;
	}
	//cout << endl;
}

void DroneKalmanFilter::transformPTAMObservation(double x,double y,double z, double yaw, double transObs[3])
{
	double yawRad = yaw * 3.14159268 / 180;
	transObs[0] = x_offset + xy_scale* x - 0.2*sin(yawRad);
	transObs[1] = y_offset + xy_scale* y - 0.2*cos(yawRad);
	transObs[2] = z_offset + z_scale* z;
}

void DroneKalmanFilter::transformPTAMObservation(double obs[6], double transObs[6])
{
	double tempObs[3];
	transformPTAMObservation(obs[0], obs[1], obs[2], obs[5], tempObs);

	transObs[0] = tempObs[0];
	transObs[1] = tempObs[1];
	transObs[2] = tempObs[2];
	transObs[3] = obs[3] + roll_offset;
	transObs[4] = obs[4] + pitch_offset;
	transObs[5] = obs[5] + yaw_offset;
}

void DroneKalmanFilter::backTransformPTAMObservation(double obs[10], double transObs[6])
{
//	ROS_INFO("offsets: %f, %f, %f, %f, %f, %f\n",
//			x_offset, y_offset, z_offset,
//			roll_offset, pitch_offset, yaw_offset);

	transObs[3] = obs[3] - roll_offset;
	transObs[4] = obs[4] - pitch_offset;
	transObs[5] = obs[5] - yaw_offset;

	double yawRad = obs[5] * 3.14159268 / 180;
	transObs[0] = (- x_offset + obs[0] + 0.2*sin(yawRad))/xy_scale;
	transObs[1] = (- y_offset + obs[1] + 0.2*cos(yawRad))/xy_scale;
	transObs[2] = (- z_offset + obs[2])/z_scale;
}

tf::Transform DroneKalmanFilter::scalePredictedCamPoseToSlamUnit(tf::Transform worldToCamTf){
	tf::Transform scaledPredictedCamPose;
	scaledPredictedCamPose = worldToCamTf;
	tf::Vector3 scaledOrigin = worldToCamTf.getOrigin() - _offsets;
	scaledOrigin /= _scaleGlobalToSlam;

	scaledPredictedCamPose.setOrigin(scaledOrigin);
	return scaledPredictedCamPose;
}

tf::Transform DroneKalmanFilter::scaleCamPoseToGlobalUnit(tf::Transform camPoseFromSlam){
	tf::Transform scaledCamPose;
	scaledCamPose = camPoseFromSlam;
	tf::Vector3 scaledOrigin = camPoseFromSlam.getOrigin();
	scaledOrigin *= _scaleGlobalToSlam;
	scaledOrigin = scaledOrigin + _offsets;

	scaledCamPose.setOrigin(scaledOrigin);
	return scaledCamPose;
}

void DroneKalmanFilter::getCurrentPose(double currentPose[6])
{
	currentPose[0] = _xFilter._state.data[0];
	currentPose[1] = _yFilter._state.data[0];
	currentPose[2] = _zFilter._state.data[0];
	currentPose[3] = _rollFilter._state;
	currentPose[4] = _pitchFilter._state;
	currentPose[5] = _yawFilter._state.data[0];
}


monoslam_ros_test::filter_state DroneKalmanFilter::getCurrentPoseSpeed()
{
	monoslam_ros_test::filter_state s;
	s.x = _xFilter._state.data[0];
	s.y = _yFilter._state.data[0];
	s.z = _zFilter._state.data[0];
	s.yaw = _yawFilter._state.data[0];
	s.dx = _xFilter._state.data[1];
	s.dy = _yFilter._state.data[1];
	s.dz = _zFilter._state.data[1];
	s.dyaw = _yawFilter._state.data[1];
	s.roll = _rollFilter._state;
	s.pitch = _pitchFilter._state;

	if(s.roll*s.roll < 0.001) s.roll = 0;
	if(s.pitch*s.pitch < 0.001) s.pitch = 0;
	if(s.yaw*s.yaw < 0.001) s.yaw = 0;
	if(s.dx*s.dx < 0.001) s.dx = 0;
	if(s.dy*s.dy < 0.001) s.dy = 0;
	if(s.dz*s.dz < 0.001) s.dz = 0;
	if(s.x*s.x < 0.001) s.x = 0;
	if(s.y*s.y < 0.001) s.y = 0;
	if(s.z*s.z < 0.001) s.z = 0;

	return s;
}

void DroneKalmanFilter::getCurrentPoseSpeedAsVec(double currentPoseSpeedVec[10])
{
	//cout << "_xFilter._state.data[0] " << _xFilter._state.data[0] << endl;
	currentPoseSpeedVec[0] = _xFilter._state.data[0];
	currentPoseSpeedVec[1] = _yFilter._state.data[0];
	currentPoseSpeedVec[2] = _zFilter._state.data[0];
	currentPoseSpeedVec[3] = _rollFilter._state;
	currentPoseSpeedVec[4] = _pitchFilter._state;
	currentPoseSpeedVec[5] = _yawFilter._state.data[0];
	currentPoseSpeedVec[6] = _xFilter._state.data[1];
	currentPoseSpeedVec[7] = _yFilter._state.data[1];
	currentPoseSpeedVec[8] = _zFilter._state.data[1];
	currentPoseSpeedVec[9] = _yawFilter._state.data[1];
}

tf::Transform DroneKalmanFilter::getCurrentPoseAsTf(){
	double x = _xFilter._state.data[0];
	double y = _yFilter._state.data[0];
	double z = _zFilter._state.data[0];
	double roll = _rollFilter._state;
	double pitch = _pitchFilter._state;
	double yaw = _yawFilter._state.data[0];

	tf::Transform dronePose;
	dronePose.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion rot;
	rot.setRPY(roll, pitch, yaw);
	dronePose.setRotation(rot);

	tf::Transform globalPose;
	globalPose = EstimationNode::_globalToDrone * dronePose * EstimationNode::_droneToGlobal;
	globalPose.setOrigin(tf::Vector3(x,y,z));
	return globalPose;
}

void DroneKalmanFilter::getCurrentPoseSpeedVariances(double currentPoseSpeedVariance[10])
{
	currentPoseSpeedVariance[0] = _xFilter._var(0,0);
	currentPoseSpeedVariance[1] = _yFilter._var(0,0);
	currentPoseSpeedVariance[2] = _zFilter._var(0,0);
	currentPoseSpeedVariance[3] = _rollFilter._var;
	currentPoseSpeedVariance[4] = _pitchFilter._var;
	currentPoseSpeedVariance[5] = _yawFilter._var(0,0);
	currentPoseSpeedVariance[6] = _xFilter._var(1,1);
	currentPoseSpeedVariance[7] = _yFilter._var(1,1);
	currentPoseSpeedVariance[8] = _zFilter._var(1,1);
	currentPoseSpeedVariance[9] = _yawFilter._var(1,1);
}

void DroneKalmanFilter::getCurrentPoseVariances(double currentPoseVariance[6])
{
	currentPoseVariance[0] = _xFilter._var(0,0);
	currentPoseVariance[1] = _yFilter._var(0,0);
	currentPoseVariance[2] = _zFilter._var(0,0);
	currentPoseVariance[3] = _rollFilter._var;
	currentPoseVariance[4] = _pitchFilter._var;
	currentPoseVariance[5] = _yawFilter._var(0,0);
}

void DroneKalmanFilter::getCurrentOffsets(double currentOffsets[6])
{
	if(offsets_xyz_initialized){
		currentOffsets[0] = x_offset;
		currentOffsets[1] = y_offset;
		currentOffsets[2] = z_offset;
	}
	if(rp_offset_framesContributed > 1){
		currentOffsets[3] = roll_offset;
		currentOffsets[4] = pitch_offset;
		currentOffsets[5] = yaw_offset;
}
}
void DroneKalmanFilter::getCurrentScales(double currentScales[3])
{
	//return TooN::makeVector(scale_xyz_initialized ? xy_scale : 1, scale_xyz_initialized ? xy_scale : 1, scale_xyz_initialized ? z_scale : 1);
}
void  DroneKalmanFilter::getCurrentScalesForLog(double currentScalesForLog[3])
{
	//return TooN::makeVector(scale_xyz_initialized ? scale_from_xy : 1, scale_xyz_initialized ? scale_from_z : 1, scale_xyz_initialized ? xy_scale : 1);
}

void DroneKalmanFilter::setCurrentScales(double scales)
{
	if(allSyncLocked) return;
	xy_scale = scales;
	z_scale = scales;
	scale_from_xy = scale_from_z = scales;

	xyz_sum_IMUxIMU = 0.2 * scales;
	xyz_sum_PTAMxPTAM = 0.2 / scales;
	xyz_sum_PTAMxIMU = 0.2;

	(*scalePairs).clear();

	double slamMeas[3] = {0.2 / sqrt(scales), 0.2 / sqrt(scales), 0.2 / sqrt(scales)};
	double imuMeas[3] = {0.2 * sqrt(scales), 0.2 * sqrt(scales), 0.2 * sqrt(scales)};
	(*scalePairs).push_back(ScaleEstimator(slamMeas, imuMeas));

	scale_xyz_initialized = true;
	offsets_xyz_initialized = false;

	initialScaleSet = scales;
}

void DroneKalmanFilter::addPTAMObservation(double trans[6], int time)
{
	if(time > predictdUpToTimestamp)
		predictUpTo(time, true,true);
	//printf("trans: %f, %f, %f, %f, %f, %f\n", trans[0], trans[1], trans[2], trans[3], trans[4], trans[5]);
	double filterState[10];
	//printf("trans: %f, %f, %f, %f, %f, %f\n", trans[0], trans[1], trans[2], trans[3], trans[4], trans[5]);

	observePTAM(trans);
	//getCurrentPoseSpeedAsVec(filterState);

	numGoodPTAMObservations++;
}

void DroneKalmanFilter::addSlamObservation(double pose[6], int time)
{
	if(time > predictdUpToTimestamp)
		predictUpTo(time, true,true);
	//printf("trans: %f, %f, %f, %f, %f, %f\n", trans[0], trans[1], trans[2], trans[3], trans[4], trans[5]);
	double filterState[10];
	getCurrentPoseSpeedAsVec(filterState);
	//printf("trans: %f, %f, %f, %f, %f, %f\n", trans[0], trans[1], trans[2], trans[3], trans[4], trans[5]);

//	ROS_INFO("Filter state (before add SLAM obs): %f, %f, %f, %f, %f, %f\n",
//					filterState[0], filterState[1], filterState[2], filterState[3], filterState[4], filterState[5]);

//	ROS_INFO("New Slam Observation: %f, %f, %f, %f, %f, %f\n",
//			pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

	for (int i = 0; i < 6; i++){
		MyApp::log_stateBeforeSlam.push_back(filterState[i]);
		MyApp::log_currSlamRes.push_back(pose[i]);
	}

	observeSlam(pose);

	getCurrentPoseSpeedAsVec(filterState);
//	ROS_INFO("Filter state (after add SLAM obs): %f, %f, %f, %f, %f, %f\n",
//				filterState[0], filterState[1], filterState[2], filterState[3], filterState[4], filterState[5]);

	for (int i = 0; i < 6; i++){
			MyApp::log_stateAfterSlam.push_back(filterState[i]);
	}
	numGoodPTAMObservations++;
	_lastPosesValid = true;
}

void DroneKalmanFilter::addFakePTAMObservation(int time)
{
	//printf("predict from predictdUpToTimestamp %d to time %d\n", predictdUpToTimestamp, time);
	if(time > predictdUpToTimestamp)
	{
		//std::cout << " predictdUpToTimestamp at " << predictdUpToTimestamp << endl;
		predictUpTo(time, true,true);
		//std::cout << " addFakePTAMObservation at " << time << endl;
	}

	_lastPosesValid = false;
}
monoslam_ros_test::filter_state DroneKalmanFilter::getPoseAt(ros::Time t, bool useControlGains)
{
	// make shallow copy
	DroneKalmanFilter scopy = DroneKalmanFilter(*this);
	//cout << scopy.predictdUpToTimestamp << endl;

	// predict using this copy
	//scopy.predictUpTo(_estimateNode->getRelativeTime(t),false, useControlGains);
	//std::cout << "navdataQueue size: " << navdataQueue->size() << endl;
	scopy.predictUpTo(_estimateNode->getRelativeTime(t),false, useControlGains);
	//std::cout << " get Pose at: " << _estimateNode->getRelativeTime(t) << endl;
	//cout << predictdUpToTimestamp << endl;

	// return value, and discard any changes made to scopy (deleting it)
	return scopy.getCurrentPoseSpeed();
	//return getCurrentPoseSpeed();
}

void DroneKalmanFilter::getPoseAtAsVec(int timestamp, bool useControlGains, double pose[10])
{
	// make shallow copy
	DroneKalmanFilter scopy = DroneKalmanFilter(*this);

	// predict using this copy
	scopy.predictUpTo(timestamp,false, useControlGains);
	//predictUpTo(timestamp,false, useControlGains);

	// return value, and discard any changes made to scopy (deleting it)
	getCurrentPoseSpeedAsVec(pose);
}
