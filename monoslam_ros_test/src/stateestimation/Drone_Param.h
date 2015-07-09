#ifndef _DRONE_PARAM_H_
#define _DRONE_PARAM_H_

// constants (variances assumed to be present)
const double varSpeedObservation_xy = 2*2;
const double varPoseObservation_xy = 0.2*0.2;
const double varAccelerationError_xy = 8*8;

const double varPoseObservation_z_PTAM = 0.08*0.08;
const double varPoseObservation_z_IMU = 0.25*0.25;
const double varPoseObservation_z_IMU_NO_PTAM = 0.1*0.1;
const double varAccelerationError_z = 1*1;

const double varPoseObservation_rp_PTAM = 3*3;
const double varPoseObservation_rp_IMU = 1*1;
const double varSpeedError_rp = 360*360 * 16;	// increased because prediction based on control command is damn inaccurate.

const double varSpeedObservation_yaw = 5*5;
const double varPoseObservation_yaw = 3*3;
const double varAccelerationError_yaw = 360*360;


// constants (some more parameters)
const double max_z_speed = 2.5;	// maximum height speed tolerated (in m/s, everything else is considered to be due to change in floor-height).
const double scaleUpdate_min_xyDist = 0.5*0.5*0.5*0.5;
const double scaleUpdate_min_zDist = 0.1*0.1;

// motion model parameters
const	float c1 = 0.58;
const	float c2 = 17.8;
const	float c3 = 10;
const	float c4 = 35;
const	float c5 = 10;
const	float c6 = 25;
const	float c7 = 1.4;
const	float c8 = 1.0;

#endif
