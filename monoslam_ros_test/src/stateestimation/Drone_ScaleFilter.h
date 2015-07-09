/*
 * Drone_ScaleFilter.h
 *
 *  Created on: Jun 25, 2014
 *      Author: rui
 */

#ifndef DRONE_SCALEFILTER_H_
#define DRONE_SCALEFILTER_H_
#include "opencv2/opencv.hpp"
#include <vector>
using namespace cv;

class ScaleFilter{
public:
	cv::Mat _f; // process function
	cv::Mat _F; // Jacobian of process function
	cv::Mat _state; // state vector including position velocity and scale
	cv::Mat _statePred;
	cv::Mat _P; // The covariance matrix
	cv::Mat _P_pred;
	cv::Mat _Q; // process noise
	cv::Mat _Rv; // Noise Covariance for vision measurements
	double _Ri; // Noise Covariance for imu measurements
	double ts_last_update_nav; // last update ts using navdata
	double ts_last_update_vision; // last update ts using vision
	double ts_last_update;

	std::deque<double> _state_ts_vec;
	std::deque<cv::Mat> _state_vec;
	std::deque<cv::Mat> _P_vec;

	std::deque<double> _navMeas_vec;

	cv::Mat _state_last_vision;
	cv::Mat _P_last_vision;

	ScaleFilter();
	~ScaleFilter();
	void predict(double delta_t);

	void updateVision(double curr_ts, double slamPos, double slamVel);

	void updateImu(double curr_ts, double navMeas);

	void predictImu(double curr_ts, double navMeas);

	//Plan B
	void updateImu_rollForward(double curr_ts, double navMeas);
	void rollBack(double curr_ts, double slamPos, double slamVel);

	void setInitScale(double scale);
	void setRv(cv::Mat& Rv);
	void setRi(double Ri);

	double getPos();
	double getVel();
	double getScale();
};


#endif /* DRONE_SCALEFILTER_H_ */
