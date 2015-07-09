/*
 * Drone_ScaleFilter.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: rui
 */
#include "Drone_ScaleFilter.h"
#include <stdio.h>

ScaleFilter::ScaleFilter(){
	_state = Mat::zeros(3,1,CV_64F);
		_statePred = Mat::zeros(3,1,CV_64F);
		_P = Mat::zeros(3,3,CV_64F);
		_P.at<double>(2,2) = 10;
		_Ri = 0.5;
		_Rv = Mat::ones(2,2,CV_64F) * 1000;
		ts_last_update_nav = 0;
		ts_last_update_vision = 0;
		ts_last_update = 0;

		_state.at<double>(2,0) = 2000;
}

ScaleFilter::~ScaleFilter(){

}

void ScaleFilter::predict(double delta_t){
	_f = Mat::eye(3,3,CV_64F);
	_F = Mat::zeros(3,3,CV_64F);
	_f.at<double>(0,1) = delta_t;
	_f.copyTo(_F);
	_Q = Mat::zeros(3,3,CV_64F);
	_Q.at<double>(0,0) = pow(delta_t,4) / 4;
	_Q.at<double>(0,1) = pow(delta_t,3) / 2;
	_Q.at<double>(1,0) = pow(delta_t,3) / 2;
	_Q.at<double>(1,1) = pow(delta_t,2);
	_Q = _Q * 100000;
	_statePred = _f * _state;
	_P_pred = _F * _P * _F.t() + _Q;
}

void ScaleFilter::rollBack(double curr_ts, double slamPos, double slamVel){
//	printf("state ts size: %d\n", _state_ts_vec.size());
	printf("curr_ts: %lf, ts_last_update_nav: %lf\n",
			curr_ts, ts_last_update_nav);

	int ii = 0;
	for (; ii < _state_ts_vec.size()-1; ii++){
		if (_state_ts_vec[ii] < curr_ts && _state_ts_vec[ii+1] >= curr_ts){
			double delta_t = curr_ts - _state_ts_vec[ii];

			_state_vec[ii].copyTo(_state);
			_P_vec[ii].copyTo(_P);
			updateVision(delta_t, slamPos,slamVel);
			ts_last_update_vision = curr_ts;
			ts_last_update_nav = curr_ts;
			break;
		}
		else{
			_state_ts_vec.pop_front();
			_state_vec.pop_front();
			_P_vec.pop_front();
			_navMeas_vec.pop_front();
			ii = 0;
		}
		printf("state ts size: %d\n", _state_ts_vec.size());
	}

	// update the rest states
	for (int jj = 0; jj < _state_ts_vec.size(); jj++){
		updateImu_rollForward(_state_ts_vec[jj], _navMeas_vec[jj]);
		_state.copyTo(_state_vec[jj]);
		_P.copyTo(_P_vec[jj]);
//		printf("state ts size: %d\n", _state_ts_vec.size());
	}
	printf("state ts size: %d\n", _state_ts_vec.size());
}

void ScaleFilter::updateVision(double curr_ts, double slamPos, double slamVel){
	double delta_t = curr_ts - ts_last_update;

	predict(delta_t);
	cv::Mat slamMeas = Mat::zeros(2,1,CV_64F);
	slamMeas.at<double>(0,0) = slamPos;
	slamMeas.at<double>(1,0) = slamVel;
	cv::Mat H_v = Mat::zeros(2,3,CV_64F);
	cv::Mat h_v = Mat::zeros(2,3,CV_64F);

	double pos_pred = _statePred.at<double>(0,0);
	double vel_pred = _statePred.at<double>(1,0);
	double scale_pred = _statePred.at<double>(2,0);

	H_v.at<double>(0,0) = 1 / scale_pred;
	H_v.at<double>(0,2) = -1 / pow(scale_pred,2) * pos_pred;
	H_v.at<double>(1,1) = 1 / scale_pred;
	H_v.at<double>(1,2) = -1 / pow(scale_pred,2) * vel_pred;
	h_v.at<double>(0,0) = 1 / scale_pred;
	h_v.at<double>(1,1) = 1 / scale_pred;

	cv::Mat K_v;
	K_v = H_v * _P_pred * H_v.t() + _Rv;
	K_v = _P_pred * H_v.t() * K_v.inv(DECOMP_LU);
	_state = _statePred + K_v * (slamMeas - h_v * _statePred);
	_P = (Mat::eye(K_v.rows, K_v.rows, CV_64F) - K_v * H_v) * _P_pred;

	ts_last_update = curr_ts;
	ts_last_update_vision = curr_ts;

	_state.copyTo(_state_last_vision);
	_P.copyTo(_P_last_vision);
}


//Plan A
//void ScaleFilter::updateImu(double delta_t, double navMeas){
//	predict(delta_t);
//	cv::Mat H_i = Mat::zeros(1,3,CV_64F);
//	H_i.at<double>(0,1) = 1;
//	cv::Mat K_i;
//	K_i = H_i * _P_pred * H_i.t() + _Ri;
//	K_i = _P_pred * H_i.t() * K_i.inv(DECOMP_LU);
//	_state = _statePred + K_i * (navMeas - H_i * _statePred);
//	_P = (Mat::eye(K_i.rows, K_i.rows, CV_64F) - K_i * H_i) * _P_pred;
//}

//Plan B
//void ScaleFilter::updateImu(double curr_ts, double navMeas){
//	double delta_t = curr_ts - ts_last_update_nav;
//	if (curr_ts < 0){
//		printf("curr_ts is negative: return\n");
//		return;
//	}
//
//	predict(delta_t);
//	cv::Mat H_i = Mat::zeros(1,3,CV_64F);
//	H_i.at<double>(0,1) = 1;
//	cv::Mat K_i;
//	K_i = H_i * _P_pred * H_i.t() + _Ri;
//	K_i = _P_pred * H_i.t() * K_i.inv(DECOMP_LU);
//	_state = _statePred + K_i * (navMeas - H_i * _statePred);
//	_P = (Mat::eye(K_i.rows, K_i.rows, CV_64F) - K_i * H_i) * _P_pred;
//
//	_state_vec.push_back(_state);
//	_state_ts_vec.push_back(curr_ts);
//	_P_vec.push_back(_P);
//	_navMeas_vec.push_back(navMeas);
//
//	ts_last_update_nav = curr_ts;
//}

//Plan C
void ScaleFilter::updateImu(double curr_ts, double navMeas){
	double delta_t = curr_ts - ts_last_update;

	predict(delta_t);

	cv::Mat H_i = Mat::zeros(1,3,CV_64F);
	H_i.at<double>(0,1) = 1;
	cv::Mat K_i;
	K_i = H_i * _P_pred * H_i.t() + _Ri;
	K_i = _P_pred * H_i.t() * K_i.inv(DECOMP_LU);
	_state = _statePred + K_i * (navMeas - H_i * _statePred);
	_P = (Mat::eye(K_i.rows, K_i.rows, CV_64F) - K_i * H_i) * _P_pred;

	ts_last_update_nav = curr_ts;
	ts_last_update = curr_ts;
}

void ScaleFilter::predictImu(double curr_ts, double navMeas){
	double delta_t = curr_ts - ts_last_update;

	_f = Mat::eye(3,3,CV_64F);
	_f.at<double>(0,1) = delta_t;
	_state.at<double>(1,0) = navMeas;

	_statePred = _f * _state;
	_statePred.copyTo(_state);

	ts_last_update_nav = curr_ts;
	ts_last_update = curr_ts;
}


void ScaleFilter::updateImu_rollForward(double curr_ts, double navMeas){
	double delta_t = curr_ts - ts_last_update_nav;
	if (curr_ts < 0){
		printf("curr_ts is negative: return\n");
		return;
	}

	predict(delta_t);
	cv::Mat H_i = Mat::zeros(1,3,CV_64F);
	H_i.at<double>(0,1) = 1;
	cv::Mat K_i;
	K_i = H_i * _P_pred * H_i.t() + _Ri;
	K_i = _P_pred * H_i.t() * K_i.inv(DECOMP_LU);
	_state = _statePred + K_i * (navMeas - H_i * _statePred);
	_P = (Mat::eye(K_i.rows, K_i.rows, CV_64F) - K_i * H_i) * _P_pred;

	ts_last_update_nav = curr_ts;
}

void ScaleFilter::setInitScale(double scale){
	_state.at<double>(2,0) = scale;
}

double ScaleFilter::getPos(){
	return _state.at<double>(0,0);
}

double ScaleFilter::getVel(){
	return _state.at<double>(1,0);
}

double ScaleFilter::getScale(){
	return _state.at<double>(2,0);
}




