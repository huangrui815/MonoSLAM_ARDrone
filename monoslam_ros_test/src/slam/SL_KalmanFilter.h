#pragma once
#include "math/SL_Matrix.h"
#include "SL_KalmanFilter.h"

class KalmanFilter;
typedef void (*PREDICT_FUNC)(double* X_pred, double*  P_pred, const double* X_old, const double* P_old, const KalmanFilter* filter, const void* param);
typedef void (*MEASURE_FUNC)(double* h, const double* X_pred, const KalmanFilter* filter, const void* param);
typedef void (*MEASURE_JACO_FUNC)(double* H, const double* X_pred, const KalmanFilter* filter, const void* param);
class KalmanFilter
{
public:
	int _dimState;	//dimension of the state variable
	int _dimMeas;	//dimension of the measurement variable

	double* _Q; //covariance of the dynamic noise
	double* _R; //covariance of the observation noise
	double* _X; //state 
	double* _P; //state covariance
	double* _Y; //residual
protected:
	/* prediction model*/
	PREDICT_FUNC _predict;
	/* observation model*/
	MEASURE_FUNC _getMeas;
	/* Jacobian of the observation model*/
	MEASURE_JACO_FUNC _getMeasJacobian;
public:
	KalmanFilter(void):_dimState(0), _dimMeas(0), 
		_Q(0),_R(0),_X(0),_P(0), _Y(0){}
	KalmanFilter(int dimState, int dimMeas):_dimState(0),_dimMeas(0),
		_Q(0),_R(0),_X(0),_P(0),_Y(0){
			allocate(dimState, dimMeas);
	}
	~KalmanFilter(void){
		clear();
	}
	void clear();
	void allocate(int dimState, int dimMeas);

	void setPredictFunc(PREDICT_FUNC predFunc){
		this->_predict = predFunc;
	}
	void setMeasureFunc(MEASURE_FUNC measFunc){
		this->_getMeas = measFunc;
	}
	void setMeasureJacoFunc(MEASURE_JACO_FUNC measJacoFunc){
		this->_getMeasJacobian = measJacoFunc;
	}
	void setVals(const double* Q, const double* R, const double* X, const double* P){
		memcpy(_Q, Q, sizeof(double)*_dimState*_dimState);
		memcpy(_R, R, sizeof(double)*_dimMeas * _dimMeas);
		memcpy(_X, X, sizeof(double)*_dimState);
		memcpy(_P, P, sizeof(double)*_dimState*_dimState);
	}
	void correct(double* X, double* P, 
		const double* Z, const double* X_pred, const double* P_pred, const void* param);

	void update(const double* Z, const double* R, const void* param = 0){
		memcpy(_R, R, sizeof(double)*_dimMeas * _dimMeas);
		Mat_d X_pred(_dimState, 1), P_pred(_dimState, _dimState);
		_predict(X_pred.data, P_pred.data, _X, _P, this, param);
		correct(_X, _P, Z, X_pred.data, P_pred.data, param);
	}
};

