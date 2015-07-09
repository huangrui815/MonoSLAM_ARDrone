#ifndef _DRONE_FILTER_
#define _DRONE_FILTER_
#include "ros/ros.h"
#include "math/SL_LinAlgWarper.h"

// KalmanFilter with two components (pose, speed)
class PVFilter
{
public:
	Mat_d _state; // 2 x 1 state
	Mat_d _var;   // 2 x 2 covariance matrix

	// constructors
	PVFilter(Mat_d& state, Mat_d& var);

	PVFilter(double pose, double speed);

	PVFilter(double pose);

	PVFilter();

	// observe
	void observePose(double obs, double obsVar);

	void observeSpeed(double obs, double obsVar);

	// predict
	// calculates prediction variance matrix based on Gaussian acceleration as error.
	// default control gain is {0,0}
	void predict
	(double ms, double accelerationVar, double controlGains[2], double coVarFac = 1, double speedVarFac = 1);

	// predict
	// calculates prediction using the given uncertainty matrix
	// vars is var(0) var(1) covar(0,1)
	// default control gain is {0,0}
	void predict
	(double ms, double vars[3], double controlGains[2]);
};

// KalmanFilter with only one component (pose, is observed directly)
class PFilter{
public:
	double _state;
	double _var;

	PFilter() : _state(0), _var(1e10) {};
	PFilter(double initState) : _state(initState), _var(0) {};

	inline void predict(double ms, double speedVar, double controlGains = 0)
	{
		/* MATLAB:
		state = state;
		var = var + speedVar*((ms/1000)^2);
		*/
		_state += controlGains;
		_var += speedVar * ms * ms / 1000000;
	}

	inline void observe(double obs, double obsVar)
	{
		/* MATLAB:
		obs_w = var / (var + obsVar);
		state = state * (1-obs_w) + obs * obs_w;
		var = var*obsVar / (var + obsVar);
		*/
		double w = _var / (_var + obsVar);
		_state = (1-w) * _state + w * obs;
		_var = _var * obsVar / (_var + obsVar);
	}
};

// Helper functions
double angleFromTo(double angle, double min, double sup);
#endif
