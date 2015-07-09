#include "Drone_FilterHelper.h"

PVFilter::PVFilter(Mat_d& state, Mat_d& var)
{
	_state = state;
	_var = var;
}

PVFilter::PVFilter(double pose, double speed){
	_state.resize(2,1);
	_state(0,0) = pose;
	_state(1,0) = speed;
	_var.resize(2,2);
	_var.fill(0);
}

PVFilter::PVFilter(double pose){
	_state.resize(2,1);
	_var.resize(2,2);
	_state.fill(0);
	_var.fill(0);
	_state(0,0) = pose;
	_var(1,1) = 1e10;
}

PVFilter::PVFilter(){
	_state.resize(2,1);
	_var.resize(2,2);
	_state.fill(0);
	_var.fill(0);
}


void PVFilter::predict
	(double ms, double accelerationVar, double controlGains[2], double coVarFac, double speedVarFac)
{
	/* MATLAB:
	G = [1 ms/1000; 0 1];
	E = [((ms/1000)^2)/2; ms/1000]; % assume normal distributed constant ACCELERATION.
	state = G*state;
	var = G * var * G' + accelerationVarPerS*(E*E');
	*/
	ms /= 1000;
	Mat_d G;
	matEyes(2,G);
	G(0,1) = ms;

	Mat_d tmp;
	matAB(G,_state,tmp);
	_state(0,0) = tmp(0,0) + controlGains[0];
	_state(1,0) = tmp(1,0) + controlGains[1];


	Mat_d tmpVar, GT;
	matTrans(G,GT);
	matAB(G,_var,tmpVar);
	matAB(tmpVar,GT,_var);
	_var(0,0) += accelerationVar * 0.25 * ms*ms*ms*ms;
	_var(1,0) += coVarFac * accelerationVar * 0.5 * ms*ms*ms * 4;
	_var(0,1) += coVarFac * accelerationVar * 0.5 * ms*ms*ms * 4;
	_var(1,1) += speedVarFac * accelerationVar * 1 * ms*ms * 4 * 4;
}

void PVFilter::predict(double ms, double vars[3], double controlGains[2]){
	/* MATLAB:
	G = [1 ms/1000; 0 1];
	E = [((ms/1000)^2)/2; ms/1000]; % assume normal distributed constant ACCELERATION.
	state = G*state;
	var = G * var * G' + accelerationVarPerS*(E*E');
	*/
	ms /= 1000;
	Mat_d G;
	matEyes(2,G);
	G(0,1) = ms;

	Mat_d tmp;
	matAB(G,_state,tmp);
	_state(0,0) = tmp(0,0) + controlGains[0];
	_state(1,0) = tmp(1,0) + controlGains[1];


	Mat_d tmpVar, GT;
	matTrans(G,GT);
	matAB(G,_var,tmpVar);
	matAB(tmpVar,GT,_var);

	_var(0,0) += vars[0];
	_var(1,0) += vars[2];
	_var(0,1) += vars[2];
	_var(1,1) += vars[1];
}

void PVFilter::observePose(double obs, double obsVar){
	/* MATLAB:
	H = [1 0];
	K = (uncertainty * H') / ((H * uncertainty * H') + obsVar);
	state = state + K * (obs - H*state);
	var = (eye(2)-K*H) * var;
	*/
	Mat_d K;
	K.resize(2,1);
	K(0,0) = _var(0,0) / (obsVar + _var(0,0));
	K(1,0) = _var(1,0) / (obsVar + _var(0,0));
	_state(0,0) += K(0,0) * (obs - _state(0,0));
	_state(1,0) += K(1,0) * (obs - _state(0,0));

	Mat_d tmp;
	matEyes(2,tmp);
	tmp(0,0) -= K(0,0);
	tmp(1,0) -= K(1,0);
	Mat_d newVar;
	matAB(tmp,_var, newVar);
	_var = newVar;
}

void PVFilter::observeSpeed(double obs, double obsVar){
	/* MATLAB:
	H = [0 1];
	K = (uncertainty * H') / ((H * uncertainty * H') + obsVar);
	state = state + K * (obs - H*state);
	var = (eye(2)-K*H) * var;
	*/
	Mat_d K;
	K.resize(2,1);
	K(0,0) = _var(0,1) / (obsVar + _var(1,1));
	K(1,0) = _var(1,1) / (obsVar + _var(1,1));
	_state(0,0) += K(0,0) * (obs - _state(1,0));
	_state(1,0) += K(1,0) * (obs - _state(1,0));

	Mat_d tmp;
	matEyes(2,tmp);
	tmp(0,1) -= K(0,0);
	tmp(1,1) -= K(1,0);
	Mat_d newVar;
	matAB(tmp,_var, newVar);
	_var = newVar;
}

// transform degree-angle to satisfy min <= angle < sup
double angleFromTo(double angle, double min, double sup)
{
	while(angle < min) angle += 360;
	while(angle >=  sup) angle -= 360;
	return angle;
}
