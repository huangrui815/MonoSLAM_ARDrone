#include "PIDController.h"
#include <algorithm>
using namespace std;

PIDController::PIDController(double sampleT){
	pGain = 0.0;
	dGain = 0.0;
	iGain = 0.0;
	N = 0.0;
	d_out_last = 0.0;
	err_last = 0.0;
	sum_err = 0.0;
	ctrl = 0.0;
	this->sampleT = sampleT;
}

PIDController::~PIDController(){

}

void PIDController::setSampleT(double& sampleTParam){
	sampleT = sampleTParam;
}

void PIDController::clear(){
	d_out_last = 0.0;
	err_last = 0.0;
	sum_err = 0.0;
	ctrl = 0.0;
}

float PIDController::computeCtrl(double err)
{
	double p_output = err * pGain;

	sum_err += err * sampleT;
	double i_output= iGain * sum_err;

	double d_output = (d_out_last + dGain * N * (err - err_last)) / (1 + N * sampleT);

	d_out_last = d_output;
	err_last = err;
	return (float)max(-0.4, min(p_output + i_output + d_output, 0.4));
}

float PIDController::computeCtrl(double err, double vel)
{
	double p_output = err * pGain;

	sum_err += err * sampleT;
	double i_output= iGain * sum_err;

	double d_output = (d_out_last + dGain * N * (vel * sampleT)) / (1 + N * sampleT);

	d_out_last = d_output;
	err_last = err;
	return (float)max(-0.4, min(p_output + i_output + d_output, 0.4));
}

void PIDController::setParameters(double kp, double ki, double kd, double kN){
	pGain = kp;
	iGain = ki;
	dGain = kd;
	N = kN;
}
