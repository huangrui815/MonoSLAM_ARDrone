#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
class PIDController{
public:
	double pGain;
	double dGain;
	double iGain;
	double N;
	double d_out_last;
	double err_last;
	double sum_err;
	double ctrl;
	double sampleT;

	PIDController(double sampleT = 0.02);
	~PIDController();
	float computeCtrl(double err);
	float computeCtrl(double err, double vel);
	void setParameters(double kp, double ki, double kd, double N);
	void setSampleT(double& sampleT);
	void clear();
};
#endif
