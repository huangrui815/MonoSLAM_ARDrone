#ifndef _DRONE_SCALEESTIMATOR_
#define _DRONE_SCALEESTIMATOR_


// IMU measurement = SLAM measurement * lambda
class ScaleEstimator
{
public:
	ScaleEstimator(double ptamDist[3], double imuDist[3]);

	double computeEstimator
	(double spp, double sii, double spi, double stdDevPTAM = 0.2, double stdDevIMU = 0.1);

	bool operator < (const ScaleEstimator& comp) const
	{
		return _singlePairEstimate < comp._singlePairEstimate;
	}

	double _ptamMeas[3];
	double _imuMeas[3];
	double _ptamNorm;
	double _imuNorm;
	double _singlePairEstimate;
	double _pp, _ii, _pi;
};
#endif
