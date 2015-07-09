#include "Drone_ScaleEstimator.h"
#include <math.h>

double ScaleEstimator::computeEstimator
	(double spp, double sii, double spi, double stdDevPTAM, double stdDevIMU)
	{
		double sII = stdDevPTAM * stdDevPTAM * sii;
		double sPP = stdDevIMU * stdDevIMU * spp;
		double sPI = stdDevIMU * stdDevPTAM * spi;

		double tmp = (sII-sPP)*(sII-sPP) + 4*sPI*sPI;
		if(tmp <= 0) tmp = 1e-5;	// numeric issues
		if(spi == 0) spi = 1e-5;
		return 0.5*((sII-sPP)+sqrt(tmp)) / (stdDevPTAM * stdDevPTAM * spi);
	}

ScaleEstimator::ScaleEstimator(double ptamDist[3], double imuDist[3])
	{
		_pp = _ii = _pi = 0;
		for (int i = 0; i < 3; i++){
			_ptamMeas[i] = ptamDist[i];
			_imuMeas[i] = imuDist[i];
			_pp += _ptamMeas[i]*_ptamMeas[i];
			_ii += _imuMeas[i]*_imuMeas[i];
			_pi += _imuMeas[i]*_ptamMeas[i];
		}
		_ptamNorm = sqrt(_pp);
		_imuNorm = sqrt(_ii);

		_singlePairEstimate = computeEstimator(_pp,_ii,_pi);
	}
