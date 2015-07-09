#include "SL_KalmanFilter.h"
#include "math/SL_LinAlg.h"
#include "math/SL_LinAlgWarper.h"
#include "tools/SL_Print.h"

void KalmanFilter::clear(){
	if( _Q)
		delete[] _Q;
	if( _R)
		delete[] _R;
	if( _X)
		delete[] _X;
	if( _P)
		delete[] _P;
	if( _Y)
		delete[] _Y;

	_Q = 0;
	_R = 0;
	_X = 0;
	_P = 0;
	_Y = 0;

	_dimState = 0;
	_dimMeas = 0;
}
void KalmanFilter::allocate(int dimState, int dimMeas){
	clear();
	assert(dimState > 0 && dimMeas > 0);
	_dimState = dimState;
	_dimMeas = dimMeas;
	
	_Q = new double [dimState*dimState];
	_R = new double [dimMeas * dimMeas];
	_X = new double [dimState];
	_P = new double [dimState * dimState];
	_Y = new double [dimMeas];
}

void KalmanFilter::correct(double* X, double* P, 
	const double* Z, const double* X_pred, const double* P_pred, const void* param){

	Mat_d h(_dimMeas, 1), H(_dimMeas, _dimState);

	_getMeas(h.data, X_pred, this, param);
	_getMeasJacobian(H.data, X_pred, this, param);

	//compute residual
	matSub(_dimMeas, 1, Z, h.data, _Y);

	//compute residual covariance(innovation)
	Mat_d Ht(_dimState,_dimMeas), PHt(_dimState, _dimMeas);
	matTrans(H, Ht);
	matAB(_dimState, _dimState, Ht.m, Ht.n, P, Ht, PHt);

	Mat_d HPHt(_dimMeas, _dimMeas), S(_dimMeas, _dimMeas);
	matAB(H, PHt, HPHt);
	matSum(_dimMeas, _dimMeas, HPHt.data, _R, S.data);

	//Kalman gain
	Mat_d SInv(H.m, H.m);
	matInv(S.m, S.data, SInv.data);

	Mat_d K(H.m, H.m);
	Mat_d HtSInv;
	matAB(Ht, SInv, HtSInv);
	matAB(_dimState, _dimState, HtSInv.m, HtSInv.n, P_pred, HtSInv, K);

	//update state estimate
	Mat_d Ky(K.m, 1);
	matAB(K.m, K.n, _dimMeas, 1, K, _Y, Ky);
	matSum(_dimState, 1, X_pred, Ky.data, X);

	//update estimate covariance
	Mat_d HP(_dimMeas, _dimState), KHP(_dimMeas, _dimState);
	matAB(H.m, H.n, _dimState, _dimState, H.data, P_pred, HP);
	matAB(K, HP, KHP);

	matSub(_dimState, _dimState, P_pred, KHP.data, P);
}
