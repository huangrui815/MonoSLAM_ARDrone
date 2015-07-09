#include "SL_PBA.h"
#include "SL_MoSLAM.h"
#include "APP_SynObj.h"

PBABundler::PBABundler(){
	pthread_mutex_init(&mutexBundler, 0);
	pthread_cond_init(&condBundler, 0);
	running = false;
	toQuit = false;
	busy = false;
	bundleThreadId = 0;
	//Initialize the PBA
	ParallelBA::DeviceT device = ParallelBA::PBA_CPU_FLOAT;
	_pba = new ParallelBA(device);
	_pba->SetFixedIntrinsics(true);

	start();
}

PBABundler::~PBABundler(){
	toQuit = true;
	end();
	delete _pba;
}

void PBABundler::start() {
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&bundleThreadId, &attr, bundlerProc, this);
}

void PBABundler::loop() {
	running = true;
	while (!toQuit) {
		pthread_mutex_lock(&mutexBundler);
		if (_kfReqVec.empty())
			pthread_cond_wait(&condBundler, &mutexBundler);
		pthread_mutex_unlock(&mutexBundler);
		busy = true;
		vector<KeyFrame*>& kfVec = _kfReqVec.front();
		enterBACriticalSection();
		setKeyframeVec(kfVec);
		run();
		update();
		_kfReqVec.pop_front();
		leaveBACriticalSection();
		busy = false;
	}
	running = false;
}

void PBABundler::end() {
	if (!running) {
		cout << running << endl;
		toQuit = true;
		pthread_mutex_lock(&mutexBundler);
		pthread_cond_signal(&condBundler);
		pthread_mutex_unlock(&mutexBundler);
		cout << "waiting the bundler thread to quit..." << endl;
		pthread_join(bundleThreadId, 0);
		cout << "bundler thread quit!" << endl;
	}
//	//release the unhanlded requests;
//	for (size_t i = 0; i < req.size(); i++) {
//		delete req[i];
//	}
}

void* PBABundler::bundlerProc(void* param) {
	PBABundler* bundler = (PBABundler*) param;
	bundler->loop();
	return 0;
}

void PBABundler::setSLAM(MoSLAM* slam){
	_slam = slam;
}

bool PBABundler::requestRecent(int nRecentFrm){
	if ( _slam->keyFrm.size() < nRecentFrm)
	{
		printf("Keyframe size < %d\n", nRecentFrm);
		return false;
	}

	int num = 0;
	vector<KeyFrame*> kfVec;
	for (KeyFrame* kf = _slam->keyFrm.current();
				kf && num < nRecentFrm; kf = kf->prev) {
		kfVec.push_back(kf);
		num++;
	}

	pthread_mutex_lock(&mutexBundler);
	_kfReqVec.push_back(kfVec);
	pthread_cond_signal(&condBundler);
	pthread_mutex_unlock(&mutexBundler);
	return true;
}

bool PBABundler::setKeyframeVec(vector<KeyFrame*>& kfVec){
	int num = 0;
	map<MapPoint*, vector<FeaturePoint*> > mpt2fps;
	map<MapPoint*, vector<int> > mpt2viewID;
	double* K = _slam->tracker.K.data;

	for (int i = 0; i < kfVec.size(); i++) {
		KeyFrame* kf = kfVec[i];

		_kfVec.push_back(kf);
		// Add camera pose
		CameraT cam;
		for (int i = 0; i < 3; ++i){
			for (int j = 0; j < 3; ++j){
				cam.m[i][j] = (float)kf->cam->R[i * 3 + j];
			}
		}
		cam.t[0] = (float) kf->cam->t[0];
		cam.t[1] = (float) kf->cam->t[1];
		cam.t[2] = (float) kf->cam->t[2];

		cam.SetFocalLength((float)K[0]);

		// Keep the first camera constant
		if (kf == _slam->keyFrm.first())
			cam.SetConstantCamera();

		_camera_data.push_back(cam);

		for (size_t k = 0; k < kf->featPts.size(); k++) {
			FeaturePoint* fp = kf->featPts[k];
			if (fp->mpt && !fp->mpt->isFalse()) {
				mpt2fps[fp->mpt].push_back(fp);
				mpt2viewID[fp->mpt].push_back(num);
			}
		}

		num++;
	}
	cout << "camera data: " << _camera_data.size() << endl;

	int mptID = 0;
	for (map<MapPoint*, vector<FeaturePoint*> >::iterator iter = mpt2fps.begin();
			iter != mpt2fps.end(); iter++){
		_mptVec.push_back(iter->first);
		// Add 3D point
		Point3D pt3d;
		pt3d.xyz[0] = (float)iter->first->x;
		pt3d.xyz[1] = (float)iter->first->y;
		pt3d.xyz[2] = (float)iter->first->z;
		_point_data.push_back(pt3d);

		// Add 2D measurements
		for (size_t jj = 0; jj < mpt2fps[iter->first].size(); jj++){
			_camidx.push_back(mpt2viewID[iter->first][jj]);
			_ptidx.push_back(mptID);

			Point2D meas(mpt2fps[iter->first][jj]->x - K[2],
					mpt2fps[iter->first][jj]->y - K[5]);
			_measurements.push_back(meas);
		}
		mptID++;
	}

	_pba->SetCameraData(_camera_data.size(),  &_camera_data[0]);                        //set camera parameters
	_pba->SetPointData(_point_data.size(), &_point_data[0]);                            //set 3D point data
	_pba->SetProjection(_measurements.size(), &_measurements[0], &_ptidx[0], &_camidx[0]);//set the projections
	return true;
}

bool PBABundler::setRecentKeyFrms(int nRecentFrm){
	if ( _slam->keyFrm.size() < nRecentFrm)
	{
		printf("Keyframe size < %d\n", nRecentFrm);
		return false;
	}

	int num = 0;
	map<MapPoint*, vector<FeaturePoint*> > mpt2fps;
	map<MapPoint*, vector<int> > mpt2viewID;
	double* K = _slam->tracker.K.data;

	for (KeyFrame* kf = _slam->keyFrm.current();
			kf && num < nRecentFrm; kf = kf->prev) {

		_kfVec.push_back(kf);
		// Add camera pose
		CameraT cam;
		for (int i = 0; i < 3; ++i){
			for (int j = 0; j < 3; ++j){
				cam.m[i][j] = (float)kf->cam->R[i * 3 + j];
			}
		}
		cam.t[0] = (float) kf->cam->t[0];
		cam.t[1] = (float) kf->cam->t[1];
		cam.t[2] = (float) kf->cam->t[2];

		cam.SetFocalLength((float)K[0]);
		_camera_data.push_back(cam);

		for (size_t k = 0; k < kf->featPts.size(); k++) {
			FeaturePoint* fp = kf->featPts[k];
			if (fp->mpt && !fp->mpt->isFalse()) {
				mpt2fps[fp->mpt].push_back(fp);
				mpt2viewID[fp->mpt].push_back(num);
			}
		}

		num++;
	}
	cout << "camera data: " << _camera_data.size() << endl;

	int mptID = 0;
	for (map<MapPoint*, vector<FeaturePoint*> >::iterator iter = mpt2fps.begin();
			iter != mpt2fps.end(); iter++){
		_mptVec.push_back(iter->first);
		// Add 3D point
		Point3D pt3d;
		pt3d.xyz[0] = (float)iter->first->x;
		pt3d.xyz[1] = (float)iter->first->y;
		pt3d.xyz[2] = (float)iter->first->z;
		_point_data.push_back(pt3d);

		// Add 2D measurements
		for (size_t jj = 0; jj < mpt2fps[iter->first].size(); jj++){
			_camidx.push_back(mpt2viewID[iter->first][jj]);
			_ptidx.push_back(mptID);

			Point2D meas(mpt2fps[iter->first][jj]->x - K[2],
					mpt2fps[iter->first][jj]->y - K[5]);
			_measurements.push_back(meas);
		}
		mptID++;
	}

    _pba->SetCameraData(_camera_data.size(),  &_camera_data[0]);                        //set camera parameters
    _pba->SetPointData(_point_data.size(), &_point_data[0]);                            //set 3D point data
    _pba->SetProjection(_measurements.size(), &_measurements[0], &_ptidx[0], &_camidx[0]);//set the projections
    return true;
}

void PBABundler::run(){
	_pba->RunBundleAdjustment();    //run bundle adjustment, and camera_data/point_data will be modified
}

void PBABundler::update(){
	printf("Update camera data: %d\n", _camera_data.size());
	for (int viewId = 0; viewId < _camera_data.size(); viewId++){
		for (int ii = 0; ii < 3; ii++){
			for (int jj = 0; jj < 3; jj++){
				_kfVec[viewId]->cam->R[3 * ii + jj] = _camera_data[viewId].m[ii][jj];
			}
			_kfVec[viewId]->cam->t[ii] = _camera_data[viewId].t[ii];
		}
	}
	printf("Update map points data: %d\n", _point_data.size());
	for (int ptsId = 0; ptsId < _point_data.size(); ptsId++){
		_mptVec[ptsId]->x = _point_data[ptsId].xyz[0];
		_mptVec[ptsId]->y = _point_data[ptsId].xyz[1];
		_mptVec[ptsId]->z = _point_data[ptsId].xyz[2];
	}

	reset();
}

void PBABundler::reset(){
	_kfVec.clear();
	_mptVec.clear();
	_camera_data.clear();
	_point_data.clear();
	_measurements.clear();
	_camidx.clear();
	_ptidx.clear();
}
