#include "stateestimation/Drone_Filter.h"
#include "APP_EstimationNode.h"
#include "ros/package.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "APP_MyApp.h"
#include "monoslam_ros_test/filter_state.h"
#include "std_msgs/String.h"
#include "stateestimation/Drone_ScaleFilter.h"

using namespace std;

tf::Transform EstimationNode::_camToDroneInGlobal;
tf::Transform EstimationNode::_droneToCamInGlobal;
tf::Transform EstimationNode::_droneToGlobal;
tf::Transform EstimationNode::_globalToDrone;
tf::Transform EstimationNode::_camToGlobalRotOnly;
tf::Transform EstimationNode::_globalToCamRotOnly;
tf::Transform EstimationNode::_camToGlobal;
tf::Transform EstimationNode::_globalToCam;

EstimationNode::EstimationNode(int droneId)
{
	// decide topic namespace by drone's ID
	std::string droneTopicPrefixStr;
	if (droneId == 1)
		droneTopicPrefixStr = "ardrone";
	else
	{
		std::stringstream ss;
		ss << droneId;
		std::string droneIdStr = ss.str();
		droneTopicPrefixStr = "ardrone" + droneIdStr;
	}

	// Resolve topics' names
    _navdataChannel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/navdata");
    _imuChannel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/imu");
    _controlChannel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/cmd_vel");
    _predictedPoseChannel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/predictedPose");
    _videoChannel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/image_raw");
    _commandChannel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/com");
    _tmBaseChannel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/tmBase");
    _exitChannel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/exit");
    _scaleChannel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/scaleInit");

    takeoff_channel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/takeoff");
    land_channel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/land");
    toggleState_channel = _estimateNodeHandler.resolveName(droneTopicPrefixStr + "/reset");

    _packagePath = ros::package::getPath("monoslam_ros_test");

	std::string val;
	float valFloat = 0;

	_predTime = ros::Duration(25*0.001);

	ros::param::get("~publishFreq", val);
	if(val.size()>0)
		sscanf(val.c_str(), "%f", &valFloat);
	else
		valFloat = 25;
	_publishFreq = valFloat;
	cout << "set publishFreq to " << valFloat << "Hz"<< endl;

	_navdataSub       = _estimateNodeHandler.subscribe(_navdataChannel, 10, &EstimationNode::navdataCb, this);
	_imuSub           = _estimateNodeHandler.subscribe(_imuChannel, 1, &EstimationNode::imuCb, this);
	_ctrlSub          = _estimateNodeHandler.subscribe(_controlChannel,10, &EstimationNode::ctrlCb, this);
	_ctrlPub	   = _estimateNodeHandler.advertise<geometry_msgs::Twist>(_controlChannel,10);
	takeoff_pub	   = _estimateNodeHandler.advertise<std_msgs::Empty>(takeoff_channel,1);
	land_pub	   = _estimateNodeHandler.advertise<std_msgs::Empty>(land_channel,1);
	toggleState_pub	   = _estimateNodeHandler.advertise<std_msgs::Empty>(toggleState_channel,1);

	_videoSub          = _estimateNodeHandler.subscribe(_videoChannel,10, &EstimationNode::videoCb, this);
	_predictedPosePub	   = _estimateNodeHandler.advertise<monoslam_ros_test::filter_state>(_predictedPoseChannel,1);


	_comPub	   = _estimateNodeHandler.advertise<std_msgs::String>(_commandChannel,50);
	_comSub	   = _estimateNodeHandler.subscribe(_commandChannel,50, &EstimationNode::comCb, this);

	_tmBasePub = _estimateNodeHandler.advertise<std_msgs::Int32>(_tmBaseChannel,1, true);

	_exitPub = _estimateNodeHandler.advertise<std_msgs::Empty>(_exitChannel,1);
	_scaleSub = _estimateNodeHandler.subscribe(_scaleChannel,1, &EstimationNode::scaleInitCb, this);


	_estimateThreadId = 0;
	_ardroneVersion = 0;
	_timeBase = 0; // Initialize before create new drone filter
	_recording = false; // Enable the recording of video and imu data
	_droneCamReader = new DroneCamReader();
	_droneCamReader->setlogFilePath("/home/rui/Data/SFM_data/ARDrone/ts.txt",
			"/home/rui/Data/SFM_data/ARDrone/video.avi");
	_droneFilter = new DroneKalmanFilter(this);
	_scaleFilter = new ScaleFilter[3];

	//Init transformations between CS
	_body2cam_rot.setValue(0, 0, 1, 1, 0, 0, 0, 1, 0);
	double sampleT = 1.0 / _publishFreq;
	_posController.setSampleT(sampleT);
//	_pathPlanner.preloadPath("/home/rui/workspace/Vicon for Test PCTx/BsplineRefgen/ardrone_01_diag.txt");
	initCSTransforms();

	_navdata_first_set = false;
	_busyUpdateVision = false;
	y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
	y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;
}

EstimationNode::~EstimationNode(){

//
//	std::vector<double>::iterator dataIter;
//	cout << dataVec.size() << endl;
//	pFile = fopen("/home/rui/Record/data.txt","w");
//	if (!pFile)
//		cout << "Cannot open file state.txt\n";
//	for (dataIter = dataVec.begin(); dataIter != dataVec.end(); dataIter++){
//		fprintf(pFile, "%f\n", *dataIter);
//	}
//	fclose(pFile);
	logData(MyApp::timeStr);
	end();
	delete _droneFilter;
}

#include <time.h>
void EstimationNode::logData(const char* timeStr){
	using namespace std;
	char dirPath[256];
	sprintf(dirPath, "%s/%s", LOG_PATH, timeStr);

	#ifdef WIN32
		CreateDirectoryA(dirPath,NULL);
	#else
		mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	#endif

	char filePath[256];

	FILE* pFile;

	sprintf(filePath, "%s/beforeState.txt", dirPath);
	std::vector<double>::iterator it;
	pFile = fopen(filePath,"w");
	if (!pFile)
		cout << "Cannot open droneState.txt\n";

	for (it = MyApp::log_stateBeforeSlam.begin();
			it != MyApp::log_stateBeforeSlam.end(); it = it + 6){
		fprintf(pFile, "%f %f %f %f %f %f\n",
				*it, *(it + 1), *(it + 2), *(it + 3), *(it + 4), *(it + 5));
	}
	fclose(pFile);

	sprintf(filePath, "%s/slamState.txt", dirPath);
	pFile = fopen(filePath,"w");
	if (!pFile)
		cout << "Cannot open droneState.txt\n";

	for (it = MyApp::log_currSlamRes.begin();
			it != MyApp::log_currSlamRes.end(); it = it + 6){
		fprintf(pFile, "%f %f %f %f %f %f\n",
				*it, *(it + 1), *(it + 2), *(it + 3), *(it + 4), *(it + 5));
	}
	fclose(pFile);

	sprintf(filePath, "%s/afterState.txt", dirPath);
	pFile = fopen(filePath,"w");
	if (!pFile)
		cout << "Cannot open droneState.txt\n";

	for (it = MyApp::log_stateAfterSlam.begin();
			it != MyApp::log_stateAfterSlam.end(); it = it + 6){
		fprintf(pFile, "%f %f %f %f %f %f\n",
				*it, *(it + 1), *(it + 2), *(it + 3), *(it + 4), *(it + 5));
	}
	fclose(pFile);

	sprintf(filePath, "%s/scale.txt", dirPath);
	pFile = fopen(filePath,"w");
	if (!pFile)
		cout << "Cannot open droneState.txt\n";

	for (it = MyApp::log_scale.begin();
			it != MyApp::log_scale.end(); it++){
		fprintf(pFile, "%f\n", *it);
	}
	fclose(pFile);

//	std::vector<ScaleEstimator>::iterator pairsIter;
//	sprintf(filePath, "%s/scalePairs.txt", dirPath);
//	pFile = fopen(filePath,"w");
//	if (!pFile)
//		cout << "Cannot open droneState.txt\n";
//
//	for (pairsIter = MyApp::log_scalePairs.begin();
//			pairsIter != MyApp::log_scalePairs.end(); it++){
//		fprintf(pFile, "%f %f %f %f %f %f %f\n", pairsIter->_ptamMeas[0], pairsIter->_ptamMeas[1], pairsIter->_ptamMeas[2],
//				pairsIter->_imuMeas[0], pairsIter->_imuMeas[1], pairsIter->_imuMeas[2], pairsIter->_singlePairEstimate);
//	}
//	fclose(pFile);
}

void EstimationNode::initCSTransforms(){
	float pi = 3.14159265;

	// camera CS to global CS
	_camToGlobalRotOnly.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion rot;
	rot.setRPY(pi / 2,0,0);
	_camToGlobalRotOnly.setRotation(rot);
	_globalToCamRotOnly = _camToGlobalRotOnly.inverse();

	_globalToCam.setOrigin(tf::Vector3(0,0.2,0.025));
	rot.setRPY(-pi / 2,0,0);
	_globalToCam.setRotation(rot);
	_camToGlobal = _globalToCam.inverse();

	_droneToGlobal.setOrigin(tf::Vector3(0,0,-1));
	tf::Quaternion droneToGlobalRot;
	droneToGlobalRot.setRPY(pi, 0 , pi/2);
	_droneToGlobal.setRotation(droneToGlobalRot);
	_globalToDrone = _droneToGlobal.inverse();

	_droneToCamInGlobal.setOrigin(tf::Vector3(0,0.2,0.025));
	rot.setRPY(0,0,0);
	_droneToCamInGlobal.setRotation(rot);
	_camToDroneInGlobal = _droneToCamInGlobal.inverse();
}

// Plan A: update whenever the navdata is available
//void EstimationNode::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
//{
//	if (_recording){
//		time_t timer;
//		time(&timer);  /* get current time; same as: timer = time(NULL)  */
//		double seconds = difftime(timer,mktime(&y2k));
//		_navdataTsVec.push_back(seconds);
//		_navdataVec.push_back(*navdataPtr);
//	}
//	_currNavdataReceived = *navdataPtr;
//	_currNavdataReceived.rotY *= -1;
//	_currNavdataReceived.rotZ *= -1;
//	_currNavdataReceived.vy *= -1;
//
//	if (_lastNavdataReceived.altd > 0)
//		_currNavdataReceived.vz = (_currNavdataReceived.altd - _lastNavdataReceived.altd) / 0.005;
//	_currNavdataReceived.vz *= -1;
//
////	ROS_INFO("rotX: %f, rotY: %f, rotZ: %f\n", _currNavdataReceived.rotX, _currNavdataReceived.rotY, _currNavdataReceived.rotZ);
////	ROS_INFO("vx: %f, vy: %f, vz: %f, alt: %d\n", _currNavdataReceived.vx, _currNavdataReceived.vy,
////			_currNavdataReceived.vz, _currNavdataReceived.altd);
//
//	if (_navdata_first_set){
//		_world2body_first.setEulerYPR(_currNavdataReceived.rotZ / 180 * 3.1415,
//				_currNavdataReceived.rotY / 180 * 3.1415,
//				_currNavdataReceived.rotX / 180 * 3.1415);
//		_scaleFilter[0].ts_last_update = getRelativeTime(_currNavdataReceived.header.stamp);
//		_navdata_first_set = false;
//		stateTSVec.push_back(getRelativeTime(_currNavdataReceived.header.stamp));
//	}
//
//	if (_scaleFilter[0].ts_last_update > 0)
//	{
//		_world2body.setEulerYPR(_currNavdataReceived.rotZ / 180 * 3.1415,
//				_currNavdataReceived.rotY / 180 * 3.1415,
//				_currNavdataReceived.rotX / 180 * 3.1415);
//		btVector3 vel_body(_currNavdataReceived.vx, _currNavdataReceived.vy,
//				_currNavdataReceived.vz);
//		btVector3 vel_world = _world2body_first.transpose() * _world2body * vel_body;
//		pthread_mutex_lock(&MyApp::_mutexScaleFilterUpdate);
//		double delta_t = getRelativeTime(_currNavdataReceived.header.stamp) - _scaleFilter[0].ts_last_update;
//
//		printf("navdata update time stamp: %lf\n", delta_t);
//
//		if (delta_t > 0){
//			_scaleFilter[0].updateImu(delta_t, vel_world[0]);
//			_scaleFilter[1].updateImu(delta_t, vel_world[1]);
//			_scaleFilter[2].updateImu(delta_t, vel_world[2]);
//			_scaleFilter[0].ts_last_update += delta_t;
//	//		printf("%lf %lf %lf\n", vel_world[0], vel_world[1], vel_world[2]);
//	//		cout << _scaleFilter[0]._state << endl;
////			cout << _scaleFilter[1]._state << endl;
//	//		cout << _scaleFilter[2]._state << endl;
////			cout << _scaleFilter[1]._state.at<double>(0,0) << " "
////					<< _scaleFilter[1]._state.at<double>(1,0) << " "
////					<< _scaleFilter[1]._state.at<double>(2,0) << endl;
//		}
//		stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(0,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(1,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(2,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(0,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(1,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(2,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(0,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(1,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(2,0));
//		stateTSVec.push_back(getRelativeTime(_currNavdataReceived.header.stamp));
//		pthread_mutex_unlock(&MyApp::_mutexScaleFilterUpdate);
//	}
//
//	_lastNavdataReceived = _currNavdataReceived;
//}

// Plan B: update whenever navdata is available, however the filter will roll back when
// vision update is available.

//void EstimationNode::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
//{
//	if (_recording){
//		time_t timer;
//		time(&timer);  /* get current time; same as: timer = time(NULL)  */
//		double seconds = difftime(timer,mktime(&y2k));
//		_navdataTsVec.push_back(seconds);
//		_navdataVec.push_back(*navdataPtr);
//	}
//	_currNavdataReceived = *navdataPtr;
//	_currNavdataReceived.rotY *= -1;
//	_currNavdataReceived.rotZ *= -1;
//	_currNavdataReceived.vy *= -1;
//
//	if (_lastNavdataReceived.altd > 0)
//		_currNavdataReceived.vz = (_currNavdataReceived.altd - _lastNavdataReceived.altd) / 0.005;
//	_currNavdataReceived.vz *= -1;
//
////	ROS_INFO("rotX: %f, rotY: %f, rotZ: %f\n", _currNavdataReceived.rotX, _currNavdataReceived.rotY, _currNavdataReceived.rotZ);
////	ROS_INFO("vx: %f, vy: %f, vz: %f, alt: %d\n", _currNavdataReceived.vx, _currNavdataReceived.vy,
////			_currNavdataReceived.vz, _currNavdataReceived.altd);
//
//	if (_navdata_first_set){
//		_world2body_first.setEulerYPR(_currNavdataReceived.rotZ / 180 * 3.1415,
//				_currNavdataReceived.rotY / 180 * 3.1415,
//				_currNavdataReceived.rotX / 180 * 3.1415);
//		_scaleFilter[0].ts_last_update_nav = getRelativeTime(_currNavdataReceived.header.stamp);
//		_scaleFilter[1].ts_last_update_nav = getRelativeTime(_currNavdataReceived.header.stamp);
//		_scaleFilter[2].ts_last_update_nav = getRelativeTime(_currNavdataReceived.header.stamp);
//		_navdata_first_set = false;
//		stateTSVec.push_back(getRelativeTime(_currNavdataReceived.header.stamp));
//	}
//
//	if (_scaleFilter[0].ts_last_update_nav > 0)
//	{
//		_world2body.setEulerYPR(_currNavdataReceived.rotZ / 180 * 3.1415,
//				_currNavdataReceived.rotY / 180 * 3.1415,
//				_currNavdataReceived.rotX / 180 * 3.1415);
//		btVector3 vel_body(_currNavdataReceived.vx, _currNavdataReceived.vy,
//				_currNavdataReceived.vz);
//		btVector3 vel_world = _world2body_first.transpose() * _world2body * vel_body;
//
//		pthread_mutex_lock(&MyApp::_mutexScaleFilterUpdate);
//		if (_busyUpdateVision)
//			pthread_cond_wait(&MyApp::_condScaleFilterUpdate,
//					&MyApp::_mutexScaleFilterUpdate);
////		double delta_t = getRelativeTime(_currNavdataReceived.header.stamp) - _scaleFilter[0].ts_last_update;
//		double curr_t = getRelativeTime(_currNavdataReceived.header.stamp);
//		printf("navdata update time stamp: %lf\n", curr_t);
//		// delta_t should be always positive
////		if (delta_t > 0){
//			_scaleFilter[0].updateImu(curr_t, vel_world[0]);
//			_scaleFilter[1].updateImu(curr_t, vel_world[1]);
//			_scaleFilter[2].updateImu(curr_t, vel_world[2]);
//	//		printf("%lf %lf %lf\n", vel_world[0], vel_world[1], vel_world[2]);
//	//		cout << _scaleFilter[0]._state << endl;
////			cout << _scaleFilter[1]._state << endl;
//	//		cout << _scaleFilter[2]._state << endl;
////			cout << _scaleFilter[1]._state.at<double>(0,0) << " "
////					<< _scaleFilter[1]._state.at<double>(1,0) << " "
////					<< _scaleFilter[1]._state.at<double>(2,0) << endl;
////		}
//		stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(0,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(1,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(2,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(0,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(1,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(2,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(0,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(1,0));
//		stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(2,0));
//		stateTSVec.push_back(getRelativeTime(_currNavdataReceived.header.stamp));
//		pthread_mutex_unlock(&MyApp::_mutexScaleFilterUpdate);
//	}
//
//	_lastNavdataReceived = _currNavdataReceived;
//}


// Plan C: predict the state when queried, otherwise cache the states and wait for vision update
void EstimationNode::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	if (_recording){
		time_t timer;
		time(&timer);  /* get current time; same as: timer = time(NULL)  */
		double seconds = difftime(timer,mktime(&y2k));
		_navdataTsVec.push_back(seconds);
		_navdataVec.push_back(*navdataPtr);
	}
	_currNavdataReceived = *navdataPtr;
	_currNavdataReceived.rotY *= -1;
	_currNavdataReceived.rotZ *= -1;
	_currNavdataReceived.vy *= -1;

	if (_lastNavdataReceived.altd > 0)
		_currNavdataReceived.vz = (_currNavdataReceived.altd - _lastNavdataReceived.altd) / 0.005;
	_currNavdataReceived.vz *= -1;

//	ROS_INFO("rotX: %f, rotY: %f, rotZ: %f\n", _currNavdataReceived.rotX, _currNavdataReceived.rotY, _currNavdataReceived.rotZ);
//	ROS_INFO("vx: %f, vy: %f, vz: %f, alt: %d\n", _currNavdataReceived.vx, _currNavdataReceived.vy,
//			_currNavdataReceived.vz, _currNavdataReceived.altd);

	if (_navdata_first_set){
		_world2body_first.setEulerYPR(_currNavdataReceived.rotZ / 180 * 3.1415,
				_currNavdataReceived.rotY / 180 * 3.1415,
				_currNavdataReceived.rotX / 180 * 3.1415);
		_scaleFilter[0].ts_last_update = getRelativeTime(_currNavdataReceived.header.stamp);
		_scaleFilter[1].ts_last_update = getRelativeTime(_currNavdataReceived.header.stamp);
		_scaleFilter[2].ts_last_update = getRelativeTime(_currNavdataReceived.header.stamp);
		_navdata_first_set = false;
		stateTSVec.push_back(getRelativeTime(_currNavdataReceived.header.stamp));
	}

	if (_scaleFilter[0].ts_last_update > 0)
	{
		_world2body.setEulerYPR(_currNavdataReceived.rotZ / 180 * 3.1415,
				_currNavdataReceived.rotY / 180 * 3.1415,
				_currNavdataReceived.rotX / 180 * 3.1415);
		tf2::Vector3 vel_body(_currNavdataReceived.vx, _currNavdataReceived.vy,
				_currNavdataReceived.vz);
		tf2::Vector3 vel_world = _world2body_first.transpose() * _world2body * vel_body;
		double curr_t = getRelativeTime(_currNavdataReceived.header.stamp);

		vel_world_vec.push_back(vel_world);
		vel_world_ts_vec.push_back(curr_t);
	}

//	stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(0,0));
//	stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(1,0));
//	stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(2,0));
//	stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(0,0));
//	stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(1,0));
//	stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(2,0));
//	stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(0,0));
//	stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(1,0));
//	stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(2,0));
//	stateTSVec.push_back(getRelativeTime(_currNavdataReceived.header.stamp));

	_lastNavdataReceived = _currNavdataReceived;
}


void EstimationNode::imuCb(const sensor_msgs::Imu::ConstPtr imuPtr) {
	if (_recording)
		_imuVec.push_back(*imuPtr);
}

void EstimationNode::ctrlCb(const geometry_msgs::TwistConstPtr velPtr)
{
	geometry_msgs::TwistStamped ts;
	ts.header.stamp = ros::Time::now();
	ts.twist = *velPtr;

	// for some reason this needs to be inverted.
	// linear.y corresponds to ROLL
	ts.twist.linear.y *= -1;
	ts.twist.linear.x *= -1;
	ts.twist.angular.z *= -1;

	pthread_mutex_lock( &_droneFilter->s_mutexFilterState );
	_droneFilter->velQueue->push_back(ts);
	pthread_mutex_unlock( &_droneFilter->s_mutexFilterState );
}

void EstimationNode::videoCb(const sensor_msgs::ImageConstPtr img)
{
	//cout << "Video callback\n";
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img,
				sensor_msgs::image_encodings::BGR8);

//	double tmVid = getVideoTime(img->header.stamp.sec, img->header.stamp.nsec,
//			_lastNavTS.toSec());

	ros::Time frameTimeRos;
//	if(ros::Time::now() - img->header.stamp > ros::Duration(30.0))
//		frameTimeRos = (ros::Time::now()-ros::Duration(0.001));
//	else
		frameTimeRos = img->header.stamp;

	double frameTime = getRelativeTime(frameTimeRos);

//	printf("add back new frame at time stamp: %f\n", frameTime);
	_droneCamReader->addBack(frameTime, cv_ptr->image);

	if (_recording){
		time_t timer;
		time(&timer);  /* get current time; same as: timer = time(NULL)  */
		double seconds = difftime(timer,mktime(&y2k));
		_imgTsVec.push_back(seconds);
	}
//	printf("added back a new frame: %f\n", frameTime);
}

// Plan A
//void EstimationNode::updateFilterSLAM(double currTs, btVector3& slamPos, btVector3& slamVel){
//	pthread_mutex_lock(&MyApp::_mutexScaleFilterUpdate);
//	currTs = getRelativeTime(ros::Time::now());
//	double delta_t = currTs - _scaleFilter[0].ts_last_update;
//	printf("Vision update time stamp: (%lf - %lf) %lf\n", currTs, _scaleFilter[0].ts_last_update, delta_t);
//	if (delta_t > 0){
//		_scaleFilter[0].updateVision(delta_t, slamPos[0],slamVel[0]);
//		_scaleFilter[1].updateVision(delta_t, slamPos[1],slamVel[1]);
//		_scaleFilter[2].updateVision(delta_t, slamPos[2],slamVel[2]);
//		_scaleFilter[0].ts_last_update += delta_t;
//	}
//	pthread_mutex_unlock(&MyApp::_mutexScaleFilterUpdate);
//}

// Plan B
//void EstimationNode::updateFilterSLAM(double currTs, btVector3& slamPos, btVector3& slamVel){
//	pthread_mutex_lock(&MyApp::_mutexScaleFilterUpdate);
//	_busyUpdateVision = true;
//	_scaleFilter[0].rollBack(currTs, slamPos[0],slamVel[0]);
//	_scaleFilter[1].rollBack(currTs, slamPos[1],slamVel[1]);
//	_scaleFilter[2].rollBack(currTs, slamPos[2],slamVel[2]);
//	_busyUpdateVision = false;
//	pthread_cond_signal(&MyApp::_condScaleFilterUpdate);
//	pthread_mutex_unlock(&MyApp::_mutexScaleFilterUpdate);
//}

//Plan C
void EstimationNode::updateFilterSLAM(double currTs, tf2::Vector3& slamPos, tf2::Vector3& slamVel){
	pthread_mutex_lock(&MyApp::_mutexScaleFilterUpdate);
	for(int ii = 0; ii < vel_world_vec.size(); ii++){
		if ( vel_world_ts_vec[ii] <= _scaleFilter[0].ts_last_update_vision){
			vel_world_vec.pop_front();
			vel_world_ts_vec.pop_front();
			ii = 0;
		}
		else if (vel_world_ts_vec[ii] > _scaleFilter[0].ts_last_update_vision
				&& vel_world_ts_vec[ii] <= currTs){
			_scaleFilter[0].updateImu(vel_world_ts_vec[ii], vel_world_vec[ii][0]);
			_scaleFilter[1].updateImu(vel_world_ts_vec[ii], vel_world_vec[ii][1]);
			_scaleFilter[2].updateImu(vel_world_ts_vec[ii], vel_world_vec[ii][2]);
		}
		else if (vel_world_ts_vec[ii] > currTs){
			_scaleFilter[0].updateVision(currTs, slamPos[0],slamVel[0]);
			_scaleFilter[1].updateVision(currTs, slamPos[1],slamVel[1]);
			_scaleFilter[2].updateVision(currTs, slamPos[2],slamVel[2]);
			break;
		}
	}
	pthread_mutex_unlock(&MyApp::_mutexScaleFilterUpdate);
	stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(0,0));
	stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(1,0));
	stateVec_scaleFilter.push_back(_scaleFilter[0]._state.at<double>(2,0));
	stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(0,0));
	stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(1,0));
	stateVec_scaleFilter.push_back(_scaleFilter[1]._state.at<double>(2,0));
	stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(0,0));
	stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(1,0));
	stateVec_scaleFilter.push_back(_scaleFilter[2]._state.at<double>(2,0));
	stateTSVec.push_back(getRelativeTime(_currNavdataReceived.header.stamp));
	return;
}

void EstimationNode::getPredictedState(double currTs, double state[6]){
	//Make a copy of current filters
	ScaleFilter filters[3];
	for (int ii = 0; ii < 3; ii++){
		_scaleFilter[ii]._state_last_vision.copyTo(filters[ii]._state);
		_scaleFilter[ii]._P_last_vision.copyTo(filters[ii]._P);
		filters[ii].ts_last_update = _scaleFilter[ii].ts_last_update_vision;
	}
	//do a prediction
	for (int ii = 0; ii < vel_world_vec.size(); ii++){
		if( vel_world_ts_vec[ii] >= filters[0].ts_last_update &&
				vel_world_ts_vec[ii] <= currTs){
			filters[0].predictImu(vel_world_ts_vec[ii], vel_world_vec[ii][0]);
			filters[1].predictImu(vel_world_ts_vec[ii], vel_world_vec[ii][1]);
			filters[2].predictImu(vel_world_ts_vec[ii], vel_world_vec[ii][2]);
		}
		else if (vel_world_ts_vec[ii] > currTs)
			break;
	}
	state[0] = filters[0].getPos();
	state[1] = filters[0].getVel();
	state[2] = filters[1].getPos();
	state[3] = filters[1].getVel();
	state[4] = filters[2].getPos();
	state[5] = filters[2].getVel();
}

double EstimationNode::getLastUpdateTs(){
	return _scaleFilter[0].ts_last_update;
}

void EstimationNode::comCb(const std_msgs::StringConstPtr str)
{
	if(str->data.length() > 2 && str->data.substr(0,2) == "p "){
		std::string s = str->data.substr(2,str->data.length()-2);
		ROS_INFO("Receive command: %s\n", s.c_str());
		if (s == "startinit"){
			MyApp::bStartInit = !MyApp::bStartInit;
		}
	}
}

void EstimationNode::dynConfCb(monoslam_ros_test::StateestimationParamsConfig &config, uint32_t level)
{
	if(!_droneFilter->allSyncLocked && config.PTAMSyncLock)
		ROS_WARN("Ptam Sync has been disabled. This fixes scale etc.");

//	if(!ptamWrapper->mapLocked && config.PTAMMapLock)
//		ROS_WARN("Ptam Map has been locked.");


	_droneFilter->useControl =config.UseControlGains;
	_droneFilter->usePTAM =config.UsePTAM;
	_droneFilter->useNavdata =config.UseNavdata;

	_droneFilter->useScalingFixpoint = config.RescaleFixOrigin;

//	ptamWrapper->maxKF = config.PTAMMaxKF;
//	ptamWrapper->mapLocked = config.PTAMMapLock;
	_droneFilter->allSyncLocked = config.PTAMSyncLock;


	//ptamWrapper->setPTAMPars(config.PTAMMinKFTimeDiff, config.PTAMMinKFWiggleDist, config.PTAMMinKFDist);


	_droneFilter->c1 = config.c1;
	_droneFilter->c2 = config.c2;
	_droneFilter->c3 = config.c3;
	_droneFilter->c4 = config.c4;
	_droneFilter->c5 = config.c5;
	_droneFilter->c6 = config.c6;
	_droneFilter->c7 = config.c7;
	_droneFilter->c8 = config.c8;

}

pthread_mutex_t EstimationNode::s_mutextmBase;
void EstimationNode::tmBaseCb(const std_msgs::Int32::ConstPtr& tmBase){
	pthread_mutex_lock(&s_mutextmBase);
	_timeBase = tmBase->data;
	pthread_mutex_unlock(&s_mutextmBase);
}

void EstimationNode::scaleInitCb(const std_msgs::EmptyConstPtr){
	_droneFilter->_scaleStop = true;
}
void EstimationNode::start(){
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&_estimateThreadId, &attr, threadProc, this);
}

void EstimationNode::loop()
{
	cout << "starting ROS main thread...\n" << endl;
	dynamic_reconfigure::Server<monoslam_ros_test::StateestimationParamsConfig> srv;
	dynamic_reconfigure::Server<monoslam_ros_test::StateestimationParamsConfig>::CallbackType f;
	f = boost::bind(&EstimationNode::dynConfCb, MyApp::_estimationNode, _1, _2);
	srv.setCallback(f);
	ros::Rate pub_rate(_publishFreq);

	while (_estimateNodeHandler.ok() && MyApp::bRunning) {
	 // std::cout << " ROS loop is running\n";
	  ros::spinOnce();
	  if (_recording){
		  _droneCamReader->enableRecording();
	  }

	  // For hovering
	  if (_posController._bRefSet){
		  pthread_mutex_lock(&MyApp::_mutexScaleFilterUpdate);
//		  double x = _scaleFilter[0].getPos();
//		  double x_vel = _scaleFilter[0].getVel();
//		  double y = _scaleFilter[1].getPos();
//		  double y_vel = _scaleFilter[1].getVel();
////		  double z = _scaleFilter[2].getPos();
//		  double z = _lastNavdataReceived.altd * 1.0;
//		  double z_vel = _scaleFilter[2].getVel();
//		  double yaw = _lastNavdataReceived.rotZ;
		  double state[6];
		  getPredictedState(getRelativeTime(ros::Time::now()), state);
		  double x = state[0];
		  double x_vel = state[1];
		  double y = state[2];
		  double y_vel = state[3];
  //		  double z = _scaleFilter[2].getPos();
		  double z = _lastNavdataReceived.altd * 1.0;
		  double z_vel = state[5];
		  double yaw = _lastNavdataReceived.rotZ;
		  pthread_mutex_unlock(&MyApp::_mutexScaleFilterUpdate);

		  double x_err = _posController._xRef - x;
		  double y_err = _posController._yRef - y;
		  double z_err = _posController._zRef - z;
		  z_err *= -1;

		  double yaw_err = (_posController._yawRef - yaw) * 3.1415 / 180;

		  tf2::Vector3 errVec(x_err, y_err, z_err);
		  tf2::Vector3 velVec(x_vel, y_vel, z_vel);
		  tf2::Vector3 errVec_Body = _world2body.transpose() * _world2body_first * errVec;
		  tf2::Vector3 velVec_Body = _world2body.transpose() * _world2body_first * velVec;

//		  _posController.computePID(errVec_Body[0], errVec_Body[1], -errVec_Body[2], yaw_err);
		  _posController.ctrlRoll = _posController._rollController.computeCtrl(errVec_Body[1], -velVec_Body[1]);
		  _posController.ctrlPitch = _posController._pitchController.computeCtrl(errVec_Body[0], -velVec_Body[0]);
		  _posController.ctrlZRate = _posController._zController.computeCtrl(-errVec_Body[2]);
		  _posController.ctrlYawRate = _posController._yawController.computeCtrl(yaw_err);


		  time_t timer;
		  time(&timer);  /* get current time; same as: timer = time(NULL)  */
		  double seconds = difftime(timer,mktime(&y2k));

//		  stateVec_control.push_back(getRelativeTime(_lastNavdataReceived.header.stamp));
		  stateVec_control.push_back(seconds);
		  stateVec_control.push_back(x);
		  stateVec_control.push_back(y);
		  stateVec_control.push_back(z);
		  stateVec_control.push_back(yaw);
		  stateVec_control.push_back(x_vel);
		  stateVec_control.push_back(y_vel);
		  stateVec_control.push_back(_posController._xRef);
		  stateVec_control.push_back(_posController._yRef);
		  stateVec_control.push_back(_posController._zRef);
		  stateVec_control.push_back(_posController._yawRef);
		  stateVec_control.push_back(0);
		  stateVec_control.push_back(0);

		  cmdVec.push_back(getRelativeTime(_lastNavdataReceived.header.stamp));
		  cmdVec.push_back(_posController.ctrlPitch);
		  cmdVec.push_back(_posController.ctrlRoll);
		  cmdVec.push_back(_posController.ctrlYawRate);
		  cmdVec.push_back(_posController.ctrlZRate);
		  cmdVec.push_back(errVec_Body[0]);
		  cmdVec.push_back(errVec_Body[1]);
		  cmdVec.push_back(errVec_Body[2]);
		  cmdVec.push_back(yaw_err);

		  _ctrlPub.publish(_posController.sendCmd());
	  }
	  else if (_pathPlanner._pathEnabled){
		  double x_ref, y_ref, vx_ref, vy_ref;
		  _pathPlanner.getCurrentRef(x_ref, y_ref, vx_ref, vy_ref);
		  pthread_mutex_lock(&MyApp::_mutexScaleFilterUpdate);
		  double z_ref = _posController._zRef;
//		  double x = _scaleFilter[0].getPos();
//		  double x_vel = _scaleFilter[0].getVel();
//		  double y = _scaleFilter[1].getPos();
//		  double y_vel = _scaleFilter[1].getVel();
// //		  double z = _scaleFilter[2].getPos();
//		  double z = _lastNavdataReceived.altd * 1.0;
//		  double z_vel = _scaleFilter[2].getVel();
//		  double yaw = _lastNavdataReceived.rotZ;

		  double state[6];
		  getPredictedState(getRelativeTime(ros::Time::now()), state);
		  double x = state[0];
		  double x_vel = state[1];
		  double y = state[2];
		  double y_vel = state[3];
  //		  double z = _scaleFilter[2].getPos();
		  double z = _lastNavdataReceived.altd * 1.0;
		  double z_vel = state[5];
		  double yaw = _lastNavdataReceived.rotZ;
		  pthread_mutex_unlock(&MyApp::_mutexScaleFilterUpdate);

		  double x_err = x_ref - x;
		  double y_err = y_ref - y;
		  double z_err = z_ref - z;
		  z_err *= -1;

		  double yaw_err = (_posController._yawRef - yaw) * 3.1415 / 180;

		  tf2::Vector3 errVec(x_err, y_err, z_err);
		  tf2::Vector3 velVec(x_vel, y_vel, z_vel);
		  tf2::Vector3 velRefVec(vx_ref, vy_ref, 0);
		  tf2::Vector3 errVec_Body = _world2body.transpose() * _world2body_first * errVec;
		  tf2::Vector3 velVec_Body = _world2body.transpose() * _world2body_first * velVec;
		  tf2::Vector3 velRefVec_Body = _world2body.transpose() * _world2body_first * velRefVec;

 //		  _posController.computePID(errVec_Body[0], errVec_Body[1], -errVec_Body[2], yaw_err);
		  _posController.ctrlRoll = _posController._rollController.computeCtrl(errVec_Body[1], velRefVec_Body[1] - velVec_Body[1]);
		  _posController.ctrlPitch = _posController._pitchController.computeCtrl(errVec_Body[0], velRefVec_Body[0] - velVec_Body[0]);
		  _posController.ctrlZRate = _posController._zController.computeCtrl(-errVec_Body[2]);
		  _posController.ctrlYawRate = _posController._yawController.computeCtrl(yaw_err);

		  time_t timer;
		  time(&timer);  /* get current time; same as: timer = time(NULL)  */
		  double seconds = difftime(timer,mktime(&y2k));

//		  stateVec_control.push_back(getRelativeTime(_lastNavdataReceived.header.stamp));
		  stateVec_control.push_back(seconds);
		  stateVec_control.push_back(x);
		  stateVec_control.push_back(y);
		  stateVec_control.push_back(z);
		  stateVec_control.push_back(yaw);
		  stateVec_control.push_back(x_vel);
		  stateVec_control.push_back(y_vel);
		  stateVec_control.push_back(x_ref);
		  stateVec_control.push_back(y_ref);
		  stateVec_control.push_back(z_ref);
		  stateVec_control.push_back(_posController._yawRef);
		  stateVec_control.push_back(vx_ref);
		  stateVec_control.push_back(vy_ref);


		  cmdVec.push_back(getRelativeTime(_lastNavdataReceived.header.stamp));
		  cmdVec.push_back(_posController.ctrlPitch);
		  cmdVec.push_back(_posController.ctrlRoll);
		  cmdVec.push_back(_posController.ctrlYawRate);
		  cmdVec.push_back(_posController.ctrlZRate);
		  cmdVec.push_back(errVec_Body[0]);
		  cmdVec.push_back(errVec_Body[1]);
		  cmdVec.push_back(errVec_Body[2]);
		  cmdVec.push_back(yaw_err);

		  _ctrlPub.publish(_posController.sendCmd());
	  }

	  if (_pathPlanner.isPathFinished()){
		  _pathPlanner.reset();
		  ControlCommand cmd(0,0,0,0);
		  MyApp::_estimationNode->sendControlToDrone(cmd);
	  }

//	  if (_scaleFilter[0].ts_last_update > 0){
//		  stateVec_scaleFilter[0].push_back(_scaleFilter[0]._state);
//		  stateVec_scaleFilter[1].push_back(_scaleFilter[1]._state);
//		  stateVec_scaleFilter[2].push_back(_scaleFilter[2]._state);
//		  stateTSVec.push_back(getRelativeTime(ros::Time().now()));
//	  }
//	  pthread_mutex_lock( &_droneFilter->s_mutexFilterState );
//	  double predictTo = (ros::Time().now() + _predTime).toSec();
//	  movisslam_ros_test::filter_state s = _droneFilter->getPoseAt(ros::Time().now() + _predTime);
//	  stateVec.push_back(s);
//	  pthread_mutex_unlock( &_droneFilter->s_mutexFilterState );
//	  printf("update filter up to %10f\n", predictTo);
//	  stateTSVec.push_back(predictTo);
//	  _predictedPosePub.publish(s);
	  pub_rate.sleep();
	}
	_droneCamReader->_videoWriter.release();
	_droneCamReader->_tsLog.close();
	log("/home/rui/SFM_data/ARDrone/imu.txt",
			"/home/rui/SFM_data/ARDrone/navdata.txt",
			"/home/rui/SFM_data/ARDrone/state.txt");
	_exitPub.publish(std_msgs::Empty());
	cout << "EstimationNode exits!" << endl;
}

void EstimationNode::log(const char* imuPath, const char* navdataPath, const char* statePath){
	FILE* file = fopen(navdataPath, "w");
	for (int ii = 0; ii < _navdataVec.size(); ii++){
		ardrone_autonomy::Navdata& nav = _navdataVec[ii];
		fprintf(file, "%lf ", _navdataTsVec[ii]);
		fprintf(file, "%lf %f %u ", getRelativeTime(nav.header.stamp), nav.batteryPercent, nav.state);
		fprintf(file, "%f %f %f ", nav.rotX, nav.rotY, nav.rotZ);
		fprintf(file, "%d %d %d ", nav.magX, nav.magY, nav.magZ);
		fprintf(file, "%d %d %f %f %f ", nav.pressure, nav.temp, nav.wind_speed, nav.wind_angle, nav.wind_comp_angle);
		fprintf(file, "%d %f %f %f ", nav.altd, nav.vx, nav.vy, nav.vz);
		fprintf(file, "%f %f %f %f\n", nav.ax, nav.ay, nav.az, nav.tm);
	}
	fclose(file);

	file = fopen(imuPath, "w");
	for (int jj = 0; jj < _imuVec.size(); jj++){
		sensor_msgs::Imu& imu = _imuVec[jj];
		fprintf(file, "%f\n", getRelativeTime(imu.header.stamp));
		// Orientation
		fprintf(file, "%f %f %f %f\n", imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
		for (int k = 0; k < 9; k++){
			fprintf(file, "%lf ", imu.orientation_covariance[k]);
		}
		fprintf(file, "\n");

		// angular velocity
		fprintf(file, "%f %f %f\n", imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
		for (int k = 0; k < 9; k++){
			fprintf(file, "%lf ", imu.angular_velocity_covariance[k]);
		}
		fprintf(file, "\n");

		// linear_acceleration
		fprintf(file, "%f %f %f\n", imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
		for (int k = 0; k < 9; k++){
			fprintf(file, "%lf ", imu.linear_acceleration_covariance[k]);
		}
		fprintf(file, "\n");
	}
	fclose(file);

	file = fopen(statePath, "w");
	fprintf(file, "%lf\n", stateTSVec[0]);

	for (int ii = 0; ii < stateTSVec.size()-1; ii++){
		fprintf(file, "%lf ", stateTSVec[ii+1]);
		fprintf(file, "%lf %lf %lf ", stateVec_scaleFilter[9*ii],
									  stateVec_scaleFilter[9*ii+1],
									  stateVec_scaleFilter[9*ii+2]);
		fprintf(file, "%lf %lf %lf ", stateVec_scaleFilter[9*ii+3],
									  stateVec_scaleFilter[9*ii+4],
									  stateVec_scaleFilter[9*ii+5]);
		fprintf(file, "%lf %lf %lf\n",stateVec_scaleFilter[9*ii+6],
									  stateVec_scaleFilter[9*ii+7],
									  stateVec_scaleFilter[9*ii+8]);
	}
	fclose(file);

	file = fopen("/home/rui/SFM_data/ARDrone/cmdVec.txt", "w");
		for (int ii = 0; ii < cmdVec.size(); ii = ii + 9){
			fprintf(file, "%f %lf %lf %lf %lf %lf %lf %lf %lf\n", cmdVec[ii],
					cmdVec[ii+1], cmdVec[ii+2], cmdVec[ii+3], cmdVec[ii+4],
					cmdVec[ii+5], cmdVec[ii+6], cmdVec[ii+7], cmdVec[ii+8]);
	}
	fclose(file);

	file = fopen("/home/rui/SFM_data/ARDrone/state_control.txt", "w");
		for (int ii = 0; ii < stateVec_control.size(); ii = ii + 13){
			fprintf(file, "%f %lf %lf %lf %lf %lf %lf ", stateVec_control[ii],
														 stateVec_control[ii+1],
														 stateVec_control[ii+2],
														 stateVec_control[ii+3],
														 stateVec_control[ii+4],
														 stateVec_control[ii+5],
														 stateVec_control[ii+6]);
			fprintf(file, "%lf %lf %lf %lf %lf %lf\n",   stateVec_control[ii+7],
														 stateVec_control[ii+8],
														 stateVec_control[ii+9],
														 stateVec_control[ii+10],
														 stateVec_control[ii+11],
														 stateVec_control[ii+12]);
	}
	fclose(file);

	file = fopen("/home/rui/SFM_data/ARDrone/imgTs.txt", "w");
	for (int ii = 0; ii < _imgTsVec.size(); ii++){
		fprintf(file, "%lf\n", _imgTsVec[ii]);
	}
	fclose(file);
}

void EstimationNode::end()
{
	ROS_INFO("waiting the drone thread to quit...\n");
	pthread_join(_estimateThreadId, NULL);
	ROS_INFO("Drone thread quits!\n");
}

void* EstimationNode::threadProc(void* data) {
	EstimationNode* thread = (EstimationNode*) data;
	thread->loop();
	return 0;
}

pthread_mutex_t EstimationNode::s_mutexDrone = PTHREAD_MUTEX_INITIALIZER; //pthread_mutex_lock( &cs_mutex );
void EstimationNode::publishCommand(std::string c)
{
	std_msgs::String s;
	s.data = c.c_str();
	pthread_mutex_lock(&s_mutexDrone);
	_comPub.publish(s);
	pthread_mutex_unlock(&s_mutexDrone);
}

void EstimationNode::reSendInfo()
{}

double EstimationNode::getVideoTime(uint32_t sec, uint32_t usec, double imutime) {
	static int num = 0;
	static double s_ds = 0;
	static bool bCal = true;

	double tmV = (sec + usec / 1000000000.0);
	if (num < 20) {
		s_ds += tmV - imutime;
		num++;
	} else {
		if (bCal) {
			s_ds /= num;
			bCal = false;
		} else {
			return tmV - s_ds;
		}
	}
	return imutime;
}

double EstimationNode::getRelativeTime(ros::Time stamp){
	if(_timeBase == 0)
	{
		_timeBase = stamp.toSec();
		std_msgs::Int32 tmBase;
		tmBase.data = _timeBase;
		pthread_mutex_lock(&s_mutextmBase);
		_tmBasePub.publish(tmBase);
		pthread_mutex_unlock(&s_mutextmBase);
		std::cout << "set ts base to " << _timeBase << std::endl;
	}
	double mss = stamp.toSec() - _timeBase;
//	cout << "mss: " << mss << endl;

	if(mss < 0)
		std::cout << "ERROR: negative timestamp..."<< std::endl;
	return mss;
}

//drone control function

void EstimationNode::sendControlToDrone(ControlCommand cmd)
{
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = -cmd.yaw;
	cmdT.linear.z = cmd.gaz;
	cmdT.linear.x = -cmd.pitch;
	cmdT.linear.y = -cmd.roll;

	// assume that while actively controlling, the above for will never be equal to zero, so i will never hover.
	cmdT.angular.x = cmdT.angular.y = 0;

	_ctrlPub.publish(cmdT);

	lastSentControl = cmd;
	lastControlSentMS = getRelativeTime(ros::Time::now());
}

void EstimationNode::sendLand()
{
//	if(isControlling)
		land_pub.publish(std_msgs::Empty());
}
void EstimationNode::sendTakeoff()
{
//	if(isControlling)
		takeoff_pub.publish(std_msgs::Empty());
}
void EstimationNode::sendToggleState()
{
//	if(isControlling)
		toggleState_pub.publish(std_msgs::Empty());
}

void EstimationNode::setRefPos()
{
	double x = _scaleFilter[0].getPos();
	double x_vel = _scaleFilter[0].getVel();
	double y = _scaleFilter[1].getPos();
	double y_vel = _scaleFilter[1].getVel();
//	double z = _scaleFilter[2].getPos();
	double z = _lastNavdataReceived.altd * 1.0;
	double z_vel = _scaleFilter[2].getVel();
	double yaw = _lastNavdataReceived.rotZ;

	refTm = getRelativeTime(ros::Time::now());
	_posController.setRef(x, y, z, yaw);
}
void EstimationNode::enablePath(){
	double x = _scaleFilter[0].getPos();
	double x_vel = _scaleFilter[0].getVel();
	double y = _scaleFilter[1].getPos();
	double y_vel = _scaleFilter[1].getVel();
//	double z = _scaleFilter[2].getPos();
	double z = _lastNavdataReceived.altd * 1.0;
	double z_vel = _scaleFilter[2].getVel();
	double yaw = _lastNavdataReceived.rotZ;

	refTm = getRelativeTime(ros::Time::now());
	_posController.setRef(x, y, z, yaw);
	_posController._bRefSet = false; // disable this since this is not a hovering command
	_pathPlanner._pathEnabled = true;
}
