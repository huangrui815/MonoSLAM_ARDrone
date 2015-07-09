#ifndef __ESTIMATIONNODE_H
#define __ESTIMATIONNODE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
//#include "tf/tfMessage.h"
//#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include <ardrone_autonomy/Navdata.h>
//#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
//#include <LinearMath/btMatrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include "videoReader/VR_DroneCamReader.h"
//#include "stateestimation/Drone_ScaleFilter.h"

#include <dynamic_reconfigure/server.h>
#include "monoslam_ros_test/StateestimationParamsConfig.h"
#include "monoslam_ros_test/filter_state.h"

#include "ControllerNode/PositionController.h"
#include "ControllerNode/PathPlanner.h"


#ifdef WIN32
#include <Windows.h>
#define LOG_PATH "C:/logData"
#else
#define LOG_PATH "/home/rui/workspace/myROS/debug_output"
#endif

//#include "TooN/"

class DroneKalmanFilter;
class ScaleFilter;

struct ControlCommand
{
	inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
	}
	double yaw, roll, pitch, gaz;
};

class EstimationNode
{
public:
	DroneKalmanFilter* _droneFilter; // 10-dim Kalman Filter
	DroneCamReader* _droneCamReader;
	ScaleFilter* _scaleFilter;

	EstimationNode(int droneId);
	~EstimationNode();

	// ROS message callbacks
	void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
	void imuCb(const sensor_msgs::Imu::ConstPtr imuPtr);
	void ctrlCb(const geometry_msgs::TwistConstPtr velPtr);
	void videoCb(const sensor_msgs::ImageConstPtr img);
	void comCb(const std_msgs::StringConstPtr str);
	void dynConfCb(monoslam_ros_test::StateestimationParamsConfig &config, uint32_t level);
	void tmBaseCb(const std_msgs::Int32::ConstPtr& tmBase);
	void scaleInitCb(const std_msgs::EmptyConstPtr);
	static pthread_mutex_t s_mutextmBase;
	// main pose-estimation loop
	void start();
	void loop();
	void end();
	void log(const char* imuPath, const char* navdataPath, const char* statePath);

	// writes a string message to "/tum_ardrone/com".
	// is thread-safe (can be called by any thread, but may block till other calling thread finishes)
	void publishCommand(std::string c);
	void reSendInfo();
	double getVideoTime(uint32_t sec, uint32_t usec, double imutime);
	static void *threadProc(void*);

	void updateFilterSLAM(double currTs, tf2::Vector3& slamPos, tf2::Vector3& slamVel);

	double getLastUpdateTs();

	DroneCamReader* getDroneCamReader(){
		return _droneCamReader;
	}
	double getRelativeTime(ros::Time stamp = ros::Time::now());

	int _ardroneVersion;

	// control drone functions
	void sendControlToDrone(ControlCommand cmd);
	void sendLand();
	void sendTakeoff();
	void sendToggleState();
	void setRefPos();
	void enablePath();

	double lastControlSentMS;
	ControlCommand lastSentControl;

	static tf::Transform _camToDroneInGlobal, _droneToCamInGlobal;
	static tf::Transform _droneToGlobal, _globalToDrone;
	static tf::Transform _camToGlobalRotOnly, _globalToCamRotOnly;
	static tf::Transform _camToGlobal, _globalToCam;
	tf2::Matrix3x3 _world2body_first;
	tf2::Matrix3x3 _body2cam_rot;
	tf2::Matrix3x3 _world2body;
	void initCSTransforms();

public:
	std::string _packagePath; // package path
	pthread_t _estimateThreadId; // estimation thread

	// communication with drone
	ros::Subscriber _navdataSub; // subscribe drone navdata
	ros::Subscriber _imuSub; // subscribe drone imu
	ros::Subscriber _magSub; // subscribe drone magnetometer

	ros::Subscriber _ctrlSub; // subscribe control commands sent from other thread
	ros::Publisher _ctrlPub; // publish command
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher toggleState_pub;


	ros::Subscriber _videoSub; // subscribe video channel

	ros::Publisher _predictedPosePub; // output

	ros::Subscriber _comSub;
	ros::Publisher _comPub;

	ros::Publisher _tmBasePub;

	ros::Publisher _exitPub;
	ros::Subscriber _scaleSub;



	ros::ServiceClient _cameraToggleClient;
	ros::ServiceClient _ledAnimationClient;
	ros::ServiceClient _flightAnimationClient;
	ros::ServiceClient _imuCalibrationClient;
	ros::ServiceClient _flattrimClient;

	static pthread_mutex_t s_mutexDrone;

	ros::NodeHandle _estimateNodeHandler; // node handler

	tf::TransformBroadcaster _poseBroadcaster; // pose publisher

	// parameters
	// every [publishFreq]ms, the node publishes the drones predicted position [predTime]ms into the future.
	// this pose can then be used to steer the drone. obviously, the larger [predTime], the worse the estimate.
	// this pose is published on /tf, and simultaneously further info is published on /ardrone/predictedPose
	ros::Duration _predTime;
	int _publishFreq;

	std::string _navdataChannel; // resolved name for navdata topic
	std::string _imuChannel; // resolved name for imu topic
	std::string _magChannel; // resolved name for magnetometer
	std::string _controlChannel; // resolved name for control topic
	std::string _predictedPoseChannel;  // resolved name for predicted pose topic
	std::string _videoChannel; 	 // resolved name for video topic
	std::string _commandChannel; // resolved name for command topic
	std::string _tmBaseChannel; // resolved name for time base topic
	std::string _exitChannel; // publish an exit signal
	std::string _scaleChannel; // receive signal for stopping scale estimation
	std::string land_channel;
	std::string takeoff_channel;
	std::string toggleState_channel;

	// for navdata time-smoothing
	long _lastDroneTS;
	long _lastRosTS;
	long _droneRosTSOffset;

	// save last navinfo received for forwarding...
	ardrone_autonomy::Navdata _lastNavdataReceived;
	ardrone_autonomy::Navdata _currNavdataReceived;
	bool _navdata_first_set;

	PositionController _posController;
	PathPlanner _pathPlanner;

	vector<double> _navdataTsVec; // navdata system timestamp
	std::vector<ardrone_autonomy::Navdata> _navdataVec;
	vector<double> _imgTsVec; // image system timestamp
	std::vector<sensor_msgs::Imu> _imuVec;
	ros::Time _lastNavTS; // Navdata received time stamp
	double _timeBase;
	bool _recording; // start recording the video and imu data
	bool _busyUpdateVision;

	std::deque<tf2::Vector3> vel_world_vec;
	std::deque<double> vel_world_ts_vec;

	void getPredictedState(double currTs, double state[6]);

	struct tm y2k;

public:
	std::vector<monoslam_ros_test::filter_state> stateVec;
	std::vector<double> stateTSVec;
	double refTm;
	std::vector<double> dataVec;
	std::vector<double> stateVec_scaleFilter;
	std::vector<double> stateVec_control;
	std::vector<float> cmdVec;
	void logData(const char* timeStr );
};
#endif
