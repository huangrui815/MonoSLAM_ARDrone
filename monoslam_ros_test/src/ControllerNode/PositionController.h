#ifndef _POSITION_CONTROLLER_H
#define _POSITION_CONTROLLER_H
#include "ros/ros.h"
#include "PIDController.h"
//#include "ardrone_package/DronePose.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <vector>
#include <deque>

using namespace std;

class PositionController{
public:
	PIDController _rollController;
	PIDController _pitchController;
	PIDController _yawController;
	PIDController _zController;

	ros::NodeHandle _ctrlNode;

	ros::Subscriber _poseSub;
	std::string _poseChannel;

	ros::Publisher _cmdPub;
	std::string _cmdChannel;

	ros::Subscriber _quitSub;
	std::string _quitChannel;

	ros::Subscriber _sysCmdSub;
	std::string _sysCmdChannel;
	std::deque<std::string> _sysCmdQueue;

	double sampleT;
	double _loopRate;

	double _xRef, _yRef, _zRef, _yawRef;
	float ctrlRoll, ctrlPitch, ctrlZRate, ctrlYawRate;
	bool _bRefSet;

	bool _bTerminate;

//	ardrone_package::DronePose _currPose;

	PositionController(double sampleT = 0.02);
	~PositionController();

	void initTopics();

	void setRef(double& x_ref, double& y_ref, double& z_ref, double& yaw_ref);
	void computePID(double errX, double errY, double errZ, double errYaw);
//	void poseCb(const ardrone_package::DronePoseConstPtr posePtr);
	void quitCb(const std_msgs::EmptyConstPtr);
	void sysCmdCb(const std_msgs::StringConstPtr str);
	geometry_msgs::Twist sendCmd();
	void setSampleT(double& sampleT);

	void loop();

	// For debug
public:
//	vector<ardrone_package::DronePose> poseVec;
	vector<double> cmdVec;
	vector<double> errVec;

	void logData(char* filePath);
};
#endif
