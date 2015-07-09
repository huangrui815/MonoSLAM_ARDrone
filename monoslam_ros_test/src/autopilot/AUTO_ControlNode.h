#pragma once
 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __CONTROLNODE_H
#define __CONTROLNODE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "monoslam_ros_test/filter_state.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <dynamic_reconfigure/server.h>
#include "monoslam_ros_test/AutopilotParamsConfig.h"
#include "AUTO_DroneController.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include <ardrone_autonomy/Navdata.h>

class DroneKalmanFilter;
class MapView;
class PTAMWrapper;
class KIProcedure;

enum DroneState {UNKNOWN, INITED, LANDED, FLYING, HOVERING, TEST, TAKINGOFF, LANDING, LOOPING};
class ControlNode
{
private:
	ros::Subscriber dronepose_sub;
	ros::Publisher vel_pub;
	ros::Subscriber _comSub;
	ros::Publisher _comPub;
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher toggleState_pub;
	ros::Subscriber _navdataSub; // subscribe drone navdata
	ros::Subscriber _exitSub;


	ros::NodeHandle _controlNodeHandler;
	static pthread_mutex_t tum_ardrone_CS;

	bool _bRunning;

	// parameters
	int minPublishFreq;
	std::string control_channel;
	std::string dronepose_channel;
	std::string command_channel;
	std::string packagePath;
	std::string land_channel;
	std::string takeoff_channel;
	std::string toggleState_channel;
	std::string _tmBaseChannel; // resolved name for time base topic
	std::string _navdataChannel; // resolved name for navdata topic
	std::string _exitChannel;
	std::string _scaleChannel;

	// command queue & KI stuff
	std::deque<std::string> commandQueue;
	static pthread_mutex_t s_mutexCommandQueue;
	// this KI is currently responsible for setting the target etc.
	// if it is "Done", it is set to NULL,
	// if it is NULL, the next command will be popped and parsed from commandQueueu.
	KIProcedure* currentKI;

	// command parameters
	DronePosition parameter_referenceZero;
	double parameter_StayTime;
	double parameter_MaxControl;
	double parameter_InitialReachDist;
	double parameter_StayWithinDist;




	void popNextCommand(const monoslam_ros_test::filter_stateConstPtr statePtr);
	void reSendInfo();
	char buf[500];
	ControlCommand lastSentControl;
public:
	ControlNode(int droneId);
	~ControlNode();
	//void logData(const char* timeStr);


	// ROS message callbacks
	void droneposeCb(const monoslam_ros_test::filter_stateConstPtr statePtr);
	void comCb(const std_msgs::StringConstPtr str);
	void dynConfCb(monoslam_ros_test::AutopilotParamsConfig &config, uint32_t level);
	void tmBaseCb(const std_msgs::Int32::ConstPtr& tmBase);
	void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
	void exitCb(const std_msgs::EmptyConstPtr);

	// main pose-estimation loop
	void Loop();

	// writes a string message to "/tum_ardrone/com".
	// is thread-safe (can be called by any thread, but may block till other calling thread finishes)
	void publishCommand(std::string c);

	// control drone functions
	void sendControlToDrone(ControlCommand cmd);
	void sendLand();
	void sendTakeoff();
	void sendToggleState();

	int getRelativeTime(ros::Time stamp = ros::Time::now());

	// controller
	DroneController controller;
	ControlCommand hoverCommand;

	// logging stuff
	std::ofstream* logfileControl;
	static pthread_mutex_t logControl_CS;
	void toogleLogging();	// switches logging on or off.

	// other internals
	long lastControlSentMS;
	bool isControlling;

	ros::Publisher _scalePub;
	ros::Subscriber _tmBaseSub;
	ros::Publisher _tmBasePub;
	static pthread_mutex_t s_mutextmBase;
	int _timeBase;
	uint32_t _droneState;

	// For debug
	std::vector<int> _droneStateVec;
	std::vector<int> _stageVec;
	std::vector<monoslam_ros_test::filter_state> _filterStateVec;
};

#endif /* __CONTROLNODE_H */
