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
 
 
 
#include "AUTO_ControlNode.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/package.h>

#include "geometry_msgs/Twist.h"
#include "monoslam_ros_test/filter_state.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sys/stat.h>
#include <string>

// include KI's
#include "KI/KIAutoInit.h";
#include "KI/KIMyAutoInit.h"
#include "KI/KIScaleInit.h"
#include "KI/KIFlyTo.h";
#include "KI/KILand.h";
#include "KI/KIProcedure.h"


using namespace std;

pthread_mutex_t ControlNode::logControl_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ControlNode::s_mutextmBase;

ControlNode::ControlNode(int droneId)
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
		droneIdStr = "ardrone" + droneIdStr;
	}

	_navdataChannel = _controlNodeHandler.resolveName(droneTopicPrefixStr + "/navdata");
    control_channel = _controlNodeHandler.resolveName(droneTopicPrefixStr + "/cmd_vel");
    dronepose_channel = _controlNodeHandler.resolveName(droneTopicPrefixStr + "/predictedPose");
    command_channel = _controlNodeHandler.resolveName(droneTopicPrefixStr + "/com");
    takeoff_channel = _controlNodeHandler.resolveName(droneTopicPrefixStr + "/takeoff");
    land_channel = _controlNodeHandler.resolveName(droneTopicPrefixStr + "/land");
    toggleState_channel = _controlNodeHandler.resolveName(droneTopicPrefixStr + "/reset");
    _tmBaseChannel = _controlNodeHandler.resolveName(droneTopicPrefixStr + "/tmBase");
    _exitChannel = _controlNodeHandler.resolveName(droneTopicPrefixStr + "/exit");
    _scaleChannel = _controlNodeHandler.resolveName(droneTopicPrefixStr + "/scaleInit");
	packagePath = ros::package::getPath("monoslam_ros_test");

	std::string val;
	float valFloat;

	ros::param::get("~minPublishFreq", val);
	if(val.size()>0)
		sscanf(val.c_str(), "%f", &valFloat);
	else
		valFloat = 110;
	minPublishFreq = valFloat;
	cout << "set minPublishFreq to " << valFloat << "ms"<< endl;


	// other internal vars
	logfileControl = 0;
	hoverCommand.gaz = hoverCommand.pitch = hoverCommand.roll = hoverCommand.yaw = 0;
	lastControlSentMS = 0;

	// channels
	_navdataSub       = _controlNodeHandler.subscribe(_navdataChannel, 10, &ControlNode::navdataCb, this);
	dronepose_sub = _controlNodeHandler.subscribe(dronepose_channel, 10, &ControlNode::droneposeCb, this);
	vel_pub	   = _controlNodeHandler.advertise<geometry_msgs::Twist>(control_channel,1);
	_comPub	   = _controlNodeHandler.advertise<std_msgs::String>(command_channel,50);
	_comSub	   = _controlNodeHandler.subscribe(command_channel,50, &ControlNode::comCb, this);
	takeoff_pub	   = _controlNodeHandler.advertise<std_msgs::Empty>(takeoff_channel,1);
	land_pub	   = _controlNodeHandler.advertise<std_msgs::Empty>(land_channel,1);
	toggleState_pub	   = _controlNodeHandler.advertise<std_msgs::Empty>(toggleState_channel,1);

	_tmBasePub = _controlNodeHandler.advertise<std_msgs::Int32>(_tmBaseChannel,1);
	_tmBaseSub = _controlNodeHandler.subscribe(_tmBaseChannel,1,&ControlNode::tmBaseCb, this);

	_exitSub = _controlNodeHandler.subscribe(_exitChannel,1, &ControlNode::exitCb, this);
	_scalePub = _controlNodeHandler.advertise<std_msgs::Empty>(_scaleChannel,1);

	// internals

	parameter_referenceZero = DronePosition(0,0,0,0);
	parameter_MaxControl = 1;
	parameter_InitialReachDist = 0.2;
	parameter_StayWithinDist = 0.5;
	parameter_StayTime = 2;
	//isControlling = false;
	isControlling = true;
	currentKI = NULL;
	lastSentControl = ControlCommand(0,0,0,0);
	_timeBase = 0;

	// create controller
	controller = DroneController();
	controller._controlNode = this;
	_bRunning = true;
}

ControlNode::~ControlNode()
{
	string dirPath("/home/rui/workspace/myROS/debug_output");
	char filePath[256];
	sprintf(filePath, "%s/err.txt", dirPath.c_str());
	FILE* pFile;
	pFile = fopen(filePath,"w");
	std::vector<DronePosition>::iterator errIter;
	for (errIter = controller.errVec.begin(); errIter != controller.errVec.end(); errIter++){
		fprintf(pFile, "%f %f %f %f\n", errIter->_x, errIter->_y, errIter->_z, errIter->_yaw);
	}
	fclose(pFile);

	sprintf(filePath, "%s/command.txt", dirPath.c_str());
	pFile = fopen(filePath,"w");
	std::vector<ControlCommand>::iterator commandIter;
	for (commandIter = controller.commandVec.begin(); commandIter != controller.commandVec.end(); commandIter++){
		fprintf(pFile, "%f %f %f %f\n", commandIter->roll, commandIter->pitch, commandIter->yaw, commandIter->gaz);
	}
	fclose(pFile);
}

//void ControlNode::logData(const char* timeStr){
//	using namespace std;
//	char dirPath[256];
//	sprintf(dirPath, "%s/%s", LOG_PATH, timeStr);
//
//	#ifdef WIN32
//		CreateDirectoryA(dirPath,NULL);
//	#else
//		mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
//	#endif
//
//	char filePath[256];
//	sprintf(filePath, "%s/err.txt", dirPath);
//	FILE* pFile;
//	pFile = fopen(filePath,"w");
//	std::vector<DronePosition>::iterator errIter;
//	for (errIter = controller.errVec.begin(); errIter != controller.errVec.end(); errIter++){
//		fprintf(pFile, "%f %f %f %f\n", errIter->_x, errIter->_y, errIter->_z, errIter->_yaw);
//	}
//	fclose(pFile);
//
//	sprintf(filePath, "%s/command.txt", dirPath);
//	pFile = fopen(filePath,"w");
//	std::vector<ControlCommand>::iterator commandIter;
//	for (commandIter = controller.commandVec.begin(); commandIter != controller.commandVec.end(); commandIter++){
//		fprintf(pFile, "%f %f %f %f\n", commandIter->roll, commandIter->pitch, commandIter->yaw, commandIter->gaz);
//	}
//	fclose(pFile);
//}

pthread_mutex_t ControlNode::s_mutexCommandQueue = PTHREAD_MUTEX_INITIALIZER;
void ControlNode::droneposeCb(const monoslam_ros_test::filter_stateConstPtr statePtr)
{
	// do controlling
	pthread_mutex_lock(&s_mutexCommandQueue);
	//ROS_INFO("filter state: %f %f %f %f %f %f %f %f %f %f\n", statePtr->x, statePtr->y, statePtr->z, statePtr->roll, statePtr->pitch, statePtr->yaw,
	//		statePtr->dx, statePtr->dy, statePtr->dz, statePtr->dyaw);

	//_filterStateVec.push_back(*statePtr);
	// as long as no KI present:
	// pop next KI (if next KI present).
	// ROS_INFO("Current command size: %d\n", commandQueue.size());
	while(currentKI == NULL && commandQueue.size() > 0)
		popNextCommand(statePtr);

	// if there is no current KI now, we obviously have no current goal -> send drone hover
	if(currentKI != NULL)
	{
		//std::cout << "currentKI != NULL\n";
		// let current KI control.
		if(currentKI->update(statePtr))
		{
			delete currentKI;
			currentKI = NULL;
		}
	}
	else if(isControlling)
	{
		sendControlToDrone(hoverCommand);
		ROS_WARN("Autopilot is Controlling, but there is no KI -> sending HOVER");
	}


	pthread_mutex_unlock(&s_mutexCommandQueue);
}

// pops next command(s) from queue (until one is found thats not "done" yet).
// assumes propery of command queue lock exists (!)
void ControlNode::popNextCommand(const monoslam_ros_test::filter_stateConstPtr statePtr)
{
	// should actually not happen., but to make sure:
	// delete existing KI.
	if(currentKI != NULL)
	{
		delete currentKI;
		currentKI = NULL;
	}

	// read next command.
	while(currentKI == NULL && commandQueue.size() > 0)
	{
		std::string command = commandQueue.front();
		commandQueue.pop_front();
		bool commandUnderstood = false;

		// print me
		ROS_INFO("executing command: %s",command.c_str());


		int p;
		char buf[100];
		float parameters[10];

		// replace macros
		if((p = command.find("$POSE$")) != std::string::npos)
		{
			snprintf(buf,100, "%.3f %.3f %.3f %.3f",statePtr->x,statePtr->y,statePtr->z,statePtr->yaw);
			command.replace(p,6,buf);
		}
		if((p = command.find("$REFERENCE$")) != std::string::npos)
		{
			snprintf(buf,100, "%.3f %.3f %.3f %.3f",parameter_referenceZero._x,parameter_referenceZero._y,parameter_referenceZero._z,parameter_referenceZero._yaw);
			command.replace(p,11,buf);
		}

		// -------- commands -----------
		// autoInit
		if(sscanf(command.c_str(),"autoInit %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			currentKI = new KIAutoInit(true,parameters[0],parameters[1],parameters[2],parameters[3],true);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;
		}

		else if (strcmp(command.c_str(), "autoinit") == 0){
			std::cout << "start my autoinit\n";
			currentKI = new KIMyAutoInit();
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;
		}

		else if (strcmp(command.c_str(), "scaleInit") == 0){
			std::cout << "start my scale init\n";
			currentKI = new KIScaleInit();
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;
		}

		else if(sscanf(command.c_str(),"autoTakeover %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			currentKI = new KIAutoInit(true,parameters[0],parameters[1],parameters[2],parameters[3],false);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;
		}

		// takeoff
		else if(command == "takeoff")
		{
			currentKI = new KIAutoInit(false);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;
		}

		// setOffset
		else if(sscanf(command.c_str(),"setReference %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			parameter_referenceZero = DronePosition(parameters[0],parameters[1],parameters[2],parameters[3]);
			commandUnderstood = true;
		}

		// setMaxControl
		else if(sscanf(command.c_str(),"setMaxControl %f",&parameters[0]) == 1)
		{
			parameter_MaxControl = parameters[0];
			commandUnderstood = true;
		}

		// setInitialReachDist
		else if(sscanf(command.c_str(),"setInitialReachDist %f",&parameters[0]) == 1)
		{
			parameter_InitialReachDist = parameters[0];
			commandUnderstood = true;
		}

		// setStayWithinDist
		else if(sscanf(command.c_str(),"setStayWithinDist %f",&parameters[0]) == 1)
		{
			parameter_StayWithinDist = parameters[0];
			commandUnderstood = true;
		}

		// setStayTime
		else if(sscanf(command.c_str(),"setStayTime %f",&parameters[0]) == 1)
		{
			parameter_StayTime = parameters[0];
			commandUnderstood = true;
		}

		// goto
		else if(sscanf(command.c_str(),"goto %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			currentKI = new KIFlyTo(
				DronePosition(
				parameters[0] + parameter_referenceZero._x,
				parameters[1] + parameter_referenceZero._y,
				parameters[2] + parameter_referenceZero._z,
				parameters[3] + parameter_referenceZero._yaw),
				parameter_StayTime,
				parameter_MaxControl,
				parameter_InitialReachDist,
				parameter_StayWithinDist
				);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;

		}

		// moveBy
		else if(sscanf(command.c_str(),"moveBy %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			currentKI = new KIFlyTo(
				DronePosition(
				parameters[0] + controller.getCurrentTarget()._x,
				parameters[1] + controller.getCurrentTarget()._y,
				parameters[2] + controller.getCurrentTarget()._z,
				parameters[3] + controller.getCurrentTarget()._yaw),
				parameter_StayTime,
				parameter_MaxControl,
				parameter_InitialReachDist,
				parameter_StayWithinDist
				);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;

		}

		// moveByRel
		else if(sscanf(command.c_str(),"moveByRel %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			bool startHovering;
			if ( parameters[0] == 0 && parameters[1] == 0 && parameters[2] == 0 && parameters[3] == 0)
				startHovering = true;
			else
				startHovering = false;

			currentKI = new KIFlyTo(
				DronePosition(
				parameters[0]+statePtr->x,parameters[1]+statePtr->y,parameters[2]+statePtr->z,
					parameters[3] + statePtr->yaw),
				parameter_StayTime,
				parameter_MaxControl,
				parameter_InitialReachDist,
				parameter_StayWithinDist,
				startHovering
				);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;

		}

		// land
		else if(command == "land")
		{
			currentKI = new KILand();
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;
		}

		// setScaleFP
		else if(command == "lockScaleFP")
		{
			publishCommand("p lockScaleFP");
			commandUnderstood = true;
		}

		if(!commandUnderstood)
			ROS_INFO("unknown command, skipping!");
	}

}

void ControlNode::comCb(const std_msgs::StringConstPtr str)
{
	// only handle commands with prefix c
	if(str->data.length() > 2 && str->data.substr(0,2) == "c ")
	{
		std::string cmd =str->data.substr(2,str->data.length()-2);

		if(cmd.length() == 4 && cmd.substr(0,4) == "stop")
		{
			isControlling = false;
			publishCommand("u l Autopilot: Stop Controlling");
			ROS_INFO("STOP CONTROLLING!");
		}
		else if(cmd.length() == 5 && cmd.substr(0,5) == "start")
		{
			isControlling = true;
			publishCommand("u l Autopilot: Start Controlling");
			ROS_INFO("START CONTROLLING!");
		}
		else if(cmd.length() == 13 && cmd.substr(0,13) == "clearCommands")
		{
			pthread_mutex_lock(&s_mutexCommandQueue);
			commandQueue.clear();						// clear command queue.
			controller.clearTarget();					// clear current controller target
			if(currentKI != NULL) delete currentKI;	// destroy & delete KI.
			currentKI = NULL;
			pthread_mutex_unlock(&s_mutexCommandQueue);

			publishCommand("u l Autopilot: Cleared Command Queue");
			ROS_INFO("Cleared Command Queue!");
		}
		else
		{
			pthread_mutex_lock(&s_mutexCommandQueue);
			commandQueue.push_back(cmd);
			pthread_mutex_unlock(&s_mutexCommandQueue);
		}
	}

	// global command: toggle log
	if(str->data.length() == 9 && str->data.substr(0,9) == "toggleLog")
	{
		this->toogleLogging();
	}
}

void ControlNode::Loop()
{
	ros::Time last = ros::Time::now();
	ros::Time lastStateUpdate = ros::Time::now();

	while (_controlNodeHandler.ok() && _bRunning)
	{

		// -------------- 1. spin for 50ms, do main controlling part here. ---------------
		while((ros::Time::now() - last) < ros::Duration(minPublishFreq / 1000.0))
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(minPublishFreq / 1000.0 - (ros::Time::now() - last).toSec()));
		last = ros::Time::now();


		// -------------- 2. send hover (maybe). ---------------
		if(isControlling && getRelativeTime(ros::Time::now()) - lastControlSentMS > minPublishFreq)
		{
			sendControlToDrone(hoverCommand);
			ROS_WARN("Autopilot enabled, but no estimated pose received - sending HOVER.");
		}

		// -------------- 2. update info. ---------------
		if((ros::Time::now() - lastStateUpdate) > ros::Duration(0.4))
		{
			reSendInfo();
			lastStateUpdate = ros::Time::now();
		}
	}

	ROS_INFO("Control node exits\n");
}
void ControlNode::dynConfCb(monoslam_ros_test::AutopilotParamsConfig &config, uint32_t level)
{
	controller.Ki_gaz = config.Ki_gaz;
	controller.Kd_gaz = config.Kd_gaz;
	controller.Kp_gaz = config.Kp_gaz;

	controller.Ki_rp = config.Ki_rp;
	controller.Kd_rp = config.Kd_rp;
	controller.Kp_rp = config.Kp_rp;

	controller.Ki_yaw = config.Ki_yaw;
	controller.Kd_yaw = config.Kd_yaw;
	controller.Kp_yaw = config.Kp_yaw;

	controller.max_gaz_drop = config.max_gaz_drop;
	controller.max_gaz_rise = config.max_gaz_rise;
	controller.max_rp = config.max_rp;
	controller.max_yaw = config.max_yaw;
	controller.agressiveness = config.agressiveness;
	controller.rise_fac = config.rise_fac;
}

void ControlNode::tmBaseCb(const std_msgs::Int32::ConstPtr& tmBase){
	pthread_mutex_lock(&s_mutextmBase);
	_timeBase = tmBase->data;
	ROS_INFO("set time base to %d", _timeBase);
	pthread_mutex_unlock(&s_mutextmBase);
}

void ControlNode::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr){
//	if ( navdataPtr->state == UNKNOWN)
//		std::cout << " ARDrone in unknown state\n";
//	else if ( navdataPtr->state == INITED)
//		std::cout << " ARDrone is inited\n";
//	else if ( navdataPtr->state == LANDED)
//		std::cout << " ARDrone has landed\n";
//	else if ( navdataPtr->state == FLYING)
//		std::cout << " ARDrone is flying\n";
//	else if ( navdataPtr->state == HOVERING)
//		std::cout << " ARDrone is hovering\n";
//	else if ( navdataPtr->state == TEST)
//		std::cout << " ARDrone is in test mode\n";
//	else if ( navdataPtr->state == TAKINGOFF)
//		std::cout << " ARDrone is taking off\n";
//	else if ( navdataPtr->state == LANDING)
//		std::cout << " ARDrone is landing\n";
//	else if ( navdataPtr->state == LOOPING)
//		std::cout << " ARDrone is in looping mode\n";
//
//	_droneStateVec.push_back(navdataPtr->state);
	_droneState = navdataPtr->state;
}

void ControlNode::exitCb(std_msgs::EmptyConstPtr){
	_bRunning = false;
}
pthread_mutex_t ControlNode::tum_ardrone_CS = PTHREAD_MUTEX_INITIALIZER;
void ControlNode::publishCommand(std::string c)
{
	std_msgs::String s;
	s.data = c.c_str();
	pthread_mutex_lock(&tum_ardrone_CS);
	_comPub.publish(s);
	pthread_mutex_unlock(&tum_ardrone_CS);
}

void ControlNode::toogleLogging()
{
	// logging has yet to be integrated.
}

void ControlNode::sendControlToDrone(ControlCommand cmd)
{
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = -cmd.yaw;
	cmdT.linear.z = cmd.gaz;
	cmdT.linear.x = -cmd.pitch;
	cmdT.linear.y = -cmd.roll;

	// assume that while actively controlling, the above for will never be equal to zero, so i will never hover.
	cmdT.angular.x = cmdT.angular.y = 0;

	if(isControlling)
	{
		vel_pub.publish(cmdT);
		lastSentControl = cmd;
	}

	lastControlSentMS = getRelativeTime(ros::Time::now());
}

void ControlNode::sendLand()
{
	if(isControlling)
		land_pub.publish(std_msgs::Empty());
}
void ControlNode::sendTakeoff()
{
	if(isControlling)
		takeoff_pub.publish(std_msgs::Empty());
}
void ControlNode::sendToggleState()
{
	if(isControlling)
		toggleState_pub.publish(std_msgs::Empty());
}
void ControlNode::reSendInfo()
{
	/*
	Idle / Controlling (Queue: X)
	Current:
	Next:
	Target: X,X,X,X
	Error: X,X,X,X
	*/

	DronePosition p = controller.getCurrentTarget();
	double e[4];
	controller.getLastErr(e);
	double ea = sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
	snprintf(buf,500,"u c %s (Queue: %d)\nCurrent: %s\nNext: %s\nTarget: (%.2f,  %.2f,  %.2f), %.1f\nError: (%.2f,  %.2f,  %.2f), %.1f (|.| %.2f)\nCont.: r %.2f, p %.2f, g %.2f, y %.2f",
			isControlling ? "Controlling" : "Idle",
			(int)commandQueue.size(),
			currentKI == NULL ? "NULL" : currentKI->command.c_str(),
			commandQueue.size() > 0 ? commandQueue.front().c_str() : "NULL",
			p._x,p._y,p._z,p._yaw,
			e[0],e[1],e[2],e[3], ea,
			lastSentControl.roll, lastSentControl.pitch, lastSentControl.gaz, lastSentControl.yaw);

	publishCommand(buf);
}

int ControlNode::getRelativeTime(ros::Time stamp){
	if(_timeBase == 0)
		return -1;
	int mss = (stamp.sec - _timeBase) * 1000 + stamp.nsec/1000000;

	if(mss < 0)
		std::cout << "ERROR: negative timestamp..."<< std::endl;
	return mss;
}
