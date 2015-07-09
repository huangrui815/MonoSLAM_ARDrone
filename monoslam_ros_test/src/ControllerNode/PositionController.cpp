#include "PositionController.h"


PositionController::PositionController(double sampleT){
	ROS_INFO("Create Controller Node...\n");
	_rollController.pGain = 0.000247;
	_rollController.dGain = 0.000241;
	_rollController.iGain = 1.595655 * 0.00001;
	_rollController.N = 186.795788820061;

//	_rollController.pGain = 0.000144;
//	_rollController.dGain = 0.000267;
//	_rollController.iGain = 6.36200 * 0.000001;
//	_rollController.N = 203.644790484222;

	_pitchController.pGain = -0.000144;
	_pitchController.dGain = -0.000267;
	_pitchController.iGain = -6.36200 * 0.000001;
	_pitchController.N = 203.644790484222;

	_yawController.pGain = 6.56418083379724;
	_yawController.dGain = 0.0;
	_yawController.iGain = 0.0;
	_yawController.N = 0.0;

//	_zController.pGain = 0.0013300809554015;
//	_zController.dGain = -9.5784686466965e-06;
//	_zController.iGain = 1.41426761093392e-05;
//	_zController.N = 138.86154504042;

//	_rollController.pGain = 0.5 * 0.001; //0.000247;
//	_rollController.dGain = 0.35 * 0.001; //0.000241;
//	_rollController.iGain = 0.0; //1.595655 * 0.00001;
//	_rollController.N = 186.795788820061;
//
//	_pitchController.pGain = -0.5 * 0.001;
//	_pitchController.dGain = -0.35 * 0.001;
//	_pitchController.iGain = -0.0;
//	_pitchController.N = 203.644790484222;
//
//	_yawController.pGain = 6.56418083379724;
//	_yawController.dGain = 0.0;
//	_yawController.iGain = 0.0;
//	_yawController.N = 0.0;
//
	_zController.pGain = 0.6 * 0.001; //0.0013300809554015;
	_zController.dGain = 0.1 * 0.001; //-9.5784686466965e-06;
	_zController.iGain = 0.001 * 0.001; //1.41426761093392e-05;
	_zController.N = 138.86154504042;

	this->sampleT = sampleT;
	_rollController.sampleT = sampleT;
	_pitchController.sampleT = sampleT;
	_yawController.sampleT = sampleT;
	_zController.sampleT = sampleT;
	_loopRate = 1.0 / sampleT;

	_xRef = _yRef = _zRef = _yawRef = 0.0;
	_bRefSet = false;
	_bTerminate = false;

	_sysCmdQueue.clear();
	initTopics();
}

PositionController::~PositionController(){
}

void PositionController::initTopics(){
//	_poseChannel = _ctrlNode.resolveName("ardrone/pose");
//	_poseSub = _ctrlNode.subscribe(_poseChannel, 10, &ControllerNode::poseCb, this);
//
//	_cmdChannel = _ctrlNode.resolveName("ardrone/cmd_vel");
//	_cmdPub	   = _ctrlNode.advertise<geometry_msgs::Twist>(_cmdChannel,100);
//
//	_quitChannel = _ctrlNode.resolveName("ardrone/quit");
//	_quitSub     = _ctrlNode.subscribe(_quitChannel, 10, &ControllerNode::quitCb, this);
//
//	_sysCmdChannel = _ctrlNode.resolveName("ardrone/sysCmd");
//	_sysCmdSub     = _ctrlNode.subscribe(_sysCmdChannel, 10, &ControllerNode::sysCmdCb, this);
}

void PositionController::setRef(double& x_ref, double& y_ref, double& z_ref, double& yaw_ref){
	_xRef = x_ref;
	_yRef = y_ref;
	_zRef = z_ref;
	_yawRef = yaw_ref;
	_bRefSet = true;
}

geometry_msgs::Twist PositionController::sendCmd()
{
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = -ctrlYawRate;
	cmdT.linear.z = ctrlZRate;
	cmdT.linear.x = -ctrlPitch;
	cmdT.linear.y = -ctrlRoll;

	cmdT.angular.x = cmdT.angular.y = 0.0;
	return cmdT;
//	_cmdPub.publish(cmdT);
}

void PositionController::setSampleT(double& sampleT){
	this->sampleT = sampleT;
	_rollController.setSampleT(sampleT);
	_pitchController.setSampleT(sampleT);
	_yawController.setSampleT(sampleT);
	_zController.setSampleT(sampleT);
}

//void ControllerNode::poseCb(const ardrone_package::DronePoseConstPtr posePtr){
//	_currPose = *posePtr;
//	if (!_sysCmdQueue.empty()){
//		std::string command = _sysCmdQueue.front();
//		ROS_INFO("executing command: %s",command.c_str());
//		float parameters[10];
//		if(sscanf(command.c_str(),"setReference %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
//		{
//			_xRef = _currPose.x + parameters[0];
//			_yRef = _currPose.y + parameters[1];
//			_zRef = _currPose.z + parameters[2];
//			_yawRef = _currPose.yaw + parameters[3];
//			_bRefSet = true;
//			ROS_INFO("Target set: %f %f %f %f\n", _xRef, _yRef, _zRef, _yawRef);
//		}
//		_sysCmdQueue.pop_front();
//	}
//}

void PositionController::quitCb(const std_msgs::EmptyConstPtr){
	_bTerminate = true;
}

void PositionController::sysCmdCb(const std_msgs::StringConstPtr str){
	ROS_INFO("Received command: %s", str->data.c_str());
	if(str->data.length() > 2 && str->data.substr(0,2) == "c "){
		std::string cmd =str->data.substr(2,str->data.length()-2);
		_sysCmdQueue.push_back(cmd);
	}
}

void PositionController::computePID(double errX, double errY, double errZ, double errYaw){
	ctrlRoll = _rollController.computeCtrl(errY);
	ctrlPitch = _pitchController.computeCtrl(errX);
	ctrlZRate = _zController.computeCtrl(errZ);
	ctrlYawRate = _yawController.computeCtrl(errYaw);
//	sendCmd(ctrlRoll, ctrlPitch, ctrlYawRate, ctrlZRate);
	//ROS_INFO("Sending command...\n");
}

void PositionController::loop(){
//	ros::Rate loopRate(_loopRate);
//
//	while(_ctrlNode.ok() && !_bTerminate){
//		ros::spinOnce();
//		if (_bRefSet){
////			ardrone_package::DronePose currPose = _currPose;
////			double x = currPose.x;
////			double y = currPose.y;
////			double z = currPose.z;
////			double yaw = currPose.yaw;
//
//			double x_err = _xRef - x;
//			double y_err = _yRef - y;
//
//			double x_err_body = x_err * cos(yaw) + y_err * sin(yaw);
//			double y_err_body = -x_err * sin(yaw) + y_err * cos(yaw);
//			float ctrlRoll = _rollController.computeCtrl(y_err_body);
//			float ctrlPitch = _pitchController.computeCtrl(x_err_body);
//			float ctrlZRate = _zController.computeCtrl(_zRef - z);
//			float ctrlYawRate = _yawController.computeCtrl(_yawRef - yaw);
//			sendCmd(ctrlRoll, ctrlPitch, ctrlYawRate, ctrlZRate);
//			//ROS_INFO("Sending command...\n");
////			poseVec.push_back(currPose);
////			cmdVec.push_back(ctrlRoll);
////			cmdVec.push_back(ctrlPitch);
////			cmdVec.push_back(ctrlZRate);
////			cmdVec.push_back(ctrlYawRate);
////			errVec.push_back(x_err_body);
////			errVec.push_back(y_err_body);
////			errVec.push_back(_zRef - z);
////			errVec.push_back(_yawRef - yaw);
//		}
//		loopRate.sleep();
//	}
//	logData("/home/rui/Record/controlRes.txt");
}


void PositionController::logData(char* filePath){
//	FILE* pFile;
//	pFile = fopen(filePath, "w");
//	for (int i = 0, j = 0; i < poseVec.size(); i++, j = j+4)
//		fprintf(pFile, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
//				poseVec[i].x, poseVec[i].y, poseVec[i].z, poseVec[i].roll, poseVec[i].pitch, poseVec[i].yaw,
//				cmdVec[j], cmdVec[j+1], cmdVec[j+2], cmdVec[j+3],
//				errVec[j], errVec[j+1], errVec[j+2], errVec[j+3]);
//	fclose(pFile);
}






