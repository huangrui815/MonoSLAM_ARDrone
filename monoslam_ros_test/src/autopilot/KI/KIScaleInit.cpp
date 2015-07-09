#include "KIScaleInit.h"
#include "../AUTO_DroneController.h"
#include "../AUTO_ControlNode.h"

KIScaleInit::KIScaleInit(){
	_bNextUp = false;
	_bNextDown = true;
	_bRising = _bDropping = false;
	_controlGain = 1;
	_numCycle = 0;
}

KIScaleInit::~KIScaleInit(){}


bool KIScaleInit::update(const monoslam_ros_test::filter_stateConstPtr statePtr)
{
	if (_numCycle == 2){
		node->_scalePub.publish(std_msgs::Empty());
		return true;
	}

	if ( node->_droneState == HOVERING && _bNextUp){
		_startTS = node->getRelativeTime(ros::Time::now());
		ROS_INFO("Ready to rise\n");
		ROS_INFO("Current time: %d\n", _startTS);
		_bNextUp = false;
		node->sendControlToDrone(ControlCommand(0,0,0,0.4*_controlGain));
		_bRising = true;
		return false;
	}
	else if (_bRising)
	{
		if( (node->getRelativeTime(ros::Time::now()) - _startTS) < 1500)
			node->sendControlToDrone(ControlCommand(0,0,0,0.4*_controlGain));
		else{
			node->sendControlToDrone(node->hoverCommand);
			_bRising = false;
			_bNextDown = true;
			_numCycle++;
		}
		return false;
	}
	else if ( node->_droneState == HOVERING && _bNextDown){
			_startTS = node->getRelativeTime(ros::Time::now());
			ROS_INFO("Ready to drop\n");
			ROS_INFO("Current time: %d\n", _startTS);
			_bNextDown = false;
			node->sendControlToDrone(ControlCommand(0,0,0,-0.4*_controlGain));
			_bDropping = true;
			return false;
	}
	else if (_bDropping){
		if( (node->getRelativeTime(ros::Time::now()) - _startTS) < 1500)
			node->sendControlToDrone(ControlCommand(0,0,0,-0.4*_controlGain));
		else{
			node->sendControlToDrone(node->hoverCommand);
			_bDropping = false;
			_bNextUp = true;
		}
		return false;
	}
	return false;
}
