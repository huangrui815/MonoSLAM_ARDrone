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

#include "KIMyAutoInit.h"
#include "../AUTO_DroneController.h"
#include "../AUTO_ControlNode.h"

KIMyAutoInit::KIMyAutoInit(bool resetMap, int imoveTimeMS, int iwaitTimeMS, int reachHeightMS, float controlGain, bool takeoff)
{
	stage = NONE;
	this->resetMap = resetMap;
	moveTimeMS = imoveTimeMS;
	waitTimeMS = iwaitTimeMS;
	this->reachHeightMS = reachHeightMS;
	this->_controlGain = controlGain;

	nextGoUp = false;
	stageStarted = false;
//
//	if(!takeoff)
//		stage = WAIT_FOR_FIRST;

	char buf[200];
	if(resetMap)
		snprintf(buf,200,"autoInit %d %d", imoveTimeMS, iwaitTimeMS);
	else
		snprintf(buf,200,"takeoff");

	command = buf;
}


KIMyAutoInit::~KIMyAutoInit(void)
{
}


bool KIMyAutoInit::update(const monoslam_ros_test::filter_stateConstPtr statePtr)
{
//	std::cout << "node->_droneState: " << node->_droneState << std::endl;
//	std::cout << "stage: " << stage << std::endl;

//	node->_stageVec.push_back(stage);
//	node->_droneStateVec.push_back(node->_droneState);

	// when the drone is landed, start to take off
	if (node->_droneState == LANDED && stage == NONE){
		ROS_INFO("ardrone is landed\n");
		node->sendTakeoff();
		stage = STARTED;
		nextGoUp = true;
		stageStarted = node->getRelativeTime(ros::Time::now());
		return false;
	}
	else if ( node->_droneState == HOVERING && stage == STARTED){
		if (node->getRelativeTime(ros::Time::now()) - stageStarted > 2000){
			// Take the first keyframe
			ROS_INFO("take the first keyframe\n");
			node->publishCommand("p startinit");
			stage = TOOK_FIRST;
			stageStarted = node->getRelativeTime(ros::Time::now());
			return false;
		}
	}
	else if ( stage == TOOK_FIRST){
		ROS_INFO("Ready to rise\n");
		ROS_INFO("Current time: %d, stageStarted time: %d, moveTimeMS: %d\n",
				node->getRelativeTime(ros::Time::now()), stageStarted, moveTimeMS);

		if( (node->getRelativeTime(ros::Time::now()) - stageStarted) < moveTimeMS)
		{
			if(nextGoUp)
				node->sendControlToDrone(ControlCommand(0,0,0,0.4*_controlGain));
			else
				node->sendControlToDrone(ControlCommand(0,0,0,-0.3*_controlGain));
			return false;
		}
		else
		{
			stage = WAIT_FOR_SECOND;
			node->sendControlToDrone(node->hoverCommand);
			stageStarted = node->getRelativeTime(ros::Time::now());
			return false;
		}
	}
	else if (stage == WAIT_FOR_SECOND){
		if (node->_droneState == HOVERING){
			if (node->getRelativeTime(ros::Time::now()) - stageStarted > waitTimeMS){
				// Take the second keyframe
				std::cout << "take the second keyframe\n";
				node->publishCommand("p startinit");
				stage = DONE;
				return true;
			}
			else{
				node->sendControlToDrone(node->hoverCommand);
				ROS_INFO("Wait for second\n");
			}
		}
	}
	//std::cout << "ardrone is in unknown state\n";
	return false;
}
