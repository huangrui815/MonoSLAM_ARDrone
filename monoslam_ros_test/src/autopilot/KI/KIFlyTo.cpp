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
 
#include "KIFlyTo.h"
#include "../AUTO_DroneController.h"
#include "../AUTO_ControlNode.h"


KIFlyTo::KIFlyTo(DronePosition checkpointP, 
		double stayTime,
		double maxControlFactorP,
		double initialReachedDistP,
		double stayWithinDistP,
		bool startHovering)
{
	stayTimeMs = (int)(1000*stayTime);
	maxControlFactor = maxControlFactorP;
	initialReachedDist = initialReachedDistP;
	stayWithinDist = stayWithinDistP;

	checkpoint = checkpointP;

	reachedAtClock = -1;
	reached = false;
	
	targetSet = false;

	isCompleted = false;

	_startHovering = startHovering;

	char buf[200];
	snprintf(buf,200,"goto %.2f %.2f %.2f %.2f", checkpointP._x, checkpointP._y, checkpointP._z, checkpointP._yaw);
	command = buf;
}


KIFlyTo::~KIFlyTo(void)
{
}


bool KIFlyTo::update(const monoslam_ros_test::filter_stateConstPtr statePtr)
{
	if(!targetSet)
		controller->setTarget(checkpoint);
	targetSet = true;

	if (_startHovering){
		node->sendControlToDrone(controller->update(statePtr));
		return false;	// not done yet (!)
	}

	// target reached?
	if(!isCompleted && reached && (node->getRelativeTime() - reachedAtClock) > stayTimeMs)
	{
		printf("checkpoint done!\n");
		isCompleted = true;
	}
	if(isCompleted)
	{
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}


	// get target dist:
	double diffs[3] = {statePtr->x - checkpoint._x,
			statePtr->y - checkpoint._y,
			statePtr->z - checkpoint._z};

	double diffYaw = statePtr->yaw - checkpoint._yaw;
	double diffDistSquared = diffs[0] * diffs[0] + diffs[1] * diffs[1] + diffs[2] * diffs[2];

	// if not reached yet, need to get within small radius to count.
	if(!reached && diffDistSquared < initialReachedDist * initialReachedDist && diffYaw*diffYaw < 25)
	{
		reached = true;
		reachedAtClock = node->getRelativeTime();
		printf("target reached initially!\n");
	}

	// if too far away again: revoke reached status...
	if(reached && (diffDistSquared > stayWithinDist * stayWithinDist || diffYaw*diffYaw > 25))
	{
		reached = false;
		printf("target lost again!\n");
	}

	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}
