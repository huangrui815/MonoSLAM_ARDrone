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
#ifndef __DRONECONTROLLER_H
#define __DRONECONTROLLER_H
 

#include <queue>
#include "geometry_msgs/Twist.h"
#include "monoslam_ros_test/filter_state.h"

class ControlNode;

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


struct DronePosition
{
public:
	double _yaw;
	double _x,_y,_z;
	inline DronePosition(double pos[3], double yaw)
		: _yaw(yaw) {_x = pos[0]; _y = pos[1]; _z = pos[2];}
	inline DronePosition(){ _yaw=0; _x = _y = _z = 0;}
	inline DronePosition(double x, double y, double z, double yaw):_x(x),_y(y),_z(z),_yaw(yaw){}
};

class DroneController
{
private:
	ControlCommand lastSentControl;
	
	// currentTarget.
	DronePosition _target;
	bool _targetValid;

	// used for integral term
	double targetNew[4];	// 0=target has been reached before
								// 1=target is new

	// need to keep track of integral terms
	double i_term[4];
	double last_err[4];
	double speedAverages[4];

	double lastTimeStamp;
	double targetSetAtClock;
	ControlCommand hoverCommand;



	// filled with info (on update)
	bool  ptamIsGood;
	double scaleAccuracy;
	void calcControl(double new_err[4], double d_error[4], double yaw);

public:

	// generates and sends a new control command to the drone, based on the currently active command ant the drone's position.
	ControlCommand update(monoslam_ros_test::filter_stateConstPtr);

	ControlNode* _controlNode;

	// for logging, gets filled with recent infos on control.
	double logInfo[28];

	// adds a waypoint
	void setTarget(DronePosition newTarget);
	void clearTarget();
	DronePosition getCurrentTarget();
	ControlCommand getLastControl();

	// gets last error
	void getLastErr(double lastErr[4]);

	DroneController(void);
	~DroneController(void);





	// PID control parameters. settable via dynamic_reconfigure
	// target is where i want to get to.
	// pose and yaw are where i am.
	double max_yaw;
	double max_rp;
	double max_gaz_rise;
	double max_gaz_drop;

	double rise_fac;
	double agressiveness;

	double Ki_yaw;
	double Kd_yaw;
	double Kp_yaw;

	double Ki_gaz;
	double Kd_gaz;
	double Kp_gaz;

	double Ki_rp;
	double Kd_rp;
	double Kp_rp;

	//for debug
	std::vector<DronePosition> errVec;
	std::vector<ControlCommand> commandVec;

};
#endif /* __DRONECONTROLLER_H */

