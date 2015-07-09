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
#include "ros/package.h"
#include "boost/thread.hpp"
#include <signal.h>

// this global var is used in getMS(ros::Time t) to convert to a consistent integer timestamp used internally pretty much everywhere.
// kind of an artifact from Windows-Version, where only that was available / used.
unsigned int ros_header_timestamp_base = 0;


int checkDroneId(int argc, char** argv){
	char *drone_ip_address = NULL;

	argc--; argv++;
	while( argc && *argv[0] == '-' )
	{
		if( !strcmp(*argv, "-ip") && ( argc > 1 ) )
		{
			drone_ip_address = *(argv+1);
			printf("Using custom ip address %s\n",drone_ip_address);
			argc--; argv++;
		}
		argc--; argv++;
	}
	if(drone_ip_address)
   {
	 printf("===================+> %s\n", drone_ip_address);
	 std::string ipStr(drone_ip_address);
	 std::string droneIdStr;
	 std::string ipTemplate("192.168.1.");
	 std::size_t found = ipStr.find(ipTemplate);
	 if (found != std::string::npos){
		std::cout << "Find IP template: " << found << '\n';
		droneIdStr.assign(ipStr, ipTemplate.length(), std::string::npos);
		std::cout << "Find Drone ID: " << droneIdStr << '\n';
	 }
	 int ardrone_id = atoi(droneIdStr.c_str());
	 return ardrone_id;
   }
	else
		return 1;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_autopilot");

  ROS_INFO("Started ArDrone Autopilot Node.");

  ControlNode controlNode(checkDroneId(argc, argv));

  dynamic_reconfigure::Server<monoslam_ros_test::AutopilotParamsConfig> srv;
  dynamic_reconfigure::Server<monoslam_ros_test::AutopilotParamsConfig>::CallbackType f;
  f = boost::bind(&ControlNode::dynConfCb, &controlNode, _1, _2);
  srv.setCallback(f);

  controlNode.Loop();

  return 0;
}
