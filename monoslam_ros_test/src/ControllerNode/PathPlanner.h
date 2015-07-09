/*
 * PathPlanner.h
 *
 *  Created on: Jul 8, 2014
 *      Author: rui
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

//#define MAX_PATH_SIZE 24000
//#define MAX_PATH_DATA 11

#include <vector>
using namespace std;
class PathPlanner{
public:
	PathPlanner();
	~PathPlanner();

	bool preloadPath(char* filePath);
	void getCurrentRef(double& ref_x, double& ref_y, double& ref_vx, double& ref_vy);
	bool isPathFinished();
	void reset();
	void setWayCounter(int way_counter_param);

	int _numWayPts;
//	double _path_data[MAX_PATH_SIZE][MAX_PATH_DATA];
	vector<vector<double> > _path_data;
	bool _pathEnabled;
	int path_counter;
	int way_counter;
};



#endif /* PATHPLANNER_H_ */
