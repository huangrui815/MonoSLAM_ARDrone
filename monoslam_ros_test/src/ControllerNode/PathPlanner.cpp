/*
 * PathPlanner.cpp
 *
 *  Created on: Jul 8, 2014
 *      Author: rui
 */
#include "PathPlanner.h"
#include "stdio.h"

PathPlanner::PathPlanner(){
	_pathEnabled = false;
	path_counter = 0;
	way_counter = 0;
}

PathPlanner::~PathPlanner(){

}

void PathPlanner::reset(){
	_pathEnabled = false;
	path_counter = 0;
	way_counter = 0;
	_path_data.clear();
}

bool PathPlanner::isPathFinished(){
	if (path_counter >= _numWayPts && path_counter > 0){
		_pathEnabled = false;
		return true;
	}
	else
		return false;
}

void PathPlanner::getCurrentRef(double& ref_x, double& ref_y, double& ref_vx, double& ref_vy){
	if (!_pathEnabled || path_counter >= _numWayPts){
		ref_x = 0;
		ref_y = 0;
		ref_vx = 0;
		ref_vy = 0;
		return;
	}

	ref_x = _path_data[path_counter][1];
	ref_y = _path_data[path_counter][2];
	ref_vx = _path_data[path_counter][5];
	ref_vy = _path_data[path_counter][6];
	path_counter++;
}

void PathPlanner::setWayCounter(int way_counter_param){
	way_counter = way_counter_param;
}

bool PathPlanner::preloadPath(char* filePath){
	FILE *fr;
	double temp;
	int row = 0;
	int column = 0;


	//fr = fopen("D:\\UAV_Project\\Vicon for Test PCTx\\Vicon\\diag_2.txt","r");
	//fr = fopen("D:\\UAV_Project\\Vicon for Test PCTx\\Vicon\\square_2.txt","r");
	//fr = fopen("D:\\UAV_Project\\Vicon for Test PCTx\\Vicon\\circle_1_low.txt","r");
	//fr = fopen("D:\\UAV_Project\\Vicon for Test PCTx\\Vicon\\diamond_1_5.txt","r");
	fr = fopen(filePath,"r");

	//fr = fopen("D:\\UAV_Project\\Vicon for Test PCTx\\Vicon\\diag_1_5.txt","r");
	//fr = fopen("D:\\UAV_Project\\Vicon for Test PCTx\\Vicon\\diag_2_5.txt","r");

	//fr = fopen("D:\\UAV_Project\\Vicon for Test PCTx\\Vicon\\square_1_5.txt","r");
	//fr = fopen("D:\\UAV_Project\\Vicon for Test PCTx\\Vicon\\square_2_5.txt","r");

	if (fr==NULL)
	{
	  printf("No file found\n");
	  return false;
	}

	_numWayPts = 0;
	vector<double> wayPt;
	while(fscanf(fr, "%lf", &temp)!= EOF){
		wayPt.push_back(temp * 1000);
		column++;
		column = column%11;
		if(column == 0){
			_path_data.push_back(wayPt);
			wayPt.clear();
			_numWayPts++;
		}
	}

	fclose(fr);
	printf("numWayPts loaded: %d\n", _numWayPts);
	return true;
}



