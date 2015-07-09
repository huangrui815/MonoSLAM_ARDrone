/*
 * SL_KeyFrmLst.cpp
 *
 *  Created on: 2010-11-25
 *      Author: Danping Zou
 */

#include "SL_KeyFrame.h"
#include "SL_MapPoint.h"
#include "SL_error.h"
using namespace std;
KeyFrameList::KeyFrameList() :
	num(0), head(), tail(0) {
}

KeyFrameList::~KeyFrameList() {
	clear();
}
void KeyFrameList::clear() {
	KeyFrame* p = head.next;
	while (p) {
		KeyFrame* q = p;
		p = p->next;
		delete q;
	}
	head.next = 0;
	tail = 0;
	num = 0;
}
KeyFrame* KeyFrameList::add(int f, CamPoseItem* cam_) {
	if (!cam_)
		repErr("KeyFrmLst::add() error!");
	if (tail == 0) {
		KeyFrame* kf = new KeyFrame(f, cam_);
		head.next = kf;
		tail = kf;
	} else {
		if (cam_->f < tail->cam->f)
			repErr("KeyFrmLst::add() cam_->f < tail->cam->f");
		KeyFrame* frm = new KeyFrame(f, cam_);

		tail->next = frm;
		frm->prev = tail;
		tail = frm;
	}
	num++;
	return tail;
}
void KeyFrame::releaseFeatPts(){
	for( size_t i = 0; i < featPts.size(); i++){
		delete featPts[i];
	}
	featPts.clear();
}
void KeyFrame::getMapPoints(Mat_d& pts3d) const{
	vector<Point3d> vecPts;
	for( size_t i = 0; i < featPts.size(); i++){
		FeaturePoint* fp = featPts[i];
		if( fp->mpt && !fp->mpt->isFalse()){
			vecPts.push_back(Point3d( fp->mpt->M[0], fp->mpt->M[1], fp->mpt->M[2]));
		}
	}

	pts3d.resize((int) vecPts.size(),3);
	for( size_t i = 0 ; i < vecPts.size(); i++){
		pts3d[3*i] = vecPts[i].x;
		pts3d[3*i+1]  = vecPts[i].y;
		pts3d[3*i+2] = vecPts[i].z;
	}
}

void KeyFrame::getMapPoints(vector<MapPoint*>& mappts) const{
	for( size_t i = 0; i < featPts.size(); i++){
		FeaturePoint* fp = featPts[i];
		if( fp->mpt && !fp->mpt->isFalse()){
			mappts.push_back(fp->mpt);
		}
	}
}
