/*
 * SL_Track2D.h
 *
 *  Created on: 2010-11-10
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#ifndef SL_TRACK2D_H_
#define SL_TRACK2D_H_
#include "imgproc/SL_Image.h"
#include "slam/SL_FeaturePoint.h"

#include <list>
#include <vector>

class TrackedFeaturePoint {
public:
	/* track id*/
	int tkid;
	/* pointer to the image point*/
	FeaturePoint pt;
	FeaturePoint* ptInKeyFrm;

	TrackedFeaturePoint* pre;
	TrackedFeaturePoint* next;
public:
	TrackedFeaturePoint() :
			tkid(0), pt(), ptInKeyFrm(0), pre(0), next(0) {
	}
	TrackedFeaturePoint(FeaturePoint p) :
			tkid(0), pt(p), ptInKeyFrm(0), pre(0), next(0) {
	}
};

class Track2D {
public:
	unsigned int id;
	/* trajectory duration */
	int f1, f2;
	/* list head*/
	TrackedFeaturePoint head;
	/* pointer to list tail*/
	TrackedFeaturePoint* tail;
public:
	int length() const {
		return f2 - f1 + 1;
	}
	void clear() {
		TrackedFeaturePoint* p = head.next;
		TrackedFeaturePoint* q = 0;
		while (p) {
			q = p;
			p = p->next;
			delete q;
		}
		f1 = f2 = -1;
		head.next = 0;
		tail = 0;
	}
	bool empty() const {
		return tail == 0;
	}
	bool valid(){
		if (tail->pt.id == -1)
			return false;
		else
			return true;
	}
	void add(TrackedFeaturePoint* node) {
		node->tkid = id;

		if (node->pt.f < f1 || f1 < 0)
			f1 = node->pt.f;
		if (node->pt.f > f2 || f2 < 0)
			f2 = node->pt.f;

		/* add to tail*/
		if (tail == 0) {
			head.next = node;
			node->pre = 0;
		} else {
			tail->next = node;
			node->pre = tail;
		}
		node->next = 0;
		tail = node;
	}
	void add(FeaturePoint pt) {
		TrackedFeaturePoint* node = new TrackedFeaturePoint(pt);
		add(node);
	}
public:
	Track2D();
	~Track2D();
};

//////////////////////////////////////////////////////////////////////
//The following will be removed
typedef std::list<Track2D*> Track2DPtrList;
typedef Track2D* Track2DPtr;
typedef Track2D** Track2DPtrPtr;

/* select tracks from f1*/
void trackSelect(const Track2DPtrList& tks, int f1, Track2DPtrList& cmptks);

/* get corresponding points from the tracks*/
int trackGetCorrespondenceNum(const Track2DPtrList& tks, int f1, int f2);
int trackGetCorrespondence(const Track2DPtrList& tks, int f1, int f2,
		double* pts1, double* pts2);
int trackGetCorrespondence(const Track2DPtrList& tks, int f1, int f2,
		std::vector<TrackedFeaturePoint*>& nodes1,
		std::vector<TrackedFeaturePoint*>& nodes2);
void trackNodes2Arr(std::vector<TrackedFeaturePoint*>& nodes, int i1, int i2,
		double* pts);

#endif /* SL_TRACK2D_H_ */
