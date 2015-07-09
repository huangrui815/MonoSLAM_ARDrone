#ifndef __KISCALEINIT_H
#define __KISCALEINIT_H

#include "KIProcedure.h"

class KIScaleInit : public KIProcedure
{
public:
	bool _bNextUp;
	bool _bNextDown;
	bool _bRising;
	bool _bDropping;

	int _numCycle;
	int _startTS;
	float _controlGain;
	KIScaleInit();
	~KIScaleInit();
	bool update(const monoslam_ros_test::filter_stateConstPtr statePtr);
};

#endif
