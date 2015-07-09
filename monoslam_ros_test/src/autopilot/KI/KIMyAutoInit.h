#ifndef __KIMYAUTOINIT_H
#define __KIMYAUTOINIT_H

#include "KIProcedure.h"

class KIMyAutoInit : public KIProcedure
{
private:
	enum {NONE, STARTED, WAIT_FOR_FIRST, TOOK_FIRST, WAIT_FOR_SECOND, DONE} stage;
	int stageStarted;
	bool nextGoUp;
	bool resetMap;
	int moveTimeMS;
	int waitTimeMS;
	int reachHeightMS;
	float _controlGain;
public:
	KIMyAutoInit(bool resetMap = true, int imoveTimeMS=1500, int iwaitTimeMS=1500, int reachHeightMS=6000, float controlMult = 1.0, bool takeoff=true);
	~KIMyAutoInit(void);
	bool update(const monoslam_ros_test::filter_stateConstPtr statePtr);
};

#endif /* __KIAUTOINIT_H */
