#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/Singleton.h>
#include <flawless/util/Callback.h>


namespace hall {

struct Timeout {
	float    delaySinceLastTick;
	int      currentHallValues;
	int      currentPos; // state indicates the angle and ranges from 0 to 11
};

struct Tick {
	float    prevTickDelay;
	int      currentHallValues;
	int      currentPos; // state indicates the angle and ranges from 0 to 11
};

int getNextStep(int curStep, bool cw);
}
