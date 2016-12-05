#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/Singleton.h>
#include <flawless/util/Callback.h>


namespace hall {

struct Timeout {
	float    delaySinceLastTick;
	int      currentHallValues;
};

struct Tick {
	float    prevTickDelay;
	float    tickFreq_Hz;
	int      currentHallValues;
};

int getNextStep(int curStep, bool cw);
}
