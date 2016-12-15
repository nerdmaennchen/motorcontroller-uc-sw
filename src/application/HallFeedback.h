#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/Singleton.h>
#include <flawless/util/Callback.h>


namespace hall {

struct Timeout {
	uint64_t delaySinceLastTickUS;
	int      currentHallValues;
	int      currentPos; // state indicates the angle and ranges from 0 to 11
	bool     movingCW;   // true if the motor previously moved clockwise false otherwise
	bool     moving;     // true if the motor is moving (if at least two ticks in the same direction happened)
};

struct Tick {
	uint64_t prevTickDelayUS;
	int      currentHallValues;
	int      currentPos; // state indicates the angle and ranges from 0 to 11
	bool     movingCW; // true if the motor moves clockwise false otherwise
	bool     moving;     // true if the motor is moving (if at least two ticks in the same direction happened)
};

int getNextStep(int curStep, bool cw);
}
