#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/Singleton.h>
#include <flawless/util/Callback.h>


namespace hall {

struct Feedback {
	uint64_t lastTickDelay_us;
	float   tickFreq_Hz;
	int currentHallValues;
};

int getNextStep(int curStep, bool cw);
}
