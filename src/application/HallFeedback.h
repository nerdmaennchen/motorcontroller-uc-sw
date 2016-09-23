#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/Singleton.h>
#include <flawless/util/Callback.h>


namespace hall {

struct Feedback {
	uint64_t lastTickDelay_us;
	float   tickFreq_Hz;
	uint8_t currentHallValues;
};

uint8_t getNextStep(uint8_t curStep, bool cw);
}
