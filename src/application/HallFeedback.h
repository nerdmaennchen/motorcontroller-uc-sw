#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/Singleton.h>
#include <flawless/util/Callback.h>


struct HallFeedback {
	uint64_t lastTickDelay;
	bool movingClockwise;
	uint8_t currentHallValues;
};

namespace hallUtils {
uint8_t getNextStep(uint8_t curStep, bool cw);
}
