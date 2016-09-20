#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/Singleton.h>
#include <flawless/util/Callback.h>

enum class MovingDirection : int {
	None,
	CW,
	CCW,
};

struct HallFeedback {
	int64_t lastTickDelay_us;
	int64_t averagedTickDelay_us;
	int64_t lastTickFreq_mHz;
	int64_t averageTickFreq_mHz;
	MovingDirection movingDirection;
	uint8_t currentHallValues;
};

namespace hallUtils {
uint8_t getNextStep(uint8_t curStep, bool cw);
}
