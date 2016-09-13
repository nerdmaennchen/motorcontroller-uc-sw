#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/Array.h>

#define SMOOTHING_CNT (4)

struct MotorCurrentMeasure {
	// current in uA
	Array<float, SMOOTHING_CNT> vals;
};

using MotorCurrent = float;
