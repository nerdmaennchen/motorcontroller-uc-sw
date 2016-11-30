#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/Array.h>
#include <interfaces/systemTime.h>

using Voltage = float;

struct VoltageMeasure {
	Voltage motorCurrent;
	Voltage voltage_phase_U;
	Voltage voltage_phase_V;
	Voltage voltage_phase_W;
	Voltage voltage_VPP;
};

