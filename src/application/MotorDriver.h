#pragma once
#include <flawless/stdtypes.h>

namespace motordriver
{

// off time is before the PWM and after to there is always a PWM_OFF_TIME*2 off time between two consecutive PWM pulses
constexpr uint32_t PWM_OFF_TIME      = 128;
constexpr uint32_t PWM_AMPLITUDE     = 512;

constexpr uint32_t StepsCount = 600;
constexpr uint32_t TicksPerStep = 4;
constexpr uint32_t TotalTickCnt = StepsCount * TicksPerStep;

}
