#pragma once


#include <application/PWMDriver.h_>

class PWMLookupTableGenerator
{
public:
	void generateLookupTable(uint8_t stepsPerPhase, uint32_t numSteps, Array<uint16_t, pwmdriver::TicksPerStep> *pattern);
	void generateLookupTableReversed(uint8_t stepsPerPhase, uint32_t numSteps, Array<uint16_t, pwmdriver::TicksPerStep> *pattern);
};


class PWMLookupTableGeneratorShaped
{
public:
	void generateLookupTable(uint8_t stepsPerPhase, uint32_t numSteps, float shapeFactor,Array<uint16_t, pwmdriver::TicksPerStep> *pattern);
	void generateLookupTableReversed(uint8_t stepsPerPhase, uint32_t numSteps, float shapeFactor, Array<uint16_t, pwmdriver::TicksPerStep> *pattern);
};

