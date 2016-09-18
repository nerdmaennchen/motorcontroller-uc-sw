#pragma once


#include "PWMDriver.h"

class PWMLookupTableGenerator
{
public:
	PWMLookupTableGenerator();
	virtual ~PWMLookupTableGenerator();

	void generateLookupTable(uint8_t stepsPerPhase, uint32_t numSteps, Array<uint16_t, pwmdriver::TicksPerStep> *pattern);
	void generateLookupTableReversed(uint8_t stepsPerPhase, uint32_t numSteps, Array<uint16_t, pwmdriver::TicksPerStep> *pattern);
};

