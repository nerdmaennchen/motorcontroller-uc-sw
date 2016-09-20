#pragma once

#include <interfaces/systemTime.h>

class ISRTime final
{
public:
	systemTime_t startTime;
	ISRTime() {
		startTime = SystemTime::get().getSystemTimeUS();
	}
	~ISRTime() {
		timeSpentInISRs += SystemTime::get().getSystemTimeUS() - startTime;
	}

	static systemTime_t timeSpentInISRs;
};
