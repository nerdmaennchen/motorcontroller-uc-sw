#pragma once

#include <inttypes.h>
#include <flawless/util/Singleton.h>

using systemTime_t = uint64_t; //In nS

class SystemTime : public flawless::util::Singleton<SystemTime>
{
public:
	systemTime_t getSystemTimeUS() const;
	systemTime_t getCurrentTime() const;

	void sleep(systemTime_t duration) const;
};


