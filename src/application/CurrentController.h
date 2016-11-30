#pragma once

#include <flawless/util/Singleton.h>

class CurrentController : public flawless::util::Singleton<CurrentController>
{
public:
	// set a power scaling factor (between 0 and 1)
	void setPower(float power);
	float getPower() const;
};
