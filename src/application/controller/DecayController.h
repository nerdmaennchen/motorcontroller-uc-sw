#pragma once


#include <flawless/applicationConfig/ApplicationConfig.h>
#include <algorithm>

struct DecayController final
{
	float errorP;
	float errorI;

	float controllP;
	float controllI;

	float decay;

	float outputP;
	float outputI;

	float update(float error, float dT) {
		errorI = (1-decay) * errorI + decay * (error * dT);
		errorP = error;

		if (controllI) {
			float helper =  1.f / controllI;
			errorI = std::max(-helper, std::min(helper, errorI));
		}

		outputP = errorP * controllP;
		outputI = errorI * controllI;

		return outputP + outputI;
	}
};
