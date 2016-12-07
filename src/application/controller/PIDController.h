#pragma once

#include <flawless/applicationConfig/ApplicationConfig.h>
#include <algorithm>

struct PIDController final
{
	float errorP;
	float errorI;
	float errorD;

	float controllP;
	float controllI;
	float controllD;

	float outputP;
	float outputI;
	float outputD;

	float update(float error, float dT) {
		if (dT) {
			errorD = (error - errorP) / dT;
			errorI += error * dT;
		} else {
			errorD = 0.f;
		}
		errorP = error;

		if (controllI) {
			float helper =  1.f / controllI;
			errorI = std::max(-helper, std::min(helper, errorI));
		}

		outputP = errorP * controllP;
		outputI = errorI * controllI;
		outputD = errorD * controllD;

		return outputP + outputI + outputD;
	}
};

struct PIController final
{
	float errorP;
	float errorI;

	float controllP;
	float controllI;

	float update(float error, float dT) {
		errorI += error * dT;
		errorP = error;

		float helper =  1.f / controllI;
		errorI = std::max(-helper, std::min(helper, errorI));

		float cP = errorP * controllP;
		float cI = errorI * controllI;
		return  cP + cI;
	}
};

