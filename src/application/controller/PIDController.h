#pragma once

#include <flawless/applicationConfig/ApplicationConfig.h>
#include <algorithm>

struct PIDControllerParams {
	float errorP;
	float errorI;
	float errorD;

	float controllP;
	float controllI;
	float controllD;
};

struct PIDController final
{
	PIDControllerParams *mParams;

	PIDController(PIDControllerParams *params) : mParams(params) {}

	void setError(float error, float dT) {
		mParams->errorI += error * dT;
		mParams->errorD = (error - mParams->errorP); // / dT;
		mParams->errorP = error;

		float helper =  1.f / mParams->controllI;
		mParams->errorI = std::max(-helper, std::min(helper, mParams->errorI));
	}

	float getControll() {
		float cP = mParams->errorP * mParams->controllP;
		float cI = mParams->errorI * mParams->controllI;
		float cD = mParams->errorD * mParams->controllD;
		return  cP + cI + cD;
	}
};

struct PIControllerParams {
	float errorP;
	float errorI;

	float controllP;
	float controllI;
};

struct PIController final
{
	PIControllerParams *mParams;
	PIController(PIControllerParams *params) : mParams(params) {}

	void setError(float error, float dT) {
		mParams->errorI += error * dT;
		mParams->errorP = error;

		float helper =  1.f / mParams->controllI;
		mParams->errorI = std::max(-helper, std::min(helper, mParams->errorI));
	}

	float getControll() {
		float cP = mParams->errorP * mParams->controllP;
		float cI = mParams->errorI * mParams->controllI;
		return  cP + cI;
	}
};

