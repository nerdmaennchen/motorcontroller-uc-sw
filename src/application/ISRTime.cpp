/*
 * ISRTime.cpp
 *
 *  Created on: 19.09.2016
 *      Author: lutz
 */

#include <interfaces/ISRTime.h>

#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/util/Array.h>

systemTime_t ISRTime::timeSpentInISRs;
namespace {
struct : flawless::Callback<Array<systemTime_t, 2>&, bool>{
	Array<systemTime_t, 2> mLastTimes;

	void callback(Array<systemTime_t, 2>& vals, bool setter) {
		if (not setter) {
			systemTime_t curTime = SystemTime::get().getSystemTimeUS();
			vals[0] = curTime-mLastTimes[0];
			vals[1] = ISRTime::timeSpentInISRs-mLastTimes[1];
			mLastTimes[0] = curTime;
			mLastTimes[1] = ISRTime::timeSpentInISRs;
		}
	}
	flawless::ApplicationConfig<Array<systemTime_t, 2>> times{"isrTimings", "2Q", this};
} isrTimeHelper;
}

