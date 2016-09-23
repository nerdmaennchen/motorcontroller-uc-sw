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
systemTime_t WorkingTime::timeSpentBusy;
namespace {
struct : flawless::Callback<Array<systemTime_t, 3>&, bool>{
	Array<systemTime_t, 3> mLastTimes;

	void callback(Array<systemTime_t, 3>& vals, bool setter) {
		if (not setter) {
			systemTime_t curTime = SystemTime::get().getSystemTimeUS();
			vals[0] = curTime-mLastTimes[0];
			vals[1] = ISRTime::timeSpentInISRs-mLastTimes[1];
			vals[2] = WorkingTime::timeSpentBusy-mLastTimes[2];
			mLastTimes[0] = curTime;
			mLastTimes[1] = ISRTime::timeSpentInISRs;
			mLastTimes[2] = WorkingTime::timeSpentBusy;
		}
	}
	flawless::ApplicationConfig<Array<systemTime_t, 3>> times{"isrTimings", "3Q", this};
} isrTimeHelper;
}

