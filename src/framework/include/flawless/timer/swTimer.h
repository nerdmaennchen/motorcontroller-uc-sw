#pragma once

#include "../stdtypes.h"
#include <flawless/module/Module.h>
#include <flawless/platform/system.h>
#include <flawless/util/Callback.h>


namespace flawless
{

using timerInterval_t = uint32_t;

class TimerCallback;
class SWTimer final: public flawless::util::Singleton<SWTimer>
{
public:
	void registerTimer(TimerCallback* callback);
	void unregisterTimer(TimerCallback* callback);

	void trigger();
private:
	TimerCallback* mNextCallback {nullptr};
	bool mIsHandlingClinents {false};
	void setupTimer();
};

class TimerCallback : public virtual flawless::Callback<void> {
public:
	TimerCallback() {}
	TimerCallback(timerInterval_t interval, bool repeating) {start(interval, repeating);}
	~TimerCallback() {
		stop();
	}

	void start(timerInterval_t interval, bool repeating) {
		LockGuard lock;
		if (0 == mInterval) {
			mRepeating = repeating;
			mInterval = interval;
			SWTimer::get().registerTimer(this);
		}
	}

	void stop() {
		LockGuard lock;
		SWTimer::get().unregisterTimer(this);
		mRepeating = false;
		mInterval = 0;
		mNextCallback = nullptr;
	}

	void invoke() {
		callback();
		if (not mRepeating) {
			stop();
		}
	}

private:
	friend class flawless::SWTimer;
	bool mRepeating {false};
	timerInterval_t mInterval {0};
	timerInterval_t mTicksToGo {0};

	TimerCallback* mNextCallback {nullptr};
};

}
