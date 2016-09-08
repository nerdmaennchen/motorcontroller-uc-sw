#pragma once

#include <flawless/platform/system.h>

namespace flawless
{


class MessageContainerBase {
protected:
	using referenceCountType_t = int;
public:
	bool locked() const {
		flawless::LockGuard lock;
		return mReferenceCount != 0;
	}
	void lock() {
		flawless::LockGuard lock;
		++mReferenceCount;
	}
	void unlock() {
		flawless::LockGuard lock;
		--mReferenceCount;
	}

protected:
	virtual ~MessageContainerBase() {}
	referenceCountType_t mReferenceCount {0};
};

}
