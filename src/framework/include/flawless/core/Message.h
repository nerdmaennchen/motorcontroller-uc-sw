#pragma once

#include "MessageContainer.h"
#include "MessagePump.h"
#include <flawless/platform/system.h>

namespace flawless
{


template<typename T>
class Message final {
public:
	Message() {}
	Message(MessageContainer<T>* msg) : mMsg(msg) {
		if (mMsg) {
			mMsg->lock();
		}
	}

	~Message() {
		if (mMsg) {
			mMsg->unlock();
		}
	}

	operator bool() const {
		return mMsg;
	}

	T& operator*() {
		return **mMsg;
	}
	T const& operator*() const {
		return **mMsg;
	}

	Message<T>& operator=(Message<T> const& rhs) {
		rhs.mMsg->lock();
		if (mMsg) {
			mMsg->unlock();
		}
		mMsg = rhs.mMsg;
		return *this;
	}

	Message<T>& operator=(T const& rhs) {
		*mMsg = rhs;
		return *this;
	}

	MessageContainer<T>* get() {
		return mMsg;
	}

	template<msgID_t msgID>
	void post();

	template<msgID_t msgID>
	void invokeDirectly();

private:
	MessageContainer<T>* mMsg {nullptr};
};

}

#include "ListenerManager.h"

namespace flawless
{
template<typename T>
template<msgID_t msgID>
void Message<T>::post()
{
	flawless::LockGuard lock;

	if (not *this) {
		return;
	}

	flawless::ListenerManagerBase *listenerManager = &(flawless::ListenerManager<T, msgID>::get());
	bool dispatchSuccess = flawless::MessagePump::get().dispatch(Post(mMsg, listenerManager));
	if (dispatchSuccess) {
		mMsg->unlock();
		mMsg = nullptr;
	}
}

template<typename T>
template<msgID_t msgID>
void Message<T>::invokeDirectly()
{
	if (not *this) {
		return;
	}
	flawless::ListenerManager<T, msgID>::get().invoke(mMsg);
}


}
