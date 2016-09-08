#pragma once

#include "MessageContainer.h"
#include "MessageBufferMemoryBase.h"
#include <flawless/util/Array.h>

namespace flawless
{

template<typename T, size_t N>
class MessageBufferMemory final : public MessageBufferMemoryBase<T> {
public:
	~MessageBufferMemory() {}

	MessageContainer<T>* getFreeMessage() override {
		flawless::LockGuard lock;
		for (size_t i(0); i < mMessages.size(); ++i) {
			MessageContainer<T> &msg = mMessages[i];
			if (not msg.locked()) {
				return &msg;
			}
		}
		return nullptr;
	}
protected:
	Array<MessageContainer<T>, N> mMessages;
};

}

