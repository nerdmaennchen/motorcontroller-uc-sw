#pragma once

#include <flawless/util/LinkedList.h>
#include "MessageBufferMemoryBase.h"
#include "Message.h"


namespace flawless
{

template<typename T>
Message<T> getFreeMessage() {
	flawless::LockGuard lock;
	static flawless::util::LinkedList<MessageBufferMemoryBase<T>> const& messageBufferMemoryManager = flawless::util::LinkedList<MessageBufferMemoryBase<T>>::get();
	MessageBufferMemoryBase<T>* buffer = messageBufferMemoryManager.mFirst;
	while (buffer) {
		MessageContainer<T> * msg = buffer->getFreeMessage();
		if (msg) {
			return Message<T>(msg);
		}
		buffer = buffer->mNext;
	}
	return Message<T>();
}

}

