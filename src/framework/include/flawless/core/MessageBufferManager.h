#pragma once

#include <flawless/util/LinkedList.h>
#include "MessageBufferMemoryBase.h"
#include "Message.h"


namespace flawless
{

template<typename T>
class MessageBufferManager final
		: public flawless::util::Singleton<MessageBufferManager<T>>
{
public:
	Message<T> getFreeMessage() {
		MessageBufferMemoryBase<T>* buffer = flawless::util::LinkedList<MessageBufferMemoryBase<T>>::get().mFirst;
		flawless::LockGuard lock;
		while (buffer) {
			MessageContainer<T> * msg = buffer->getFreeMessage();
			if (msg) {
				return Message<T>(msg);
			}
			buffer = buffer->mNext;
		}
		return Message<T>();
	}

	~MessageBufferManager() {}
};

}
