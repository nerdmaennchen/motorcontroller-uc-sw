#pragma once

#include <flawless/util/LinkedList.h>
#include "MessageContainer.h"

namespace flawless
{

template<typename T>
class MessageBufferMemoryBase : public flawless::util::LinkedListNode<MessageBufferMemoryBase<T>> {
protected:
	virtual ~MessageBufferMemoryBase() {}
public:
	virtual MessageContainer<T>* getFreeMessage() = 0;
};

}
