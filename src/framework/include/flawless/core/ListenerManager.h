#pragma once

#include "MessageContainer.h"
#include "ListenerManagerBase.h"
#include "Listener.h"

namespace flawless
{

template<typename T, msgID_t msgID>
class ListenerManager : public flawless::util::Singleton<ListenerManager<T, msgID>>, public ListenerManagerBase {
public:
	void invoke(MessageContainerBase* msg) override {
		Listener<T, msgID>* listener = flawless::util::LinkedList<Listener<T, msgID>>::get().mFirst;
		Message<T> typedMsg = Message<T>(reinterpret_cast<MessageContainer<T>*>(msg));
		while (listener) {
			listener->callback(typedMsg);
			listener = listener->mNext;
		}
	}
};

}
