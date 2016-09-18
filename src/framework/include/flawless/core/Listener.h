#pragma once

#include "ListenerBase.h"
#include "Message.h"
#include <flawless/util/LinkedList.h>
#include <flawless/util/Callback.h>
namespace flawless
{

template<typename T, msgID_t msgID>
class Listener : public flawless::util::LinkedListNode<Listener<T, msgID>>, public virtual flawless::RefCallback<Message<T>>
{};

} /* namespace flawless */

