#pragma once

#include "Message.h"
#include <flawless/util/LinkedList.h>
#include <flawless/util/Callback.h>

namespace flawless
{

template<typename T, msgID_t msgID=0>
class Listener : public flawless::util::LinkedListNode<Listener<T, msgID>>, public flawless::RefCallback<Message<T>>
{};

} /* namespace flawless */

