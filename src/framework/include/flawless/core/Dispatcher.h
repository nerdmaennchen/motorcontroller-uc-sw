#pragma once

#include <flawless/util/Singleton.h>
#include "Listener.h"

namespace flawless
{

template<typename T, msgID_t msgID>
class Dispatcher : public flawless::util::Singleton<Dispatcher<T, msgID>> {
};

}
