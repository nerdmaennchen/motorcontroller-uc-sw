#pragma once

#include "MessageContainerBase.h"

namespace flawless
{

class ListenerManagerBase {
public:
	virtual void invoke(MessageContainerBase* msg) = 0;
protected:
	virtual ~ListenerManagerBase() {}
};

}
