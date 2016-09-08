#pragma once

#include <flawless/util/LinkedList.h>

namespace flawless
{


class Module : public flawless::util::LinkedListNode<Module> {
public:
	Module(unsigned int initLevel) : mInitLevel(initLevel) {}
	virtual ~Module() {}
	virtual void init(unsigned int level) = 0;

	const unsigned int mInitLevel;
};

void InitializeModules();

}

