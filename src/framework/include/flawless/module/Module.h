#pragma once

#include <flawless/util/LinkedList.h>

namespace flawless
{

struct ModuleBase : flawless::util::SortedListNode<ModuleBase> {
	ModuleBase(int level) : SortedListNode<ModuleBase>(level) {}
	virtual void init() {};
};

template<int initLevel = 0>
class Module : ModuleBase {
public:
	Module() : ModuleBase(initLevel) {}
	virtual ~Module() {}
};

void InitializeModules();

}

