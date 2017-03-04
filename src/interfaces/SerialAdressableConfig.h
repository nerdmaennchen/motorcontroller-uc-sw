#pragma once

#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/util/LinkedList.h>

struct SerialAdressableConfig : public flawless::util::LinkedListNode<SerialAdressableConfig>
{
	flawless::ApplicationConfigBase& mConfig;
	SerialAdressableConfig(flawless::ApplicationConfigBase& config) : mConfig(config) {}
	SerialAdressableConfig *mNextSubscribed {nullptr};
};
