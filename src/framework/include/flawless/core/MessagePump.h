#pragma once

#include <flawless/module/Module.h>
#include <flawless/util/Singleton.h>

#include "Post.h"

namespace flawless
{

class MessagePump final : public flawless::util::Singleton<MessagePump>
{
public:
	bool dispatch(Post post);

	void run();
};

} /* namespace flawless */
