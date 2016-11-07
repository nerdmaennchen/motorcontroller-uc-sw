#include "MessagePump.h"

#include "flawless/config/msgPump_config.h"
#include "flawless/platform/system.h"

#include <flawless/util/Array.h>
#include <flawless/util/FiFo.h>
#include "interfaces/ISRTime.h"

namespace flawless
{

namespace {
	FIFO<Post, MSG_PUMP_MSG_QUEUE_MAX_SIZE, LockGuard> gMsgQueue;
}

bool MessagePump::dispatch(Post post)
{
	return gMsgQueue.put(post);
}


void MessagePump::run()
{
	while (true)
	{
		if (gMsgQueue.count()) {
			WorkingTime workingTime;
			Post& head = gMsgQueue[0];
			head.invoke();
			gMsgQueue.pop(1);
		}
	}
}

} /* namespace flawless */
