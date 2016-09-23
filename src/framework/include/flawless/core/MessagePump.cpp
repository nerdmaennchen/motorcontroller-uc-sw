#include "MessagePump.h"

#include "flawless/config/msgPump_config.h"
#include "flawless/platform/system.h"

#include <flawless/util/Array.h>
#include "interfaces/ISRTime.h"

namespace flawless
{

namespace {
	Array<Post, MSG_PUMP_MSG_QUEUE_MAX_SIZE> gMsgQueue;
	volatile size_t gMsgQueueHead = 0;
	volatile size_t gMsgQueueSize = 0;
}

bool MessagePump::dispatch(Post post)
{
	flawless::LockGuard lock;
	if (gMsgQueueSize < MSG_PUMP_MSG_QUEUE_MAX_SIZE) {
		size_t nextIdx = (gMsgQueueHead + gMsgQueueSize) % MSG_PUMP_MSG_QUEUE_MAX_SIZE;
		gMsgQueue[nextIdx] = post;
		++gMsgQueueSize;
		return true;
	}
	return false;
}


void MessagePump::run()
{
	while (true)
	{
		if (gMsgQueueSize) {
			WorkingTime workingTime;
			Post& head = gMsgQueue[gMsgQueueHead];
			head.invoke();
			{
				flawless::LockGuard lock;
				++gMsgQueueHead;
				if (gMsgQueueHead >= MSG_PUMP_MSG_QUEUE_MAX_SIZE) {
					gMsgQueueHead = 0;
				}
				--gMsgQueueSize;
			}
		}
	}
}

} /* namespace flawless */
