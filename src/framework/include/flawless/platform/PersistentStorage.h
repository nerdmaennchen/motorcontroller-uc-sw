#pragma once

#include <flawless/util/Singleton.h>

namespace flawless
{
namespace platform
{
/**
 * you need to implement this class to make persistent storage working
 */

class PersistentStorage : public flawless::util::Singleton<PersistentStorage> {
public:
	volatile void* getMemoryBasePtr();
	int getMemorySize();

	void eraseMemory();
	void unlockMemory();
	void lockMemory();
	void writeData(volatile void *targetPtr, void const* srcPtr, int size);
};

}
}
