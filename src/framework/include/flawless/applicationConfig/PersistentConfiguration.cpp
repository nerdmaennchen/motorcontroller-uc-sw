
#include <flawless/applicationConfig/PersistentApplicationConfig.h>
#include <flawless/platform/PersistentStorage.h>
#include <interfaces/systemTime.h>
#include <array>
#include <string.h>


namespace {
/*
 * the layout is like:
 * ConfigHandle;DescriptorString;Content
 */
struct ConfigHandle {
	int offsetTillNextHandle;
	int descriptorStrLen;
};

bool handleValid(volatile ConfigHandle const* handle) {
	return handle and handle->descriptorStrLen != 0 and handle->descriptorStrLen != -1 and handle->offsetTillNextHandle > handle->descriptorStrLen;
}

volatile ConfigHandle const* findHandle(flawless::PersistentConfigBase const* config)
{
	long int offset = 0;
	char const* configName = config->getName();
	flawless::platform::PersistentStorage& persistentStorage = flawless::platform::PersistentStorage::get();
	volatile char const* storage = reinterpret_cast<volatile char const*>(persistentStorage.getMemoryBasePtr());
	while (offset < persistentStorage.getMemorySize()) {
		volatile ConfigHandle const* handle = reinterpret_cast<volatile ConfigHandle const*>(&(storage[offset]));
		if (not handleValid(handle)) {
			break; // bail out if not found
		}
		volatile char const* descriptorString = reinterpret_cast<volatile char const*>(handle) + sizeof(*handle);
		if (0 == strcmp(const_cast<const char*>(descriptorString), const_cast<const char*>(configName))) {
			// set data accordingly
			return handle;
		} else {
			offset += handle->offsetTillNextHandle;
		}
	}
	return nullptr;
}

}

namespace flawless
{

bool PersistentConfigurationManager::dirty() const {
	PersistentConfigBase* config = flawless::util::LinkedList<PersistentConfigBase>::get().mFirst;
	while (config) {
		volatile ConfigHandle const* handle = findHandle(config);
		if (not handle) { // if the handle cannot be found there we are obviously dirty
			return true;
		}
		// test if a value changed
		volatile char const* basePtr = reinterpret_cast<volatile char const*>(handle);
		volatile char const* contentPtr = reinterpret_cast<volatile char const*>(basePtr + sizeof(*handle) + handle->descriptorStrLen);
		if (0 != memcmp(const_cast<const char*>(contentPtr), config->getValue(), config->getSize())) {
			return true;
		}
		config = config->getNext();
	}
	return false;
}

bool PersistentConfigurationManager::loadFromStorage(PersistentConfigBase *config)
{
	volatile ConfigHandle const* handle = findHandle(config);
	if (handle) {
		volatile char const* basePtr = reinterpret_cast<volatile char const*>(handle);
		volatile void const* srcContentPtr = reinterpret_cast<volatile void const*>(&(basePtr[sizeof(*handle) + handle->descriptorStrLen]));
		config->setValue(const_cast<void const*>(srcContentPtr));
		return true;
	}
	return false;
}

bool PersistentConfigurationManager::updateStorage()
{
	if (not dirty()) {
		return false;
	}
	SystemTime::get().sleep(500000); // sleep for .5 seconds to force no permanent flash writes
	std::size_t offset = 0;
	flawless::platform::PersistentStorage& persistentStorage = flawless::platform::PersistentStorage::get();
	volatile char* storage = reinterpret_cast<volatile char*>(persistentStorage.getMemoryBasePtr());
	PersistentConfigBase* config = flawless::util::LinkedList<PersistentConfigBase>::get().mFirst;
	persistentStorage.unlockMemory();
	persistentStorage.eraseMemory();
	while (config) {
		int nameLen = strlen(config->getName())+1;
		int offsetTillNextHandle = sizeof(ConfigHandle) + nameLen + config->getSize();
		offsetTillNextHandle = ((offsetTillNextHandle) + 4) & ~3; // padding if needed
		ConfigHandle handle = {
			offsetTillNextHandle,
			nameLen
		};
		volatile void* dstHandlePtr  = reinterpret_cast<volatile void*>(&(storage[offset]));
		volatile void* dstNamePtr    = reinterpret_cast<volatile void*>(&(storage[offset + sizeof(handle)]));
		volatile void* dstContentPtr = reinterpret_cast<volatile void*>(&(storage[offset + sizeof(handle) + handle.descriptorStrLen]));
		persistentStorage.writeData(dstHandlePtr,  &handle,            sizeof(handle));
		persistentStorage.writeData(dstNamePtr,    config->getName(),  nameLen);
		persistentStorage.writeData(dstContentPtr, config->getValue(), config->getSize());

		offset += offsetTillNextHandle;
		config = config->getNext();
	}
	persistentStorage.lockMemory();
	return true;
}
}

