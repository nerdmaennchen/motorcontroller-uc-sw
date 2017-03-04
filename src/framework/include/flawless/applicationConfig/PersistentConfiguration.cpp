
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

bool handleValid(ConfigHandle const* handle) {
	return handle and handle->descriptorStrLen != 0 and handle->descriptorStrLen != -1 and handle->offsetTillNextHandle > handle->descriptorStrLen;
}

ConfigHandle const* findHandle(flawless::PersistentConfigBase const* config)
{
	long int offset = 0;
	char const* configName = config->getName();
	flawless::platform::PersistentStorage& persistentStorage = flawless::platform::PersistentStorage::get();
	char const* storage = reinterpret_cast<char const*>(persistentStorage.getMemoryBasePtr());
	while (offset < persistentStorage.getMemorySize()) {
		ConfigHandle const* handle = reinterpret_cast<ConfigHandle const*>(&(storage[offset]));
		if (not handleValid(handle)) {
			break; // bail out if not found
		}
		char const* descriptorString = reinterpret_cast<char const*>(handle) + sizeof(*handle);
		if (0 == strcmp(descriptorString, configName)) {
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
		ConfigHandle const* handle = findHandle(config);
		if (not handle) { // if the handle cannot be found there we are obviously dirty
			return true;
		}
		// test if a value changed
		char const* basePtr = reinterpret_cast<char const*>(handle);
		char const* contentPtr = reinterpret_cast<char const*>(basePtr + sizeof(*handle) + handle->descriptorStrLen);
		if (0 != memcmp(contentPtr, config->getValue(), config->getSize())) {
			return true;
		}
		config = config->getNext();
	}
	return false;
}

bool PersistentConfigurationManager::loadFromStorage(PersistentConfigBase *config)
{
	ConfigHandle const* handle = findHandle(config);
	if (handle) {
		char const* basePtr = reinterpret_cast<char const*>(handle);
		void const* srcContentPtr = reinterpret_cast<void const*>(&(basePtr[sizeof(*handle) + handle->descriptorStrLen]));
		config->setValue(srcContentPtr);
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
	char* storage = reinterpret_cast<char*>(persistentStorage.getMemoryBasePtr());
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
		void* dstHandlePtr  = reinterpret_cast<void*>(&(storage[offset]));
		void* dstNamePtr    = reinterpret_cast<void*>(&(storage[offset + sizeof(handle)]));
		void* dstContentPtr = reinterpret_cast<void*>(&(storage[offset + sizeof(handle) + handle.descriptorStrLen]));
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

