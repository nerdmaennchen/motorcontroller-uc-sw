
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/applicationConfig/PersistentApplicationConfig.h>
#include <flawless/platform/PersistentStorage.h>

#include <libopencm3/stm32/f4/flash.h>

extern "C" {
extern volatile char _flashConfigROMBegin;
extern volatile char _flashConfigROMEnd;
}

#define SECTOR 1

namespace
{

void waitForFlashOperation() {
	while ((FLASH_SR & FLASH_BSY) == FLASH_BSY);
}

}

namespace flawless
{
namespace platform
{

volatile void* PersistentStorage::getMemoryBasePtr()
{
	return &_flashConfigROMBegin;
}

int PersistentStorage::getMemorySize()
{
	return int(&_flashConfigROMEnd - &_flashConfigROMBegin);
}

void PersistentStorage::eraseMemory()
{
	waitForFlashOperation();
	FLASH_CR = (SECTOR << 3);
	FLASH_CR |= FLASH_SER;
	FLASH_CR |= FLASH_STRT;

	waitForFlashOperation();
	FLASH_CR = 0;
}

void PersistentStorage::unlockMemory() {
	waitForFlashOperation();
	FLASH_KEYR = FLASH_KEY1;
	FLASH_KEYR = FLASH_KEY2;
}

void PersistentStorage::lockMemory() {
	waitForFlashOperation();
	FLASH_CR |= FLASH_LOCK;
}

void PersistentStorage::writeData(volatile void *targetPtr, void const* srcPtr, int size) {
	volatile char* target    = reinterpret_cast<volatile char*>(targetPtr);
	char const* src = reinterpret_cast<char const*>(srcPtr);

	waitForFlashOperation();
	FLASH_CR &= ~(3 << 8);
	FLASH_CR |= 0 << 8;
	FLASH_CR |= FLASH_PG;

	for (int i(0); i < size; ++i) {
		*target = *src;
		++target;
		++src;
	}

	FLASH_CR &= ~FLASH_PG;
	waitForFlashOperation();
}

}
}

namespace
{

class : flawless::Callback<void> {
public:
	void callback() override {
		flawless::PersistentConfigurationManager::get().updateStorage();
	}
private:
flawless::ApplicationConfig<void> enableConfig{"persistentconfig.writetoflash", this};
} writeToFlash;

class : flawless::Callback<void> {
public:
	void callback() override {
		flawless::PersistentConfigBase* handle = flawless::util::LinkedList<flawless::PersistentConfigBase>::get().mFirst;
		while (handle) {
			flawless::PersistentConfigurationManager::get().loadFromStorage(handle);
			handle = handle->getNext();
		}
	}
private:
flawless::ApplicationConfig<void> enableConfig{"persistentconfig.reloadfromflash", this};
} loadFromFlash;

}
