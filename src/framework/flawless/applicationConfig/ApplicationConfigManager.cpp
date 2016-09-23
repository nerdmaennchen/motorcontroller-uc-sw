#include <flawless/module/Module.h>
#include <flawless/protocol/PacketHandler.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/platform/system.h>
#include <string.h>

namespace {


class ApplicationConfigManager: public flawless::PacketHandler
{
public:
	ApplicationConfigManager() : flawless::PacketHandler(1) {}

	void handlePacket(flawless::PhyInterface* iface, uint8_t const* packet, uint32_t len) override {
		if (0 == len) {
			// return the description
			uint16_t totalPackeLen = 0;

			auto* handle0 = flawless::util::LinkedList<flawless::ApplicationConfigBase>::get().mFirst;
			auto* handle = handle0;
			while (handle) {
				totalPackeLen += 2;
				totalPackeLen += strlen(handle->getName())+1;
				totalPackeLen += strlen(handle->getFormat())+1;
				handle = handle->mNext;
			}
			iface->startPacket(mEPNum, totalPackeLen);

			handle = handle0;
			while (handle) {
				uint16_t paramLen = handle->getSize();
				iface->sendPacket(&paramLen, sizeof(paramLen));
				iface->sendPacket(handle->getName(), strlen(handle->getName())+1);
				iface->sendPacket(handle->getFormat(), strlen(handle->getFormat())+1);
				handle = handle->mNext;
			}
		} else if (1 == len) {
			auto* handle = flawless::util::LinkedList<flawless::ApplicationConfigBase>::get().mFirst;
			uint8_t idx = 0;
			uint8_t target = packet[0];
			while (handle) {
				if (target == idx) {
					uint16_t paramLen = handle->getSize();
					iface->startPacket(mEPNum, paramLen);
					flawless::LockGuard lock;
					iface->sendPacket(handle->getValue(), paramLen);
					break;
				}
				++idx;
				handle = handle->mNext;
			}
		} else {
			// first two bytes are the index the rest is the content
			auto* handle = flawless::util::LinkedList<flawless::ApplicationConfigBase>::get().mFirst;
			uint8_t idx = 0;
			uint8_t target = packet[0];
			while (handle) {
				if (target == idx and len == handle->getSize() + 1U) {
					handle->setValue(&(packet[1]));
					break;
				}
				++idx;
				handle = handle->mNext;
			}
		}
	}
} appConfManager;

}

