#include <flawless/module/Module.h>
#include <flawless/protocol/PacketHandler.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <string.h>

namespace {


class ApplicationConfigManager: public flawless::PacketHandler
{
public:
	ApplicationConfigManager() : flawless::PacketHandler(1) {}

	void handlePacket(flawless::PhyInterface* iface, uint8_t const* packet, size_t len) override {
		if (0 == len) {
			// return the description
			uint16_t totalPackeLen = 0;

			auto* handle0 = flawless::util::LinkedList<flawless::ApplicationConfigBase>::get().mFirst;
			auto* handle = handle0;
			while (handle) {
				totalPackeLen += 2;
				totalPackeLen += strlen(handle->getName())+1;
				handle = handle->mNext;
			}
			iface->startPacket(mEPNum, totalPackeLen);

			handle = handle0;
			while (handle) {
				uint16_t paramLen = handle->getSize();
				iface->sendPacket(&paramLen, sizeof(paramLen));
				iface->sendPacket(handle->getName(), strlen(handle->getName())+1);
				handle = handle->mNext;
			}
		} else if (1 == len) {
			auto* handle = flawless::util::LinkedList<flawless::ApplicationConfigBase>::get().mFirst;
			uint8_t idx = 0;
			uint8_t target = packet[0];
			while (handle) {
				if (target == idx) {
					uint16_t paramLen = handle->getSize();
					if (0 == paramLen) { // either call the callback or read the content
						if (handle->mOnValueChangedCB) {
							handle->mOnValueChangedCB->callback();
						}
					} else {
						iface->startPacket(mEPNum, paramLen);
						iface->sendPacket(handle->getDataPtr(), paramLen);
					}
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
				if (target == idx and len == handle->getSize() + 1) {
					handle->setValue(&(packet[1]));
					if (handle->mOnValueChangedCB) {
						handle->mOnValueChangedCB->callback();
					}
					break;
				}
				++idx;
				handle = handle->mNext;
			}
		}
	}
} appConfManager;

}

