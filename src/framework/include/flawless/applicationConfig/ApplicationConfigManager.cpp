#include <flawless/module/Module.h>
#include <flawless/protocol/PacketHandler.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/applicationConfig/PersistentApplicationConfig.h>
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
				handle = handle->getNext();
			}

			auto* perHandle0 = flawless::util::LinkedList<flawless::PersistentConfigBase>::get().mFirst;
			auto* perHandle = perHandle0;
			while (perHandle) {
				totalPackeLen += 2;
				totalPackeLen += strlen(perHandle->getName())+1;
				totalPackeLen += strlen(perHandle->getFormat())+1;
				perHandle = perHandle->getNext();
			}

			iface->startPacket(mEPNum, totalPackeLen);

			handle = handle0;
			while (handle) {
				uint16_t paramLen = handle->getSize();
				iface->sendPacket(&paramLen, sizeof(paramLen));
				iface->sendPacket(handle->getName(), strlen(handle->getName())+1);
				iface->sendPacket(handle->getFormat(), strlen(handle->getFormat())+1);
				handle = handle->getNext();
			}

			perHandle = perHandle0;
			while (perHandle) {
				uint16_t paramLen = perHandle->getSize();
				iface->sendPacket(&paramLen, sizeof(paramLen));
				iface->sendPacket(perHandle->getName(), strlen(perHandle->getName())+1);
				iface->sendPacket(perHandle->getFormat(), strlen(perHandle->getFormat())+1);
				perHandle = perHandle->getNext();
			}
		} else if (1 == len) {
			auto* handle = flawless::util::LinkedList<flawless::ApplicationConfigBase>::get().mFirst;
			uint8_t idx = 0;
			uint8_t target = packet[0];
			while (handle) {
				if (target == idx) {
					uint16_t paramLen = handle->getSize();
					if (0 == paramLen) {
						// special case: if the handle does not have any size its a simple rpc
						handle->setValue(nullptr);
					} else {
						iface->startPacket(mEPNum, paramLen);
						iface->sendPacket(handle->getValue(), paramLen);
					}
					return;
				}
				++idx;
				handle = handle->getNext();
			}

			auto* perHandle = flawless::util::LinkedList<flawless::PersistentConfigBase>::get().mFirst;
			while (perHandle) {
				if (target == idx) {
					uint16_t paramLen = perHandle->getSize();
					if (0 == paramLen) {
						// special case: if the handle does not have any size its a simple rpc
						perHandle->setValue(nullptr);
					} else {
						iface->startPacket(mEPNum, paramLen);
						iface->sendPacket(perHandle->getValue(), paramLen);
					}
					return;
				}
				++idx;
				perHandle = perHandle->getNext();
			}
		} else {
			// first byte is the index the rest is the content
			auto* handle = flawless::util::LinkedList<flawless::ApplicationConfigBase>::get().mFirst;
			uint8_t idx = 0;
			uint8_t target = packet[0];
			while (handle) {
				if (target == idx and len == handle->getSize() + 1U) {
					handle->setValue(&(packet[1]));
					break;
				}
				++idx;
				handle = handle->getNext();
			}

			auto* perHandle = flawless::util::LinkedList<flawless::PersistentConfigBase>::get().mFirst;
			while (perHandle) {
				if (target == idx and len == perHandle->getSize() + 1U) {
					perHandle->setValue(&(packet[1]));
					break;
				}
				++idx;
				perHandle = perHandle->getNext();
			}
		}
	}
} appConfManager;

}

