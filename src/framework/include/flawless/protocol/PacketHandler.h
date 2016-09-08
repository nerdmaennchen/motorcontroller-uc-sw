#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/LinkedList.h>
#include <flawless/protocol/PhyInterface.h>

namespace flawless
{

class PacketHandler : public flawless::util::LinkedListNode<PacketHandler> {
public:
	PacketHandler(uint8_t epNum) : mEPNum(epNum) {}

	virtual void handlePacket(PhyInterface* iface, uint8_t const* data, size_t len) = 0;

	const uint8_t mEPNum;
};
}
