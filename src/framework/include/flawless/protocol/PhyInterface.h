#pragma once

#include <flawless/util/LinkedList.h>
#include <flawless/stdtypes.h>

#include <flawless/protocol/PhyInterface.h>

namespace flawless
{

class PhyInterface : public flawless::util::LinkedListNode<PhyInterface> {
public:
	PhyInterface(uint8_t ifaceNo) : mIfaceNo(ifaceNo) {}

	virtual void startPacket(uint8_t, uint16_t) {}
	virtual void sendPacket(void const*, uint16_t) {}
	virtual void sendPacket(uint8_t epNum, void const* msg, uint16_t len) = 0;

	const uint8_t mIfaceNo;
};

}
