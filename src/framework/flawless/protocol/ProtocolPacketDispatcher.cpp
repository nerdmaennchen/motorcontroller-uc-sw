/*
 * ProtocolPacketDispatcher.cpp
 *
 *  Created on: 04.09.2016
 *      Author: lutz
 */

#include <flawless/protocol/ProtocolPacketDispatcher.h>
#include <flawless/protocol/PacketHandler.h>
#include <flawless/module/Module.h>

namespace {

flawless::MessageBufferMemory<flawless::Packet_, 10> packetMessageBuffer;

class : public flawless::Listener<flawless::Packet_, MSG_ID_INCOMMING_PACKET> {
	void callback(flawless::Packet const& packet) override {
		if (packet->len > 0) {
			uint8_t ep = packet->buffer[0];

			auto* handler = flawless::util::LinkedList<flawless::PacketHandler>::get().mFirst;
			while (handler) {
				if (handler->mEPNum == ep) {
					handler->handlePacket(packet->iface, &(packet->buffer[1]), packet->len - 1);
				}
				handler = handler->mNext;
			}
		}
	}
} pcketListener;


}
