#pragma once

#include <flawless/stdtypes.h>
#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>
#include <flawless/core/Message.h>
#include <flawless/config/flawLessProtocol_config.h>
#include <flawless/config/msgIDs.h>
#include <flawless/util/Array.h>

#include <flawless/protocol/PhyInterface.h>

namespace flawless
{

struct Packet_ {
	PhyInterface *iface;
	Array<uint8_t, FLAWLESS_PROTOCOL_MAX_PACKET_LEN> buffer;
	size_t len;
};
using Packet = flawless::Message<Packet_>;


} /* namespace flawless */

