/*
 * genericFlawlessProtocol_phy.h
 *
 *  Created on: Jun 10, 2012
 *      Author: lutz
 */

/*
 * this is the interface for the physiacal interface implementation of the generic flawless protocol.
 *
 */

#ifndef GENERICFLAWLESSPROTOCOL_PHY_H_
#define GENERICFLAWLESSPROTOCOL_PHY_H_

#include <flawless/stdtypes.h>

/*
 * prototype for the raw send function
 *
 * @param data: the data to send.
 * @param packetLen: the amount of bytes to send
 */
typedef void (*phySendFunction_t)(const flawLessTransportSymbol_t *data, uint16_t packetLen);
typedef void (*phyStartOfFrameMarkerFunction)();
typedef void (*phyEndOfFrameMarkerFunction)();

typedef enum {
	GENRIC_PROTOCOL_CAPABILITIES_DEFALT,
	GENRIC_PROTOCOL_CAPABILITIES_CAN_PACKETIZE
} genericProtocol_PhyCapabilities;

typedef struct {
	phySendFunction_t sendFunction;
	phyStartOfFrameMarkerFunction frameStartFunction;
	phyEndOfFrameMarkerFunction frameEndFunction;
	genericProtocol_PhyCapabilities capabilities;
} phySendInfoStruct_t;

void registerPhySendFunction(phySendInfoStruct_t const* info, uint8_t interfaceNumber);

#endif /* GENERICFLAWLESSPROTOCOL_PHY_H_ */
