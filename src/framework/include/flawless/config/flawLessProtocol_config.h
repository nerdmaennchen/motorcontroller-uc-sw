#pragma once

#include <stdint.h>


/*
 * the maximum length of a single packet
 */
#define FLAWLESS_PROTOCOL_MAX_PACKET_LEN 512U

/*
 * those three chars are used for packet synchronization
 */

#define FLAWLESS_PROTOCOL_PACKET_CHAR_ESCAPE 0x3c
#define FLAWLESS_PROTOCOL_PACKET_CHAR_BEGINNING 0xf0
#define FLAWLESS_PROTOCOL_PACKET_CHAR_END 0x0f

/**
 * packets below this size are created in memory and sent as a single chunk.
 * bigger packets might be sent byte by byte.
 */
#define FLAWLESS_PROTOCOL_SHORT_PACKET_MAX_SIZE 256

typedef enum tag_flawLessProtocolStatus
{
	FLAWLESS_OK,
	FLAWLESS_ERROR,
	FLAWLESS_BUSY
} flawLessProtocolStatus_t;


/*
 * the interface descriptor for each phy interface.
 * Used by the higher layer to separate data sources
 */
typedef uint8_t flawLessInterfaceDescriptor_t;

typedef uint8_t genericProtocol_subProtocolIdentifier_t;


/*
 * the type of data to send
 */
typedef uint8_t flawLessTransportSymbol_t;

/*
 * include profiling of packet errors
 */
#define FLAWLESS_PROTOCOL_INTERFACE_STATISTICS 1
#define FLAWLESS_PROTOCOL_STATISTICS_INTERVAL_MS 5000

