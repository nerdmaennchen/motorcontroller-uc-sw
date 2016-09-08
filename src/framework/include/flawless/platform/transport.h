/*
 * transport.h
 *
 *  Created on: Jun 8, 2012
 *      Author: lutz
 */

#ifndef TRANSPORT_H_
#define TRANSPORT_H_

#include <flawless/config/transport_config.h>
#include <flawless/stdtypes.h>


typedef uint8_t transportSymbol_t;

void transport_send_string(transport_device_values_t bus, const char *string);

void transport_send_bytes(transport_device_values_t bus, uint8_t dataLength, const transportSymbol_t* pData);

void transport_setBaudrate(transport_device_values_t device, uint32_t baudrate);

#endif
