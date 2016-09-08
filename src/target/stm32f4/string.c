/*
 * string.c
 *
 *  Created on: Jul 17, 2012
 *      Author: lutz
 */

#include <flawless/stdtypes.h>
#include <string.h>

void * memset(void *dst, int val, size_t len)
{
	uint32_t i = 0U;
	uint8_t *dstPtr = (uint8_t*)dst;
	for (i = 0U; i < len; ++i)
	{
		*dstPtr = val;
		++dstPtr;
	}
	return dst;
}


void * memcpy(void *dst, const void *src, size_t count)
{
	uint32_t i = 0U;
	uint8_t *dstPtr = (uint8_t*)dst;
	const uint8_t *srcPtr = (uint8_t*)src;
	for (i = 0U; i < count; ++i)
	{
		*dstPtr = *srcPtr;
		++dstPtr;
		++srcPtr;
	}
	return dst;
}
