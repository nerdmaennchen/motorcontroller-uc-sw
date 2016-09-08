/*
 * main.c
 *
 *  Created on: Oct 17, 2012
 *      Author: lutz
 */

#include <flawless/module/Module.h>

#include <flawless/core/MessagePump.h>

#include <interfaces/clock.h>
#include <interfaces/memory.h>

int main(void)
{
	clock_init();
	memory_init();
	flawless::InitializeModules();

	flawless::MessagePump::get().run();

	return 0;
}

void* __dso_handle;
