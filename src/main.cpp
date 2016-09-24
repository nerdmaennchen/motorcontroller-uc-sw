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

extern "C" {

void* __dso_handle;
void __cxa_pure_virtual() { while(1); }
void __cxa__atexit() { while(1); }
void atexit() {
}

}

