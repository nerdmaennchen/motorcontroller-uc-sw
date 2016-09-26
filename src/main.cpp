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


#include <cstdlib>
extern "C" {

void* __dso_handle;
void __cxa_pure_virtual() { while(1); }
// this function is intended to register functions that need to be called upon unloading of a shared library or upon exit()...
// we dont need this here -> empty implementation
int __cxa_atexit(void (*) (void *), void *, void *) {return 0;}
void __register_exitproc() {}
void free (void*) { while(1); }
void* malloc(size_t) { while(1); return nullptr;}

int atexit(void (*)()) { return 0;}

}

