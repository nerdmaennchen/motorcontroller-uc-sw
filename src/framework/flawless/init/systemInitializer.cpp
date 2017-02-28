/*
 * systemInitializer.c
 *
 *  Created on: Apr 24, 2012
 *      Author: lutz
 */

#include <flawless/module/Module.h>
#include <flawless/stdtypes.h>

#include <algorithm>

void flawless::InitializeModules()
{
	ModuleBase* module = flawless::util::SortedList<ModuleBase>::get().mFirst;
	while (module) {
		module->init();
		module = module->mNext;
	}
}


