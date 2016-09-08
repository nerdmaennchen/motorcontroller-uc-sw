/*
 * configuration.h
 *
 *  Created on: Mar 28, 2013
 *      Author: lutz
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_


#define CONFIG_VARIABLE(type, name, defaultValue) \
	__attribute__((unused))\
	__attribute__ ((section(".applicationConfig"))) \
	static type name = defaultValue;

#define CONFIG_VARIABLE2(type, name) \
	__attribute__((unused))\
	__attribute__ ((section(".applicationConfig"))) \
	static type name


void config_updateToFlash();


#endif /* CONFIGURATION_H_ */
