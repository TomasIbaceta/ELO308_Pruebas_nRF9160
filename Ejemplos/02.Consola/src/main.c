/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 * ---------------
 * Tomás Ibaceta.
 * @brief Classic hello world type test to check for console access.
 * @note origin: this example is modified from the base code provided by nrf/samples/nrf9160/hello_world.
 * Is has no modifications at all.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
int main(void)
{
	while(1){
		printk("Hello World! \n");
		k_msleep(2000);
	}
	
	return 0;
}
