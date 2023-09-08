/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

/* To strictly comply with UART timing, enable external XTAL oscillator */
void enable_xtal(void)
{
	struct onoff_manager *clk_mgr;
	static struct onoff_client cli = {};

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	sys_notify_init_spinwait(&cli.notify);
	(void)onoff_request(clk_mgr, &cli);
}

void readModemResponse(void)
{
        int err;
        char response[64];

        err = nrf_modem_at_cmd(response, sizeof(response), "AT+CGMI"); //Request Manufacture information
        if (err) {
                /* error */
        }

        /* buffer contains the whole response */
        printf("Modem response:\n%s", response);
}

/**
 * @brief Allows to send a command to the modem, the response will be printed on the screen.
 * 
 * @param command the AT command to be sent, for example: myModem_SendCommandPrintResponse("AT+CGMI");
 * you could format strings, or use pointer to char, just make sure that you're sending a proper string instead
 * of a simple character array.
 */
void myModem_SendCommandPrintResponse(char* command){
	int err;
	char response[64];

	err = nrf_modem_at_cmd(response, sizeof(response), command); //Request Manufacture information
	if (err) {
		printk("Error! my_readModemResponse\n error line: %s\n", command);
		return;
	}

	/* buffer contains the whole response */
	printk("Command: %s\nModem response: %s\n", command, response);
	return;
}

int main(void)
{
	int err;

	printk("The AT host sample started - 1\n");
	k_msleep(2000);
	printk("The AT host sample started - 2\n");

	err = nrf_modem_lib_init();
	if (err) {
		printk("Modem library initialization failed, error: %d\n", err);
		return 0;
	}
	enable_xtal();
	printk("Ready\n");

	if (!nrf_modem_is_initialized){
		printk("modem not init\n");
	}

//main.c
	printk("modem init check OK\n");
	
	//All the following commands work OK.
	printk("MyCommand Test 1:\n");
	myModem_SendCommandPrintResponse("AT+CGMI");

	printk("MyCommand Test 4:\n");
	myModem_SendCommandPrintResponse("AT%%XCOEX0?"); 

	printk("MyCommand Test 5:\n");
	myModem_SendCommandPrintResponse("AT%%XMAGPIO?");

	myModem_SendCommandPrintResponse("AT+CGMR");

	//The following Syntax does not work

	// printk("MyCommand Test 2:\n");
	// myModem_SendCommandPrintResponse("AT%XCOEX0?");

	// printk("MyCommand Test 3:\n");
	// myModem_SendCommandPrintResponse("AT\%XCOEX0?");

	



	return 0;
}
