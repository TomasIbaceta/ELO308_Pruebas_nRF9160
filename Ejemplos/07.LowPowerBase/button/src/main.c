/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * @brief This sets a led as gpio_output, a button as gpio_input with interrupts and connects a callback function to it. Then, it turns the led to turn
 * on when and while the button is pressed.
 * 
 * @notes ORIGIN: This is the example as-is, only minor changes to read better. 
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>


// For low power

#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

//for system-off.
#include <hal/nrf_power.h>

//for modem (just to disable it)
#include <modem/nrf_modem_lib.h>

/** ---------- My tests ---------- **/

#define REGISTER_LEVEL_SYSTEMOFF 0
#define DISABLE_UART_FOR_LOWER_POWER 0
#define DISABLE_MANY_PERIPHERALS     0
#define INIT_MODEM_TO_DISABLE     1


/** ---------- Defines ---------- **/

#define SLEEP_TIME_MS	1

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

/** ---------- Get power node  ---------- **/
#define MY_POWER DT_NODELABEL(power)

/** ---------- Functions ---------- **/

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	//printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {

		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	if (led.port && !device_is_ready(led.port)) {
		led.port = NULL;
	}
	if (led.port) {
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0) {

			led.port = NULL;
		} else {

		}
	}



/**
 * @brief This test turns on the System Off mode by going directly to the registers.
 * Reference 5.1.2 on the User manual for more information.
 * System off mode is supposed to be the highest energy saving mode there is. Everything is turned off
 * besides a couple of memory regions to save data. It's so low that after coming back from this mode it
 * has to reset.
 * I have not yet found conclusive evidence that this turns off all peripherals.
 * So far, when running this test, the device goes down to 2.5mA. This is still incredibly high,
 * operation here should be a couple of uA. Problem is, I know that disabling UART gives me around 500uA,
 * yet whether I disable uart or not this example goes down to 2.5mA anyways.
 * 
 * This is telling me that maybe 2.5mA is the baseline of whatever else the whole DK is running.
 * I'll try to get more conclusive evidence though.
 * 
 * UPDATE: Completely disabling the console from the prj.conf lowers the measurements to
 * 1.93 mA on another test, so the threshold can be lowered.
 * 
 * #completely remove UART.
	CONFIG_UART_CONSOLE=n
	CONFIG_SERIAL=n
 * 
 */
#if REGISTER_LEVEL_SYSTEMOFF
	//const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	//pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
	k_sleep(K_SECONDS(3) );
	printk("Force SystemOFF at the register level:\n");
	k_sleep(K_SECONDS(3) );
	
	#define BASE_ADDR 0x40004000
	#define SYSTEMOFF_OFFSET 0x500
	#define SYSTEMOFF_ADDR (BASE_ADDR + SYSTEMOFF_OFFSET)
	volatile uint32_t *systemoff_reg = (uint32_t *)SYSTEMOFF_ADDR;
	
	while(1){ //if it were to wakeup for some reason (it won't), go right back.
		*systemoff_reg = 1; //SystemOFF.
	}
#endif

/**
 * @brief This test confirms that power_manager_device API is working OK.
 * 
 * 
 */
#if DISABLE_UART_FOR_LOWER_POWER
// while(1){ //Doing this keeps a constant 3.20mA, doesn't change.
//	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
// 	pm_device_action_run(cons, PM_DEVICE_ACTION_TURN_OFF); //turn UART off.
// 	k_sleep(K_SECONDS(5) );
// 	pm_device_action_run(cons, PM_DEVICE_ACTION_TURN_ON); //turn UART on.
// 	k_sleep(K_SECONDS(5) );
// }

while(1){ //Doing this oscilates between 3.09 to 2.57mA, a 520uA difference.
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND); //Suspend UART.
	k_sleep(K_SECONDS(2) );
	pm_device_action_run(cons, PM_DEVICE_ACTION_RESUME); //Resume UART.
	k_sleep(K_SECONDS(2) );
}
#endif

#if DISABLE_MANY_PERIPHERALS
	// const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	// pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND); //Suspend UART.
	
	k_sleep(K_SECONDS(2) );
	while(1){
		k_sleep(K_SECONDS(2) );
	}
#endif

#if INIT_MODEM_TO_DISABLE
	int err = nrf_modem_lib_init();
    if (err) {
        printk("Modem initialization failed with error: %d\n", err);
		
		while(1){
			gpio_pin_set_dt(&led, 1);
			k_sleep(K_SECONDS(4) );
			gpio_pin_set_dt(&led, 0);
			k_sleep(K_SECONDS(4) );
		}
        return;
    }

    nrf_modem_lib_shutdown();
	while(1){
		gpio_pin_set_dt(&led, 1);
		k_sleep(K_SECONDS(1) );
		gpio_pin_set_dt(&led, 0);
		k_sleep(K_SECONDS(1) );
	}

#endif

while(1){
	/* force entry to deep sleep next time it goes into IDLE. */
	pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});

	gpio_pin_set_dt(&led, 1);
	k_sleep(K_SECONDS(5) ); //give up the thread, now the power module will send to SOFT_OFF
	gpio_pin_set_dt(&led, 0); //because it's system-off, the led should never turn off,
	                          //as process never gets to this line.
}

	// printk("Press the button\n");
	// if (led.port) {
	// 	while (1) {
	// 		/* If we have an LED, match its state to the button's. */
	// 		int val = gpio_pin_get_dt(&button);

	// 		if (val >= 0) {
	// 			gpio_pin_set_dt(&led, val);
	// 		}
	// 		k_msleep(SLEEP_TIME_MS);
	// 	}
	// }
	return 0;
}
