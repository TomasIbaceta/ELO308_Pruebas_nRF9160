/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 * 
 * ---------------
 * Tomás Ibaceta.
 * @brief 
 * @note origin: this example is modified from the base code provided by nrf/samples/nrf9160/gnss.
 * it's modified so
 */

/** ---------- includes from the example ---------- **/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <date_time.h>

/** ---------- Includes to have a console ---------- **/
#include <zephyr/sys/printk.h>

/** ---------- Includes to have GPIO ---------- **/
#include <zephyr/drivers/gpio.h>
//#include <zephyr/device.h>


LOG_MODULE_REGISTER(gnss_sample, CONFIG_GNSS_SAMPLE_LOG_LEVEL);

#define PI 3.14159265358979323846
#define EARTH_RADIUS_METERS (6371.0 * 1000.0)

#if !defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE) || defined(CONFIG_GNSS_SAMPLE_MODE_TTFF_TEST)
static struct k_work_q gnss_work_q;

#define GNSS_WORKQ_THREAD_STACK_SIZE 2304
#define GNSS_WORKQ_THREAD_PRIORITY   5

K_THREAD_STACK_DEFINE(gnss_workq_stack_area, GNSS_WORKQ_THREAD_STACK_SIZE);
#endif /* !CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE || CONFIG_GNSS_SAMPLE_MODE_TTFF_TEST */

#if !defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE)
#include "assistance.h"

static struct nrf_modem_gnss_agps_data_frame last_agps;
static struct k_work agps_data_get_work;
static volatile bool requesting_assistance;
#endif /* !CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE */

#if defined(CONFIG_GNSS_SAMPLE_MODE_TTFF_TEST)
static struct k_work_delayable ttff_test_got_fix_work;
static struct k_work_delayable ttff_test_prepare_work;
static struct k_work ttff_test_start_work;
static uint32_t time_to_fix;
#endif

/** ---------- My Globals ---------- **/

static bool isTakingValidFixes = false;

/** ---------- Example Globals ---------- **/

static const char update_indicator[] = {'\\', '|', '/', '-'};

static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static uint64_t fix_timestamp;
static uint32_t time_blocked;

/* Reference position. */
static bool ref_used;
static double ref_latitude;
static double ref_longitude;

K_MSGQ_DEFINE(nmea_queue, sizeof(struct nrf_modem_gnss_nmea_data_frame *), 10, 4);
static K_SEM_DEFINE(pvt_data_sem, 0, 1);
static K_SEM_DEFINE(time_sem, 0, 1);

static struct k_poll_event events[2] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&pvt_data_sem, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&nmea_queue, 0),
};

BUILD_ASSERT(IS_ENABLED(CONFIG_LTE_NETWORK_MODE_LTE_M_GPS) ||
	     IS_ENABLED(CONFIG_LTE_NETWORK_MODE_NBIOT_GPS) ||
	     IS_ENABLED(CONFIG_LTE_NETWORK_MODE_LTE_M_NBIOT_GPS),
	     "CONFIG_LTE_NETWORK_MODE_LTE_M_GPS, "
	     "CONFIG_LTE_NETWORK_MODE_NBIOT_GPS or "
	     "CONFIG_LTE_NETWORK_MODE_LTE_M_NBIOT_GPS must be enabled");

BUILD_ASSERT((sizeof(CONFIG_GNSS_SAMPLE_REFERENCE_LATITUDE) == 1 &&
	      sizeof(CONFIG_GNSS_SAMPLE_REFERENCE_LONGITUDE) == 1) ||
	     (sizeof(CONFIG_GNSS_SAMPLE_REFERENCE_LATITUDE) > 1 &&
	      sizeof(CONFIG_GNSS_SAMPLE_REFERENCE_LONGITUDE) > 1),
	     "CONFIG_GNSS_SAMPLE_REFERENCE_LATITUDE and "
	     "CONFIG_GNSS_SAMPLE_REFERENCE_LONGITUDE must be both either set or empty");

static void gnss_event_handler(int event)
{
	int retval;
	struct nrf_modem_gnss_nmea_data_frame *nmea_data;

	switch (event) {
	case NRF_MODEM_GNSS_EVT_PVT:
		retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
		if (retval == 0) {
			k_sem_give(&pvt_data_sem);
		}
		break;

	case NRF_MODEM_GNSS_EVT_NMEA:
		nmea_data = k_malloc(sizeof(struct nrf_modem_gnss_nmea_data_frame));
		if (nmea_data == NULL) {
			printk("Failed to allocate memory for NMEA\n");
			break;
		}

		retval = nrf_modem_gnss_read(nmea_data,
					     sizeof(struct nrf_modem_gnss_nmea_data_frame),
					     NRF_MODEM_GNSS_DATA_NMEA);
		if (retval == 0) {
			retval = k_msgq_put(&nmea_queue, &nmea_data, K_NO_WAIT);
		}

		if (retval != 0) {
			k_free(nmea_data);
		}
		break;

	case NRF_MODEM_GNSS_EVT_AGPS_REQ:
#if !defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE)
		retval = nrf_modem_gnss_read(&last_agps,
					     sizeof(last_agps),
					     NRF_MODEM_GNSS_DATA_AGPS_REQ);
		if (retval == 0) {
			k_work_submit_to_queue(&gnss_work_q, &agps_data_get_work);
		}
#endif /* !CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE */
		break;

	default:
		break;
	}
}

#if !defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE)
#if defined(CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND)
K_SEM_DEFINE(lte_ready, 0, 1);

static void lte_lc_event_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
		    (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			printk("Connected to LTE network");
			k_sem_give(&lte_ready);
		}
		break;

	default:
		break;
	}
}

void lte_connect(void)
{
	int err;

	printk("Connecting to LTE network");

	err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_LTE);
	if (err) {
		printk("Failed to activate LTE, error: %d", err);
		return;
	}

	k_sem_take(&lte_ready, K_FOREVER);

	/* Wait for a while, because with IPv4v6 PDN the IPv6 activation takes a bit more time. */
	k_sleep(K_SECONDS(1));
}

void lte_disconnect(void)
{
	int err;

	err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);
	if (err) {
		printk("Failed to deactivate LTE, error: %d", err);
		return;
	}

	printk("LTE disconnected");
}
#endif /* CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND */

static void agps_data_get_work_fn(struct k_work *item)
{
	ARG_UNUSED(item);

	int err;

#if defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_SUPL)
	/* SUPL doesn't usually provide NeQuick ionospheric corrections and satellite real time
	 * integrity information. If GNSS asks only for those, the request should be ignored.
	 */
	if (last_agps.sv_mask_ephe == 0 &&
	    last_agps.sv_mask_alm == 0 &&
	    (last_agps.data_flags & ~(NRF_MODEM_GNSS_AGPS_NEQUICK_REQUEST |
				      NRF_MODEM_GNSS_AGPS_INTEGRITY_REQUEST)) == 0) {
		printk("Ignoring assistance request for only NeQuick and/or integrity\n");
		return;
	}
#endif /* CONFIG_GNSS_SAMPLE_ASSISTANCE_SUPL */

#if defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_MINIMAL)
	/* With minimal assistance, the request should be ignored if no GPS time or position
	 * is requested.
	 */
	if (!(last_agps.data_flags & NRF_MODEM_GNSS_AGPS_SYS_TIME_AND_SV_TOW_REQUEST) &&
	    !(last_agps.data_flags & NRF_MODEM_GNSS_AGPS_POSITION_REQUEST)) {
		printk("Ignoring assistance request because no GPS time or position is requested\n");
		return;
	}
#endif /* CONFIG_GNSS_SAMPLE_ASSISTANCE_MINIMAL */

	requesting_assistance = true;

	printk("Assistance data needed, ephe 0x%08x, alm 0x%08x, flags 0x%02x\n",
		last_agps.sv_mask_ephe,
		last_agps.sv_mask_alm,
		last_agps.data_flags);

#if defined(CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND)
	lte_connect();
#endif /* CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND */

	err = assistance_request(&last_agps);
	if (err) {
		printk("Failed to request assistance data\n");
	}

#if defined(CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND)
	lte_disconnect();
#endif /* CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND */

	requesting_assistance = false;
}
#endif /* !CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE */

static void date_time_evt_handler(const struct date_time_evt *evt)
{
	k_sem_give(&time_sem);
}

static int modem_init(void)
{
	if (IS_ENABLED(CONFIG_DATE_TIME)) {
		date_time_register_handler(date_time_evt_handler);
	}

	if (lte_lc_init() != 0) {
		printk("Failed to initialize LTE link controller");
		return -1;
	}

#if defined(CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND)
	lte_lc_register_handler(lte_lc_event_handler);
#elif !defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE)
	lte_lc_psm_req(true);

	printk("Connecting to LTE network\n");

	if (lte_lc_connect() != 0) {
		printk("Failed to connect to LTE network\n");
		return -1;
	}

	printk("Connected to LTE network\n");

	if (IS_ENABLED(CONFIG_DATE_TIME)) {
		printk("Waiting for current time\n");

		/* Wait for an event from the Date Time library. */
		k_sem_take(&time_sem, K_MINUTES(10));

		if (!date_time_is_valid()) {
			printk("Failed to get current time, continuing anyway\n");
		}
	}
#endif

	return 0;
}

static int sample_init(void)
{
	int err = 0;

#if !defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE) || defined(CONFIG_GNSS_SAMPLE_MODE_TTFF_TEST)
	struct k_work_queue_config cfg = {
		.name = "gnss_work_q",
		.no_yield = false
	};

	k_work_queue_start(
		&gnss_work_q,
		gnss_workq_stack_area,
		K_THREAD_STACK_SIZEOF(gnss_workq_stack_area),
		GNSS_WORKQ_THREAD_PRIORITY,
		&cfg);
#endif /* !CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE || CONFIG_GNSS_SAMPLE_MODE_TTFF_TEST */

#if !defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE)
	k_work_init(&agps_data_get_work, agps_data_get_work_fn);

	err = assistance_init(&gnss_work_q);
#endif /* !CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE */

#if defined(CONFIG_GNSS_SAMPLE_MODE_TTFF_TEST)
	k_work_init_delayable(&ttff_test_got_fix_work, ttff_test_got_fix_work_fn);
	k_work_init_delayable(&ttff_test_prepare_work, ttff_test_prepare_work_fn);
	k_work_init(&ttff_test_start_work, ttff_test_start_work_fn);
#endif /* CONFIG_GNSS_SAMPLE_MODE_TTFF_TEST */

	return err;
}

static int gnss_init_and_start(void)
{
#if defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE) || defined(CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND)
	/* Enable GNSS. */
	if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS) != 0) {
		printk("Failed to activate GNSS functional mode");
		return -1;
	}
#endif /* CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE || CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND */

	/* Configure GNSS. */
	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0) {
		printk("Failed to set GNSS event handler");
		return -1;
	}

	/* Enable all supported NMEA messages. */
	uint16_t nmea_mask = NRF_MODEM_GNSS_NMEA_RMC_MASK |
			     NRF_MODEM_GNSS_NMEA_GGA_MASK |
			     NRF_MODEM_GNSS_NMEA_GLL_MASK |
			     NRF_MODEM_GNSS_NMEA_GSA_MASK |
			     NRF_MODEM_GNSS_NMEA_GSV_MASK;

	if (nrf_modem_gnss_nmea_mask_set(nmea_mask) != 0) {
		printk("Failed to set GNSS NMEA mask");
		return -1;
	}

	/* This use case flag should always be set. */
	uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;

	if (IS_ENABLED(CONFIG_GNSS_SAMPLE_MODE_PERIODIC) &&
	    !IS_ENABLED(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE)) {
		/* Disable GNSS scheduled downloads when assistance is used. */
		use_case |= NRF_MODEM_GNSS_USE_CASE_SCHED_DOWNLOAD_DISABLE;
	}

	if (IS_ENABLED(CONFIG_GNSS_SAMPLE_LOW_ACCURACY)) {
		use_case |= NRF_MODEM_GNSS_USE_CASE_LOW_ACCURACY;
	}

	if (nrf_modem_gnss_use_case_set(use_case) != 0) {
		printk("Failed to set GNSS use case");
	}

#if defined(CONFIG_NRF_CLOUD_AGPS_ELEVATION_MASK)
	if (nrf_modem_gnss_elevation_threshold_set(CONFIG_NRF_CLOUD_AGPS_ELEVATION_MASK) != 0) {
		printk("Failed to set elevation threshold");
		return -1;
	}
	LOG_DBG("Set elevation threshold to %u", CONFIG_NRF_CLOUD_AGPS_ELEVATION_MASK);
#endif

#if defined(CONFIG_GNSS_SAMPLE_MODE_CONTINUOUS)
	/* Default to no power saving. */
	uint8_t power_mode = NRF_MODEM_GNSS_PSM_DISABLED;

#if defined(GNSS_SAMPLE_POWER_SAVING_MODERATE)
	power_mode = NRF_MODEM_GNSS_PSM_DUTY_CYCLING_PERFORMANCE;
#elif defined(GNSS_SAMPLE_POWER_SAVING_HIGH)
	power_mode = NRF_MODEM_GNSS_PSM_DUTY_CYCLING_POWER;
#endif

	if (nrf_modem_gnss_power_mode_set(power_mode) != 0) {
		printk("Failed to set GNSS power saving mode");
		return -1;
	}
#endif /* CONFIG_GNSS_SAMPLE_MODE_CONTINUOUS */

	/* Default to continuous tracking. */
	uint16_t fix_retry = 0;
	uint16_t fix_interval = 1;

#if defined(CONFIG_GNSS_SAMPLE_MODE_PERIODIC)
	fix_retry = CONFIG_GNSS_SAMPLE_PERIODIC_TIMEOUT;
	fix_interval = CONFIG_GNSS_SAMPLE_PERIODIC_INTERVAL;
#elif defined(CONFIG_GNSS_SAMPLE_MODE_TTFF_TEST)
	/* Single fix for TTFF test mode. */
	fix_retry = 0;
	fix_interval = 0;
#endif

	if (nrf_modem_gnss_fix_retry_set(fix_retry) != 0) {
		printk("Failed to set GNSS fix retry");
		return -1;
	}

	if (nrf_modem_gnss_fix_interval_set(fix_interval) != 0) {
		printk("Failed to set GNSS fix interval");
		return -1;
	}

#if defined(CONFIG_GNSS_SAMPLE_MODE_TTFF_TEST)
	k_work_schedule_for_queue(&gnss_work_q, &ttff_test_prepare_work, K_NO_WAIT);
#else /* !CONFIG_GNSS_SAMPLE_MODE_TTFF_TEST */
	if (nrf_modem_gnss_start() != 0) {
		printk("Failed to start GNSS");
		return -1;
	}
#endif

	return 0;
}

static bool output_paused(void)
{
#if defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE) || defined(CONFIG_GNSS_SAMPLE_LOG_LEVEL_OFF)
	return false;
#else
	return (requesting_assistance || assistance_is_active()) ? true : false;
#endif
}

static void print_satellite_stats(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	uint8_t tracked   = 0;
	uint8_t in_fix    = 0;
	uint8_t unhealthy = 0;

	for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i) {
		if (pvt_data->sv[i].sv > 0) {
			tracked++;

			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) {
				in_fix++;
			}

			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_UNHEALTHY) {
				unhealthy++;
			}
		}
	}
	if(isTakingValidFixes == false){ //I don't want to print extra data when having valid fixes.
		printk("Tracking: %2d Using: %2d Unhealthy: %d\n", tracked, in_fix, unhealthy);
	}
	
}

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	printk("  (%.06f, %.06f), \n", pvt_data->latitude, pvt_data->longitude);
	return;
}

static void print_fix_data_old(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	printk("Latitude:       %.06f\n", pvt_data->latitude);
	printk("Longitude:      %.06f\n", pvt_data->longitude);
	printk("Altitude:       %.01f m\n", pvt_data->altitude);
	printk("Accuracy:       %.01f m\n", pvt_data->accuracy);
	printk("Speed:          %.01f m/s\n", pvt_data->speed);
	printk("Speed accuracy: %.01f m/s\n", pvt_data->speed_accuracy);
	printk("Heading:        %.01f deg\n", pvt_data->heading);
	printk("Date:           %04u-%02u-%02u\n",
	       pvt_data->datetime.year,
	       pvt_data->datetime.month,
	       pvt_data->datetime.day);
	printk("Time (UTC):     %02u:%02u:%02u.%03u\n",
	       pvt_data->datetime.hour,
	       pvt_data->datetime.minute,
	       pvt_data->datetime.seconds,
	       pvt_data->datetime.ms);
	printk("PDOP:           %.01f\n", pvt_data->pdop);
	printk("HDOP:           %.01f\n", pvt_data->hdop);
	printk("VDOP:           %.01f\n", pvt_data->vdop);
	printk("TDOP:           %.01f\n", pvt_data->tdop);
}

/** ---------- GPIO Configuration ---------- **/

#define SW0_NODE	DT_ALIAS(sw0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

static bool isButtonPressed = false;
void button_pressed_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	isButtonPressed = true;
}

/**
 * @brief inits and configure the button
 * 
 * @param button 
 * @return int 0 if OK, negative values for some error
 */
int my_button_init(struct gpio_dt_spec* button){

	if (!gpio_is_ready_dt(button)) {
		printk("Error: button device %s is not ready\n",
		       button->port->name);
		return -1;
	}
	int ret = 0;
	ret = gpio_pin_configure_dt(button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button->port->name, button->pin);
		return -2;
	}

	ret = gpio_pin_interrupt_configure_dt(button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button->port->name, button->pin);
		return -3;
	}
	return 0;
	
}

/**
 * @brief inits and configure the led
 * 
 * @param led 
 * @return int 0 if OK, negative values for errors.
 */
int my_led_init(struct gpio_dt_spec *led){
	int ret=0;
	if (led->port && !device_is_ready(led->port)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led->port->name);
		led->port = NULL;
		return -1;
	}
	if (led->port) {
		ret = gpio_pin_configure_dt(led, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led->port->name, led->pin);
			led->port = NULL;
			return -2;
		} else { //init and config OK
			return 0;
		}
	}
}

/** ---------- My Functions ---------- **/

/**
 * @brief there are issues with having the console-out on while debugging, so I need time to plug it in.
 * 
 */
void init_sequence(void){
	printf("3...");
	printk("3...");
	k_msleep(1000);

	printf("2...");
	printk("2...");
	k_msleep(1000);

	printf("1...");
	printk("1...\n");
	k_msleep(1000);

	return;
}

/**
 * @brief This application uses ANSI escape codes to manipulate the terminals.
 * \033 octal is the ESC character. then [ and then code.
 * 1;1H sets the cursore on row 1, column 1 (top left corner)
 * 2J clears the screen from the cursor position onwards.
 * 
 * So, these two commands "reset the terminal screen" if it supports ANSI escape codes.
 */
void reset_screen(void){
	printk("\033[1;1H");
	printk("\033[2J");
	return;
}



/** ---------- Main ---------- **/
int main(void)
{
	
	int err;
	uint8_t cnt = 0;
	struct nrf_modem_gnss_nmea_data_frame *nmea_data;

	init_sequence();
	k_msleep(2000);
	printk("Starting GNSS sample\n");

	/** ---------- Init and config button ---------- **/
	int retVal = 0;
	retVal = my_button_init(&button);
	if( retVal !=0 ){ //This is just a test, it's ok the program just ends if something fails here.
		printk("button init failed: &d\n", retVal);
		return 0;
	}

	/** ---------- Register Callback ---------- **/
	gpio_init_callback(&button_cb_data, button_pressed_callback, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	/** ---------- Init and config LED ---------- **/
	retVal = my_led_init(&led);
	if(retVal != 0){
		printk("led init failed: &d\n", retVal);
		return 0;
	}
	gpio_pin_set_dt(&led, 0); //turn off led
	printk("Waiting for button press\n");

	//------------- Other Power Inits

	nrf_modem_lib_init();
	nrf_modem_lib_shutdown();

	//------------------------------

	while(1){ //wait for button press.
		if(isButtonPressed == true){
			isButtonPressed = false;
			break;
		}
		k_yield();
	}
	printk("button pressed!\n");

	gpio_pin_set_dt(&led, 1);
	k_sleep(K_SECONDS(1) );
	gpio_pin_set_dt(&led, 0);

	err = nrf_modem_lib_init();
	if (err) {
		printk("Modem library initialization failed, error: %d\n", err);
		return err;
	}

	/* Initialize reference coordinates (if used). */
	if (sizeof(CONFIG_GNSS_SAMPLE_REFERENCE_LATITUDE) > 1 &&
	    sizeof(CONFIG_GNSS_SAMPLE_REFERENCE_LONGITUDE) > 1) {
		ref_used = true;
		ref_latitude = atof(CONFIG_GNSS_SAMPLE_REFERENCE_LATITUDE);
		ref_longitude = atof(CONFIG_GNSS_SAMPLE_REFERENCE_LONGITUDE);
	}

	if (modem_init() != 0) {
		printk("Failed to initialize modem\n");
		return -1;
	}

	if (sample_init() != 0) {
		printk("Failed to initialize sample\n");
		return -1;
	}

	if (gnss_init_and_start() != 0) {
		printk("Failed to initialize and start GNSS\n");
		return -1;
	}

	nrf_modem_gnss_prio_mode_enable();

	printk("All init OK\n");

	fix_timestamp = k_uptime_get();

	for (;;) {
		(void)k_poll(events, 2, K_FOREVER);

		if (events[0].state == K_POLL_STATE_SEM_AVAILABLE &&
		    k_sem_take(events[0].sem, K_NO_WAIT) == 0) {
			/* New PVT data available */

			if (output_paused()) {
				goto handle_nmea;
			}

			// reset_screen(); // we are not resetting the screen anymore because  we want to copypaste the valid fixes.
			print_satellite_stats(&last_pvt); //read the flags to see that happened

			/** ---------- Manage/Log Errors ---------- **/
			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED) {
				printk("GNSS operation blocked by LTE\n");
			}
			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME) {
				printk("Insufficient GNSS time windows\n");
			}
			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_SLEEP_BETWEEN_PVT) {
				printk("Sleep period(s) between PVT notifications\n");
			}


			//printk("-----------\n");

			/** ---------- If got a valid fix, print the data ---------- **/

			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
				isTakingValidFixes = true;
				gpio_pin_set_dt(&led, 1);
				fix_timestamp = k_uptime_get();
				print_fix_data(&last_pvt);
			} else { // if no valid fix, print more logs
				isTakingValidFixes = false;
				gpio_pin_set_dt(&led, 0);
				printk("Seconds since last fix: %d\n",
						(uint32_t)((k_uptime_get() - fix_timestamp) / 1000));
				cnt++;
				printk("Searching [%c]\n", update_indicator[cnt%4]);
			}
			
		}

handle_nmea:
		if (events[1].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE &&
		    k_msgq_get(events[1].msgq, &nmea_data, K_NO_WAIT) == 0) {
			/* New NMEA data available */

			if (!output_paused()) {
				//printk("%s", nmea_data->nmea_str); 
			}
			k_free(nmea_data);
		}

		events[0].state = K_POLL_STATE_NOT_READY;
		events[1].state = K_POLL_STATE_NOT_READY;
	}

	return 0;
}
