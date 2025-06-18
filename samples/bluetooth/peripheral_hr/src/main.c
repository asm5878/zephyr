/* main.c - Application main entry point */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#define ST_TRACE 0

static bool hrf_ntf_enabled;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
#if defined(CONFIG_BT_EXT_ADV)
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
#endif /* CONFIG_BT_EXT_ADV */
};

#if !defined(CONFIG_BT_EXT_ADV)
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
#endif /* !CONFIG_BT_EXT_ADV */

/* Use atomic variable, 2 bits for connection and disconnection state */
static ATOMIC_DEFINE(state, 2U);

#define STATE_CONNECTED    1U
#define STATE_DISCONNECTED 2U

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		printk("Connected\n");

		(void)atomic_set_bit(state, STATE_CONNECTED);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	(void)atomic_set_bit(state, STATE_DISCONNECTED);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void hrs_ntf_changed(bool enabled)
{
	hrf_ntf_enabled = enabled;

	printk("HRS notification status changed: %s\n",
	       enabled ? "enabled" : "disabled");
}

static struct bt_hrs_cb hrs_cb = {
	.ntf_changed = hrs_ntf_changed,
};

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}

static void hrs_notify(void)
{
	static uint8_t heartrate = 90U;

	/* Heartrate measurements simulation */
	heartrate++;
	if (heartrate == 160U) {
		heartrate = 90U;
	}

	if (hrf_ntf_enabled) {
		bt_hrs_notify(heartrate);
	}
}

#if defined(CONFIG_GPIO)
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS_OKAY(LED0_NODE)
#include <zephyr/drivers/gpio.h>
// #define HAS_LED     1
#if defined(HAS_LED)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define BLINK_ONOFF K_MSEC(500)

static struct k_work_delayable blink_work;
static bool                  led_is_on;

static void blink_timeout(struct k_work *work)
{
	led_is_on = !led_is_on;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);

	k_work_schedule(&blink_work, BLINK_ONOFF);
}

static int blink_setup(void)
{
	int err;

	printk("Checking LED device...");
	if (!gpio_is_ready_dt(&led)) {
		printk("failed.\n");
		return -EIO;
	}
	printk("done.\n");

	printk("Configuring GPIO pin...");
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err) {
		printk("failed.\n");
		return -EIO;
	}
	printk("done.\n");

	k_work_init_delayable(&blink_work, blink_timeout);

	return 0;
}

static void blink_start(void)
{
	printk("Start blinking LED...\n");
	led_is_on = false;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);
	k_work_schedule(&blink_work, BLINK_ONOFF);
}

static void blink_stop(void)
{
	struct k_work_sync work_sync;

	printk("Stop blinking LED.\n");
	k_work_cancel_delayable_sync(&blink_work, &work_sync);

	/* Keep LED on */
	led_is_on = true;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);
}
#endif
#endif /* LED0_NODE */
#endif /* CONFIG_GPIO */
# if (ST_TRACE == 1)
typedef struct evt_s {
  int evt_code;
  uint32_t evt_data;
  uint32_t evt_data_aux;
} evt_t;

char *evt_strings[] = {
  /* 0 */ "sys_clock_announce",
  /* 1 */ ">lptim_irq_handler",
  /* 2 */ ">sys_clock_set_timeout",
  /* 3 */ ">arch_suspend_to_ram",
  /* 4 */ "<arch_suspend_to_ram",
  /* 5 */ ">suspend_to_ram",
  /* 6 */ "<suspend_to_ram",
  /* 7 */ "Csys_clock_announce",
  /* 8 */ "update_cache",
  /* 9 */ "Csys_clock_announce1",
  /* 10 */ ">sys_clock_idle_exit",
  /* 11 */ "<sys_clock_idle_exit",
  /* 12 */ ">pm_system_suspend",
  /* 13 */ "pm_system_suspend WU kernel_ticks=%d, dummy=%d",
  /* 14 */ "<pm_system_suspend(true)",
  /* 15 */ ">pm_system_resume",
  /* 16 */ "<pm_system_resume",
  /* 17 */ "<pm_system_suspend(false)",
  /* 18 */ ">_isr_wrapper",
  /* 19 */ "<_isr_wrapper",
  /* 20 */ "_isr_wrapper(WU from idle)",
  /* 21 */ "main->sleep last_uptime=%d, dummy=%d",
  /* 22 */ "main->wkup",
  /* 23 */ "lptim ck off",
  /* 24 */ "lptim ck on",
  /* 25 */ "sys_clock_set_timeout debug",
  /* 26 */ "lptim_irq_handler debug",
  /* 27 */ "<lptim_irq_handler",
  /* 28 */ "pm_system_suspend debug",
  /* 29 */ ">pm_suspend_devices",
  /* 30 */ "<pm_suspend_devices",
  /* 31 */ "pm_suspend_devices debug",
  /* 32 */ ">lptim_set_autoreload",
  /* 33 */ "<lptim_set_autoreload",
  /* 34 */ ">z_reset_time_slice",
  /* 35 */ "<z_reset_time_slice",
  /* 36 */ "z_reset_time_slice debug",
  /* 37 */ ">unready_thread",
  /* 38 */ "<unready_thread",
  /* 39 */ ">z_tick_sleep",
  /* 40 */ "<z_tick_sleep",
  /* 41 */ "idle debug",
  /* 42 */ ">sys_clock_driver_init",
  /* 43 */ "<sys_clock_driver_init",
  /* 44 */ ">set_mode_stop, substate_id=%d, dummy=%08x",
  /* 45 */ "<set_mode_stop, substate_id=%d, dummy=%08x",
  /* 46 */ ">Clock_Switching",
  /* 47 */ "<Clock_Switching",
  /* 48 */ ">radio_high_prio_isr",
  /* 49 */ "<radio_high_prio_isr",
  /* 50 */ ">radio_low_prio_isr",
  /* 51 */ "<radio_low_prio_isr",
  /* 52 */ "stop 1 wake-up",
  
};
#define EVENTS_SIZE 2048
evt_t events[EVENTS_SIZE];
int evt_count=0;
void add_event(int code, uint32_t val1, uint32_t val2) {
  if (evt_count < EVENTS_SIZE) {
    events[evt_count].evt_code = code;
    events[evt_count].evt_data = val1;
    events[evt_count].evt_data_aux = val2;
    evt_count++;
  }
}
#define PRE_FORMAT "%-4d(%-2d) "
int print_events(int start)
{
  volatile int limit = evt_count;
  printf ("%-4s %-32s %-16s %-16s\n", "ID", "CODE", "DATA", "AUX");
  for (int i=start; i < limit; i++) {
	char format[128];  
    char * message = "Unknown";
    if (events[i].evt_code < sizeof(evt_strings)/sizeof(char*)) {
      message = evt_strings[events[i].evt_code];
	}
	if (strchr(message, '%') == NULL) {
		  sprintf(format,
					"%s%-32s %s\n",
					PRE_FORMAT,
					message,
					"data=%08x, data_aux=%08x");
	} else {
		  sprintf(format, "%s%s\n", PRE_FORMAT, message);
	}
    printf (format, 
			i,
			events[i].evt_code,
			events[i].evt_data,
			events[i].evt_data_aux);
  }
  return limit;
}
#endif /* ST_TRACE */
int main(void)
{
	int err;
#if (ST_TRACE == 1)
	int start = 0;
	int count = 0;
#endif
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	bt_conn_auth_cb_register(&auth_cb_display);

	bt_hrs_cb_register(&hrs_cb);

#if !defined(CONFIG_BT_EXT_ADV)
	printk("Starting Legacy Advertising (connectable and scannable)\n");
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

#else /* CONFIG_BT_EXT_ADV */
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0U,
		.secondary_max_skip = 0U,
		.options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_CODED),
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};
	struct bt_le_ext_adv *adv;

	printk("Creating a Coded PHY connectable non-scannable advertising set\n");
	err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
	if (err) {
		printk("Failed to create Coded PHY extended advertising set (err %d)\n", err);

		printk("Creating a non-Coded PHY connectable non-scannable advertising set\n");
		adv_param.options &= ~BT_LE_ADV_OPT_CODED;
		err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
		if (err) {
			printk("Failed to create extended advertising set (err %d)\n", err);
			return 0;
		}
	}

	printk("Setting extended advertising data\n");
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set extended advertising data (err %d)\n", err);
		return 0;
	}

	printk("Starting Extended Advertising (connectable non-scannable)\n");
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising set (err %d)\n", err);
		return 0;
	}
#endif /* CONFIG_BT_EXT_ADV */

#if (ST_TRACE == 1)
	k_busy_wait(5000000);
#endif
	printk("Advertising successfully started\n");

#if defined(HAS_LED)
	err = blink_setup();
	if (err) {
		return 0;
	}

	blink_start();
#endif /* HAS_LED */
	/* Implement notification. */
	while (1) {
#if (ST_TRACE == 1)
		add_event(21, k_uptime_ticks(), 0);
#endif
		k_sleep(K_SECONDS(1));
#if (ST_TRACE == 1)
		add_event(22, 0, 0);
		printk("Wakeup %d\n", count++);
		// start = print_events(start);
#endif
		/* Heartrate measurements simulation */
		hrs_notify();

		/* Battery level simulation */
		bas_notify();

		if (atomic_test_and_clear_bit(state, STATE_CONNECTED)) {
			/* Connected callback executed */

#if defined(HAS_LED)
			blink_stop();
#endif /* HAS_LED */
		} else if (atomic_test_and_clear_bit(state, STATE_DISCONNECTED)) {
#if !defined(CONFIG_BT_EXT_ADV)
			printk("Starting Legacy Advertising (connectable and scannable)\n");
			err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd,
					      ARRAY_SIZE(sd));
			if (err) {
				printk("Advertising failed to start (err %d)\n", err);
				return 0;
			}

#else /* CONFIG_BT_EXT_ADV */
			printk("Starting Extended Advertising (connectable and non-scannable)\n");
			err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
			if (err) {
				printk("Failed to start extended advertising set (err %d)\n", err);
				return 0;
			}
#endif /* CONFIG_BT_EXT_ADV */

#if defined(HAS_LED)
			blink_start();
#endif /* HAS_LED */
		}
	}

	return 0;
}
