/*
 * Copyright (c) 2025 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/debug_gpios.h>

const struct gpio_dt_spec debug_gpios[DEBUG_GPIO_COUNT] = {
	GPIO_DT_SPEC_GET(DEBUG1_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG2_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG3_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG4_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG5_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG6_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG7_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG8_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG9_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG11_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG14_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG15_NODE, gpios),
	GPIO_DT_SPEC_GET(DEBUG16_NODE, gpios),
};

int init_debug_gpios(void)
{
	int ret = 0;

	for (uint8_t i=0; i<DEBUG_GPIO_COUNT; i++) {
		if (!gpio_is_ready_dt(&debug_gpios[i])) {
			printk("debug GPIO %d: gpio_is_ready_dt KO!!\n", i);
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&debug_gpios[i], GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			printk("debug GPIO %d: gpio_pin_configure_dt KO!!\n", i);
			return ret;
		}
	}

	return 0;
}

int set_debug_gpio(uint8_t gpio_number, int value)
{
	if (gpio_number > DEBUG_GPIO_COUNT) {
		return -EINVAL;
	}

	return gpio_pin_set_dt(&debug_gpios[gpio_number - 1], value);
}

int toggle_debug_gpio(uint8_t gpio_number)
{
	if (gpio_number > DEBUG_GPIO_COUNT) {
		return -EINVAL;
	}

	return gpio_pin_toggle_dt(&debug_gpios[gpio_number - 1]);
}
