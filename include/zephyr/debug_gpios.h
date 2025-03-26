                                                   /*
 * Copyright (c) 2025 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DEBUG_GPIOS_H
#define DEBUG_GPIOS_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define DEBUG_GPIO_COUNT	(13)

#define DBG_SLP_TIME_US (10000)

#define DEBUG1_NODE DT_ALIAS(debug1)
#define DEBUG2_NODE DT_ALIAS(debug2)
#define DEBUG3_NODE DT_ALIAS(debug3)
#define DEBUG4_NODE DT_ALIAS(debug4)
#define DEBUG5_NODE DT_ALIAS(debug5)
#define DEBUG6_NODE DT_ALIAS(debug6)
#define DEBUG7_NODE DT_ALIAS(debug7)
#define DEBUG8_NODE DT_ALIAS(debug8)
#define DEBUG9_NODE DT_ALIAS(debug9)
#define DEBUG11_NODE DT_ALIAS(debug11)
#define DEBUG14_NODE DT_ALIAS(debug14)
#define DEBUG15_NODE DT_ALIAS(debug15)
#define DEBUG16_NODE DT_ALIAS(debug16)

/* Check if the nodes are correctly defined */
#if !DT_NODE_HAS_STATUS(DEBUG1_NODE, okay) || !DT_NODE_HAS_STATUS(DEBUG2_NODE, okay) || \
    !DT_NODE_HAS_STATUS(DEBUG3_NODE, okay) || !DT_NODE_HAS_STATUS(DEBUG4_NODE, okay) || \
    !DT_NODE_HAS_STATUS(DEBUG5_NODE, okay) || !DT_NODE_HAS_STATUS(DEBUG6_NODE, okay) || \
	!DT_NODE_HAS_STATUS(DEBUG7_NODE, okay) || !DT_NODE_HAS_STATUS(DEBUG8_NODE, okay) || \
	!DT_NODE_HAS_STATUS(DEBUG9_NODE, okay) || !DT_NODE_HAS_STATUS(DEBUG11_NODE, okay) || \
    !DT_NODE_HAS_STATUS(DEBUG14_NODE, okay) || !DT_NODE_HAS_STATUS(DEBUG15_NODE, okay) || \
    !DT_NODE_HAS_STATUS(DEBUG16_NODE, okay)
#error "One or more debug GPIOs are not defined in the device tree"
#endif

extern const struct gpio_dt_spec debug_gpios[DEBUG_GPIO_COUNT];

extern int init_debug_gpios(void);

extern int set_debug_gpio(uint8_t gpio_number, int value);
extern int toggle_debug_gpio(uint8_t gpio_number);

#endif /* DEBUG_GPIOS_H */
