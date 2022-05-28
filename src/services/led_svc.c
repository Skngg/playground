/** @file
 *  @brief Button Service sample
 */

/*
 * Copyright (c) 2019 Marcio Montenegro <mtuxpe@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "led_svc.h"

#include <zephyr.h>
#include <drivers/gpio.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(led_svc);

static const struct gpio_dt_spec led[2] = {GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios), GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios)};
static bool led_state[2]; /* Tracking state here supports GPIO expander-based LEDs. */
static bool led_ok[2];

void led_update(uint8_t index)
{
	if (!led_ok[index]) {
		return;
	}

	led_state[index] = !led_state[index];
	LOG_INF("Turn %s LED%u", led_state[index] ? "on" : "off", index);
	gpio_pin_set(led[index].port, led[index].pin, led_state[index]);
}

int led_init(void)
{
	int ret;
	uint8_t i = 0;
	for (i = 0; i < 2; ++i)
	{
		led_ok[i] = device_is_ready(led[i].port);
		if (!led_ok[i])
		{
			LOG_ERR("Error: LED on GPIO %s pin %d is not ready",
					led[i].port->name, led[i].pin);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&led[i], GPIO_OUTPUT_INACTIVE);
		if (ret < 0)
		{
			LOG_ERR("Error %d: failed to configure GPIO %s pin %d",
					ret, led[i].port->name, led[i].pin);
			return ret;
		}
	}

	return ret;
}
