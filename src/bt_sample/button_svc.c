/** @file
 *  @brief Button Service sample
 */

/*
 * Copyright (c) 2019 Marcio Montenegro <mtuxpe@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "button_svc.h"

#include <zephyr.h>
#include <logging/log.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

LOG_MODULE_REGISTER(button_svc);

static const struct gpio_dt_spec button[2] = {GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios), GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios)};
static struct gpio_callback gpio_cb[2];

int button_init(gpio_callback_handler_t handler)
{
	int ret;
	uint8_t i = 0;
	for (i = 0; i < 2; ++i)
	{

		if (!device_is_ready(button[i].port))
		{
			LOG_ERR("Error: button GPIO device %s is not ready",
					button[i].port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&button[i], GPIO_INPUT);
		if (ret != 0)
		{
			LOG_ERR("Error %d: can't configure button on GPIO %s pin %d",
					ret, button[i].port->name, button[i].pin);
			return ret;
		}

		gpio_init_callback(&gpio_cb[i], handler, BIT(button[i].pin));
		gpio_add_callback(button[i].port, &gpio_cb[i]);
		ret = gpio_pin_interrupt_configure_dt(&button[i], GPIO_INT_EDGE_TO_ACTIVE);
		if (ret != 0)
		{
			LOG_ERR("Error %d: can't configure button interrupt on "
					"GPIO %s pin %d",
					ret, button[i].port->name, button[i].pin);
			return ret;
		}
	}
	return 0;
}
