/** @file
 *  @brief Button Service sample
 */

/*
 * Copyright (c) 2019 Marcio Montenegro <mtuxpe@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "button_svc.h"
#include "segger_svc.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <logging/log.h>
#include <zephyr.h>

LOG_MODULE_REGISTER(button_svc, 4);
K_TIMER_DEFINE(button_debounce_timer, NULL, NULL);

#define NUMBER_OF_BUTTONS 4

static const struct gpio_dt_spec button[NUMBER_OF_BUTTONS] = {GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
															  GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
															  GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios),
															  GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios)};
static struct gpio_callback gpio_cb[NUMBER_OF_BUTTONS];

int button_init(gpio_callback_handler_t handler)
{
	int ret;
	for (uint8_t i = 0; i < NUMBER_OF_BUTTONS; ++i)
	{

		if (!device_is_ready(button[i].port))
		{
			LOG_ERR("Error: button GPIO device %s is not ready",
					button[i].port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&button[i], GPIO_INPUT | GPIO_INT_DEBOUNCE);
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

void button_callback(const struct device *gpiob, struct gpio_callback *cb,
					 uint32_t pins)
{
	extern atomic_t imu_drdy_flag;
	extern struct k_work imus_irq_toggle, read_slow_all_imu_work, toggle_imu_fifo;
	if (0 == k_timer_remaining_get(&button_debounce_timer))
	{
		k_timer_start(&button_debounce_timer, K_MSEC(200), K_NO_WAIT);
		if (pins == GPIO_IN_PIN11_Msk)
		{
			/* Toggle IMUs IRQ handling */
			LOG_DBG("Toggle IMUs IRQ handling");
			k_work_submit(&imus_irq_toggle);
		}
		else if (pins == GPIO_IN_PIN12_Msk)
		{
			/* Start capturing measurments */
			k_work_submit(&toggle_imu_fifo);
		}
		else if (pins == GPIO_IN_PIN24_Msk)
		{
			/* Update BT PHY parameters */
			extern struct bt_conn *conn;
			if (!conn)
			{
				LOG_WRN("Device not connected");
				return;
			}
			int ret = bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(6, 6, 0, 400));
			if (0 != ret)
			{
				LOG_ERR("Connection parameter change failed with error: %d", ret);
			}
		}
		else if (pins == GPIO_IN_PIN25_Msk)
		{
			/* Read all imus (NO FIFO) */
			k_work_submit(&read_slow_all_imu_work);
			atomic_clear(&imu_drdy_flag);
		}
	}
}