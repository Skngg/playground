/** @file
 *  @brief IRQ Service sample
 */

#include "irq_svc.h"

#include <zephyr.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(irq_svc, 4);

const struct gpio_dt_spec eirq[NUMBER_OF_EIRQ] = {GPIO_DT_SPEC_GET(DT_NODELABEL(imu_0), gpios),
												  GPIO_DT_SPEC_GET(DT_NODELABEL(imu_1), gpios),
												  GPIO_DT_SPEC_GET(DT_NODELABEL(imu_2), gpios),
												  GPIO_DT_SPEC_GET(DT_NODELABEL(imu_3), gpios)};
static struct gpio_callback gpio_irq_cb[NUMBER_OF_EIRQ];
atomic_t imu_drdy_flag = 0;

int eirq_init(gpio_callback_handler_t handler)
{
	int ret;
	for (uint8_t i = 0; i < NUMBER_OF_EIRQ; ++i)
	{

		if (!device_is_ready(eirq[i].port))
		{
			LOG_ERR("Error: eirq GPIO device %s is not ready",
					eirq[i].port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&eirq[i], GPIO_INPUT | GPIO_INT_DEBOUNCE);
		if (ret != 0)
		{
			LOG_ERR("Error %d: can't configure eirq on GPIO %s pin %d",
					ret, eirq[i].port->name, eirq[i].pin);
			return ret;
		}

		gpio_init_callback(&gpio_irq_cb[i], handler, BIT(eirq[i].pin));
		gpio_add_callback(eirq[i].port, &gpio_irq_cb[i]);
		ret = gpio_pin_interrupt_configure_dt(&eirq[i], GPIO_INT_EDGE_TO_ACTIVE);
		LOG_DBG("Configured ext IRQ at %s pin %d", eirq[i].port->name, eirq[i].pin);
		if (ret != 0)
		{
			LOG_ERR("Error %d: can't configure eirq interrupt on "
					"GPIO %s pin %d",
					ret, eirq[i].port->name, eirq[i].pin);
			return ret;
		}
	}
	return 0;
}

void eirq_halt_all()
{
	for (uint8_t i = 0; i < NUMBER_OF_EIRQ; ++i)
	{
		eirq_halt(i);
	}
}

void eirq_resume_all()
{
	for (uint8_t i = 0; i < NUMBER_OF_EIRQ; ++i)
	{
		eirq_resume(i);
	}
}

int eirq_halt(uint8_t i)
{
	int ret = gpio_pin_interrupt_configure_dt(&eirq[i], GPIO_INT_DISABLE);
	LOG_DBG("Halted ext IRQ at %s pin %d", eirq[i].port->name, eirq[i].pin);
	if (ret != 0)
	{
		LOG_ERR("Error %d: can't halt eirq interrupt on "
				"GPIO %s pin %d",
				ret, eirq[i].port->name, eirq[i].pin);
		return ret;
	}
	return 0;
}

int eirq_resume(uint8_t i)
{
	int ret = gpio_pin_interrupt_configure_dt(&eirq[i], GPIO_INT_EDGE_TO_ACTIVE);
	LOG_DBG("Resumed ext IRQ at %s pin %d", eirq[i].port->name, eirq[i].pin);
	if (ret != 0)
	{
		LOG_ERR("Error %d: can't resume eirq interrupt on "
				"GPIO %s pin %d",
				ret, eirq[i].port->name, eirq[i].pin);
		return ret;
	}
	return 0;
}
