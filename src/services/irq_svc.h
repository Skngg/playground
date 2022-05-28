/** @file
 *  @brief IRQ Service
 */

#ifndef IRQ_SVC_H_
#define IRQ_SVC_H_

#include <drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUMBER_OF_EIRQ 4

extern atomic_t imu_drdy_flag;

int eirq_init(gpio_callback_handler_t handler);
int eirq_halt(uint8_t i);
int eirq_resume(uint8_t i);
void eirq_resume_all();
void eirq_halt_all();

#ifdef __cplusplus
}
#endif

#endif
