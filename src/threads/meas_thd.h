/** @file
 *  @brief Measurements thread
 */

#ifndef MEAS_THD_H_
#define MEAS_THD_H_

#include <stdio.h>
#include <zephyr.h>
#include <kernel.h>

#include "../services/irq_svc.h"
#include "../services/imu_svc.h"
#include "../services/segger_svc.h"

#define MEAS_THD_PRIORITY   -1
#define MEAS_THD_STACK_SIZE 4096

#ifdef __cplusplus
extern "C" {
#endif

void meas_thd_entry(void *p1, void *p2, void *p3);
void meas_thd_0_entry(void *p1, void *p2, void *p3);
void meas_thd_1_entry(void *p1, void *p2, void *p3);
void meas_thd_2_entry(void *p1, void *p2, void *p3);
void meas_thd_3_entry(void *p1, void *p2, void *p3);

#ifdef __cplusplus
}
#endif

#endif // MEAS_THD_H_
