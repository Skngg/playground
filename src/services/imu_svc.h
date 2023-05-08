/** @file
 *  @brief IMU Service
 */

#ifndef IMU_SVC_H_
#define IMU_SVC_H_

#include "irq_svc.h"
#include "../ext_drivers/lsm6ds3/lsm6ds3_reg.h"
#include <zephyr.h>
#include <logging/log.h>

#define SAMPLE_BATCH_SIZE 5
#define SETTLING_SIZE 10

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct sensor_t
{
    const stmdev_ctx_t *stm_dev;
    const char *name;
    uint8_t fifo_pattern_counter;
    uint8_t settling_counter;
} sensor_t;

extern sensor_t *imu_sensor_list[NUMBER_OF_EIRQ];
extern atomic_t read_fifo;

void reset_imu(stmdev_ctx_t *dev, const char *name);
void setup_imu(stmdev_ctx_t *dev);
void reset_all_imus();
void setup_all_imus();
void read_slow_imu(sensor_t *sensor);
void read_slow_all_imus();
int read_fast_imu(sensor_t *sensor);
void read_fast_all_imus();
int read_fifo_imu(sensor_t *sensor);

void imu_irq_triggered(const struct device *dev, struct gpio_callback *cb,
                       uint32_t pins);
void imu_irq_toggle_handler();
void toggle_imu_fifo_handler(struct k_work *work);

#ifdef __cplusplus
}
#endif

#endif // IMU_SVC_H_
