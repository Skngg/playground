/** @file
 *  @brief Measurements thread source
 */

#include "meas_thd.h"

LOG_MODULE_REGISTER(meas_thd, 4);

K_THREAD_DEFINE(meas_thd, MEAS_THD_STACK_SIZE, meas_thd_entry, NULL, NULL, NULL, MEAS_THD_PRIORITY, 0, 3000);

static int (*imu_read_fn)(sensor_t *sensor) = &read_fast_imu;

void meas_thd_entry(void *p1, void *p2, void *p3)
{
    while (1)
    {
        if (atomic_test_and_clear_bit(&read_fifo, 1))
        {
            if (!atomic_test_bit(&read_fifo, 0))
            {
                imu_read_fn = &read_fast_imu;
            }
            else
            {
                imu_read_fn = &read_fifo_imu;
            }
        }
        if (atomic_test_bit(&imu_drdy_flag, 0))
        {
            imu_read_fn(imu_sensor_list[0]);
            atomic_clear_bit(&imu_drdy_flag, 0);
        }
        if (atomic_test_bit(&imu_drdy_flag, 1))
        {
            imu_read_fn(imu_sensor_list[1]);
            atomic_clear_bit(&imu_drdy_flag, 1);
        }
        if (atomic_test_bit(&imu_drdy_flag, 2))
        {
            imu_read_fn(imu_sensor_list[2]);
            atomic_clear_bit(&imu_drdy_flag, 2);
        }
        if (atomic_test_bit(&imu_drdy_flag, 3))
        {
            imu_read_fn(imu_sensor_list[3]);
            atomic_clear_bit(&imu_drdy_flag, 3);
        }
    }
}