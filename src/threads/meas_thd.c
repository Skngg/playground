/** @file
 *  @brief Measurements thread source
 */

#include "meas_thd.h"

LOG_MODULE_REGISTER(meas_thd, 4);

K_THREAD_DEFINE(meas_thd_0, MEAS_THD_STACK_SIZE, meas_thd_0_entry, NULL, NULL, NULL, MEAS_THD_PRIORITY, 0, 5000);
K_THREAD_DEFINE(meas_thd_1, MEAS_THD_STACK_SIZE, meas_thd_1_entry, NULL, NULL, NULL, MEAS_THD_PRIORITY, 0, 5000);
K_THREAD_DEFINE(meas_thd_2, MEAS_THD_STACK_SIZE, meas_thd_2_entry, NULL, NULL, NULL, MEAS_THD_PRIORITY, 0, 5000);
K_THREAD_DEFINE(meas_thd_3, MEAS_THD_STACK_SIZE, meas_thd_3_entry, NULL, NULL, NULL, MEAS_THD_PRIORITY, 0, 5000);

static int (*imu_read_fn)(sensor_t *sensor) = &read_fast_imu;

void meas_thd_entry(void *p1, void *p2, void *p3)
{
    LOG_DBG("Meas thread entry");
    while (1)
    {
        if (atomic_test_and_clear_bit(&read_fifo, 1))
        {
            if (!atomic_test_bit(&read_fifo, 0))
            {
                imu_read_fn = &read_fast_imu;
                LOG_DBG("SET NORMAL");
            }
            else
            {
                imu_read_fn = &read_fifo_imu;
                LOG_DBG("SET FIFO");
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

void meas_thd_0_entry(void *p1, void *p2, void *p3)
{
    LOG_INF("MEAS_THD_0 started");
    while (1)
    {
        // if (0 == k_sem_take(&imu0_sem, K_FOREVER))
        if(atomic_test_bit(&imu_drdy_flag,0))
        {
            // k_thread_priority_set(&meas_thd_0,0);
            // k_sched_lock();
            k_mutex_lock(&i2c0_mutex,K_FOREVER);
            read_fifo_imu(imu_sensor_list[0]);
            k_mutex_unlock(&i2c0_mutex);
            atomic_clear_bit(&imu_drdy_flag,0);
            k_sleep(K_MSEC(20));
            // k_sched_unlock();
            // k_thread_priority_set(&meas_thd_0,MEAS_THD_PRIORITY);
        }
        else
        {
            k_yield();
        }
    }
}

void meas_thd_1_entry(void *p1, void *p2, void *p3)
{
    LOG_INF("MEAS_THD_1 started");
    while (1)
    {
        // if (0 == k_sem_take(&imu1_sem, K_FOREVER))
        if(atomic_test_bit(&imu_drdy_flag,1))
        {
            // k_thread_priority_set(&meas_thd_1,0);
            // k_sched_lock();
            k_mutex_lock(&i2c0_mutex,K_FOREVER);
            read_fifo_imu(imu_sensor_list[1]);
            k_mutex_unlock(&i2c0_mutex);
            atomic_clear_bit(&imu_drdy_flag,1);
            k_sleep(K_MSEC(20));
            // k_sched_unlock();
            // k_thread_priority_set(&meas_thd_1,MEAS_THD_PRIORITY);
        }
        else
        {
            k_yield();
        }
    }
}

void meas_thd_2_entry(void *p1, void *p2, void *p3)
{
    LOG_INF("MEAS_THD_2 started");
    while (1)
    {
        // if (0 == k_sem_take(&imu2_sem, K_FOREVER))
        if(atomic_test_bit(&imu_drdy_flag,2))
        {
            // k_thread_priority_set(&meas_thd_2,0);
            // k_sched_lock();
            k_mutex_lock(&i2c1_mutex,K_FOREVER);
            read_fifo_imu(imu_sensor_list[2]);
            k_mutex_unlock(&i2c1_mutex);
            atomic_clear_bit(&imu_drdy_flag,2);
            k_sleep(K_MSEC(20));
            // k_sched_unlock();
            // k_thread_priority_set(&meas_thd_2,MEAS_THD_PRIORITY);
        }
        else
        {
            k_yield();
        }
    }
}

void meas_thd_3_entry(void *p1, void *p2, void *p3)
{
    LOG_INF("MEAS_THD_3 started");
    while (1)
    {
        // if (0 == k_sem_take(&imu3_sem, K_FOREVER))
        if(atomic_test_bit(&imu_drdy_flag,3))
        {
            // k_thread_priority_set(&meas_thd_3,0);
            // k_sched_lock();
            k_mutex_lock(&i2c1_mutex,K_FOREVER);
            read_fifo_imu(imu_sensor_list[3]);
            k_mutex_unlock(&i2c1_mutex);
            atomic_clear_bit(&imu_drdy_flag,3);
            k_sleep(K_MSEC(20));
            // k_sched_unlock();
            // k_thread_priority_set(&meas_thd_3,MEAS_THD_PRIORITY);
        }
        else
        {
            k_yield();
        }
    }
}