/** @file
 *  @brief IMU Service sample
 */

#include "imu_svc.h"
#include "segger_svc.h"
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>

LOG_MODULE_REGISTER(imu_svc, 4);

sensor_t *imu_sensor_list[NUMBER_OF_EIRQ];
atomic_t read_fifo = 0;
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];

void reset_imu(stmdev_ctx_t *dev, const char *name)
{
	uint8_t whoAmI = 0, rst = 0;
	lsm6ds3_device_id_get(dev, &whoAmI);

	if (LSM6DS3_ID != whoAmI)
		LOG_ERR("%s failed checking WHO_AM_I register: %x", name, whoAmI);
	else
	{
		lsm6ds3_reset_set(dev, PROPERTY_ENABLE);
		LOG_DBG("Waiting to confirm reset on device: %s", name);
		do
		{
			lsm6ds3_reset_get(dev, &rst);
		} while (rst);
		LOG_DBG("Device reset");
	}
}

void setup_imu(stmdev_ctx_t *dev)
{
	lsm6ds3_int1_route_t int_1_reg;
	/* Set full scale */
	lsm6ds3_xl_full_scale_set(dev, LSM6DS3_2g);
	lsm6ds3_gy_full_scale_set(dev, LSM6DS3_250dps);
	lsm6ds3_block_data_update_set(dev, PROPERTY_ENABLE);
	lsm6ds3_pin_mode_set(dev, LSM6DS3_PUSH_PULL);
	lsm6ds3_pin_polarity_set(dev, LSM6DS3_ACTIVE_HIGH);
	lsm6ds3_gy_hp_bandwidth_set(dev, LSM6DS3_HP_CUT_OFF_2Hz07);
	lsm6ds3_xl_filter_analog_set(dev,LSM6DS3_ANTI_ALIASING_50Hz);

	/* Set FIFO */
	lsm6ds3_fifo_gy_batch_set(dev, LSM6DS3_FIFO_GY_NO_DEC);
	lsm6ds3_fifo_xl_batch_set(dev, LSM6DS3_FIFO_XL_NO_DEC);
	lsm6ds3_fifo_data_rate_set(dev, LSM6DS3_FIFO_104Hz);
	lsm6ds3_fifo_watermark_set(dev, 6 * SAMPLE_BATCH_SIZE);

	/* Enable interrupt generation on FIFO threshold INT1 pin */
	lsm6ds3_pin_int1_route_get(dev, &int_1_reg);
	int_1_reg.int1_fth = PROPERTY_ENABLE;
	lsm6ds3_pin_int1_route_set(dev, &int_1_reg);
	lsm6ds3_filter_settling_mask_set(dev, PROPERTY_ENABLE);

	/* Set Output Data Rate and power-up*/
	lsm6ds3_xl_data_rate_set(dev, LSM6DS3_XL_ODR_104Hz);
	lsm6ds3_gy_data_rate_set(dev, LSM6DS3_GY_ODR_104Hz);

	// lsm6ds3_den_mode_set(dev,);
}

void reset_all_imus()
{
	uint8_t i;
	for (i = 0; i < NUMBER_OF_EIRQ; ++i)
	{
		reset_imu(imu_sensor_list[i]->stm_dev, imu_sensor_list[i]->name);
	}
}

void setup_all_imus()
{
	uint8_t i;
	for (i = 0; i < NUMBER_OF_EIRQ; ++i)
	{
		setup_imu(imu_sensor_list[i]->stm_dev);
	}
}

void read_slow_imu(sensor_t *sensor)
{
	uint8_t reg = 0;
	stmdev_ctx_t *dev_ctx = sensor->stm_dev;
	/* Read output only if new value is available */
	lsm6ds3_xl_flag_data_ready_get(dev_ctx, &reg);

	if (reg)
	{
		/* Read acceleration field data */
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lsm6ds3_acceleration_raw_get(dev_ctx, data_raw_acceleration);
	}

	lsm6ds3_gy_flag_data_ready_get(dev_ctx, &reg);

	if (reg)
	{
		/* Read angular rate field data */
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm6ds3_angular_rate_raw_get(dev_ctx, data_raw_angular_rate);
	}

	if (0 != data_raw_acceleration[0])
	{
		acceleration_mg[0] =
			lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] =
			lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] =
			lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[2]);
		LOG_DBG("%s\tAcceleration [mg]:\t%4d\t%4d\t%4d", sensor->name, (int)acceleration_mg[0], (int)acceleration_mg[1], (int)acceleration_mg[2]);
	}
	if (0 != data_raw_angular_rate[0])
	{
		angular_rate_mdps[0] =
			lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps[1] =
			lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps[2] =
			lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate[2]);
		LOG_DBG("%s\tAngular rate [mdps]:\t%4d\t%4d\t%4d", sensor->name, (int)angular_rate_mdps[0], (int)angular_rate_mdps[1], (int)angular_rate_mdps[2]);
	}
}

void read_slow_all_imus()
{
	LOG_DBG("----------------------------------------------------");
	for (uint8_t i = 0; i < NUMBER_OF_EIRQ; ++i)
	{
		read_slow_imu(imu_sensor_list[i]);
	}
	LOG_DBG("----------------------------------------------------");
}

int read_fast_imu(sensor_t *sensor)
{
	stmdev_ctx_t *dev_ctx = sensor->stm_dev;

	/* Read acceleration field data */
	lsm6ds3_angular_rate_raw_get(dev_ctx, data_raw_angular_rate);
	lsm6ds3_acceleration_raw_get(dev_ctx, data_raw_acceleration);

	// SEGGER_RTT_printf(SEGGER_LOGGER_TERMINAL, "%s Acceleration [lsb]: 0x%.8X 0x%.8X 0x%.8X\n", sensor->name, data_raw_acceleration[0], data_raw_acceleration[1], data_raw_acceleration[2]);
	LOG_DBG("%s\tAcceleration [lsb]:\t0x%.8X\t0x%.8X\t0x%.8X", sensor->name, data_raw_acceleration[0], data_raw_acceleration[1], data_raw_acceleration[2]);

	// SEGGER_RTT_printf(SEGGER_LOGGER_TERMINAL, "%s Angular rate [lsb]: 0x%.8X 0x%.8X 0x%.8X\n", sensor->name, data_raw_angular_rate[0], data_raw_angular_rate[1], data_raw_angular_rate[2]);
	LOG_DBG("%s\tAngular rate [lsb]:\t0x%.8X\t0x%.8X\t0x%.8X", sensor->name, data_raw_angular_rate[0], data_raw_angular_rate[1], data_raw_angular_rate[2]);
	return 1;
}

void read_fast_all_imus()
{
	LOG_DBG("----------------------------------------------------");
	for (uint8_t i = 0; i < NUMBER_OF_EIRQ; ++i)
	{
		read_fast_imu(imu_sensor_list[i]);
	}
	LOG_DBG("----------------------------------------------------");
}

int read_fifo_imu(sensor_t *sensor)
{
	stmdev_ctx_t *dev_ctx = sensor->stm_dev;
	// static int16_t words[96];

	/* Read FIFO data level */
	uint16_t num_of_words = 0;
	lsm6ds3_fifo_data_level_get(dev_ctx, &num_of_words);

	if (num_of_words > 0)
	{
		if (num_of_words % 6 != 0)
		{
			LOG_ERR("FIFO data may be incomplete: num_of_words (%u) not a multiple of 6", num_of_words);
		}
		// LOG_DBG("%s\tnum of words: %u", sensor->name, num_of_words);
		const uint16_t num_of_sets = num_of_words % 6 == 0 ? (num_of_words / 6) : (num_of_words / 6) + 1;
		int16_t words[6 * num_of_sets];
		
		memset(words, 0, 12 * num_of_sets);

		// lsm6ds3_fifo_raw_data_get(dev_ctx, &(words[sensor->fifo_pattern_counter]), num_of_words);
		lsm6ds3_fifo_raw_data_get(dev_ctx, (uint8_t*)words, num_of_words);
		// sensor->fifo_pattern_counter = (sensor->fifo_pattern_counter + num_of_words % 6) % 6;

		/* Prevent printing data from sensors with unsettled filters */
		if (!(sensor->settling_counter < SETTLING_SIZE))
		{
			#ifdef JLINK_CONNECTED
			for (uint16_t i = 0; i < num_of_sets; ++i)
			{
				// LOG_DBG("%s\tAcceleration [lsb]:\t0x%.8X\t0x%.8X\t0x%.8X", sensor->name, words[6*i + 3], words[6*i + 4], words[6*i+5]);
				// LOG_DBG("%s\tAngular rate [lsb]:\t0x%.8X\t0x%.8X\t0x%.8X", sensor->name, words[6*i + 0], words[6*i + 1], words[6*i+2]);
				SEGGER_RTT_printf(SEGGER_LOGGER_TERMINAL, "%s\tAcc:\t0x%.8X\t0x%.8X\t0x%.8X\n\t\t\t\tAng:\t0x%.8X\t0x%.8X\t0x%.8X\n",
								  sensor->name,
								  words[6 * i + 3], words[6 * i + 4], words[6 * i + 5],
								  words[6 * i + 0], words[6 * i + 1], words[6 * i + 2]);
			}
			#endif
			notify_bt_acc(sensor, words, 2 * num_of_words);
			return num_of_sets;
		}
		else
		{
			sensor->settling_counter += num_of_sets;
		}
	}
	return 0;
}

// |--IMU IRQ callback---
#pragma region IMU_IRQ_callback

void imu_irq_triggered(const struct device *dev, struct gpio_callback *cb,
					   uint32_t pins)
{
	switch (pins)
	{
	case GPIO_IN_PIN3_Msk:
		atomic_set_bit(&imu_drdy_flag, 0);
		// k_sem_give(&imu0_sem);
		// LOG_DBG("1");
		break;
	case GPIO_IN_PIN4_Msk:
		atomic_set_bit(&imu_drdy_flag, 1);
		// k_sem_give(&imu1_sem);
		// LOG_DBG("2");
		break;
	case GPIO_IN_PIN9_Msk:
		atomic_set_bit(&imu_drdy_flag, 2);
		// k_sem_give(&imu2_sem);
		// LOG_DBG("3");
		break;
	case GPIO_IN_PIN10_Msk:
		atomic_set_bit(&imu_drdy_flag, 3);
		// k_sem_give(&imu3_sem);
		// LOG_DBG("4");
		break;
	default:
		return;
		break;
	}
}

void imu_irq_toggle_handler()
{
	lsm6ds3_int1_route_t int_1_reg;
	lsm6ds3_pin_int1_route_get(imu_sensor_list[0]->stm_dev, &int_1_reg);
	if (PROPERTY_ENABLE == int_1_reg.int1_fth)
	{
		eirq_halt_all();
		atomic_clear(&imu_drdy_flag);
		// atomic_clear(&imu1_drdy_flag);
		// atomic_clear(&imu2_drdy_flag);
		// atomic_clear(&imu3_drdy_flag);
		int_1_reg.int1_fth = PROPERTY_DISABLE;
		LOG_DBG("Disable DRDY INT1");
	}
	else
	{
		int_1_reg.int1_fth = PROPERTY_ENABLE;
		LOG_DBG("Enable DRDY INT1");
	}

	for (uint8_t i = 0; i < NUMBER_OF_EIRQ; ++i)
	{
		lsm6ds3_pin_int1_route_set(imu_sensor_list[i]->stm_dev, &int_1_reg);
	}

	if (PROPERTY_ENABLE == int_1_reg.int1_fth)
	{
		eirq_resume_all();
	}
}
K_WORK_DEFINE(imus_irq_toggle, imu_irq_toggle_handler);

void read_slow_all_imus_handler(struct k_work *work)
{
	read_slow_all_imus();
}
K_WORK_DEFINE(read_slow_all_imu_work, read_slow_all_imus_handler);

void toggle_imu_fifo_handler(struct k_work *work)
{
	static lsm6ds3_fifo_md_t fifo_mode = LSM6DS3_BYPASS_MODE;

	// lsm6ds3_fifo_mode_get(imu_sensor_list[0]->stm_dev, &fifo_mode);
	if (fifo_mode == LSM6DS3_BYPASS_MODE)
	{
		fifo_mode = LSM6DS3_STREAM_MODE;
		atomic_set_bit(&read_fifo, 0);		// indicate FIFO read fn for meas_thd
		atomic_set_bit(&read_fifo, 1); 		// indicate bit change
		LOG_DBG("Set FIFO mode to LSM6DS3_STREAM_MODE");
	}
	else
	{
		fifo_mode = LSM6DS3_BYPASS_MODE;
		atomic_clear_bit(&read_fifo, 0);	// indicate DATA read fn for meas_thd
		atomic_set_bit(&read_fifo, 1);		// indicate bit change
		LOG_DBG("Set FIFO mode to LSM6DS3_BYPASS_MODE");
	}

	for (uint8_t i = 0; i < NUMBER_OF_EIRQ; ++i)
	{
		/* Enter Power Down state for XL and GY to synchronize timings */
		lsm6ds3_xl_data_rate_set(imu_sensor_list[i]->stm_dev, LSM6DS3_XL_ODR_OFF);
		lsm6ds3_gy_data_rate_set(imu_sensor_list[i]->stm_dev, LSM6DS3_GY_ODR_OFF);

		lsm6ds3_fifo_mode_set(imu_sensor_list[i]->stm_dev, fifo_mode);
		lsm6ds3_fifo_watermark_set(imu_sensor_list[i]->stm_dev, 6 * SAMPLE_BATCH_SIZE);
		lsm6ds3_gy_hp_reset_set(imu_sensor_list[i]->stm_dev, PROPERTY_ENABLE);
		lsm6ds3_filter_settling_mask_set(imu_sensor_list[i]->stm_dev, PROPERTY_ENABLE);
		imu_sensor_list[i]->settling_counter = 0;
	}

	/* Set Output Data Rate and power-up*/
	union
	{
		struct
		{
			lsm6ds3_ctrl1_xl_t ctrl1_xl;
			lsm6ds3_ctrl2_g_t ctrl2_g;
		};
		uint8_t regs[2];
	} ctrl;
	ctrl.ctrl1_xl.odr_xl = LSM6DS3_XL_ODR_104Hz;
	ctrl.ctrl1_xl.fs_xl = LSM6DS3_2g;
	ctrl.ctrl1_xl.bw_xl = LSM6DS3_ANTI_ALIASING_50Hz;
	ctrl.ctrl2_g.odr_g = LSM6DS3_GY_ODR_104Hz;
	ctrl.ctrl2_g.fs_g = LSM6DS3_250dps;
	ctrl.ctrl2_g.not_used_01 = 0;

	for (uint8_t i = 0; i < NUMBER_OF_EIRQ; ++i)
	{
		lsm6ds3_write_reg(imu_sensor_list[i]->stm_dev, LSM6DS3_CTRL1_XL, ctrl.regs, 2);
	}

	// lsm6ds3_write_reg(imu_sensor_list[2]->stm_dev, LSM6DS3_CTRL1_XL, ctrl.regs, 2);
	// lsm6ds3_write_reg(imu_sensor_list[0]->stm_dev, LSM6DS3_CTRL1_XL, ctrl.regs, 2);
	// lsm6ds3_write_reg(imu_sensor_list[3]->stm_dev, LSM6DS3_CTRL1_XL, ctrl.regs, 2);
	// lsm6ds3_write_reg(imu_sensor_list[1]->stm_dev, LSM6DS3_CTRL1_XL, ctrl.regs, 2);
}
K_WORK_DEFINE(toggle_imu_fifo, toggle_imu_fifo_handler);

#pragma endregion
// ---IMU IRQ callback--|