#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/i2c.h>

// BT APP INC
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include "bt_sample/button_svc.h"
#include "bt_sample/led_svc.h"
// BT APP INC END

#include "ext_drivers/lsm6ds3/lsm6ds3_reg.h"

#define IMU_0 "imu_0"
#define IMU_0_HIGH IMU_0 + "_HIGH"
#define IMU_1 "imu_0"
#define IMU_1_HIGH IMU_1 + "_HIGH"
#define IMU_SENSOR_AMOUNT 4

LOG_MODULE_REGISTER(MAIN_MODULE, 4);

typedef struct sensor_t
{
	stmdev_ctx_t *stm_dev;
	char *name;
} sensor_t;

static sensor_t *imu_sensor_list[IMU_SENSOR_AMOUNT];
// static volatile uint8_t tx_buffer[1000];

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];

int32_t platform_write_high(void *i2c_dev, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	return i2c_burst_write(i2c_dev, LSM6DS3_I2C_ADD_H, reg, bufp, len);
}
int32_t platform_read_high(void *i2c_dev, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	return i2c_burst_read(i2c_dev, LSM6DS3_I2C_ADD_H, reg, bufp, len);
}

int32_t platform_write(void *i2c_dev, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	return i2c_burst_write(i2c_dev, LSM6DS3_I2C_ADD_L, reg, bufp, len);
}
int32_t platform_read(void *i2c_dev, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	return i2c_burst_read(i2c_dev, LSM6DS3_I2C_ADD_L, reg, bufp, len);
}

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
	// lsm6ds3_int1_route_t int_1_reg;
	/* Set Output Data Rate */
	lsm6ds3_xl_data_rate_set(dev, LSM6DS3_XL_ODR_416Hz);
	lsm6ds3_gy_data_rate_set(dev, LSM6DS3_GY_ODR_416Hz);
	/* Set full scale */
	lsm6ds3_xl_full_scale_set(dev, LSM6DS3_2g);
	lsm6ds3_gy_full_scale_set(dev, LSM6DS3_2000dps);
	/* Set threshold to 60 degrees */
	// lsm6ds3_6d_threshold_set(dev, LSM6DS3_DEG_50);
	/* Use HP filter */
	lsm6ds3_xl_hp_path_internal_set(dev, LSM6DS3_USE_HPF);
	/* LPF2 on 6D function selection */
	lsm6ds3_6d_feed_data_set(dev, LSM6DS3_LPF2_FEED);
	/* Enable interrupt generation on 6D INT1 pin */
	// lsm6ds3_pin_int1_route_get(dev, &int_1_reg);
	// int_1_reg.int1_6d = PROPERTY_ENABLE;
	// lsm6ds3_pin_int1_route_set(dev, &int_1_reg);
}

void read_imu_full(sensor_t *sensor)
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
		acceleration_mg[0] =
			lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] =
			lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] =
			lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[2]);

		LOG_DBG("%s\tAcceleration [mg]:\t%4d\t%4d\t%4d", sensor->name, (int)acceleration_mg[0], (int)acceleration_mg[1], (int)acceleration_mg[2]);
	}

	lsm6ds3_gy_flag_data_ready_get(dev_ctx, &reg);

	if (reg)
	{
		/* Read angular rate field data */
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm6ds3_angular_rate_raw_get(dev_ctx, data_raw_angular_rate);
		angular_rate_mdps[0] =
			lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps[1] =
			lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps[2] =
			lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);

		LOG_DBG("%s\tAngular rate [mdps]:\t%4d\t%4d\t%4d", sensor->name, (int)angular_rate_mdps[0], (int)angular_rate_mdps[1], (int)angular_rate_mdps[2]);
	}
}

/* void read_imu_orient(stmdev_ctx_t *dev_ctx)
{
	lsm6ds3_all_src_t all_source;
	// Check if 6D Orientation events
	lsm6ds3_all_sources_get(dev_ctx, &all_source);

	if (all_source.d6d_src.d6d_ia)
	{
		sprintf((char *)tx_buffer, "6D Or. switched to ");

		if (all_source.d6d_src.xh)
		{
			strcat((char *)tx_buffer, "XH");
		}

		if (all_source.d6d_src.xl)
		{
			strcat((char *)tx_buffer, "XL");
		}

		if (all_source.d6d_src.yh)
		{
			strcat((char *)tx_buffer, "YH");
		}

		if (all_source.d6d_src.yl)
		{
			strcat((char *)tx_buffer, "YL");
		}

		if (all_source.d6d_src.zh)
		{
			strcat((char *)tx_buffer, "ZH");
		}

		if (all_source.d6d_src.zl)
		{
			strcat((char *)tx_buffer, "ZL");
		}

		strcat((char *)tx_buffer, "\r\n");

		printk("%s", tx_buffer);
	}
} */

void reset_all_imus()
{
	uint8_t i;
	for (i = 0; i < IMU_SENSOR_AMOUNT; ++i)
	{
		reset_imu(imu_sensor_list[i]->stm_dev, imu_sensor_list[i]->name);
	}
}

void setup_all_imus()
{
	uint8_t i;
	for (i = 0; i < IMU_SENSOR_AMOUNT; ++i)
	{
		setup_imu(imu_sensor_list[i]->stm_dev);
	}
}

void read_all_imu_full_handler(struct k_work *work)
{
	uint8_t i;
	for (i = 0; i < IMU_SENSOR_AMOUNT; ++i)
	{
		read_imu_full(imu_sensor_list[i]);
	}
	LOG_DBG("----------------------------------------------------");
}

K_WORK_DEFINE(read_all_imu_full, read_all_imu_full_handler);

void periodic_trigger_read_all_imu_full(struct k_timer *dummy)
{
	k_work_submit(&read_all_imu_full);
}
K_TIMER_DEFINE(imu_timer_periodic, periodic_trigger_read_all_imu_full, NULL);

const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

// BT APP REQUIREMENTS
/* Button value. */
static uint16_t but_val;

/* Prototype */
static ssize_t recv(struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, const void *buf,
		    uint16_t len, uint16_t offset, uint8_t flags);

/* ST Custom Service  */
static struct bt_uuid_128 st_service_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0000fe40, 0xcc7a, 0x482a, 0x984a, 0x7f2ed5b3e58f));

/* ST LED service */
static struct bt_uuid_128 led_char_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0000fe41, 0x8e22, 0x4541, 0x9d4c, 0x21edae82ed19));

/* ST Notify button service */
static struct bt_uuid_128 but_notif_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0000fe42, 0x8e22, 0x4541, 0x9d4c, 0x21edae82ed19));

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ADV_LEN 12

/* Advertising data */
static uint8_t manuf_data[ADV_LEN] = {
	0x01 /*SKD version */,
	0x83 /* STM32WB - P2P Server 1 */,
	0x00 /* GROUP A Feature  */,
	0x00 /* GROUP A Feature */,
	0x00 /* GROUP B Feature */,
	0x00 /* GROUP B Feature */,
	0x00, /* BLE MAC start -MSB */
	0x00,
	0x00,
	0x00,
	0x00,
	0x00, /* BLE MAC stop */
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, ADV_LEN)
};

/* BLE connection */
struct bt_conn *conn;
/* Notification state */
volatile bool notify_enable;

static void mpu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notify_enable = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification %s", notify_enable ? "enabled" : "disabled");
}

/* The embedded board is acting as GATT server.
 * The ST BLE Android app is the BLE GATT client.
 */

/* ST BLE Sensor GATT services and characteristic */

BT_GATT_SERVICE_DEFINE(stsensor_svc,
BT_GATT_PRIMARY_SERVICE(&st_service_uuid),
BT_GATT_CHARACTERISTIC(&led_char_uuid.uuid,
		       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		       BT_GATT_PERM_WRITE, NULL, recv, (void *)1),
BT_GATT_CHARACTERISTIC(&but_notif_uuid.uuid, BT_GATT_CHRC_NOTIFY,
		       BT_GATT_PERM_READ, NULL, NULL, &but_val),
BT_GATT_CCC(mpu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static ssize_t recv(struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, const void *buf,
		    uint16_t len, uint16_t offset, uint8_t flags)
{
	led_update(0);

	return 0;
}

static void button_callback(const struct device *gpiob, struct gpio_callback *cb,
		     uint32_t pins)
{
	int err;

	LOG_INF("Button pressed");
	if (conn) {
		if (notify_enable) {
			err = bt_gatt_notify(NULL, &stsensor_svc.attrs[4],
					     &but_val, sizeof(but_val));
			if (err) {
				LOG_ERR("Notify error: %d", err);
			} else {
				LOG_INF("Send notify ok");
				but_val = (but_val == 0) ? 0x100 : 0;
			}
		} else {
			LOG_INF("Notify not enabled");
		}
	} else {
		LOG_INF("BLE not connected");
	}
}

static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}
	LOG_INF("Bluetooth initialized");
	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Configuration mode: waiting connections...");
}

static void connected(struct bt_conn *connected, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
	} else {
		LOG_INF("Connected");
		if (!conn) {
			conn = bt_conn_ref(connected);
		}
	}
}

static void disconnected(struct bt_conn *disconn, uint8_t reason)
{
	if (conn) {
		bt_conn_unref(conn);
		conn = NULL;
	}

	LOG_INF("Disconnected (reason %u)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};
// BT APP REQUIREMENTS END

void main(void)
{

	static stmdev_ctx_t dev_ctx_0_high, dev_ctx_0, dev_ctx_1_high, dev_ctx_1;

	dev_ctx_0_high.write_reg = platform_write_high;
	dev_ctx_0_high.read_reg = platform_read_high;
	dev_ctx_0_high.handle = i2c0_dev;
	static sensor_t imu_lt = {.stm_dev = &dev_ctx_0_high, .name = "IMU_LEFT_THIGH"};
	dev_ctx_0.write_reg = platform_write;
	dev_ctx_0.read_reg = platform_read;
	dev_ctx_0.handle = i2c0_dev;
	static sensor_t imu_lc = {.stm_dev = &dev_ctx_0, .name = "IMU_LEFT_CALF"};

	dev_ctx_1_high.write_reg = platform_write_high;
	dev_ctx_1_high.read_reg = platform_read_high;
	dev_ctx_1_high.handle = i2c1_dev;
	static sensor_t imu_rt = {.stm_dev = &dev_ctx_1_high, .name = "IMU_RIGHT_THIGH"};

	dev_ctx_1.write_reg = platform_write;
	dev_ctx_1.read_reg = platform_read;
	dev_ctx_1.handle = i2c1_dev;
	static sensor_t imu_rc = {.stm_dev = &dev_ctx_1, .name = "IMU_RIGHT_CALF"};

	imu_sensor_list[0] = &imu_lt;
	imu_sensor_list[1] = &imu_rt;
	imu_sensor_list[2] = &imu_lc;
	imu_sensor_list[3] = &imu_rc;

	if (!device_is_ready(i2c0_dev))
	{
		LOG_ERR("I2C: Device i2c0 is not ready.");
		return;
	}
	else if (!device_is_ready(i2c1_dev))
	{
		LOG_ERR("I2C: Device i2c1 is not ready.");
		return;
	}

	LOG_INF("Resetting all sensors");
	reset_all_imus();
	setup_all_imus();

	uint8_t i;
	for (i = 0; i < IMU_SENSOR_AMOUNT; ++i)
	{
		read_imu_full(imu_sensor_list[i]);
		LOG_INF("%s\tAcceleration [mg]:%4d\t%4d\t%4d", (imu_sensor_list[i])->name, (int)acceleration_mg[0], (int)acceleration_mg[1], (int)acceleration_mg[2]);
		LOG_INF("%s\tAngular rate [mdps]:%4d\t%4d\t%4d", imu_sensor_list[i]->name, (int)angular_rate_mdps[0], (int)angular_rate_mdps[1], (int)angular_rate_mdps[2]);
		memset(acceleration_mg, 0x00, 3 * sizeof(float));
		memset(angular_rate_mdps, 0x00, 3 * sizeof(float));
		k_msleep(500);
	}

	LOG_INF("Starting system");
	k_msleep(1000);
	// k_timer_start(&imu_timer_periodic, K_SECONDS(3), K_SECONDS(3));

	// BT APP
	int err;

	err = button_init(button_callback);
	if (err)
	{
		return;
	}

	err = led_init();
	if (err)
	{
		return;
	}
	k_msleep(1000);
	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err)
	{
		LOG_ERR("Bluetooth init failed (err %d)", err);
	}
	// BT APP END
}