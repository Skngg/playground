#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/i2c.h>
// #include <zephyr/modules/segger>

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
// BT APP INC END

#include "services/button_svc.h"
#include "services/led_svc.h"
#include "services/irq_svc.h"
#include "services/imu_svc.h"
#include "threads/meas_thd.h"

#define IMU_0 "imu_0"
#define IMU_0_HIGH IMU_0 + "_HIGH"
#define IMU_1 "imu_0"
#define IMU_1_HIGH IMU_1 + "_HIGH"
#define IMU_SENSOR_AMOUNT NUMBER_OF_EIRQ

LOG_MODULE_REGISTER(MAIN_MODULE, 4);

void notify_bt_acc(sensor_t *, uint8_t *, uint16_t);

extern sensor_t *imu_sensor_list[NUMBER_OF_EIRQ];
extern atomic_t imu_drdy_flag;

// |--STM device write/read handlers---
#pragma region STM_device_handlers
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
#pragma endregion
// ---STM device write/read handlers--|

// |--I2C device definition---
#pragma region I2C_device_definition
const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
#pragma endregion
// ---I2C device definition--|

// |--BLUETOOTH---
#pragma region BLUETOOTH

/* Prototype */
static ssize_t on_receive(struct bt_conn *conn,
						  const struct bt_gatt_attr *attr, const void *buf,
						  uint16_t len, uint16_t offset, uint8_t flags);
// static ssize_t on_sent(struct bt_conn *conn,
// 					const struct bt_gatt_attr *attr, const void *buf,
// 					uint16_t len, uint16_t offset);

/* Custom Service  */
static struct bt_uuid_128 primary_service_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0000fe40, 0xcc7a, 0x482a, 0x984a, 0x7f2ed5b3e58f));

/* RX service */
static struct bt_uuid_128 rx_char_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0000fe41, 0x8e22, 0x4541, 0x9d4c, 0x21edae82ed19));

/* IMU Notify service */
static struct bt_uuid_128 tx_char_uuid = BT_UUID_INIT_128(
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
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, ADV_LEN)};

/* BLE connection */
struct bt_conn *conn;

/* Notification and Indication state */
volatile bool notify_enable;
// volatile bool indicate_enable;

static void mpu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notify_enable = (value == BT_GATT_CCC_NOTIFY);
	// indicate_enable = (value == BT_GATT_CCC_INDICATE;
	LOG_INF("Notification %s", notify_enable ? "enabled" : "disabled");
	// LOG_INF("Indication %s", indicate_enable ? "enabled" : "disabled");
}

/* BLE Sensor GATT services and characteristic */
#define TX_DATA_BUFFER_SIZE 250
volatile uint8_t tx_buffer[TX_DATA_BUFFER_SIZE] = {0xAA, 0xBB, 0xCC, 0xCD, 0xBC, 0xAB};
static uint8_t msgCounter[4];
BT_GATT_SERVICE_DEFINE(stsensor_svc,
					   BT_GATT_PRIMARY_SERVICE(&primary_service_uuid),
					   BT_GATT_CHARACTERISTIC(&rx_char_uuid.uuid,
											  BT_GATT_CHRC_WRITE_WITHOUT_RESP,
											  BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
											  NULL, on_receive, NULL),
					   BT_GATT_CHARACTERISTIC(&tx_char_uuid.uuid, BT_GATT_CHRC_NOTIFY,
											  BT_GATT_PERM_READ, NULL, NULL, NULL),
					   BT_GATT_CCC(mpu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

void print_array(char *dest_hex, uint8_t* dest_raw, const uint8_t *src, uint16_t len)
{
	for (uint16_t i = 0; i < len; ++i)
	{
		char hex_byte[3];
		sprintf(hex_byte, "%2X", src[i]);
		strcat(dest_hex, hex_byte);
		dest_raw[i] = src[i];
	}
	dest_raw[len] = 0;
	strcat(dest_hex, "\0");
}

static ssize_t on_receive(struct bt_conn *conn,
						  const struct bt_gatt_attr *attr, const void *buf,
						  uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t buffer[len+1];
	char data[2*len+1];
	static bool isFifo = false;
	memset(data,0,len);
	memset(buffer,0,len);
	print_array(data, buffer, (uint8_t *)buf, len);
	LOG_DBG("Received BT %u bytes with offset %u:\nHEX: %s\nRAW: %s", len, offset, data, buffer);
	if ((strcmp(buffer, "START") == 0 && !isFifo) || (strcmp(buffer, "STOP") == 0 && isFifo))
	{
		extern struct k_work toggle_imu_fifo;
		k_work_submit(&toggle_imu_fifo);
		led_update(0);
		if(isFifo)
		{
			memset(msgCounter,0,4*sizeof(uint8_t));
			isFifo = false;
		}
		else
		{
			isFifo = true;
		}
		// isFifo = isFifo ? false : true;
	}
	return 0;
}

// static ssize_t on_sent(struct bt_conn *conn,
// 					const struct bt_gatt_attr *attr, const void *buf,
// 					uint16_t len, uint16_t offset)
// {
// 	const uint8_t *value = attr->user_data;
// 	// char array[2 * CONFIG_BT_CTLR_DATA_LENGTH_MAX + 1] = "";
// 	const uint8_t *buffer = buf;
// 	char data[len];
// 	print_array(data, value, len);
// 	LOG_DBG("SEND %s", data);
// 	// LOG_DBG("send %s",print_array(array,(uint8_t*)buf,len,0));

// 	return bt_gatt_attr_read(conn, attr, buf, len, offset, data,
// 							 strlen(data));
// }

static void bt_ready(int err)
{
	if (err)
	{
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}
	LOG_INF("Bluetooth initialized");
	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err)
	{
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Configuration mode: waiting connections...");
}

static void connected(struct bt_conn *connected, uint8_t err)
{
	if (err)
	{
		LOG_ERR("Connection failed (err %u)", err);
	}
	else
	{
		LOG_INF("Connected");
		if (!conn)
		{
			conn = bt_conn_ref(connected);
		}

		led_update(1);
	}
}

static void disconnected(struct bt_conn *disconn, uint8_t reason)
{
	if (conn)
	{
		bt_conn_unref(conn);
		conn = NULL;
	}

	LOG_INF("Disconnected (reason %u)", reason);
	led_update(1);
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	if (bt_conn_get_info(conn, &info))
	{
		LOG_ERR("Could not parse connection info");
	}
	else
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

		LOG_INF("Connection parameters updated!	\n\
		Connected to: %s						\n\
		New Connection Interval: %u				\n\
		New Slave Latency: %u					\n\
		New Connection Supervisory Timeout: %u	\n",
				addr, info.le.interval, info.le.latency, info.le.timeout);
	}
}

void notify_bt_acc(sensor_t *sensor, uint8_t *data_arr, uint16_t size)
{
	int err;
	if (conn)
	{
		if (notify_enable)
		{
			uint8_t imuId = 0;

			if (strcmp(sensor->name,"IMU_LEFT_THIGH") == 0)
			{
				imuId = 0x1;
			}
			else if (strcmp(sensor->name,"IMU_LEFT_CALF") == 0)
			{
				imuId = 0x2;
			}
			else if (strcmp(sensor->name,"IMU_RIGHT_THIGH") == 0)
			{
				imuId = 0x3;
			}
			else if (strcmp(sensor->name,"IMU_RIGHT_CALF") == 0)
			{
				imuId = 0x4;
			}

			if (imuId == 0)
			{
				LOG_ERR("Notification failed with invalid imuId: %u", imuId);
				return;
			}

			memset(tx_buffer, 0, TX_DATA_BUFFER_SIZE);
			if (size + 2 <= TX_DATA_BUFFER_SIZE)
			{
				tx_buffer[0] = imuId;
				tx_buffer[1] = msgCounter[imuId - 1];
				memcpy(tx_buffer + 2, data_arr, size * sizeof(uint8_t));
			}
			else
			{
				LOG_ERR("Notification failed with requested msg size = %u too big for the tx_buffer", size);
				return;
			}
			
			const struct bt_gatt_attr *attr = &stsensor_svc.attrs[4]; 

			struct bt_gatt_notify_params params = 
			{
				.uuid   = &tx_char_uuid.uuid,
				.attr   = attr,
				.data   = tx_buffer,
				.len    = size+2,
				.func   = NULL
			};

			err = bt_gatt_notify_cb(conn, &params);


			if (err)
			{
				LOG_ERR("Notify error: %d", err);
			}
			else
			{
				++msgCounter[imuId - 1];
			}
		}
		else
		{
			LOG_INF("Notify not enabled");
		}
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
};

void att_mtu_updated_cb(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("MTU updated!	\n\
	Connected to:\t%s		\n\
	TX MTU:\t%u				\n\
	RX MTU:\t%u				\n",
			addr, tx, rx);
}

static struct bt_gatt_cb gatt_cb = {
	.att_mtu_updated = att_mtu_updated_cb
};
#pragma endregion
// ---BLUETOOTH--|

// |--MAIN---
#pragma region MAIN
void main(void)
{
	static stmdev_ctx_t dev_ctx_0_high, dev_ctx_0, dev_ctx_1_high, dev_ctx_1;

	dev_ctx_0_high.write_reg = platform_write_high;
	dev_ctx_0_high.read_reg = platform_read_high;
	dev_ctx_0_high.handle = i2c0_dev;
	static sensor_t imu_lt = {.stm_dev = &dev_ctx_0_high, .name = "IMU_LEFT_THIGH", .fifo_pattern_counter = 0, .settling_counter = 0};
	dev_ctx_0.write_reg = platform_write;
	dev_ctx_0.read_reg = platform_read;
	dev_ctx_0.handle = i2c0_dev;
	static sensor_t imu_lc = {.stm_dev = &dev_ctx_0, .name = "IMU_LEFT_CALF", .fifo_pattern_counter = 0, .settling_counter = 0};

	dev_ctx_1_high.write_reg = platform_write_high;
	dev_ctx_1_high.read_reg = platform_read_high;
	dev_ctx_1_high.handle = i2c1_dev;
	static sensor_t imu_rt = {.stm_dev = &dev_ctx_1_high, .name = "IMU_RIGHT_THIGH", .fifo_pattern_counter = 0, .settling_counter = 0};

	dev_ctx_1.write_reg = platform_write;
	dev_ctx_1.read_reg = platform_read;
	dev_ctx_1.handle = i2c1_dev;
	static sensor_t imu_rc = {.stm_dev = &dev_ctx_1, .name = "IMU_RIGHT_CALF", .fifo_pattern_counter = 0, .settling_counter = 0};

	imu_sensor_list[0] = &imu_lt;
	imu_sensor_list[1] = &imu_lc;
	imu_sensor_list[2] = &imu_rt;
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
	read_slow_all_imus();

	LOG_INF("Starting system");
	k_msleep(1000);

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

	bt_gatt_cb_register(&gatt_cb);

	err = eirq_init(imu_irq_triggered);
	if (err)
	{
		return;
	}
}
#pragma endregion
// ---MAIN--|