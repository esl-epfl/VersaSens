/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : versa_ble.c                                                  **
** version  : 1                                                            **
** date     : DD/MM/YY                                                     **
**                                                                         **
*****************************************************************************
**                                                                         **
** Copyright (c) EPFL                                                      **
** All rights reserved.                                                    **
**                                                                         **
*****************************************************************************
	 
VERSION HISTORY:
----------------
Version     : 1
Date        : 10/02/2021
Revised by  : Benjamin Duc
Description : Original version.


*/

/***************************************************************************/
/***************************************************************************/

/**
* @file   versa_ble.c
* @date   DD/MM/YY
* @brief  This is the main header of versa_ble.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _VERSA_BLE_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#include <dk_buttons_and_leds.h>
#include "versa_ble.h"
#include "thread_config.h"
#include <nrfx_gpiote.h>

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(versa_ble, LOG_LEVEL_INF);

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED          DK_LED1
#define CON_STATUS_LED          DK_LED2
#define RUN_LED_BLINK_INTERVAL  1000
#define NOTIFY_INTERVAL         1000

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

static struct bt_le_ext_adv *adv;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/*
 * @brief Function to start advertising
 * 
 * @param work : work structure
 * 
 * @retval None
 */
static void start_advertising_coded(struct k_work *work);

/*
 * @brief Function to set value of the status characteristic
 * 
 * @param conn : connection structure
 * @param attr : attribute structure
 * @param buf : buffer
 * @param len : length
 * @param offset : offset
 * @param flags : flags
 * 
 * @retval None
 */
static ssize_t read_status(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

static K_WORK_DEFINE(start_advertising_worker, start_advertising_coded);

BT_GATT_SERVICE_DEFINE(TEST, BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERVICE)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_CUSTOM_CHARA), BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_CUSTOM_CHARA_WRITE), BT_GATT_CHRC_WRITE,
						BT_GATT_PERM_WRITE, NULL, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_CUSTOM_CHARA_STATUS), BT_GATT_CHRC_READ,
						BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_status, NULL, NULL),
);

K_FIFO_DEFINE(sensor_fifo);
K_THREAD_STACK_DEFINE(sensor_thread_stack, 1024);
struct k_thread sensor_thread_data;

static struct bt_gatt_exchange_params exchange_params;

bool stream_data = false;

uint8_t status[1] = {0x00};

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

static void mtu_exchanged(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_exchange_params *params)
{
    if (err) {
        LOG_ERR("MTU exchange failed (err %d)\n", err);
    } else {
        LOG_INF("MTU exchange successful\n");
    }
}

/****************************************************************************/
/****************************************************************************/

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_ERR("Connection failed (err %d)\n", conn_err);
		return;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info (err %d)\n", err);
	} else {
		const struct bt_conn_le_phy_info *phy_info;
		phy_info = info.le.phy;

		LOG_INF("Connected: %s, tx_phy %u, rx_phy %u\n",
		       addr, phy_info->tx_phy, phy_info->rx_phy);
	}

	err = bt_conn_le_phy_update(conn, BT_CONN_LE_PHY_PARAM_2M);
	if (err) {
		LOG_INF("bt_conn_le_phy_update() returned %d", err);
	}
	
	struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };
    err = bt_conn_le_data_len_update(conn, &my_data_len);
    if (err) {
        LOG_ERR("data_len_update failed (err %d)", err);
    }

	exchange_params.func = mtu_exchanged;

    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }

	dk_set_led_on(CON_STATUS_LED);
}

/****************************************************************************/
/****************************************************************************/

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)\n", reason);

	k_work_submit(&start_advertising_worker);

	dk_set_led_off(CON_STATUS_LED);
}

/****************************************************************************/
/****************************************************************************/

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/****************************************************************************/
/****************************************************************************/

static int create_advertising_coded(void)
{
	int err;
	struct bt_le_adv_param param =
		BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE,
				     BT_GAP_ADV_FAST_INT_MIN_2,
				     BT_GAP_ADV_FAST_INT_MAX_2,
				     NULL);

	err = bt_le_ext_adv_create(&param, NULL, &adv);
	if (err) {
		printk("Failed to create advertiser set (err %d)\n", err);
		return err;
	}

	printk("Created adv: %p\n", adv);

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Failed to set advertising data (err %d)\n", err);
		return err;
	}

	return 0;
}

/****************************************************************************/
/****************************************************************************/

static void hrs_notify(void)
{
	static uint8_t heartrate = 100;

	heartrate++;
	if (heartrate == 160) {
		heartrate = 100;
	}

	bt_hrs_notify(heartrate);
}

/****************************************************************************/
/****************************************************************************/

void sensor_thread(void)
{
	while (1) {
    	// Get sensor data
    	struct sensor_data *data = k_fifo_get(&sensor_fifo, K_FOREVER);

    	// Send notification
    	int ret = bt_gatt_notify(NULL, &TEST.attrs[1], data->data, data->size);
		if (ret) {
			LOG_ERR("Failed to notify, error code: %d\n", ret);
			if (ret != -ENOTCONN) {
				// nrf_gpio_cfg_output(27);
            	// nrf_gpio_pin_clear(27);
			}
		}

    	// Free the sensor data
    	k_free(data);
	}
}

/****************************************************************************/
/****************************************************************************/

void receive_sensor_data(uint8_t *data, size_t size)
{
	if(!stream_data) {
		return;
	}
	// Allocate a new sensor data
	struct sensor_data *new_data = k_malloc(sizeof(*new_data));

	// Set the sensor data
	new_data->size = size < MAX_DATA_SIZE ? size : MAX_DATA_SIZE;
	memcpy(new_data->data, data, new_data->size);

	// Put the sensor data in the FIFO
	k_fifo_put(&sensor_fifo, new_data);
}

/****************************************************************************/
/****************************************************************************/

void set_battery_data(uint16_t *data)
{
	// Convert the voltage to a battery percentage
	uint8_t percent = (uint8_t)(((data[0] - 40960) * 100) / 7680);
	LOG_INF("Battery : %d\n", percent);
	bt_bas_set_battery_level(percent); 
}

/****************************************************************************/
/****************************************************************************/

int start_ble(void)
{
	uint32_t led_status = 0;
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	LOG_INF("Bluetooth initialized\n");

	err = create_advertising_coded();
	if (err) {
		LOG_ERR("Advertising failed to create (err %d)\n", err);
		return 0;
	}

	k_work_submit(&start_advertising_worker);
	// k_work_schedule(&notify_work, K_NO_WAIT);
	k_thread_create(&sensor_thread_data, sensor_thread_stack,
			K_THREAD_STACK_SIZEOF(sensor_thread_stack),
			(k_thread_entry_t)sensor_thread, NULL, NULL, NULL,
			BLE_PRIO, 0, K_NO_WAIT);

	return 0;
}

/****************************************************************************/
/****************************************************************************/

void enable_stream_data(void)
{
	stream_data = true;
}

/****************************************************************************/
/****************************************************************************/

void disable_stream_data(void)
{
	stream_data = false;
}

/****************************************************************************/
/****************************************************************************/

void set_status(uint8_t new_status)
{
	status[0] = new_status;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

static ssize_t read_status(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset, uint8_t flags){
	
	return bt_gatt_attr_read(conn, attr, buf, len, offset, status, sizeof(status));			
}

/****************************************************************************/
/****************************************************************************/

static void start_advertising_coded(struct k_work *work)
{
	int err;

	err = bt_le_ext_adv_start(adv, NULL);
	if (err) {
		LOG_ERR("Failed to start advertising set (err %d)\n", err);
		return;
	}

	printk("Advertiser %p set started\n", adv);
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/






