/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <sys/byteorder.h>

#include <string.h>
#include <stdio.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <settings/settings.h>
#include <stdlib.h>

static struct bt_conn *default_conn;

static void start_scan(void);
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable);
static void scan_connecting_error(struct bt_scan_device_info *device_info);
static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn);
static int scan_init(void);

#define NAME_LEN 30

/**
 * @brief Convertes four passed bytes to a float.
 *
 * @param[in] b0  Byte 0
 * @param[in] b1  Byte 1
 * @param[in] b2  Byte 2
 * @param[in] b3  Byte 3
 *
 * @retval Converted float from passed bytes
 *
 */
static float bytesToFloat(unsigned char b0, unsigned char b1, unsigned char b2,
			  unsigned char b3)
{
	//printk("B0: %X , B1: %X , B2: %X , B3: %X \n", b0, b1, b2, b3);
	union {
		float convertedfloat;
		unsigned long convert;
	} converter;

	converter.convert = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
	return converter.convertedfloat;
}

static void parse_sensor_data(struct bt_data *data, void *sensor_data)
{
	printk("parse sesnor: %d\n", sizeof(*sensor_data));
	// memcpy(sensor_data->manufacturer, data->data, 2 * sizeof(uint8_t));
	// memcpy(&sensor_data->idType, data->data + 2, 1 * sizeof(uint8_t));

	// sensor_data->battery_capacity = data->data[5];
	// sensor_data->gas_value = bytesToFloat(data->data[7], data->data[8],
	// 				      data->data[9], data->data[10]);

	// printk("manufacturer-> %x-%x \n", sensor_data->manufacturer[0],
	//        sensor_data->manufacturer[1]);
	// printk("idType: %x, %x", sensor_data->idType, data->data[2]);

	// for (int i = 0; i < 14; i++) {
	// 	uint8_t mask = (0x01 << i);
	// 	if (i < 8) {
	// 		sensor_data->alarms[i] =
	// 			mask & data->data[3] ? true : false;
	// 		// printk("mask: %x, val-> %x set-> %x \n", mask,
	// 		//        mask & data->data[3], sensor_data->alarms[i]);
	// 	} else {
	// 		sensor_data->alarms[i] =
	// 			mask & data->data[4] ? true : false;
	// 		;
	// 	}
	// }
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error,
		scan_connecting);

static void start_scan(void)
{
	int err;

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_load();
		if (err) {
			printk("Failed loading settings (err %d)", err);
			return;
		}
	}

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		printk("Scanning failed to start (err %d)", err);
		return;
	}

	printk("Scanning successfully started\n");
	return;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", addr);

	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

static bool data_cb(struct bt_data *data, void *user_data)

{
	printk("\n--------------------------\n");
	printk("data type: %x \n", data->type);
	// if (data->type == 0xff) {
	// 	// for (int i = 0; i < data->data_len; i++) {
	// 	// 	printk("%d = %x\n", i, data->data[i]);
	// 	// }
	// 	parse_sensor_data(data, &SENSOR_DATA);
	// } else if (data->type == 0x08) {
	// 	char name[data->data_len];
	// 	memcpy(name, data->data, data->data_len);
	// 	printk("data name: %s \n", (char *)name);
	// }

	// else if (data->type == 0x01) {
	// 	for (int i = 0; i < data->data_len; i++) {
	// 		printk("flag %d = %x\n", i, data->data[i]);
	// 	}
	// } else if (data->type == 0x02) {
	// 	for (int i = 0; i < data->data_len; i++) {
	// 		printk("uuid tag: %d = %x\n", i, data->data[i]);
	// 	}
	// }

	printk("\n---------------------------\n");
	return true;
	// char *name = user_data;

	// switch (data->type) {
	// case BT_DATA_NAME_SHORTENED:
	// case BT_DATA_NAME_COMPLETE:
	// 	// memcpy(name, data->data, MIN(data->data_len, NAME_LEN - 1));
	// 	printk("cb: name: %s",(char*) user_data);
	// 	return false;
	// default:
	// 	return true;
	// }
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	// char addr[BT_ADDR_LE_STR_LEN];
	char name[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, name, sizeof(name));
	// memcpy(name, device_info->adv_data+2,device_info->adv_data->size-2);

	printk("\n before name : %s \n", (char *)(name));

	bt_data_parse(device_info->adv_data, data_cb, name);
	// printk("\n after parse name : %s \n",(char*) (name));

	//snprintf(adv_buffer, sizeof(), (char*)device_info->adv_data->data);

	printk("Filters matched. Address: %s connectable: %d RSSI: %d\n", name,
	       connectable, device_info->recv_info->rssi);

	// printk("Advertisement Data: %s, Size: %d\n", my_data->name, device_info->adv_data->len);

	// memset(name, 0, sizeof(my_data->name));
	// strncpy(name, "abcd", 4);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	default_conn = bt_conn_ref(conn);
}

static int scan_init(void)
{
	int err;

	struct bt_scan_init_param scan_init = { .connect_if_match = 1 };

	// uint8_t manufacturer_data_array[2] = { 0xBC, 0x04 };
	// struct bt_scan_manufacturer_data manufacturer_data = {
	// 	.data = manufacturer_data_array, .data_len = 2
	// };

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);
	// 00:80:25:D1:29:3A
	// clang-format off
	bt_addr_t f_add ={ { 0x3A, 0x29, 0xD1, 0x25, 0x80, 0x0 }};
	bt_addr_le_t filter_add = { .type = BT_ADDR_LE_PUBLIC, .a = f_add };
	// clang-format on

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_ADDR, &filter_add);

	// err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_MANUFACTURER_DATA,
	// 			 &manufacturer_data);
	if (err) {
		printk("Scanning filters cannot be set (err %d)\n", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_MANUFACTURER_DATA_FILTER, false);
	if (err) {
		printk("Filters cannot be turned on (err %d)\n", err);
		return err;
	}
	return 0;
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

void main(void)
{
	int err;

	printk("Starting Scanning for Draeger BLE Devices\n");
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	k_msleep(3000);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	bt_conn_cb_register(&conn_callbacks);

	err = scan_init();
	if (err) {
		return;
	}

	start_scan();
}