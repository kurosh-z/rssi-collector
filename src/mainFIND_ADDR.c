/* find device mac adrr: find the strongest which is rssi > -70 and print the mac adress of the divice */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
// 8899
#include <stdio.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <devicetree.h>
#include <sys/__assert.h>
#include <string.h>
#include <drivers/uart.h>
#include <kernel.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

// #include "UART_handler.h"
// #include <SEGGER_RTT.h>

#define TIME_THRESHOLD 15000

typedef struct Found_Dev {
	char addr[BT_ADDR_LE_STR_LEN];
	int rssi;
	int64_t lastUpdate;
} Found_Dev;

Found_Dev *devinfo;

static void start_scan(void);

static struct bt_conn *default_conn;

int is_same_addr(char *addr, char *target_addr)
{
	int cond = 0;
	for (int i = 0; i < BT_ADDR_LE_STR_LEN; i++) {
		cond = *(addr + i) == *(target_addr + i) ? 1 : 0;
		if (!cond) {
			return 0;
		}
	}
	return 1;
}

void get_addr_type(char *addr, char *type)
{
	uint8_t started = 0;
	int j = 0;
	for (int i = 0; i < BT_ADDR_LE_STR_LEN; i++) {
		if (*(addr + i) == *"(") {
			started = 1;
		}
		if (started) {
			*(type + j) = addr[i + 1];
			j++;
		}
		if (j > 5) {
			break;
		}
	}
	type[7] = *"\0";
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	char addr_type[7] = "123456";
	int err;

	if (default_conn) {
		return;
	}
	if (rssi < -70) {
		return;
	}
	int64_t currTime = k_uptime_get();
	// if device is not there anymore reset rssi
	if (currTime - devinfo->lastUpdate > TIME_THRESHOLD) {
		devinfo->rssi = -200;
		printk("time threshold (%lld ms) reached, reseting rssi ... \n",
		       currTime - devinfo->lastUpdate);
	}

	if (rssi > devinfo->rssi) {
		devinfo->rssi = rssi;
		bt_addr_le_to_str(addr, devinfo->addr, sizeof(addr_str));
		get_addr_type(devinfo->addr, addr_type);

		if (strcmp(addr_type, "random") == 0) {
			return;
		}
		// strcpy(devinfo->addr, addr_str);
		devinfo->lastUpdate = currTime;

		printk("========================= \n\n");

		printk("mac: %s \n", devinfo->addr);
		printk("rssi: %d \n", devinfo->rssi);
		printk("lastUpdate: %lld \n", devinfo->lastUpdate);
		printk("========================= \n\n");
	}

	// printk("------------------------- \n");
	// bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	// printk("mac: %s \n", addr_str);
	// printk("rssi: %d \n", rssi);
	// printk("lastUpdate: %lld \n", devinfo->lastUpdate);
	// printk("------------------------- \n");

	if (bt_le_scan_stop()) {
		return;
	}

	start_scan();
}

static void start_scan(void)
{
	int err;

	/* pasive scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	// printk("Scanning successfully started\n");
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

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

void main(void)

{
	// SEGGER_RTT_Init();
	k_msleep(3000);

	printk("starting the program ...\n");

	devinfo = (struct Found_Dev *)k_malloc(sizeof(struct Found_Dev));
	devinfo->lastUpdate = 0;
	devinfo->rssi = -200;

	int err;

	err = bt_enable(NULL);
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	bt_conn_cb_register(&conn_callbacks);

	start_scan();

	// while (true) {
	// 	int64_t currTime = k_uptime_get();
	// 	// printk("curr time-> %lld , time diff-> %lld \n", currTime,
	// 	//        currTime - lastTime);
	// 	if (currTime - lastTime > SENT_TIME_THRESHOLD) {
	// 		blt_force_stop();
	// 		prepare_payload(&payload, currTime);
	// 		//sed data
	// 		nbiot_send(&payload);
	// 		lastTime = k_uptime_get();
	// 		start_scan();
	// 	}
	// }
}