/* main.c - Application main entry point */

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

#include "UART_handler.h"
#include <SEGGER_RTT.h>

#define DEVICE_LABEL "ble_loc_d-02_"
#define SENT_TIME_THRESHOLD 3000
#define NUM_OF_BEACONS 10
#define PAYLOAD_BUFF_LEN NUM_OF_BEACONS * 7 + 8

#define P_STATUS_IDLE 0
#define P_STATUS_SENT 1
#define P_STATUS_READY_TO_SEND 2

#define STACKSIZE 2048
#define PRIORITY 7

typedef struct Found_Anchor {
	int anch_num;
	char *addr;
	int8_t rssi;
	int64_t lastUpdate;
} Found_Anchor;

struct Found_Anchor anchor_array[NUM_OF_BEACONS];

typedef struct Payload {
	char data[PAYLOAD_BUFF_LEN];
	int len;
} Payload;

struct send_data_t {
	// void *fifo_reserved;
	uint8_t status;
	struct Payload payload;
};

struct send_data_t SEND_DATA = { .status = P_STATUS_READY_TO_SEND,
				 .payload = { .data = "empty", .len = 0 } };

K_FIFO_DEFINE(lte_send_payload);

char BECON_ADDRS[NUM_OF_BEACONS][17] = {
	// "00:80:25:D1:29:B8", // 1-> 1003
	"00:80:25:E5:D3:0A",
	"00:80:25:D1:29:3A", //2-> 1006
	"00:80:25:D1:29:22", // 3-> 1031
	"00:80:25:D1:29:A9", // 4->1027
	"00:80:25:D1:29:55", //5 ->1015
	"00:80:25:CF:2F:C4", // 6 -> 2202
	"00:80:25:E5:D3:CA", // 7 -> 2101
	"88:6B:0F:6B:2B:30", // 8
	"88:6B:0F:6B:2D:2E", // 9
	"00:80:25:D1:28:BB", // 10
};

/*----------------------------------------------------------------
compares addr with all beacons addrs defined in BECON_ADDRS return beacon's index if it 
matches with at leas one of them otherwise returns -1
*/
int is_draeger_beacon(char *addr)
{
	int cond = 0;
	// memset(cond, 0, NUM_OF_BEACONS);

	for (int beacon_i = 0; beacon_i < NUM_OF_BEACONS; beacon_i++) {
		for (int i = 0; i < 17; i++) {
			cond = *(addr + i) == *(BECON_ADDRS[beacon_i] + i) ? 1 :
										   0;
			if (!cond) {
				break;
			}
		}
		if (cond) {
			return beacon_i;
		}
	}
	return -1;
}
static void start_scan(void);

static struct bt_conn *default_conn;

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	if (default_conn) {
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	/* We're only interested in connectable events */
	// if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	//     type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
	// 	return;
	// }

	char just_addr[17] = { "\0" };
	memcpy(just_addr, addr_str, 17);
	// printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

	int i_beacon = is_draeger_beacon(addr_str);
	// printk("i_beacon: %d \n", i_beacon);
	if (i_beacon != -1) {
		// printk("------------------------- \n");
		// printk("fount it! \n");
		printk("Device found: %s (RSSI %d)\n", addr_str, rssi);
		// printk("i_beacon: %d \n", i_beacon);
		int64_t currTime = k_uptime_get();
		anchor_array[i_beacon].lastUpdate = currTime;
		anchor_array[i_beacon].rssi = -1 * rssi;
		anchor_array[i_beacon].anch_num = i_beacon + 1;
		memcpy(anchor_array[i_beacon].addr, addr_str, 17);
		anchor_array[i_beacon].addr[17] = *"\0";
		// printk("anch_num: %d \n", anchor_array[i_beacon].anch_num);
		// printk("anch_rssi: %d \n", anchor_array[i_beacon].rssi);
		// printk("anch_addr %s \n", anchor_array[i_beacon].addr);
		// printk("lastUpdate %lld \n", anchor_array[i_beacon].lastUpdate);

		// printk("------------------------- \n");
	}

	/* connect only to devices in close proximity */

	if (bt_le_scan_stop()) {
		return;
	}

	// err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
	// 			BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	// if (err) {
	// 	printk("Create conn to %s failed (%u)\n", addr_str, err);
	// 	start_scan();
	// }
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

const struct device *uart1_dev;
static void init_uart()
{
	// uint8_t err = 0;
	// const struct device *uart1_dev;
	uart1_dev = init_uart_dev("UART_1");
	if (!uart1_dev) {
		printk("[UART] Failed to get device!\n");
	}
	uart_irq_rx_enable(uart1_dev);
}

static void nbiot_send(struct Payload *payload)
{
	int err;
	char tx_buffer[UART_TX_LINE_BUF_SIZE] = { '\0' };
	char rx_buffer[UART_RX_LINE_BUF_SIZE] = { '\0' };
	int len = 0;

	// memcpy(tx_buffer, payload->data, payload->len);
	strcpy(tx_buffer, payload->data);
	printk("Send to nRF9160: %s\n", tx_buffer);
	uart_irq_rx_enable(uart1_dev);
	err = tx_message_uart(tx_buffer, strlen(tx_buffer), uart1_dev);
	if (err < 0) {
		printk("FAILED UART TX\n");
	}
	// memset(tx_buffer, 0, sizeof(tx_buffer));

	len = rx_message_uart(rx_buffer, K_MSEC(1000), uart1_dev);
	if (len <= 0) {
		printk("Failed UART RX Receiption from nRF9160\n");
	} else {
		printk("UART RX from nRF9160: %s\n", rx_buffer);
		if (!strcmp(rx_buffer, "ACK\n") == 0) {
			printk("faliled to recieve acknowledgement!\n");
		}
	}
	printk("out ...\n");
	k_msleep(2000);

	uart_irq_rx_disable(uart1_dev);
}

int blt_force_stop()
{
	int flag = 1;
	int ret;
	int MAX_ITER = 6000;
	int iter = 0;
	while (flag) {
		ret = bt_scan_stop();
		if (ret == 0) {
			flag = 0;
		}
		if (iter > MAX_ITER) {
			flag = 0;
		}
		iter++;
	}

	return ret;
}

int get_last_index(char *data)
{
	int tip = 0;
	while (*(data + tip) != '\0') {
		tip++;
	}
	return tip;
}

void prepare_payload(struct Payload *payload, int64_t currTime)
{
	memset(payload->data, 0, PAYLOAD_BUFF_LEN);
	// int EPS = 1000;
	strcpy(payload->data, DEVICE_LABEL);
	int data_tip = get_last_index(payload->data);
	printk("\n\n++++++++++++++++++++++++++++++++++++++\n");

	for (int i_anchor = 0; i_anchor < NUM_OF_BEACONS; i_anchor++) {
		// printk("index: %d\n", i_anchor);
		// printk("diff: %lld \n",
		//    currTime - anchor_array[i_anchor].lastUpdate);
		if (currTime - anchor_array[i_anchor].lastUpdate <
		    SENT_TIME_THRESHOLD) {
			// printk("anchor_num: %d\n",
			//        anchor_array[i_anchor].anch_num);
			u8_to_dec(payload->data + data_tip, 3,
				  anchor_array[i_anchor].anch_num);

			data_tip = get_last_index(payload->data);
			strcpy(payload->data + data_tip, "-");
			data_tip += 1;
			u8_to_dec(payload->data + data_tip, 3,
				  anchor_array[i_anchor].rssi);
			// printk("anchor_rssi: %d\n",
			//        anchor_array[i_anchor].rssi);

			data_tip = get_last_index(payload->data);
			strcpy(payload->data + data_tip, "-");
			data_tip += 1;
		}
	}
	if (data_tip != 8) {
		// printk("data_tip %d\n", data_tip);
		payload->data[data_tip - 1] = *"\n";
	}
	payload->data[data_tip] = *"\0";
	payload->len = data_tip;

	printk("final payload->len: %d\n", payload->len);
	printk("final payload->data: %s", payload->data);
	printk("\n++++++++++++++++++++++++++++++++++++++\n\n");

	SEND_DATA.status = P_STATUS_READY_TO_SEND;
	memcpy(&SEND_DATA.payload, payload, sizeof(payload));
	// size_t size = sizeof(struct send_data_t);
	// char *mem_ptr = k_malloc(size);
	// __ASSET_NO_MSG(mem_ptr != 0);
	// memcpy(mem_ptr, &send_data, size);
	// k_fifo_put(&lte_send_payload, mem_ptr);
}

void init_anchor_array()
{
	for (int i = 0; i < NUM_OF_BEACONS; i++) {
		anchor_array[i] = *(struct Found_Anchor *)k_malloc(
			sizeof(struct Found_Anchor));
		anchor_array[i].addr = (char *)k_malloc(17 * sizeof(char *));
		// memset(anchor_array[i].addr, 0, 17);
		anchor_array[i].anch_num = i + 1;
		anchor_array[i].rssi = 0;
		anchor_array[i].lastUpdate = 0;
	}
}

void lte_send_thread(void)
{
	SEGGER_RTT_Init();
	k_msleep(3000);
	printk("starting sending lte ...\n");
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	init_uart();
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	while (1) {
		// struct send_data_t *data =
		// 	k_fifo_get(&lte_send_payload, K_FOREVER);
		if (SEND_DATA.status == P_STATUS_READY_TO_SEND) {
			nbiot_send(&SEND_DATA.payload);
			SEND_DATA.status = P_STATUS_SENT;
			// size_t size = sizeof(struct send_data_t);
			// char *mem_ptr = k_malloc(size);
			// __ASSET_NO_MSG(mem_ptr != 0);
			// memcpy(mem_ptr, &send_data, size);
			// k_fifo_put(&lte_send_payload, mem_ptr);
		}
		// k_free(data);
	}
}

void search_beacons_thread(void)
{
	SEGGER_RTT_Init();
	k_msleep(3000);
	printk("starting beacon_search ...\n");
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	init_anchor_array();
	struct Payload payload =
		*(struct Payload *)k_malloc(sizeof(struct Payload));

	int err;
	err = bt_enable(NULL);
	printk("Bluetooth initialized status: %d\n", err);
	bt_conn_cb_register(&conn_callbacks);
	start_scan();
	int64_t lastTime = 0;

	while (true) {
		int64_t currTime = k_uptime_get();
		// printk("curr time-> %lld , time diff-> %lld \n", currTime,
		//        currTime - lastTime);
		if (currTime - lastTime > SENT_TIME_THRESHOLD) {
			blt_force_stop();
			prepare_payload(&payload, currTime);
			//sed data
			// nbiot_send(&payload);
			k_msleep(4000);
			lastTime = k_uptime_get();
			start_scan();
		}
	}
}

// void main(void)

// {
// 	SEGGER_RTT_Init();
// 	k_msleep(3000);

// 	printk("starting the program ...\n");

// 	init_uart();
// 	init_anchor_array();
// 	struct Payload payload =
// 		*(struct Payload *)k_malloc(sizeof(struct Payload));

// 	int err;

// 	err = bt_enable(NULL);
// 	if (IS_ENABLED(CONFIG_SETTINGS)) {
// 		settings_load();
// 	}
// 	if (err) {
// 		printk("Bluetooth init failed (err %d)\n", err);
// 		return;
// 	}

// 	printk("Bluetooth initialized\n");

// 	bt_conn_cb_register(&conn_callbacks);

// 	start_scan();

// 	int64_t lastTime = 0;

// 	while (true) {
// 		int64_t currTime = k_uptime_get();
// 		// printk("curr time-> %lld , time diff-> %lld \n", currTime,
// 		//        currTime - lastTime);
// 		if (currTime - lastTime > SENT_TIME_THRESHOLD) {
// 			blt_force_stop();
// 			prepare_payload(&payload, currTime);
// 			//sed data
// 			nbiot_send(&payload);
// 			// k_msleep(4000);
// 			lastTime = k_uptime_get();
// 			start_scan();
// 		}
// 	}
// }

// K_THREAD_DEFINE(search_id, STACKSIZE, search_beacons_thread, NULL, NULL, NULL,
// 		PRIORITY, 0, 0);
K_THREAD_DEFINE(send_id, STACKSIZE, lte_send_thread, NULL, NULL, NULL, PRIORITY,
		0, 0);