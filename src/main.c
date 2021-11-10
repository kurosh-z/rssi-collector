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

#define STACKSIZE1 2048
#define STACKSIZE2 1024
#define DEVICE_LABEL "ble_loc_d-02_"
#define SENT_TIME_THRESHOLD 3000
#define NUM_OF_BEACONS 4
#define SEND_BUFF_LEN 350
#define NUM_OF_RSSI_COLECTION 5
#define MAX_NUM_NEIGHBOUR_ANCHORS 4

#define C_STATUS_COLLECTING 0
#define C_STATUS_SENT 1
#define C_STATUS_READY_TO_BE_SENT 2

#define BUF_STATUS_PENDING 0
#define BUF_STATUS_TRYING_TO_SEND 1
#define BUF_STATUS_SENT 2

int num_of_sent_neighbouring_anchors = 0;

char BECON_ADDRS[NUM_OF_BEACONS][17] = {
	"20:C3:8F:FA:1C:28", //1
	"A0:E6:F8:54:4F:F6", //2
	"98:07:2D:3B:8B:89", //3
	"A0:E6:F8:54:4F:AE" //4
};

typedef struct RssiInfo {
	int beacon_num;
	int counter;
	uint8_t status;
	int8_t rssi_array[NUM_OF_RSSI_COLECTION];
} RssiInfo;

struct RssiInfo *rssi_collection[NUM_OF_BEACONS];

typedef struct Send_Buff {
	char data[SEND_BUFF_LEN];
	int len;
	uint8_t status;
} Send_Buff;

// Send_Buff send_buff = { .data = "ble_loc_test", .len = 19 };

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
		// printk("fount it! beac %d \n", i_beacon);
		// printk("Device found: %s (RSSI %d)\n", addr_str, rssi);
		// printk("i_beacon: %d \n", i_beacon);
		// int64_t currTime = k_uptime_get();
		// if we gattered enought rssi return
		if (rssi_collection[i_beacon]->status ==
		    C_STATUS_READY_TO_BE_SENT) {
			return;
		}
		if (rssi_collection[i_beacon]->status == C_STATUS_SENT) {
			return;
		}
		rssi_collection[i_beacon]->beacon_num = i_beacon;
		int curr_idx = rssi_collection[i_beacon]->counter;
		rssi_collection[i_beacon]->rssi_array[curr_idx] = -1 * rssi;
		rssi_collection[i_beacon]->counter = curr_idx + 1;
		if (rssi_collection[i_beacon]->counter ==
		    NUM_OF_RSSI_COLECTION) {
			rssi_collection[i_beacon]->status =
				C_STATUS_READY_TO_BE_SENT;
		}

		// printk("beacon_num: %d \n",
		//        rssi_collection[i_beacon]->beacon_num);
		// printk("curr_idx: %d \n", curr_idx);
		// printk("rssi found: %d \n", rssi);
		// printk("counter: %d \n", rssi_collection[i_beacon]->counter);
		// printk("curr_idx: %d \n", curr_idx);
		// printk("status %d \n", rssi_collection[i_beacon]->status);

		// printk("------------------------- \n");
		// k_msleep(2000);
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
		printk("[UART1] Failed to get device!\n");
	}
	uart_irq_rx_enable(uart1_dev);
}
const struct device *uart0_dev;
static void init_uart0()
{
	// uint8_t err = 0;
	// const struct device *uart1_dev;
	uart0_dev = init_uart_dev("UART_0");
	if (!uart0_dev) {
		printk("[UART0] Failed to get device!\n");
	}
	uart_irq_rx_enable(uart0_dev);
}

static void send_uart0(char *send_buff)
{
	int err;
	char tx_buffer[UART_TX_LINE_BUF_SIZE] = { '\0' };

	strcpy(tx_buffer, send_buff);
	printk("Send to uart0: %s\n", tx_buffer);

	err = tx_message_uart(tx_buffer, strlen(tx_buffer), uart0_dev);
	if (err < 0) {
		printk("FAILED UART0 TX\n");
	}
}

static void receive_uart0()
{
	char rx_buffer[UART_RX_LINE_BUF_SIZE] = { '\0' };
	int len = 0;
	uart_irq_rx_enable(uart0_dev);
	while (1) {
		printk("test:\n");
		// if (!strcmp(rx_buffer, "ack\n") == 0) {
		// 	printk("recived:\n");
		// }
		k_msleep(3000);
	}
	// len = rx_message_uart(rx_buffer, K_MSEC(2000), uart1_dev);
	len = rx_message_uart(rx_buffer, K_MSEC(20000), uart0_dev);
	while (1) {
		printk("recived:\n %s", rx_buffer);
		// if (!strcmp(rx_buffer, "ack\n") == 0) {
		// 	printk("recived:\n");
		// }
		k_msleep(1000);
	}
	// uart_irq_rx_disable(uart0_dev);
}

static void nbiot_send(struct Send_Buff *send_buff)
{
	int err;
	char tx_buffer[UART_TX_LINE_BUF_SIZE] = { '\0' };
	char rx_buffer[UART_RX_LINE_BUF_SIZE] = { '\0' };
	int len = 0;

	// memcpy(tx_buffer, payload->data, payload->len);
	strcpy(tx_buffer, send_buff->data);
	send_buff->status = BUF_STATUS_TRYING_TO_SEND;
	printk("Send to nRF9160: %s\n", tx_buffer);
	uart_irq_rx_enable(uart1_dev);
	err = tx_message_uart(tx_buffer, strlen(tx_buffer), uart1_dev);
	if (err < 0) {
		printk("FAILED UART TX\n");
	}
	// memset(tx_buffer, 0, sizeof(tx_buffer));

	len = rx_message_uart(rx_buffer, K_MSEC(2000), uart1_dev);
	if (len <= 0) {
		printk("Failed UART RX Receiption from nRF9160\n");
	} else {
		printk("UART RX from nRF9160: %s\n", rx_buffer);
		if (!strcmp(rx_buffer, "ACK\n") == 0) {
			printk("faliled to recieve acknowledgement!\n");
		} else {
			send_buff->status = BUF_STATUS_SENT;
			num_of_sent_neighbouring_anchors++;
		}
	}

	// printk("out ...\n");
	// k_msleep(2000);

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

void prepare_send_data(Send_Buff *buff, int beacon_idx, uint8_t pX, uint8_t pY)
{
	memset(buff->data, 0, SEND_BUFF_LEN);
	int data_tip = 0;
	strcpy(buff->data, "ble_loc_COLL_pX");
	u8_to_dec(buff->data + strlen(buff->data), 3, pX);
	strcat(buff->data, "_pY");
	data_tip = get_last_index(buff->data);
	u8_to_dec(buff->data + data_tip, 3, pY);
	strcat(buff->data, "_from_");
	data_tip = get_last_index(buff->data);
	u8_to_dec(buff->data + data_tip, 3,
		  rssi_collection[beacon_idx]->beacon_num + 1);
	strcat(buff->data, "_rssi_");
	data_tip = get_last_index(buff->data);

	printk("\n\n++++++++++++++++++++++++++++++++++++++\n");
	printk("preparing beacon number: %d", beacon_idx);

	for (int idx = 0; idx < NUM_OF_RSSI_COLECTION - 1; idx++) {
		// printk("index: %d\n", i_anchor);
		// printk("anchor_num: %d\n",
		//        anchor_array[i_anchor].anch_num);
		u8_to_dec(buff->data + data_tip, 3,
			  rssi_collection[beacon_idx]->rssi_array[idx]);
		strcat(buff->data, "-");
		data_tip = get_last_index(buff->data);
	}
	buff->data[data_tip - 1] = *"\n";
	// buff->data[data_tip+] = *"\0";
	buff->len = data_tip - 1;

	// printk("final payload->len: %d\n", payload->len);
	printk("fbuf->data: %s", buff->data);
	printk("\n++++++++++++++++++++++++++++++++++++++\n\n");
}

void send_lte(void)

{
	SEGGER_RTT_Init();
	k_msleep(4000);
	printk("starting the lte send ...\n");

	init_uart();
	printk("uart initialized \n");

	struct Send_Buff send_buff = *(Send_Buff *)k_malloc(sizeof(Send_Buff));

	//just for the test
	strcpy(send_buff.data, "ble_loc_02-12-56-57-code1234\n");
	send_buff.len = strlen(send_buff.data);
	send_buff.status = BUF_STATUS_PENDING;
	printk("data is: %s ,data len:%d", send_buff.data, send_buff.len);
	while (1) {
		printk("sendin ...\n");
		nbiot_send(&send_buff);
		k_msleep(10000);
	}

	while (num_of_sent_neighbouring_anchors < MAX_NUM_NEIGHBOUR_ANCHORS) {
		for (int idx = 0; idx < NUM_OF_BEACONS; idx++) {
			// printf("------------------\n");
			// printf("counter_%d -> %d \n", idx,
			//        rssi_collection[idx]->counter);
			if (rssi_collection[idx]->status ==
			    C_STATUS_READY_TO_BE_SENT) {
				// printf("sendstatus %d \n", send_buff.status);
				if (send_buff.status ==
				    BUF_STATUS_TRYING_TO_SEND) {
					nbiot_send(&send_buff);
				} else if (send_buff.status ==
					   BUF_STATUS_PENDING) {
					prepare_send_data(&send_buff, idx, 0,
							  0);
					nbiot_send(&send_buff);
				} else if (send_buff.status ==
					   BUF_STATUS_SENT) {
					rssi_collection[idx]->status =
						C_STATUS_SENT;
					send_buff.status = BUF_STATUS_PENDING;
				}
			}
			// printf("------------------\n");
			// k_msleep(2000);
		}
	}
	printk("finished sendig neighbouring anchors, count= %d",
	       num_of_sent_neighbouring_anchors);
	k_cpu_idle();
}

void search_beacons(void)

{
	SEGGER_RTT_Init();
	k_msleep(2000);

	printk("starting search_beacons-entry ...\n");
	// init_uart();
	// init_rssi_collection();
	// struct Payload payload =
	// 	*(struct Payload *)k_malloc(sizeof(struct Payload));
	RssiInfo *myrssi_collection =
		(RssiInfo *)k_malloc(NUM_OF_BEACONS * sizeof(struct RssiInfo));

	for (int i = 0; i < NUM_OF_BEACONS; i++) {
		// rssi_collection[i] =
		// 	*(struct RssiInfo *)k_malloc(sizeof(struct RssiInfo));
		rssi_collection[i] = myrssi_collection + i;
		rssi_collection[i]->beacon_num = i;
		rssi_collection[i]->counter = 0;
		rssi_collection[i]->status = C_STATUS_COLLECTING;
		// printk("becnum %d\n", rssi_collection[i]->beacon_num);

		memset(rssi_collection[i]->rssi_array, 1,
		       NUM_OF_RSSI_COLECTION);
	}

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

	// uint8_t flag = 1;
	// while (flag) {
	// 	if (num_of_sent_neighbouring_anchors >=
	// 	    MAX_NUM_NEIGHBOUR_ANCHORS) {
	// 		blt_force_stop();
	// 		flag = 0;
	// 	}
	// }
	// k_cpu_idle();

	// int64_t lastTime = 0;

	// k_cpu_idle();

	// while (true) {
	// 	int64_t currTime = k_uptime_get();
	// 	// printk("curr time-> %lld , time diff-> %lld \n", currTime,
	// 	//        currTime - lastTime);
	// 	if (currTime - lastTime > SENT_TIME_THRESHOLD) {
	// 		blt_force_stop();
	// 		prepare_payload(&PAYLOAD, currTime);
	// 		//sed data
	// 		// nbiot_send(&PAYLOAD);
	// 		// k_msleep(2000);
	// 		lastTime = k_uptime_get();
	// 		start_scan();
	// 	}
	// }
}

void send_uart_entry()
{
	SEGGER_RTT_Init();
	k_msleep(2000);

	printk("starting the uart_entry\n");

	init_uart0();
	// receive_uart0();
	k_cpu_idle();
	// char *buff = "testing uart0\r\n";
	// while (1) {
	// 	send_uart0(buff);
	// 	k_msleep(3000);
	// }
}

// K_THREAD_DEFINE(search_id, STACKSIZE1, search_beacons, NULL, NULL, NULL, 7, 0,
// 		0);
// K_THREAD_DEFINE(send_id, STACKSIZE1, send_lte, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(send_uart_id, STACKSIZE1, send_uart_entry, NULL, NULL, NULL, 6,
		0, 0);
