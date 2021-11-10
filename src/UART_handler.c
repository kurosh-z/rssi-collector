/*
Author: Björn Sievers
Project: UART_handler
Purpose: Enables easier integration of UART message handling in the desired project.
This project uses ISR & the Zephyr OS FIFO feature to store & send messages from a desired UART port
This handler has the feature to listen for specific "endMarkers" which are expected as an indicator of the end of a UART message.
The endMarkers to be listend to can be specified in the "UART_handler.h" file.
IMPORTANT: If the endMarker does not appear in the received UART string the received message will not be made stored into the FIFO structure!
*/

//++++++++++++++++++++++++++ INCLUDES START +++++++++++++++++++++++++++
#include <zephyr.h>
#include <kernel.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <device.h>
#include <devicetree.h>
#include <string.h>
#include <drivers/uart.h>
#include <stdio.h>
#include "UART_handler.h"
//++++++++++++++++++++++++++ INCLUDES END +++++++++++++++++++++++++++

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart1_tx_data);

static K_FIFO_DEFINE(fifo_uart_rx_data);
static K_FIFO_DEFINE(fifo_uart1_rx_data);

uart_rx_helper uart0_rx = { 0, { 0 }, false };
uart_rx_helper uart1_rx = { 0, { 0 }, false };

const struct device *uart_dev_0;
const struct device *uart_dev_1;

static void rx_isr_uart_0(const struct device *x)
{
	int data_length = 0;
	uint8_t byte;
	data_length = uart_fifo_read(x, &byte, 1);
	if (uart0_rx.uart_rx_buff_len < UART_RX_LINE_BUF_SIZE) {
		uart0_rx.uart_rx_buf[uart0_rx.uart_rx_buff_len] = byte;
		uart0_rx.uart_rx_buff_len += data_length;
	}
	//If received byte stream exceeds maximum buffer size of UART_RX_LINE_BUF_SIZE
	//remaining incomming data will be ignored and the already received data stream will be
	//treated as invalid data which will be tossed away as soon as an EndMarker was found.
	else {
		uart0_rx.dirty = true;
		uart0_rx.uart_rx_buff_len += data_length;
	}
	if (byte == COM_RX_ENDMARKER_1_UART0 ||
	    byte == COM_RX_ENDMARKER_2_UART0) {
		if (!uart0_rx.dirty) {
			uart_rx_fifo_data_t *rx_data =
				k_malloc(sizeof(uart_rx_fifo_data_t));
			if (rx_data) {
				rx_data->received_line =
					k_malloc(UART_RX_LINE_BUF_SIZE);
				if (rx_data->received_line) {
					memcpy(rx_data->received_line,
					       uart0_rx.uart_rx_buf,
					       uart0_rx.uart_rx_buff_len);
					rx_data->len =
						uart0_rx.uart_rx_buff_len;
					rx_data->received_line[rx_data->len] =
						'\0';
					k_fifo_put(&fifo_uart_rx_data, rx_data);
					memset(uart0_rx.uart_rx_buf, 0,
					       UART_RX_LINE_BUF_SIZE);
					uart0_rx.uart_rx_buff_len = 0;
					uart0_rx.dirty = false;
					return;
				}
			}
		} else {
			printk("UART0: Received data stream exceeds buffer size! Len: %d\n",
			       uart0_rx.uart_rx_buff_len);
			memset(uart0_rx.uart_rx_buf, 0, UART_RX_LINE_BUF_SIZE);
			uart0_rx.uart_rx_buff_len = 0;
			uart0_rx.dirty = false;
		}
	}
}

static void rx_isr_uart_1(const struct device *x)
{
	int data_length = 0;
	uint8_t byte;
	data_length = uart_fifo_read(x, &byte, 1);
	if (uart1_rx.uart_rx_buff_len < UART_RX_LINE_BUF_SIZE) {
		uart1_rx.uart_rx_buf[uart1_rx.uart_rx_buff_len] = byte;
		uart1_rx.uart_rx_buff_len += data_length;
	}
	//If received byte stream exceeds maximum buffer size of UART_RX_LINE_BUF_SIZE
	//remaining incomming data will be ignored and the already received data stream will be
	//treated as invalid data which will be tossed away as soon as an EndMarker was found.
	else {
		uart1_rx.dirty = true;
		uart1_rx.uart_rx_buff_len += data_length;
	}
	// printk("Byte: %s\n", (char *)byte);
	if (byte == COM_RX_ENDMARKER_1_UART1 ||
	    byte == COM_RX_ENDMARKER_2_UART1) {
		if (!uart1_rx.dirty) {
			uart_rx_fifo_data_t *rx_data =
				k_malloc(sizeof(uart_rx_fifo_data_t));
			if (rx_data) {
				rx_data->received_line =
					k_malloc(UART_RX_LINE_BUF_SIZE);
				if (rx_data->received_line) {
					memcpy(rx_data->received_line,
					       uart1_rx.uart_rx_buf,
					       uart1_rx.uart_rx_buff_len);
					//printk("Received before FIFO: %s\n", rx_data->received_line);
					rx_data->len =
						uart1_rx.uart_rx_buff_len;
					rx_data->received_line[rx_data->len] =
						'\0';
					k_fifo_put(&fifo_uart1_rx_data,
						   rx_data);
					memset(uart1_rx.uart_rx_buf, 0,
					       UART_RX_LINE_BUF_SIZE);
					uart1_rx.uart_rx_buff_len = 0;
					uart1_rx.dirty = false;
				}
			}
		} else {
			printk("UART1: Received data stream exceeds buffer size! Len: %d\n",
			       uart1_rx.uart_rx_buff_len);
			memset(uart1_rx.uart_rx_buf, 0, UART_RX_LINE_BUF_SIZE);
			uart1_rx.uart_rx_buff_len = 0;
			uart1_rx.dirty = false;
		}
	}
}

static void tx_isr_uart_0(const struct device *x)
{
	static uart_data_t *buf = NULL;
	if (!buf) {
		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		//Check if there is data in the FIFO and if so does it comply with specified max. message length.
		if (buf) {
			if (buf->len > UART_TX_LINE_BUF_SIZE) {
				uart_irq_tx_disable(x);
				buf = NULL;
				printk("UART0: Received data string exceeds maximum specified buffer length: Max.: %d",
				       UART_TX_LINE_BUF_SIZE);
				return;
			}

			uint16_t written = 0;
			while (buf->len > written) {
				written +=
					uart_fifo_fill(x, &buf->data[written],
						       buf->len - written);
				//printk("Written: %d, Send: %s", written, buf->data);
			}
			while (!uart_irq_tx_complete(x)) {
				/* Wait for the last byte to get
		 	* shifted out of the module
			*/
			}
			if (k_fifo_is_empty(&fifo_uart_tx_data)) {
				memset(buf->data, 0, sizeof(buf->data));
				if (buf != NULL) {
					k_free(buf);
				}
				buf = NULL;
				uart_irq_tx_disable(x);
			}
		}
		/* Nothing in the FIFO, nothing to send or received databuffer exceeds maximum length*/
		else {
			uart_irq_tx_disable(x);
			return;
		}
		if (buf != NULL) {
			buf = NULL;
		}
	}
}

static void tx_isr_uart_1(const struct device *x)
{
	uart_data_t *buf2 = NULL;
	if (!buf2) {
		buf2 = k_fifo_get(&fifo_uart1_tx_data, K_NO_WAIT);
		//Check if there is data in the FIFO and if so does it comply with specified max. message length.
		if (buf2) {
			if (buf2->len > UART_TX_LINE_BUF_SIZE) {
				uart_irq_tx_disable(x);
				buf2 = NULL;
				printk("UART1: Received data string exceeds maximum specified buffer length: Max.: %d",
				       UART_TX_LINE_BUF_SIZE);
				return;
			}

			uint16_t written = 0;
			//printk("+++++++++++++++++++++SENDING: %s", buf2->data);
			while (buf2->len > written) {
				written +=
					uart_fifo_fill(x, &buf2->data[written],
						       buf2->len - written);
				printk("Written: %d, Send: %s", written,
				       buf2->data);
			}
			while (!uart_irq_tx_complete(x)) {
				/* Wait for the last byte to get
		 	* shifted out of the module
			*/
			}
			if (k_fifo_is_empty(&fifo_uart1_tx_data)) {
				memset(buf2->data, 0, sizeof(buf2->data));
				if (buf2 != NULL) {
					k_free(buf2);
				}
				buf2 = NULL;
				uart_irq_tx_disable(x);
			}
		}
		/* Nothing in the FIFO, nothing to send or received databuffer exceeds maximum length*/
		else {
			uart_irq_tx_disable(x);
			return;
		}
		if (buf2 != NULL) {
			buf2 = NULL;
		}
	}
}

static void uart_cb(const struct device *x, void *user_data)
{
	uart_irq_update(x);

	//RX_HANDLING
	if (uart_irq_rx_ready(x)) {
		if (x == uart_dev_0) {
			rx_isr_uart_0(x);
		}
		if (x == uart_dev_1) {
			rx_isr_uart_1(x);
		}
	}

	//TX_HANDLING
	if (uart_irq_tx_ready(x)) {
		if (x == uart_dev_0) {
			tx_isr_uart_0(x);
		}
		if (x == uart_dev_1) {
			tx_isr_uart_1(x);
		}
	}
}

int tx_message_uart(char *tx_buf, size_t msg_len, const struct device *x)
{
	// printk("+=+=+=+=+=+=+=+=+=+=+=+=\n");
	// printk("this is tx_message_uart\n");
	if (tx_buf == NULL || msg_len < 0) {
		// printk("1,tx_buff:%s \n", tx_buf);
		return -EINVAL;
	}
	if ((msg_len > CONFIG_UART_0_NRF_TX_BUFFER_SIZE) && (x == uart_dev_0)) {
		// printk("2, msg_len:%d\n ", msg_len);
		return -EFBIG;
	}
	if ((msg_len > CONFIG_UART_1_NRF_TX_BUFFER_SIZE) && (x == uart_dev_1)) {
		// printk("msg_len %d:\n", msg_len);
		return -EFBIG;
	}

	uart_data_t *tx_data = k_malloc(sizeof(uart_data_t));
	if (tx_data == NULL) {
		printk("NO MEMORY ALLOCATED!!\n");
		return -EINVAL;
	}
	tx_data->len = msg_len;
	memcpy(tx_data->data, tx_buf, msg_len);

	if (x == uart_dev_0) {
		k_fifo_put(&fifo_uart_tx_data, tx_data);
		//k_free(tx_data);
		uart_irq_tx_enable(x);
		// printk("return 0 1\n");
		return 0;
	}
	if (x == uart_dev_1) {
		k_fifo_put(&fifo_uart1_tx_data, tx_data);
		//k_free(tx_data);
		uart_irq_tx_enable(x);
		// printk("return 0 2\n");
		return 0;
	} else
		k_free(tx_data);
	// printk("k_free ...\n");
	return -EINVAL;
}

int rx_message_uart(char *rx_buf, k_timeout_t timeout, const struct device *x)
{
	if (rx_buf == NULL || x == NULL) {
		return -EINVAL;
	}

	int len = 0;
	uart_rx_fifo_data_t *received = NULL;
	if (x == uart_dev_0) {
		received = k_fifo_get(&fifo_uart_rx_data, timeout);
	}
	if (x == uart_dev_1) {
		received = k_fifo_get(&fifo_uart1_rx_data, timeout);
	}
	if (received) {
		memcpy(rx_buf, received->received_line, received->len);
		printk("rx_buf: %s \n", rx_buf);
		len = received->len;
		k_free(received->received_line);
		k_free(received);
	} else {
		return -EINVAL;
	}
	return len;
}

//ZERO WHEN AVAILABLE
int check_rx_uart_available(const struct device *x)
{
	int ret;

	if (x == uart_dev_0) {
		//ZERO WHEN AVAILABLE
		ret = k_fifo_is_empty(&fifo_uart_rx_data);
		printk("RETURN FIFO_CHECK %d, non-zero if empty\n", ret);
		if (ret != 0) {
			return -EINVAL;
		}
	}

	if (x == uart_dev_1) {
		ret = k_fifo_is_empty(&fifo_uart1_rx_data);
		if (ret != 0) {
			return -EINVAL;
		}
	}
	return 0;
}

const struct device *init_uart_dev(const char *uart_num)
{
	if (uart_num == NULL) {
		return NULL;
	}
	int comp = strcmp(uart_num, "UART_0");
	if (comp == 0) {
		uart_dev_0 = device_get_binding(uart_num);
		if (!uart_dev_0) {
			printk("[UART] Failed to get device!\n");
			return NULL;
		}
		// and initialise
		uart_irq_rx_disable(uart_dev_0);
		uart_irq_tx_disable(uart_dev_0);
		uart_irq_callback_set(uart_dev_0, uart_cb);
		return uart_dev_0;
	}
	comp = strcmp(uart_num, "UART_1");
	if (comp == 0) {
		uart_dev_1 = device_get_binding(uart_num);
		if (!uart_dev_1) {
			printk("[UART] Failed to get device!\n");
			return NULL;
		}
		// and initialise
		uart_irq_rx_disable(uart_dev_1);
		uart_irq_tx_disable(uart_dev_1);
		uart_irq_callback_set(uart_dev_1, uart_cb);
		return uart_dev_1;
	}
	return NULL;
}

const struct device *init_uart_dev_and_ISR(const char *uart_num)
{
	if (uart_num == NULL) {
		return NULL;
	}

	int comp = strcmp(uart_num, "UART_0");
	if (comp == 0) {
		uart_dev_0 = device_get_binding(uart_num);
		if (!uart_dev_0) {
			printk("[UART] Failed to get device!\n");
			return NULL;
		}
		// and initialise
		uart_irq_rx_disable(uart_dev_0);
		uart_irq_tx_disable(uart_dev_0);
		uart_irq_callback_set(uart_dev_0, uart_cb);
		uart_irq_rx_enable(uart_dev_0);
		return uart_dev_0;
	}
	comp = strcmp(uart_num, "UART_1");
	if (comp == 0) {
		uart_dev_1 = device_get_binding(uart_num);
		if (!uart_dev_1) {
			printk("[UART] Failed to get device!\n");
			return NULL;
		}
		// and initialise
		uart_irq_rx_disable(uart_dev_1);
		uart_irq_tx_disable(uart_dev_1);
		uart_irq_callback_set(uart_dev_1, uart_cb);
		uart_irq_rx_enable(uart_dev_1);
		return uart_dev_1;
	}
	return NULL;
}