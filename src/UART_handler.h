#ifndef UART_HANDLER_LIBRARY_H_
#define UART_HANDLER_LIBRARY_H_


#define UART_RX_LINE_BUF_SIZE 128
#define UART_TX_LINE_BUF_SIZE 128

//ENDMARKERS the UART RX ISR is listening on. As soon as one of the ENDMARKERS occurs in
//the received bytes the UART RX ISR feeds the received message into a FIFO.
//IMPORTANT: If no ENDMARKER is found in the RX message, the message wont be forwarded to FIFO and is not available to the upper application!
#define COM_RX_ENDMARKER_1_UART0 0xC0 
#define COM_RX_ENDMARKER_2_UART0 0xC0

#define COM_RX_ENDMARKER_1_UART1 '\n' 
#define COM_RX_ENDMARKER_2_UART1 '\r'

//FIFO struct which helps for UART message transmission.
typedef struct {
	void  *fifo_reserved;
	uint8_t    data[UART_TX_LINE_BUF_SIZE];
	uint16_t   len;
} uart_data_t;

//FIFO struct which stores received UART messages
typedef struct {
	void *fifo_reserved;
	uint8_t *received_line;
	uint8_t len;
} uart_rx_fifo_data_t;

//Struct which helps storing UART_RX data in a FIFO queue.
typedef struct {
	uint8_t uart_rx_buff_len;
	uint8_t uart_rx_buf[UART_RX_LINE_BUF_SIZE];
	bool dirty;
} uart_rx_helper;

/**
 * @brief Checks if data is available from selected UART port
 *
 * @param[in] uart_dev   UART device which should be checked on available data
 *
 * @retval Zero when data is available, non-zero if no data is available
 */
int check_rx_uart_available(const struct device* uart_dev);


/**
 * @brief Sends message via chosen UART port.
 * Single or multiple bytes possible. Maximum message length depends on configured
 * CONFIG_UART_0_NRF_TX_BUFFER_SIZE and CONFIG_UART_1_NRF_TX_BUFFER_SIZE in the prj.conf file (when not specified the standard size of 32 bytes is applied). 
 *
 *
 * @param[in] tx_buf	Byte string of message to be send.     
 * @param[in] msg_len   Length of message to be send.
 * @param[in] uart_dev   UART device which shall send the message.
 *
 * @retval Zero on success, negative otherwise.
 */
int tx_message_uart(char* tx_buf, size_t msg_len, const struct device* uart_dev);


/**
 * @brief Reads message on chosen UART port. 
 *
 *
 * @param[in] rx_buf	Byte string of message to be send.     
 * @param[in] timeout	Maximum waiting period to obtain data.
 * @param[in] uart_dev   UART from which a potential message shall be received.
 *
 * @retval Zero on success, negative otherwise.
 */
int rx_message_uart(char* rx_buf, k_timeout_t timeout, const struct device* uart_dev);

/**
 * @brief Initialization of chosen UART device. The RX ISR UART receiption will not be initialized
 * using this function call and has to be activated manually.
 * This function supports to initialize the UART devices "UART_0" and "UART_1"
 * 
 * @param[in] uart_num	Name of UART device which shall be initialized.
 *
 * @retval Pointer to initialized UART device, NULL otherwise.
 */
const struct device* init_uart_dev(const char* uart_num);

/**
 * @brief Initialization of chosen UART device and activation of UART RX ISR receiption.
 * This function supports to initialize the UART devices "UART_0" and "UART_1"
 *
 * @param[in] uart_num	Name of UART device which shall be initialized.
 *
 * @retval Pointer to initialized UART device, NULL otherwise.
 */
const struct device* init_uart_dev_and_ISR(const char* uart_num);

#endif /* UART_HANDLER_LIBRARY_H_ */