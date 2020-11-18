#ifndef UART_H
#define UART_H

#include "nrf_gpio.h"

#define BLE_COLLECTOR_RX           6
#define BLE_COLLECTOR_TX           5
#define BLE_COLLECTOR_RTS          UART_PIN_DISCONNECTED
#define BLE_COLLECTOR_CTS          UART_PIN_DISCONNECTED
#define BLE_COLLECTOR_HWFC         false

#define MAX_TEST_DATA_BYTES     (15U)           /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE        16              /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        16              /**< UART RX buffer size. */

void init_uart(void);
void uart_put(char ch);
void uart_get(uint8_t *ch);

#endif /* UART_H */
