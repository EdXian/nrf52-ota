#include "uart.h"
#include "app_error.h"
#include "app_uart.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void uart_error_handle(app_uart_evt_t *p_event) {
  if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) {
    //APP_ERROR_HANDLER(p_event->data.error_communication);
    __asm("nop");
  } 
  else if (p_event->evt_type == APP_UART_FIFO_ERROR) {
    //APP_ERROR_HANDLER(p_event->data.error_code);
    __asm("nop");
  }
}

void init_uart(void) {
  uint32_t err_code;
  const app_uart_comm_params_t comm_params =
      {
          BLE_COLLECTOR_RX,
          BLE_COLLECTOR_TX,
          BLE_COLLECTOR_RTS,
          BLE_COLLECTOR_CTS,
          BLE_COLLECTOR_HWFC,
          false,
          NRF_UART_BAUDRATE_115200};

  APP_UART_FIFO_INIT(&comm_params,
      UART_RX_BUF_SIZE,
      UART_TX_BUF_SIZE,
      uart_error_handle,
      APP_IRQ_PRIORITY_LOWEST,
      err_code);
  APP_ERROR_CHECK(err_code);
}

void uart_put(char ch) {
  app_uart_put(ch);
}

void uart_get(uint8_t *ch) {
  app_uart_get(ch);
}

