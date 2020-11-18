/**
* @file hal_ads1018.c
* @author Danny Deng
* @Company Biologue Co.Ltd
* @Date 2020/xx/xx
* 
* @brief ADS1018 ADC SPI interface hardware link file.
*
* @pin Definition
*  DIN : 30 
*  DO  : 31
*  SCLK: 29
*  /CS : 28
*/

#include "hal_ads1018.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_spi.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "ad8232.h"
#include <stdint.h>

/* Pin definition */
#define ADC_CS_PIN_NUM       28
#define ADC_MOSI_PIN_NUM     30
#define ADC_MISO_PIN_NUM     31
#define ADC_SCK_PIN_NUM      29

#define TIMEOUTCNT           10000
#define ADC_TEST_PIN         (11)

#define SPI_INSTANCE  2 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

static void spi_init(void);
static uint8_t ads1018_spi_swap(uint8_t trm_data);
static void ads1018_spi_multi_swap(uint8_t *trm_datas, uint8_t *rec_datas, uint16_t data_sz);
static void ads1018_cs_low(void);
static void ads1018_cs_high(void);
static uint8_t ads1018_get_drdy(void);

static volatile bool spi_xfer_done = true;

ads1018_drv_t ads1018_drv = {                \
  .init = spi_init,                          \
  .spi_swap = ads1018_spi_swap,              \
  .spi_multi_swap = ads1018_spi_multi_swap,  \
  .cs_low = ads1018_cs_low,                  \
  .cs_high = ads1018_cs_high,                \
  .get_drdy = ads1018_get_drdy               \
};
extern uint8_t send_buf[4];
extern uint8_t rec_buf[4];
extern ads1018_t ads1018;
static void nrf_adc_spi_evt_handler(nrf_drv_spi_evt_t const * p_event, void *p_context) {
  uint16_t check_config;
  switch(p_event->type) {
    case NRF_DRV_SPI_EVENT_DONE:
#ifdef ADC_TEST_PIN
        nrf_gpio_pin_set(ADC_TEST_PIN);
#endif
        spi_xfer_done = true;
        ads1018.adc_cb_handler();

      break;
    default:
      NRF_LOG_INFO("SPI EVENT: %d", p_event->type);
     break;
  }

}

static void spi_init(void) {
  ret_code_t err_code;
  // Setting GPIO
  //nrf_gpio_cfg(ADC_CS_PIN_NUM, GPIO_PIN_CNF_DIR_Output, GPIO_PIN_CNF_INPUT_Disconnect, GPIO_PIN_CNF_PULL_Disabled, 
  //             NRF_GPIO_PIN_S0S1, GPIO_PIN_CNF_SENSE_Disabled);

  // Setting SPI
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.frequency = SPI_FREQUENCY_FREQUENCY_M2;
  spi_config.ss_pin   = ADC_CS_PIN_NUM;
  spi_config.miso_pin = ADC_MISO_PIN_NUM;
  spi_config.mosi_pin = ADC_MOSI_PIN_NUM;
  spi_config.sck_pin  = ADC_SCK_PIN_NUM;
  err_code = nrf_drv_spi_init(&spi, &spi_config, nrf_adc_spi_evt_handler, NULL);
  APP_ERROR_CHECK(err_code);

#ifdef ADC_TEST_PIN
  nrf_gpio_cfg_output(ADC_TEST_PIN);
  nrf_gpio_pin_clear(ADC_TEST_PIN);
#endif
}

static uint8_t ads1018_spi_swap(uint8_t trm_data) {
  ret_code_t err_code;
  spi_xfer_done = false;
  uint8_t rx_temp;
  err_code = nrf_drv_spi_transfer(&spi, &trm_data, 1, &rx_temp, 1);
  APP_ERROR_CHECK(err_code);

  int timeoutcnt = TIMEOUTCNT;
  while ((!spi_xfer_done) && ((timeoutcnt--)>=0)) {
    __asm("nop");
    timeoutcnt--;
    if(timeoutcnt < 0) {
      // timeout
      //NRF_LOG_INFO("SPI SWAP timeout");
      break;
    }
  }
  return rx_temp;
}

static void ads1018_spi_multi_swap(uint8_t *trm_datas, uint8_t *rec_datas, uint16_t data_sz) {
#ifdef ADC_TEST_PIN
  nrf_gpio_pin_clear(ADC_TEST_PIN);
#endif
  ret_code_t err_code;
  int timeoutcnt = TIMEOUTCNT;
  while ((!spi_xfer_done) && ((timeoutcnt--)>=0)) {
    __asm("nop");
    timeoutcnt--;
    if(timeoutcnt < 0) {
      // timeout
      NRF_LOG_INFO("SPI SWAP timeout");
      break;
    }
  }
  spi_xfer_done = false;
  err_code = nrf_drv_spi_transfer(&spi, trm_datas, data_sz, rec_datas, data_sz);
}

static void ads1018_cs_low(void) {
  nrf_gpio_pin_clear(ADC_CS_PIN_NUM);
}

static void ads1018_cs_high(void) {
  nrf_gpio_pin_set(ADC_CS_PIN_NUM);
}

static uint8_t ads1018_get_drdy(void) {
  // TODO: Need check
  nrf_gpio_pin_sense_t pin_sense;
  pin_sense = nrf_gpio_pin_sense_get(ADC_MISO_PIN_NUM);
  if(pin_sense == NRF_GPIO_PIN_NOSENSE ) {
    return 0xFF;
  }
  else if(pin_sense == NRF_GPIO_PIN_SENSE_HIGH) {
    return 0;
  }
   else if(pin_sense == NRF_GPIO_PIN_SENSE_LOW) {
    return 1;
  }
}
