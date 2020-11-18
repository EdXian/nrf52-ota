#include "rtc.h"
#include "nrfx.h"
#include "nrfx_rtc.h"
#include "nrf_gpio.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "boards.h"
#include "app_error.h"
#include "ble_biologue_service.h"
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

/* Setup the default time */
struct tm rtc_tm = {
  .tm_year = RTC_DEFAULT_YEAR,  \
  .tm_mon = RTC_DEFAULT_MONTH,  \
  .tm_mday = RTC_DEFAULT_DAY,   \
  .tm_hour = RTC_DEFAULT_HOUR,  \
  .tm_min = RTC_DEFAULT_MIN,    \
  .tm_sec = RTC_DEFAULT_S,      \
};

extern char rtc_buf[VALUE_LEN_TIME];
volatile uint32_t global_tick = 0;

#define COMPARE_COUNTERTIME  (3UL) 
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);
volatile uint32_t rtc_tick = 0;

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type) {
  if(int_type == NRF_DRV_RTC_INT_TICK) {
    rtc_tick++;
    if((rtc_tick % RTC_TICK_MS_TO_S) == 0) {
#ifdef RTC_TEST_PIN
      nrf_gpio_pin_set(RTC_TEST_PIN);
#endif
      rtc_handle();
#ifdef RTC_TEST_PIN
      nrf_gpio_pin_clear(RTC_TEST_PIN);
#endif
#if DEBUG_SHOW_TIME
      show_rtc();
#endif
    }
  }
  else if(int_type == NRF_DRV_RTC_INT_OVERFLOW) {
    
  }
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void) {
  memset(rtc_buf, 0, sizeof(rtc_buf));
  //sprintf("%2d/%2d;%2d:%2d:%3d", rtc_tm);
  nrfx_err_t err_code;
  nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
  config.prescaler = 4095;
  //Initialize RTC instance
  err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);

  //Enable tick event & interrupt
  nrf_drv_rtc_tick_enable(&rtc,true);

  //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
  err_code = nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8,true);
  APP_ERROR_CHECK(err_code);

  //Power on RTC instance
  nrf_drv_rtc_enable(&rtc);
}

void init_rtc(void) {
   rtc_config();
#ifdef RTC_TEST_PIN
   nrf_gpio_cfg_output(RTC_TEST_PIN);
#endif
}

#if DEBUG_SHOW_TIME

char temp_show[20];
void show_rtc(void) {  
  memset(temp_show, 0, sizeof(temp_show));
  strftime(temp_show, sizeof(temp_show), "%x;%X", &rtc_tm);
  NRF_LOG_INFO("now: %s", temp_show);
}
#endif

void rtc_client_handle(uint8_t *packet) {
  uint16_t checksum = 0;
  time_t setting_time;
  struct tm tm_temp;
  struct tm *tm_p = &tm_temp;
  for(int i=0;i<VALUE_LEN_TIME-2;i++) {
    checksum += packet[i];
    NRF_LOG_INFO("[rec]: %x", packet[i]);
  }
  if((packet[VALUE_LEN_TIME-2] == ((checksum&0xFF00) >> 8)) &&
     ((packet[VALUE_LEN_TIME-1] == ((checksum&0x00FF))))) {
    // Overwrite the RTC data
    setting_time = (packet[0]<<24)|(packet[1]<<16)|(packet[2]<<8)|(packet[3]<<0);
    tm_p = gmtime(&setting_time);
    memcpy(&rtc_tm, tm_p, sizeof(struct tm));
    global_tick = 0;
 
    NRF_LOG_INFO("Success, overwrite the RTC !");
  }
  else {
        NRF_LOG_INFO("Fail to overwrite the RTC ! checksum error");
    }
}

// Handle every one second, consume about 71.32 us  
void rtc_handle(void) {
  uint16_t checksum = 0;
  rtc_tm.tm_sec++;
  time_t time_now = mktime(&rtc_tm);
  memcpy(rtc_buf, (uint8_t *)&time_now, 4);
  for(int i=0;i<VALUE_LEN_TIME-2;i++) {
    checksum += rtc_buf[i];
  }
  rtc_buf[VALUE_LEN_TIME-2] = (checksum&0xFF00) >> 8;
  rtc_buf[VALUE_LEN_TIME-1] = (checksum&0x00FF);
}
