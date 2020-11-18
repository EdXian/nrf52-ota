/**
* @file hal_ads1018.c
* @author Danny Deng
* @Company Biologue Co.Ltd
* @Date 2020/xx/xx
* 
* @brief ADS1018 ADC hardware abstract file
*/

#include "hal_ads1018.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

extern ads1018_drv_t ads1018_drv;
uint8_t send_buf[4] = {0, 0, 0, 0};
uint8_t rec_buf[4] = {0, 0, 0, 0};

static void ads1018_init(void);
static void ads1018_set_mux(uint16_t mux);
static void ads1018_set_pga(uint16_t pga);
static void ads1018_set_mode(uint16_t mode);
static void ads1018_set_dr(uint16_t dr);
static void ads1018_set_ts(uint16_t ts);
static void ads1018_set_pull(uint16_t pull_en);
static void ads1018_set_nop(uint16_t nop);
static void ads1018_start_conversion(void);
static void ads1018_temp_conversion(void);
static void ads1018_adc_process(void);
static void ads1018_adc_cb_handler(void);


ads1018_t ads1018 = {
  .drv = &ads1018_drv,                            \
  .config = ADS1018_DEFAULT_CONFIG,               \
  .config_index = 0,                              \
  .temp = 0,                                      \
  .count = 0,                                     \
  .spi_flag = true,                               \
  .ok_flag = false,                               \
  .init = ads1018_init,                           \
  .set_mux = ads1018_set_mux,                     \
  .set_pga = ads1018_set_pga,                     \
  .set_mode = ads1018_set_mode,                   \
  .set_dr = ads1018_set_dr,                       \
  .set_ts = ads1018_set_ts,                       \
  .set_pull = ads1018_set_pull,                   \
  .set_nop = ads1018_set_nop,                     \
  .start_conversion = ads1018_start_conversion,   \
  .adc_process = ads1018_adc_process,             \
  .adc_cb_handler = ads1018_adc_cb_handler,       \
  .index = 0,                                     \
  .data = {0},                                    \
};


static void ads1018_init(void) {
  // initialize the SPI hardware
  ads1018.drv->init();
  ads1018.index = 0;
  memset(ads1018.data, 0, sizeof(ads1018.data));
  ads1018_start_conversion();
}

static void ads1018_set_mux(uint16_t mux) {
  mux &= ADS1018_CONFIG_MUX_MSK;
  ads1018.config &= ~ADS1018_CONFIG_MUX_MSK;
  ads1018.config |= mux;
}

static void ads1018_set_pga(uint16_t pga) {
  pga &= ADS1018_CONFIG_PGA_MSK;
  ads1018.config &= ~ADS1018_CONFIG_PGA_MSK;
  ads1018.config |= pga;
}

static void ads1018_set_mode(uint16_t mode) {
  mode &= ADS1018_CONFIG_MODE_MSK;
  ads1018.config &= ~ADS1018_CONFIG_MODE_MSK;
  ads1018.config |= mode;
}

static void ads1018_set_dr(uint16_t dr) {
  dr &= ADS1018_CONFIG_DR_MSK;
  ads1018.config &= ~ADS1018_CONFIG_DR_MSK;
  ads1018.config |= dr;
}

static void ads1018_set_ts(uint16_t ts) {
  ts &= ADS1018_CONFIG_TS_MSK;
  ads1018.config &= ~ADS1018_CONFIG_TS_MSK;
  ads1018.config |= ts;
}

static void ads1018_set_pull(uint16_t pull_en) {
  pull_en &= ADS1018_CONFIG_PULL_MSK;
  ads1018.config &= ~ADS1018_CONFIG_PULL_MSK;
  ads1018.config |= pull_en;
}

static void ads1018_set_nop(uint16_t nop) {
  nop &= ADS1018_CONFIG_NOP_MSK;
  ads1018.config &= ~ADS1018_CONFIG_NOP_MSK;
  ads1018.config |= nop;
}

static void ads1018_start_conversion(void) {
  ads1018.config |= (ADS1018_CONFIG_SS_MSK);
  
  send_buf[0] = (ads1018.config&0xFF00)>>8;
  send_buf[1] = (ads1018.config&0x00FF);
  memset(rec_buf, 0, sizeof(rec_buf));
  ads1018.drv->spi_multi_swap(send_buf, rec_buf, 2);
}

static void ads1018_temp_conversion(void) {
  ads1018.set_ts(ADS1018_TS_TEMP);
  ads1018.config |= (ADS1018_CONFIG_SS_MSK);
  
  send_buf[0] = (ads1018.config&0xFF00)>>8;
  send_buf[1] = (ads1018.config&0x00FF);
  memset(rec_buf, 0, sizeof(rec_buf));
  ads1018.drv->spi_multi_swap(send_buf, rec_buf, 2);

  
  ads1018.set_ts(ADS1018_TS_ADC);
  send_buf[0] = (ads1018.config&0xFF00)>>8;
  send_buf[1] = (ads1018.config&0x00FF);
  ads1018.drv->spi_multi_swap(send_buf, rec_buf, 2);

  ads1018.temp = (rec_buf[0] << 8) | (rec_buf[1]);

}

static void ads1018_adc_process(void){
  // Compare whether next conversion need to get temperature
  ads1018.start_conversion();
}

/* Call it function when SPI transfer complete */
static void ads1018_adc_cb_handler(void) {
  uint16_t recData = ((rec_buf[0] << 8) | (rec_buf[1]))>> 4;
  
  if(ads1018.count < 64) {
    ads1018.data[ads1018.index++] = recData;
    if(ads1018.index >= ADS1018_BUF_SIZE) {
      ads1018.index = 0;
    }
  }
  else if(ads1018.count == 64) {
      // get temperature
      ads1018.adc_process();
  }
  else {
    ads1018.temp = recData;
    ads1018.count = 0;
    if(ads1018.ok_flag == false) {
      ads1018.ok_flag = true;
    }
    else {
      while(true) __asm("nop");
    }
  }

  ads1018.count++;
  if(ads1018.count == 63) {
    ads1018.set_ts(ADS1018_TS_TEMP);
  }
  else {
    ads1018.set_ts(ADS1018_TS_ADC);
  }
  ads1018.config &= ~(ADS1018_CONFIG_SS_MSK);
  rec_buf[0] = 0;
  rec_buf[1] = 0;
}

