/**
* @file hal_ads1018.h
* @author Danny Deng
* @Company Biologue Co.Ltd
* @Date 2020/xx/xx
* 
* @brief ADS1018 ADC hardware abstract file
*/

#ifndef HAL_ADS1018_H
#define HAL_ADS1018_H

#include <stdint.h>

#define ADS1018_BUF_SIZE             (256U)

#define ADS1018_CONFIG_SS_POS        (15U)
#define ADS1018_CONFIG_SS(x)         (x<<ADS1018_CONFIG_SS_POS)
#define ADS1018_CONFIG_SS_MSK        (1<<ADS1018_CONFIG_SS_POS)
#define ADS1018_CONFIG_MUX_POS       (12U)
#define ADS1018_CONFIG_MUX(x)        (x<<ADS1018_CONFIG_MUX_POS)
#define ADS1018_CONFIG_MUX_MSK       (7<<ADS1018_CONFIG_MUX_POS)
#define ADS1018_CONFIG_PGA_POS       (9U)
#define ADS1018_CONFIG_PGA(x)        (x<<ADS1018_CONFIG_PGA_POS)
#define ADS1018_CONFIG_PGA_MSK       (7<<ADS1018_CONFIG_PGA_POS)
#define ADS1018_CONFIG_MODE_POS      (8U)
#define ADS1018_CONFIG_MODE(x)       (x<<ADS1018_CONFIG_MODE_POS)
#define ADS1018_CONFIG_MODE_MSK      (1<<ADS1018_CONFIG_MODE_POS)
#define ADS1018_CONFIG_DR_POS        (5U)
#define ADS1018_CONFIG_DR(x)         (x<<ADS1018_CONFIG_DR_POS)
#define ADS1018_CONFIG_DR_MSK        (7<<ADS1018_CONFIG_DR_POS)
#define ADS1018_CONFIG_TS_POS        (4U)
#define ADS1018_CONFIG_TS(x)         (x<<ADS1018_CONFIG_TS_POS)
#define ADS1018_CONFIG_TS_MSK        (1<<ADS1018_CONFIG_TS_POS)
#define ADS1018_CONFIG_PULL_POS      (3U)
#define ADS1018_CONFIG_PULL(x)       (x<<ADS1018_CONFIG_PULL_POS)
#define ADS1018_CONFIG_PULL_MSK      (1<<ADS1018_CONFIG_PULL_POS)
#define ADS1018_CONFIG_NOP_POS       (1U)
#define ADS1018_CONFIG_NOP(x)        (x<<ADS1018_CONFIG_NOP_POS)
#define ADS1018_CONFIG_NOP_MSK       (3<<ADS1018_CONFIG_NOP_POS)

#define ADS1018_MUX_P0_N1            (0<<ADS1018_CONFIG_MUX_POS)
#define ADS1018_MUX_P0_N3            (1<<ADS1018_CONFIG_MUX_POS)
#define ADS1018_MUX_P1_N3            (2<<ADS1018_CONFIG_MUX_POS)
#define ADS1018_MUX_P2_N3            (3<<ADS1018_CONFIG_MUX_POS)
#define ADS1018_MUX_P0_GND           (4<<ADS1018_CONFIG_MUX_POS)
#define ADS1018_MUX_P1_GND           (5<<ADS1018_CONFIG_MUX_POS)
#define ADS1018_MUX_P2_GND           (6<<ADS1018_CONFIG_MUX_POS)
#define ADS1018_MUX_P3_GND           (7<<ADS1018_CONFIG_MUX_POS)

#define ADS1018_PGA_6144             (0<<ADS1018_CONFIG_PGA_POS)
#define ADS1018_PGA_4096             (1<<ADS1018_CONFIG_PGA_POS)
#define ADS1018_PGA_2048             (2<<ADS1018_CONFIG_PGA_POS)
#define ADS1018_PGA_1024             (3<<ADS1018_CONFIG_PGA_POS)
#define ADS1018_PGA_0512             (4<<ADS1018_CONFIG_PGA_POS)
#define ADS1018_PGA_0256             (5<<ADS1018_CONFIG_PGA_POS)

#define ADS1018_MODE_CONTI           (0<<ADS1018_CONFIG_MODE_POS)
#define ADS1018_MODE_SINGLE          (1<<ADS1018_CONFIG_MODE_POS)

#define ADS1018_DR_128               (0<<ADS1018_CONFIG_DR_POS)
#define ADS1018_DR_250               (1<<ADS1018_CONFIG_DR_POS)
#define ADS1018_DR_490               (2<<ADS1018_CONFIG_DR_POS)
#define ADS1018_DR_920               (3<<ADS1018_CONFIG_DR_POS)
#define ADS1018_DR_1600              (4<<ADS1018_CONFIG_DR_POS)
#define ADS1018_DR_2400              (5<<ADS1018_CONFIG_DR_POS)
#define ADS1018_DR_3300              (6<<ADS1018_CONFIG_DR_POS)
#define ADS1018_DR_NOT_USE           (7<<ADS1018_CONFIG_DR_POS)

#define ADS1018_TS_ADC               (0<<ADS1018_CONFIG_TS_POS)
#define ADS1018_TS_TEMP              (1<<ADS1018_CONFIG_TS_POS)

#define ADS1018_PULL_DISEN           (0<<ADS1018_CONFIG_PULL_POS)
#define ADS1018_PULL_EN              (1<<ADS1018_CONFIG_PULL_POS)

#define ADS1018_NOP_VALID            (1<<ADS1018_CONFIG_NOP_POS)
#define ADS1018_NOP_INVALID          (2<<ADS1018_CONFIG_NOP_POS)


/**
* @Default ADC 
*/
#define ADS1018_DEFAULT_CONFIG       (ADS1018_MUX_P3_GND | ADS1018_PGA_2048 | ADS1018_MODE_SINGLE \
                                     | ADS1018_DR_1600 | ADS1018_TS_ADC | ADS1018_PULL_EN         \
                                     | ADS1018_NOP_VALID)

typedef struct _ads1018_drv_t {
  void (*init)(void);
  uint8_t (*spi_swap)(uint8_t trm_data);
  void (*spi_multi_swap)(uint8_t *trm_datas, uint8_t *rec_datas, uint16_t data_sz);
  void (*cs_low)(void);
  void (*cs_high)(void);
  uint8_t (*get_drdy)(void);
}ads1018_drv_t;

typedef struct ads1018 {
  ads1018_drv_t *drv;
  uint16_t config;
  uint8_t config_index;
  uint16_t count;
  uint8_t spi_flag;
  uint8_t ok_flag;
  uint16_t temp;
  void (*init)(void);
  void (*set_mux)(uint16_t mux);
  void (*set_pga)(uint16_t pga);
  void (*set_mode)(uint16_t mode);
  void (*set_dr)(uint16_t dr);
  void (*set_ts)(uint16_t ts);
  void (*set_pull)(uint16_t pull_en);
  void (*set_nop)(uint16_t nop);
  void (*start_conversion)(void);
  void (*adc_process)(void);
  void (*adc_cb_handler)(void);
  int index;
  uint16_t data[ADS1018_BUF_SIZE];
}ads1018_t;

#endif /* HAL_ADS1018_H */
