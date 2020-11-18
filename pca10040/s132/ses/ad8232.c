#include "ad8232.h"
#include "nrf_gpio.h"

#define ad8232_operation()      nrf_gpio_pin_set(AD8232_SDH_PIN_NUM)
#define ad8232_sleep()          nrf_gpio_pin_clear(AD8232_SDH_PIN_NUM)
#define ad8232_get_LOD_minus()  nrf_gpio_pin_read(AD8232_LOD_MINUS_PIN_NUM)
#define ad8232_get_LOD_plus()   nrf_gpio_pin_read(AD8232_LOD_PLUS_PIN_NUM)

static void init_ad8232(void);
static void ad8232_lod_update(void);

ad8232_t ad8232 = {                 \
  .lod_status = 0,                    \
  .init = init_ad8232,               \
  .update_lod = ad8232_lod_update,   \
};

static void init_ad8232(void) {
  nrf_gpio_cfg_output(AD8232_SDH_PIN_NUM);
  nrf_gpio_cfg_input(AD8232_LOD_MINUS_PIN_NUM, GPIO_PIN_CNF_PULL_Disabled);
  nrf_gpio_cfg_input(AD8232_LOD_PLUS_PIN_NUM, GPIO_PIN_CNF_PULL_Disabled);
  ad8232_operation();
}

static void ad8232_lod_update(void) {
  ad8232.lod_status = (ad8232_get_LOD_plus()<<LOD_PLUS_POS) | (ad8232_get_LOD_minus()<<LOD_MINUS_POS);
}

