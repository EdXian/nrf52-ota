/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "fds.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "sensorsim.h"
#include "ble_biologue_service.h"
#include "nrf_timer.h"
#include "nrf_drv_timer.h"
#include "nrfx_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_biologue_ota.h"
#include "ble_nus.h"
#include "ble_biologue_service.h"  // Include Biologue service file
#include "hal_ads1018.h"           // Include ADC file
#include "ad8232.h"                // Include ECG AFE file
#include "rtc.h"                   // Include RTC file
#include "sdc.h"                   // Include SDC file
#include "uart.h"                  // Include UART file

#define DEVICE_NAME "SMART_SEAT1.3"                  /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME "NordicSemiconductor" /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL 300                    /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION 18000  /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG 1  /**< A tag identifying the SoftDevice BLE configuration. */

// MTU 137 bytes, payload size is 137-3-4-2 = 128 => 128*(1/256)=0.5
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(50, UNIT_1_25_MS)  /**< Minimum acceptable connection interval (50 ms). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Maximum acceptable connection interval (100 ms). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 0                               /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/* Define the test parameters */
#define TIMER_TEST_PIN        (12)
#define TIMER_TEST_PIN2       (8)
#define SEND_TEST_DATA        (false)

NRF_BLE_GATT_DEF(m_gatt);              /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);    /**< Advertising module instance. */
BLE_BIOLOGUE_DEF(m_biologue);          /**< Biologue BLE sensor collector service. */
APP_TIMER_DEF(adc_timer_id);           /**< ADC timer define, set as 256 Hz */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
                                                        /**< Context for the Queued Write module.*/



const nrf_drv_timer_t TIMER_ADC = NRF_DRV_TIMER_INSTANCE(1);
extern volatile uint32_t global_tick;
extern ads1018_t ads1018;
extern ad8232_t ad8232;
extern time_t time_now;
extern volatile uint8_t notify_data[VALUE_LEN_NOTIFY];
extern uint8_t ecg_buf[VALUE_LEN_ECG_RAW];
extern uint8_t temp_buf[VALUE_LEN_TEMP];
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
extern volatile uint8_t flag_notify;
extern volatile uint8_t flag_notify_ok;

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifiers. */
    {
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};

static void advertising_start(bool erase_bonds);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt) {
  pm_handler_on_pm_evt(p_evt);
  pm_handler_flash_clean(p_evt);

  switch (p_evt->evt_id) {
  case PM_EVT_PEERS_DELETE_SUCCEEDED:
    advertising_start(false);
    break;

  default:
    break;
  }
}

#if SEND_TEST_DATA
volatile uint8_t ccc = 0;
#endif
volatile uint16_t st = 0;
// timer 256 Hz
void adc_timer_handler(nrf_timer_event_t event_type, void * p_contex) {
  uint32_t err_code;
#ifdef TIMER_TEST_PIN
  nrf_gpio_pin_set(TIMER_TEST_PIN);
  nrf_gpio_pin_set(TIMER_TEST_PIN2);
#endif
  ads1018.adc_process();
  int start_index;
  int temp_index = 0;
  uint8_t *t_p;
  uint16_t checksum = 0;
  
  // conversion complete, prepare data to send
  if(ads1018.ok_flag) {
    ad8232.update_lod();
    start_index = st;
    t_p = (uint8_t *)&global_tick;
#ifdef TIMER_TEST_PIN
    nrf_gpio_pin_clear(TIMER_TEST_PIN2);
#endif
    for(int i=0;i<4;i++) {
        checksum += t_p[3-i];
        notify_data[i] = t_p[3-i];
    }
    memcpy((uint8_t *)&notify_data[4], (uint8_t *)&ads1018.data[start_index], 128);
    for(int i=0;i<128;i++) {
      checksum += notify_data[i+4];
    }
#if SEND_TEST_DATA
    memset((char *)&notify_data[4], ccc++, VALUE_LEN_ECG_RAW-8);
#endif /* SEND_TEST_DATA */

    ads1018.temp |= ad8232.lod_status;
    notify_data[VALUE_LEN_ECG_RAW-4] = (ads1018.temp&0xFF00) >> 8;
    notify_data[VALUE_LEN_ECG_RAW-3] = (ads1018.temp&0x00FF);
    notify_data[VALUE_LEN_ECG_RAW-2] = (checksum&0xFF00) >> 8;
    notify_data[VALUE_LEN_ECG_RAW-1] = (checksum&0x00FF);
    if(flag_notify) {   // send command to replace this flag
      err_code = ecg_data_notify(&m_biologue);
      if(err_code != NRF_SUCCESS) {
          NRF_LOG_INFO("Notify error %x", err_code);
      }
    }
    st += 64;
    if(st >= ADS1018_BUF_SIZE) st = 0;
    ads1018.ok_flag = false;
  }
  global_tick++;
#ifdef TIMER_TEST_PIN
  nrf_gpio_pin_clear(TIMER_TEST_PIN);
#endif
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {
  uint32_t err_code = NRF_SUCCESS;
  nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
  err_code = nrf_drv_timer_init(&TIMER_ADC, &timer_cfg, adc_timer_handler);
  APP_ERROR_CHECK(err_code);

  /* Sampling period = 1/256 */
  nrf_drv_timer_extended_compare(
       &TIMER_ADC, NRF_TIMER_CC_CHANNEL0, 62500, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

#ifdef TIMER_TEST_PIN
  nrf_gpio_cfg_output(TIMER_TEST_PIN);
  nrf_gpio_cfg_output(TIMER_TEST_PIN2);
#endif

}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
  ret_code_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)DEVICE_NAME,
      strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}



void ota_resposnse_ack(ota_state state, ota_ack_type ack_type){
	uint8_t data[2] = {state, ack_type};
	uint16_t size = sizeof(data);
	ble_nus_data_send(&m_nus, data, &size, NULL);
}

void ota_resposnse_ver(uint32_t ver){
  uint8_t data[4] = {check_version_state , (ver>>0) , (ver>>8) , (ver>>16)};
  uint16_t size = sizeof(data);
  ble_nus_data_send(&m_nus, data,&size,NULL);
}

extern uint8_t ota_buffer[OTA_BUFFER_SIZE];
ota_control_t m_ota_ctrl;
ota_start_cmd_t start_cmd;
ota_write_cmd_t write_cmd;
ota_verify_cmd_t verify_cmd;
ota_clearrom_cmd_t clearrom_cmd;
ota_key_cmd_t key_cmd;


int8_t checksum_error(uint8_t* data, uint16_t len){
    int8_t sum = 0;
    for(uint16_t i=0;i<len;i++){
      sum+=data[i]; 
    }
  return sum;
}



uint8_t data_compare(uint8_t* data, uint16_t len){
  for(unsigned int i=0;i<len;i++){
    if(data[i] !=  ota_buffer[i]) return 1;
  }
  return 0;
}


uint8_t err_count = 0;
void ota_update_handle(uint8_t* flag){
    uint8_t ret = 0 ;
	switch (*flag){
              case idle_flag:
              *flag = idle_flag;
                      break;
              case flash_flag:
                      
                      ret = ota_flash_write(start_cmd.addr, ota_buffer, start_cmd.recv_len);
                      m_ota_ctrl.checksum = checksum_error((uint8_t*)start_cmd.addr, start_cmd.recv_len);
                      
                      if( (ret==0) ){
                         ota_resposnse_ack(flash_state,ota_write_pass);
                         printf("write pass\r\n");
                      }else{

                        ota_resposnse_ack(flash_state,ota_write_fail);
                        printf("write fail\r\n");

                      }
                      
                      *flag = idle_flag;
                      break;

              case clear_flag:
                      
                      ret = ota_flash_erase(clearrom_cmd.start_addr, clearrom_cmd.end_addr);
                      //nrf_delay_ms(500);
                      if(ret == 0){
                        ota_resposnse_ack(clearrom_state,ota_clear_pass);
                        printf("clear pass \r\n");
                      }else{
                        
                        ota_resposnse_ack(clearrom_state,ota_clear_fail);
                        printf("clear fail\r\n");
                      }
                      
                      *flag = idle_flag;

                      break;
              case start_flag:
                     
                      break;
              case write_flag:
                      
                 
                      break;
              
              case verify_flag:
                       


                      break;
              case burn_flag:
                      
                    //update mcu here
                      break;
              default:
                      break;
      }
}

static void ota_data_handler(ble_nus_evt_t * p_evt)
{
    uint32_t* tmp;
    uint8_t* tmp8_t;
    uint8_t head = 0x00;
    uint32_t ver = 0;
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
      uint32_t err_code;
      head = p_evt->params.rx_data.p_data[0];
      
      switch (head){
              case key_state:
                    tmp = (uint32_t*)&p_evt->params.rx_data.p_data[0];
                    key_cmd = *(ota_key_cmd_t*)tmp;
                    //compare key
                    if(1){
                       ota_resposnse_ack(key_state,ota_key_pass);
                    }else{
                      ota_resposnse_ack(key_state,ota_key_fail);
                    }


                   
                    //printf("key_state\r\n");
                    
                    break;
              case start_state:						
                     
                    tmp = (uint32_t*)&p_evt->params.rx_data.p_data[0];
                    start_cmd = *(ota_start_cmd_t*)tmp;
                    
                    memset(ota_buffer , 0 , OTA_BUFFER_SIZE);    //clear ota tmp buffer
                    
                    m_ota_ctrl.write_count = 0x00;
                    m_ota_ctrl.checksum =  0 ;
                    //check size
                    if(  (start_cmd.recv_len < otadata_max_size)  || 
                          ((start_cmd.addr)>=otadata_start_address && (start_cmd.addr)<otadata_end_address)
                      ){
                      ota_resposnse_ack(start_state,ota_start_pass);
                      m_ota_ctrl.write_enable = true;
                      m_ota_ctrl.flag = start_flag;
                    }else{

                      if((start_cmd.recv_len >= otadata_max_size)){
                        ota_resposnse_ack(start_state,ota_size_overflow);
                      }else{
                        ota_resposnse_ack(start_state,ota_address_fail);
                      }
                      
                      m_ota_ctrl.write_enable = false;
                      m_ota_ctrl.flag = idle_flag;

                   }
                  
                    //printf("start_state\r\n");
                    break;
              
              case write_state:

                    m_ota_ctrl.flag = write_flag;
                    tmp = (uint32_t*)&p_evt->params.rx_data.p_data[0];
                    tmp8_t = (uint8_t*)&p_evt->params.rx_data.p_data[2];
                    write_cmd = *(ota_write_cmd_t*)tmp;
                    if( m_ota_ctrl.write_count < OTA_BUFFER_SIZE ){
                      memcpy(&ota_buffer[m_ota_ctrl.write_count], tmp8_t, write_cmd.len);
                     
                    }
                    
                    m_ota_ctrl.write_count += write_cmd.len;
                    ota_resposnse_ack(write_state,ota_write_pass);
                    m_ota_ctrl.state = write_state;

                    //printf("write_state\r\n");
                    //if(m_ota_ctrl.write_enable){
                      
                    

                    //}else{

                    //    ota_resposnse_ack(write_state,ack_is_invalid);
                    //}
                  break;
                      
              case flash_state:
                      
                      m_ota_ctrl.flag = flash_flag;
                      //printf("flash_state\r\n");
                      break;
              
              case verify_state:  // verify
                      
                      tmp = (uint32_t*)&p_evt->params.rx_data.p_data[0];
                      verify_cmd = *(ota_verify_cmd_t*)tmp;						
                      
                      m_ota_ctrl.state = verify_state;
                      m_ota_ctrl.flag = verify_flag;

                      //use checksum to verify flash
                      if(m_ota_ctrl.checksum == verify_cmd.crc_value){
                        ota_resposnse_ack(verify_state,ota_verify_pass);
                       // printf("verify pass\r\n");
                      }else{
                        ota_resposnse_ack(verify_state,ota_verify_fail);
                       //  printf("verify failed\r\n");
                      }

                      break;
              
             case clearrom_state:  // clear rom
                      
                    m_ota_ctrl.flag = clear_flag;
                    tmp = (uint32_t*)&p_evt->params.rx_data.p_data[0];
                    clearrom_cmd = *(ota_clearrom_cmd_t*)tmp;
                    //ota_resposnse_ack(clearrom_state,ack_is_valid);
                    m_ota_ctrl.state = clearrom_state;
                    //printf("clear_state\r\n");
                    break;

              case check_version_state:
                    //check the ver. of the Frimware.                     
                    ver =  ota_get_version();
                    ota_resposnse_ver(ver);

                    break;

              case label_state:


                    break;
              default:	
                      break;
      }
    }
}


static void services_init(void) {
    ret_code_t err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_nus_init_t     nus_init;
    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = ota_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
  ret_code_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
  //APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
  ret_code_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
 */
static void application_timers_start(void) {
  nrf_drv_timer_enable(&TIMER_ADC);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void) {
  ret_code_t err_code;

  //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  //APP_ERROR_CHECK(err_code);

  // Prepare wakeup buttons.
  //err_code = bsp_btn_ble_sleep_mode_prepare();
  //APP_ERROR_CHECK(err_code);

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  ret_code_t err_code;
  
  switch (ble_adv_evt) {
    case BLE_ADV_EVT_FAST:
      NRF_LOG_INFO("Fast advertising.");
      //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
      //APP_ERROR_CHECK(err_code);
      break;

    case BLE_ADV_EVT_IDLE:
      // sleep_mode_enter();
      NRF_LOG_INFO("BLE idle.");
      break;

    default:
        // No implementation needed.
        break;
  }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  ret_code_t err_code = NRF_SUCCESS;
  // NRF_LOG_INFO("ble_evt_handler, %x", p_ble_evt->header.evt_id);
  switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Disconnected.");
      // LED indication will be changed when advertising starts.
      break;

    case BLE_GAP_EVT_CONNECTED:
      NRF_LOG_INFO("Connected.");
      //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
      //APP_ERROR_CHECK(err_code);
      //m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      //err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
      //APP_ERROR_CHECK(err_code);
      break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
      NRF_LOG_DEBUG("PHY update request.");
      ble_gap_phys_t const phys =
          {
              .rx_phys = BLE_GAP_PHY_AUTO,
              .tx_phys = BLE_GAP_PHY_AUTO,
          };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GATTC_EVT_TIMEOUT:
      // Disconnect on GATT Client timeout event.
      NRF_LOG_DEBUG("GATT Client Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
      NRF_LOG_DEBUG("GATT Server Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK(err_code);
        break;

    default:
      // No implementation needed.
      break;
  }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
  ble_cfg_t ble_cfg;
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void) {
  ble_gap_sec_params_t sec_param;
  ret_code_t err_code;
  err_code = pm_init();
  APP_ERROR_CHECK(err_code);
  memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

  // Security parameters to be used for all security procedures.
  sec_param.bond = SEC_PARAM_BOND;
  sec_param.mitm = SEC_PARAM_MITM;
  sec_param.lesc = SEC_PARAM_LESC;
  sec_param.keypress = SEC_PARAM_KEYPRESS;
  sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
  sec_param.oob = SEC_PARAM_OOB;
  sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
  sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
  sec_param.kdist_own.enc = 1;
  sec_param.kdist_own.id = 1;
  sec_param.kdist_peer.enc = 1;
  sec_param.kdist_peer.id = 1;
  err_code = pm_sec_params_set(&sec_param);
  APP_ERROR_CHECK(err_code);

  err_code = pm_register(pm_evt_handler);
  APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void) {
  ret_code_t err_code;

  NRF_LOG_INFO("Erase bonds!");

  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
  ret_code_t err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = true;
  init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.advdata.uuids_complete.p_uuids = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds) {
  if (erase_bonds == true) {
    delete_bonds();
    // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
  } else {
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
  }
}

static void biologue_evt_handler(ble_biologue_service_t *p_biologue_service, ble_biologue_evt_t *p_evt) {
  // Action to perform when the Data I/O characteristic notifications are enabled
  // Add your implementation here
  if (p_evt->evt_type == BLE_ECG_RAW_EVT_NOTIFICATION_ENABLED) {
    // Possibly save to a global variable to know that notifications are ENABLED
    NRF_LOG_INFO("Notifications ENABLED on ECG Characteristic");
  } else if (p_evt->evt_type == BLE_ECG_RAW_EVT_NOTIFICATION_DISABLED) {
    // Possibly save to a global variable to know that notifications are DISABLED
    NRF_LOG_INFO("Notifications DISABLED on ECG Characteristic");
  }

  // Handle any other events necessary...
}

static void biologue_services_init(void) {
    ret_code_t err_code;
    ble_biologue_service_init_t biologue_init;
  

    // Initialize the biologue service
    memset(&biologue_init, 0, sizeof(biologue_init));
    biologue_init.evt_handler = biologue_evt_handler;

    err_code = ble_biologue_service_init(&m_biologue, &biologue_init);
    NRF_LOG_INFO("Done with services_init()");
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */


int main(void) {  
  bool erase_bonds = false;
  // Initialize.
  log_init();
  power_management_init();
  ble_stack_init();
  gap_params_init();
  gatt_init();
  advertising_init();
  services_init();
  biologue_services_init();
  conn_params_init();
  peer_manager_init();
  timers_init();
  init_rtc();
  init_uart();
  //fatfs_example ();
  ad8232.init();
  ads1018.init();
  ota_flash_init();
 // Start execution.
  //NRF_LOG_INFO("Template example started. %p", &__otadata);
  NRF_LOG_INFO("GAP Length: %d", NRF_SDH_BLE_GAP_DATA_LENGTH);
  NRF_LOG_INFO("GATT MTU: %d", NRF_SDH_BLE_GATT_MAX_MTU_SIZE);

  advertising_start(erase_bonds);
  application_timers_start();
  
  //ota_flash_erase(0x50000,0x52000);
  //nrf_delay_ms(500);

  for (;;) {
    //idle_state_handle();
    ota_update_handle(&(m_ota_ctrl.flag));
  }
}


void HardFault_Handler(void) {
  NRF_LOG_INFO("HardFault Handle");
      __ASM volatile(

    "push {lr}                      \n"

    /* reserve space on stack for error_info_t struct - preserve 8byte stack aligment */
    "sub sp, sp, %0                 \n"

    /* prepare error_info_t struct */
    "str r0, [sp, %1]               \n"
    "str r1, [sp, %3]               \n"
    "str r2, [sp, %2]               \n"

    /* prepare arguments and call function: app_error_fault_handler */
    "ldr r0, =%4                    \n"
    "mov r1, lr                     \n"
    "mov r2, sp                     \n"
    "bl  %5                         \n"

    /* release stack */
    "add sp, sp, %0                 \n"

    "pop {pc}                       \n"
    ".ltorg                         \n"

    : /* Outputs */
    : /* Inputs */
    "I" (APP_ERROR_ERROR_INFO_SIZE_ALIGNED_8BYTE),
    "I" (APP_ERROR_ERROR_INFO_OFFSET_ERR_CODE),
    "I" (APP_ERROR_ERROR_INFO_OFFSET_P_FILE_NAME),
    "I" (APP_ERROR_ERROR_INFO_OFFSET_LINE_NUM),
    "X" (NRF_FAULT_ID_SDK_ERROR),
    "X" (app_error_fault_handler)
    : /* Clobbers */
    "r0", "r1", "r2"
    );
  while(true) {
    __asm("nop");
  }
}

/**
 * @}
 */