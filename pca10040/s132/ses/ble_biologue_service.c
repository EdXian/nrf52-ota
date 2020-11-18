/**
* @file ble_biologue_service.c
* @author Danny Deng
* @Company Biologue Co.Ltd
* @Date 2020/xx/xx
* 
* @brief BLE service for BLE sensor collector, this file defines the biologue service and its characteristic.
*/

#include "ble_biologue_service.h"
#include "app_util.h"
#include "rtc.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "nrf_log.h"

/* Global value in stack for characteristic */
uint8_t ecg_buf[VALUE_LEN_ECG_RAW];
char rtc_buf[VALUE_LEN_TIME];
uint8_t temp_buf[VALUE_LEN_TEMP];
volatile uint8_t flag_notify = false;
volatile uint8_t flag_notify_ok = true;

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_biologue_service   Biologue service structure.
 * @param[in]   p_ble_evt            Event received from the BLE stack.
 */
static void on_connect(ble_biologue_service_t *p_biologue_service, ble_evt_t const *p_ble_evt) {
  p_biologue_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_biologue_service   LED Button Service structure.
 * @param[in]   p_ble_evt            Event received from the BLE stack.
 */
static void on_disconnect(ble_biologue_service_t *p_biologue_service, ble_evt_t const *p_ble_evt) {
  UNUSED_PARAMETER(p_ble_evt);
  p_biologue_service->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_biologue_service   LED Button Service structure.
 * @param[in]   p_ble_evt        Event received from the BLE stack.
 */
static void on_write(ble_biologue_service_t *p_biologue_service, ble_evt_t const *p_ble_evt) {
  ble_gatts_evt_write_t *p_evt_write = (ble_gatts_evt_write_t *)&p_ble_evt->evt.gatts_evt.params.write;
  uint16_t command = 0;
  
  // Handle characteristic
  if((p_evt_write->handle == p_biologue_service->ecg_raw_char_handles.cccd_handle) && (p_biologue_service->evt_handler != NULL)) {
    
  }
  else if((p_evt_write->handle == p_biologue_service->data_temp_handles.cccd_handle) && (p_biologue_service->evt_handler != NULL)) {
    
  }
  else if((p_evt_write->handle == p_biologue_service->data_command_handles.cccd_handle) && (p_biologue_service->evt_handler != NULL)) {

  }
  else if((p_evt_write->handle == p_biologue_service->data_time_handles.value_handle) && (p_biologue_service->evt_handler != NULL)) {
    NRF_LOG_INFO("PC/Phone write handle.");
    if(p_evt_write->len == VALUE_LEN_TIME) {
      rtc_client_handle(p_evt_write->data);
    }
    else {
      NRF_LOG_INFO("Data length does not match.");
    }
  }
  else if((p_evt_write->handle == p_biologue_service->data_command_handles.value_handle) && (p_biologue_service->evt_handler != NULL)) {
    NRF_LOG_INFO("Command write handle.");
    if((p_evt_write->len == VALUE_LEN_COMMAND_NORMAL) || (p_evt_write->len == VALUE_LEN_COMMAND_HW_SET)) {
      ble_biologue_command_handle(p_evt_write->data);
    }
    else {
      NRF_LOG_INFO("Data length does not match.");
    }
  }

  //if ((p_evt_write->handle == p_biologue_service->ecg_raw_char_handles.value_handle) &&
  //    (p_evt_write->len == 1) &&
  //    (p_biologue_service->evt_handler != NULL)) {
  //  // Handle what happens on a write event to the characteristic value
  //}

  // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
  if ((p_evt_write->handle == p_biologue_service->ecg_raw_char_handles.cccd_handle) && (p_evt_write->len == 2)) {
    // CCCD written, call application event handler
    if (p_biologue_service->evt_handler != NULL) {
      ble_biologue_evt_t evt;

      if (ble_srv_is_notification_enabled(p_evt_write->data)) {
        evt.evt_type = BLE_ECG_RAW_EVT_NOTIFICATION_ENABLED;
      } else {
        evt.evt_type = BLE_ECG_RAW_EVT_NOTIFICATION_DISABLED;
      }

      p_biologue_service->evt_handler(p_biologue_service, &evt);
    }
  }
}

// Notify data complete
static void on_hvx_tx_complete(ble_biologue_service_t *p_biologue_service, ble_evt_t const *p_ble_evt) {
  flag_notify_ok = true;
}

void ble_biologue_service_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
  ble_biologue_service_t *p_biologue_service = (ble_biologue_service_t *)p_context;

  if(p_ble_evt->header.evt_id != BLE_GATTS_EVT_HVN_TX_COMPLETE)
    NRF_LOG_INFO("BLE event received. Event type = %x", p_ble_evt->header.evt_id);

  if (p_biologue_service == NULL || p_ble_evt == NULL) {
    return;
  }

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    on_connect(p_biologue_service, p_ble_evt);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    on_disconnect(p_biologue_service, p_ble_evt);
    break;

  case BLE_GATTS_EVT_WRITE:
    on_write(p_biologue_service, p_ble_evt);
    break;
  case BLE_GATTS_EVT_HVN_TX_COMPLETE:
    on_hvx_tx_complete(p_biologue_service, p_ble_evt);
    break;
  case BLE_GATTS_EVT_SYS_ATTR_MISSING:
    NRF_LOG_ERROR("BLE_GATTS_EVT_SYS_ATTR_MISSING");
    break;
  default:
    // No implementation needed.
    break;
  }
}

/**@brief Function for adding the ECG RAW characteristic.
 * @property Read
 * @type     char
 * @length   274 bytes
 */
static uint32_t ecg_raw_char_add(ble_biologue_service_t *p_biologue_service, const ble_biologue_service_init_t *p_biologue_service_init) {
  uint32_t err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t attr_char_value;
  ble_uuid_t ble_uuid;
  ble_gatts_attr_md_t attr_md;

  // Configure the CCCD which is needed for Notifications and Indications
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  // Configure the characteristic metadata.
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 0;
  char_md.char_props.write_wo_resp = 0;
  char_md.char_props.notify = 1;
  char_md.p_char_user_desc = "ECG raw data.";
  char_md.char_user_desc_max_size = 13;
  char_md.char_user_desc_size = 13;
  char_md.p_char_pf = NULL;
  char_md.p_user_desc_md = NULL;
  char_md.p_cccd_md = &cccd_md;
  char_md.p_sccd_md = NULL;

  // Add the biologue Data I/O Characteristic UUID
  ble_uuid128_t base_uuid = {BLE_UUID_BIOLOGUE_ECG_RAW_BASE_UUID};
  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_biologue_service->uuid_type);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  ble_uuid.type = p_biologue_service->uuid_type;
  ble_uuid.uuid = BLE_UUID_BIOLOGUE_ECG_RAW_UUID;

  // Configure the characteristic value's metadata
  memset(&attr_md, 0, sizeof(attr_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc = BLE_GATTS_VLOC_USER;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen = 0;

  // Configure the characteristic value
  memset(ecg_buf, 0, sizeof(ecg_buf));
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = VALUE_LEN_ECG_RAW;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = VALUE_LEN_ECG_RAW;
  attr_char_value.p_value = ecg_buf;

  return sd_ble_gatts_characteristic_add(p_biologue_service->service_handle, &char_md,
      &attr_char_value,
      &p_biologue_service->ecg_raw_char_handles);
}

/**@brief Function for adding the temperature characteristic.
 *
 */
//static uint32_t temp_char_add(ble_biologue_service_t *p_biologue_service, const ble_biologue_service_init_t *p_biologue_service_init) {
//  uint32_t err_code;
//  ble_gatts_char_md_t char_md;
//  ble_gatts_attr_md_t cccd_md;
//  ble_gatts_attr_t attr_char_value;
//  ble_uuid_t ble_uuid;
//  ble_gatts_attr_md_t attr_md;

//  // Configure the CCCD which is needed for Notifications and Indications
//  memset(&cccd_md, 0, sizeof(cccd_md));
//  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

//  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

//  // Configure the characteristic metadata.
//  memset(&char_md, 0, sizeof(char_md));
//  char_md.char_props.read = 1;
//  char_md.char_props.write_wo_resp = 0;
//  char_md.char_props.notify = 0;
//  char_md.p_char_user_desc = "Temperature.";
//  char_md.char_user_desc_max_size = 12;
//  char_md.char_user_desc_size = 12;
//  char_md.p_char_pf = NULL;
//  char_md.p_user_desc_md = NULL;
//  // char_md.p_cccd_md = &cccd_md;
//  char_md.p_sccd_md = NULL;

//  // Add the biologue Data I/O Characteristic UUID
//  ble_uuid128_t base_uuid = {BLE_UUID_BIOLOGUE_ECG_RAW_BASE_UUID};
//  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_biologue_service->uuid_type);
//  if (err_code != NRF_SUCCESS) {
//    return err_code;
//  }

//  ble_uuid.type = p_biologue_service->uuid_type;
//  ble_uuid.uuid = BLE_UUID_BIOLOGUE_TEMP_UUID;

//  // Configure the characteristic value's metadata
//  memset(&attr_md, 0, sizeof(attr_md));
//  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
//  attr_md.vloc = BLE_GATTS_VLOC_USER;
//  attr_md.rd_auth = 0;
//  attr_md.wr_auth = 0;
//  attr_md.vlen = 0;

//  // Configure the characteristic value
//  memset(&attr_char_value, 0, sizeof(attr_char_value));
//  memset(temp_buf, 0, VALUE_LEN_TEMP);
//  attr_char_value.p_uuid = &ble_uuid;
//  attr_char_value.p_attr_md = &attr_md;
//  attr_char_value.init_len = VALUE_LEN_TEMP;
//  attr_char_value.init_offs = 0;
//  attr_char_value.max_len = VALUE_LEN_TEMP;
//  attr_char_value.p_value = temp_buf;

//  return sd_ble_gatts_characteristic_add(p_biologue_service->service_handle, &char_md,
//      &attr_char_value,
//      &p_biologue_service->data_temp_handles);
//}

/**@brief Function for adding the command characteristic.
 *
 */
static uint32_t command_char_add(ble_biologue_service_t *p_biologue_service, const ble_biologue_service_init_t *p_biologue_service_init) {
  uint32_t err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t attr_char_value;
  ble_uuid_t ble_uuid;
  ble_gatts_attr_md_t attr_md;

  // Configure the CCCD which is needed for Notifications and Indications
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  // Configure the characteristic metadata.
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.write = 1;
  char_md.char_props.read = 0;
  char_md.char_props.write_wo_resp = 0;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc = "Command.";
  char_md.char_user_desc_max_size = 8;
  char_md.char_user_desc_size = 8;
  char_md.p_char_pf = NULL;
  char_md.p_user_desc_md = NULL;
  // char_md.p_cccd_md = &cccd_md;
  char_md.p_sccd_md = NULL;

  // Add the biologue Data I/O Characteristic UUID
  ble_uuid128_t base_uuid = {BLE_UUID_BIOLOGUE_ECG_RAW_BASE_UUID};
  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_biologue_service->uuid_type);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  ble_uuid.type = p_biologue_service->uuid_type;
  ble_uuid.uuid = BLE_UUID_BIOLOGUE_COMMAND_UUID;

  // Configure the characteristic value's metadata
  memset(&attr_md, 0, sizeof(attr_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen = 1;

  // Configure the characteristic value
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = VALUE_LEN_COMMAND_NORMAL;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = VALUE_LEN_COMMAND_HW_SET;
  attr_char_value.p_value = NULL;

  return sd_ble_gatts_characteristic_add(p_biologue_service->service_handle, &char_md,
      &attr_char_value,
      &p_biologue_service->data_command_handles);
}

/**@brief Function for adding the time characteristic.
 *
 */
static uint32_t time_char_add(ble_biologue_service_t *p_biologue_service, const ble_biologue_service_init_t *p_biologue_service_init) {
  uint32_t err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t attr_char_value;
  ble_uuid_t ble_uuid;
  ble_gatts_attr_md_t attr_md;

  // Configure the CCCD which is needed for Notifications and Indications
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  // Configure the characteristic metadata.
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.write = 1;
  char_md.char_props.write_wo_resp = 0;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc = "Real-time clock.";
  char_md.char_user_desc_max_size = 16;
  char_md.char_user_desc_size = 16;
  char_md.p_char_pf = NULL;
  char_md.p_user_desc_md = NULL;
  char_md.p_cccd_md = &cccd_md;
  char_md.p_sccd_md = NULL;

  // Add the biologue Data I/O Characteristic UUID
  ble_uuid128_t base_uuid = {BLE_UUID_BIOLOGUE_ECG_RAW_BASE_UUID};
  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_biologue_service->uuid_type);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  ble_uuid.type = p_biologue_service->uuid_type;
  ble_uuid.uuid = BLE_UUID_BIOLOGUE_TIME_UUID;

  // Configure the characteristic value's metadata
  memset(&attr_md, 0, sizeof(attr_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc = BLE_GATTS_VLOC_USER;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen = 0;

  // Configure the characteristic value
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = VALUE_LEN_TIME;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = VALUE_LEN_TIME;
  attr_char_value.p_value = rtc_buf;

  return sd_ble_gatts_characteristic_add(p_biologue_service->service_handle, &char_md,
      &attr_char_value,
      &p_biologue_service->data_time_handles);
}


/*
static uint32_t time_char_add(ble_biologue_service_t *p_biologue_service, const ble_biologue_service_init_t *p_biologue_service_init) {
  uint32_t err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t attr_char_value;
  ble_uuid_t ble_uuid;
  ble_gatts_attr_md_t attr_md;

  // Configure the CCCD which is needed for Notifications and Indications
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  // Configure the characteristic metadata.
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.write = 1;
  char_md.char_props.write_wo_resp = 0;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc = "Real-time clock.";
  char_md.char_user_desc_max_size = 16;
  char_md.char_user_desc_size = 16;
  char_md.p_char_pf = NULL;
  char_md.p_user_desc_md = NULL;
  char_md.p_cccd_md = &cccd_md;
  char_md.p_sccd_md = NULL;

  // Add the biologue Data I/O Characteristic UUID
  ble_uuid128_t base_uuid = {BLE_UUID_BIOLOGUE_ECG_RAW_BASE_UUID};
  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_biologue_service->uuid_type);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  ble_uuid.type = p_biologue_service->uuid_type;
  ble_uuid.uuid = BLE_UUID_BIOLOGUE_TIME_UUID;

  // Configure the characteristic value's metadata
  memset(&attr_md, 0, sizeof(attr_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc = BLE_GATTS_VLOC_USER;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen = 0;

  // Configure the characteristic value
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = VALUE_LEN_TIME;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = VALUE_LEN_TIME;
  attr_char_value.p_value = rtc_buf;

  return sd_ble_gatts_characteristic_add(p_biologue_service->service_handle, &char_md,
      &attr_char_value,
      &p_biologue_service->data_time_handles);
}

*/







uint32_t ble_biologue_service_init(ble_biologue_service_t *p_biologue_service, const ble_biologue_service_init_t *p_biologue_service_init) {
  uint32_t err_code;
  ble_uuid_t ble_uuid;

  // Initialize service structure
  p_biologue_service->conn_handle = BLE_CONN_HANDLE_INVALID;
  p_biologue_service->evt_handler = p_biologue_service_init->evt_handler;

  // Add service
  ble_uuid128_t base_uuid = {BLE_UUID_BIOLOGUE_SERVICE_BASE_UUID};
  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_biologue_service->uuid_type);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  ble_uuid.type = p_biologue_service->uuid_type;
  ble_uuid.uuid = BLE_UUID_BIOLOGUE_SERVICE_UUID;

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_biologue_service->service_handle);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  // Add four characteristic
  err_code = ecg_raw_char_add(p_biologue_service, p_biologue_service_init);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }
  //err_code = temp_char_add(p_biologue_service, p_biologue_service_init);
  //if (err_code != NRF_SUCCESS) {
  //  return err_code;
  //}
  err_code = command_char_add(p_biologue_service, p_biologue_service_init);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }
  err_code = time_char_add(p_biologue_service, p_biologue_service_init);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  





  return NRF_SUCCESS;
}



