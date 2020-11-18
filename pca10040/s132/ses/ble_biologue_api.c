#include "ble_biologue_service.h"
#include "app_util.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "nrf_log.h"

uint8_t volatile notify_data[VALUE_LEN_NOTIFY];
extern volatile uint8_t flag_notify;

uint32_t ble_biologue_command_handle(uint8_t *packet) {
  if(packet[0] != 0xAA) return NRF_ERROR_NULL;
  uint16_t command = (packet[1] << 8) | packet[2];
  uint16_t value;
  switch(command) {
    case COMMAND_ECG_START:
        NRF_LOG_INFO("COMMAND_ECG_START");
        flag_notify = true;
      break;
    case COMMAND_ECG_STOP:
        NRF_LOG_INFO("COMMAND_ECG_STOP");
        flag_notify = false;
      break;
    case COMMAND_ECG_SLEEP:
      NRF_LOG_INFO("COMMAND_ECG_SLEEP");
      break;
    case COMMAND_HW_SETTING:
      value = (packet[3] << 8) | packet[4];
      NRF_LOG_INFO("COMMAND_HW_SETTING, config %x", value);
      break;
    default:
      NRF_LOG_INFO("command: %d", command);
      break;
  }
  return NRF_SUCCESS;
}

uint32_t ecg_data_notify(ble_biologue_service_t *p_biologue_service) {
  if (p_biologue_service == NULL) {
    return NRF_ERROR_NULL;
  }

  uint32_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  // Initialize value struct.
  memset(&gatts_value, 0, sizeof(gatts_value));

  gatts_value.len = VALUE_LEN_NOTIFY;
  gatts_value.offset = 0;
   //static uint8_t temp[VALUE_LEN_NOTIFY];
   //memcpy(temp, notify_data, VALUE_LEN_ECG_RAW);
  gatts_value.p_value = (uint8_t *)notify_data;
  
  // Update database.
  err_code = sd_ble_gatts_value_set(p_biologue_service->conn_handle,
      p_biologue_service->ecg_raw_char_handles.value_handle,
      &gatts_value);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  // Send value if connected and notifying.
  if ((p_biologue_service->conn_handle != BLE_CONN_HANDLE_INVALID)) {
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_biologue_service->ecg_raw_char_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = gatts_value.offset;
    hvx_params.p_len = &gatts_value.len;
    hvx_params.p_data = gatts_value.p_value;

    err_code = sd_ble_gatts_hvx(p_biologue_service->conn_handle, &hvx_params);
    //NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
  } 
  else {
    err_code = NRF_ERROR_INVALID_STATE;
    //NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n");
  }

  return err_code;
}