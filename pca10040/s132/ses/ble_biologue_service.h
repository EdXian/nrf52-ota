/**
* @file ble_biologue_service.c
* @author Danny Deng
* @Company Biologue Co.Ltd
* @Date 2020/xx/xx
* 
* @brief BLE service for BLE sensor collector
*/

#ifndef BLE_BIOLOGUE_SERVICE_H
#define BLE_BIOLOGUE_SERVICE_H

#include <stdint.h>
#include <string.h>
#include "ble.h"
#include "ble_srv_common.h"

#ifndef BLE_BIOLOGUE_OBSERVER_PROO
#define BLE_BIOLOGUE_OBSERVER_PROO 2
#endif

// Define the Biologue service
#define BLE_BIOLOGUE_DEF(_name)                                  \
static ble_biologue_service_t _name;                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                              \
                     BLE_BIOLOGUE_OBSERVER_PROO,                 \
                     ble_biologue_service_on_ble_evt, &_name)

// Biologue Service UUID: 0x1234E001-FFFF-1234-FFFF-111122223333
// Biologue ECG raw data Characteristic UUID: 0x1234E002-FFFF-1234-FFFF-111122223333
// Biologue temperature Characteristic UUID: 0x1234E003-FFFF-1234-FFFF-111122223333
// Biologue command Characteristic UUID: 0x1234E004-FFFF-1234-FFFF-111122223333
// Biologue time(RTC) Characteristic UUID: 0x1234E005-FFFF-1234-FFFF-111122223333
// The bytes need to be in reverse order to match the UUID from the spec
// Note: They are stored in little endian, meaning that the Least Significant Byte
// is stored first.
    
#define BLE_UUID_BIOLOGUE_SERVICE_BASE_UUID  {0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0xFF, 0xFF, 0x34, 0x12, 0xFF, 0xFF, 0x01, 0xE0, 0x34, 0x12}
#define BLE_UUID_BIOLOGUE_SERVICE_UUID        0xE001
    
#define BLE_UUID_BIOLOGUE_ECG_RAW_BASE_UUID  {0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0xFF, 0xFF, 0x34, 0x12, 0xFF, 0xFF, 0x01, 0xE0, 0x34, 0x12}
#define BLE_UUID_BIOLOGUE_ECG_RAW_UUID        0xE002

#define BLE_UUID_BIOLOGUE_TEMP_BASE_UUID     {0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0xFF, 0xFF, 0x34, 0x12, 0xFF, 0xFF, 0x01, 0xE0, 0x34, 0x12}
#define BLE_UUID_BIOLOGUE_TEMP_UUID           0xE003

#define BLE_UUID_BIOLOGUE_COMMAND_BASE_UUID  {0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0xFF, 0xFF, 0x34, 0x12, 0xFF, 0xFF, 0x01, 0xE0, 0x34, 0x12}
#define BLE_UUID_BIOLOGUE_COMMAND_UUID        0xE004

#define BLE_UUID_BIOLOGUE_TIME_BASE_UUID     {0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0xFF, 0xFF, 0x34, 0x12, 0xFF, 0xFF, 0x01, 0xE0, 0x34, 0x12}
#define BLE_UUID_BIOLOGUE_TIME_UUID           0xE005

#define BLE_ECG_DATA_NOTIFY                  (0x11)

/* Define the value size */
#define VALUE_LEN_ECG_RAW                    (NRF_SDH_BLE_GATT_MAX_MTU_SIZE-3)
#define VALUE_LEN_TEMP                       (8)
#define VALUE_LEN_COMMAND_NORMAL             (3)
#define VALUE_LEN_COMMAND_HW_SET             (5)
#define VALUE_LEN_TIME                       (6)
#define VALUE_LEN_NOTIFY                     (NRF_SDH_BLE_GATT_MAX_MTU_SIZE-3)

/* Define the command from PC/Phone */
#define COMMAND_ECG_START                    (0xF0AA)
#define COMMAND_ECG_STOP                     (0xF1AA)
#define COMMAND_ECG_SLEEP                    (0xFAFF)
#define COMMAND_HW_SETTING                   (0xE2AA)

/**@brief Biologue Service event type. */
typedef enum {
    BLE_ECG_RAW_EVT_NOTIFICATION_ENABLED,
    BLE_ECG_RAW_EVT_NOTIFICATION_DISABLED,
    BLE_DATA_ECG,
    BLE_DATA_TEMP,
    BLE_DATA_SETTING_HW,
    BLE_DATA_TIME
} ble_biologue_evt_type_t;
 
/**@brief Biologue Service event. */
typedef struct {
  ble_biologue_evt_type_t evt_type;
} ble_biologue_evt_t;

// Forward declaration of the ble_midi_service_t type. 
typedef struct ble_biologue_service_s ble_biologue_service_t;

// Define the biologue service handler
typedef void (*ble_biologue_evt_handler_t) (ble_biologue_service_t * p_biologue_service, ble_biologue_evt_t * p_evt);

typedef struct {
    ble_biologue_evt_handler_t evt_handler;  /**< Event handler to be called when a MIDI event occurs. */
} ble_biologue_service_init_t;


/**@brief Biologue service structure. This contains various status information for the service. */
struct ble_biologue_service_s {
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    ecg_raw_char_handles;
    ble_gatts_char_handles_t    data_temp_handles;
    ble_gatts_char_handles_t    data_command_handles;
    ble_gatts_char_handles_t    data_time_handles;
    uint8_t                     uuid_type;
    uint16_t                    conn_handle;
    ble_biologue_evt_handler_t  evt_handler;
};

/**@brief Function for initializing the Biologue service.
 *
 * @param[out]  p_biologue_service   Biologue service structure. This structure will have to be supplied by
 *                                   the application. It will be initialized by this function, and will later
 *                               be used to identify this particular service instance.
 * @param[in]   p_biologue_service_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_biologue_service_init(ble_biologue_service_t *p_biologue_service, const ble_biologue_service_init_t * p_biologue_service_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Biologue service.
 *
 *
 * @param[in]   p_biologue_service      LED Button Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_biologue_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/* Define the Biologue API */
/**@brief Function for sending the ECG raw data.
 *
 * @details The application calls this function
 *
 *
 * @param[in]   p_biologue_service      Biologue Service structure.
 * @param[in]   ecg_raw_value           Event received from the BLE stack.
 */
uint32_t ble_biologue_ecg_raw_value_update(ble_biologue_service_t *p_biologue_service, uint8_t ecg_raw_value);

/**@brief Function for sending the ECG raw data.
 *
 * @details The application calls this function
 *
 *
 * @param[in]   p_biologue_service      Biologue Service structure.
 * @param[in]   ecg_raw_value           Event received from the BLE stack.
 */
uint32_t ble_biologue_temp_send(ble_biologue_service_t *p_biologue_service, uint16_t temp_data);

/**@brief Function for sending the ECG raw data.
 *
 * @details The application calls this function
 *
 *
 * @param[in]   p_biologue_service      Biologue Service structure.
 * @param[in]   ecg_raw_value           Event received from the BLE stack.
 */
uint32_t ble_biologue_command_send(ble_biologue_service_t *p_biologue_service, uint8_t *command_data);

/**@brief Function for sending the ECG raw data.
 *
 * @details The application calls this function
 *
 *
 * @param[in]   p_biologue_service      Biologue Service structure.
 * @param[in]   ecg_raw_value           Event received from the BLE stack.
 */
uint32_t ble_biologue_command_rec(ble_biologue_service_t *p_biologue_service, uint8_t *command_data);

/**@brief Function for sending the ECG raw data.
 *
 * @details The application calls this function
 *
 *
 * @param[in]   p_biologue_service      Biologue Service structure.
 * @param[in]   ecg_raw_value           Event received from the BLE stack.
 */
uint32_t ble_biologue_command_handle(uint8_t *packet);
uint32_t ecg_data_notify(ble_biologue_service_t *p_biologue_service);


#endif /* BLE_BIOLOGUE_SERVICE_H */
