#ifndef BLE_BIOLOGUE_OTA_H
#define BLE_BIOLOGUE_OTA_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_fstorage_sd.h"
#include "nrf_fstorage.h"
#include "ble_nus.h"


#define label_address (0x4f000)
#define label_size   (0x1000)
#define otadata_max_size (0x20000)
#define otadata_start_address (label_address + label_size)
#define otadata_end_address (otadata_start_address + otadata_max_size)


#define PAGE_SIZE 4096
#define OTA_BUFFER_SIZE 4096
#define CRC_POLY  0xEDB88320
#define OTA_DATA_LEN  128


//#define PAGE_SIZE 4096
//#define OTA_BUFFER_SIZE 4000
//#define CRC_POLY  0xEDB88320
//#define OTA_DATA_LEN  200

typedef struct ota_control{
  uint8_t state;
  uint16_t write_count;
  uint8_t flag;
  uint8_t write_enable;
  int8_t checksum;
}ota_control_t;

typedef enum {
        key_state = 0x49,
	start_state = 0x50,
	write_state ,
	verify_state,
	clearrom_state,
	flash_state,
        check_version_state,
        label_state
}ota_state;


typedef enum {
    ack_is_valid = 1,
    ack_is_invalid = 2,
    
    ota_key_pass, 
    ota_key_fail ,

    ota_start_pass,
    ota_size_overflow,
    ota_address_fail,
    
    ota_write_pass,
    ota_write_fail,

    ota_verify_pass,
    ota_verify_fail,

    ota_clear_pass,
    ota_clear_fail

}ota_ack_type;

typedef enum{
	idle_flag = 0x00,
	flash_flag,
	clear_flag,
	start_flag,
	write_flag,
	verify_flag,
	burn_flag
}ota_flag;


#pragma pack(push,1)

typedef struct ota_label{
  uint8_t app_is_valid;
  uint8_t major_v;
  uint8_t minor_v;
  uint8_t patch;
  uint32_t key[16];
}ota_label_t;

#pragma pack(pop)

#define OTA_Label    (*(ota_label_t*)0x4f000)

#pragma pack(push,1)

typedef  struct ota_start_cmd{  //__attribute__((packed))
	uint8_t cmd;
	uint8_t len;
	uint32_t recv_len;
	uint32_t addr;
}ota_start_cmd_t;

typedef  struct ota_write_cmd{
	uint8_t cmd;
	uint8_t len;
	uint8_t first_data;   
}ota_write_cmd_t;


typedef  struct ota_verify_cmd{
	uint8_t cmd;
	uint8_t type;
	uint32_t crc_value;
}ota_verify_cmd_t;

typedef  struct ota_clearrom_cmd{
	uint8_t cmd;
	uint8_t len;
	uint32_t start_addr;
	uint32_t end_addr;
}ota_clearrom_cmd_t;

typedef  struct ota_key_cmd{
	uint8_t cmd;
	uint8_t len;
	uint8_t key[16];
}ota_key_cmd_t;


#pragma pack(pop)


// 
uint32_t ota_crc32(uint32_t crc32, uint8_t *buf, uint32_t len);
uint32_t ota_get_version(void);
//void ota_resposnse_ver(uint32_t ver);
//write label
void ota_label_clear();
void ota_label_write(ota_label_t* label);

//key
uint8_t ota_check_key(void);

//flash
ret_code_t ota_flash_init(void);
uint8_t ota_flash_write(uint32_t addr, uint8_t* buffer, uint32_t recv_len);
uint8_t ota_flash_erase(uint32_t start_addr, uint32_t end_addr);
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage);
#endif
