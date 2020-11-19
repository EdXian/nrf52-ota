#include "ble_biologue_ota.h"

extern uint8_t* __otalabel;
extern uint8_t* __otadata;
NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    .evt_handler = fstorage_evt_handler,
    .start_addr = otadata_start_address,
    .end_addr   = otadata_end_address,
};

uint8_t ota_buffer[OTA_BUFFER_SIZE];

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {

        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {

        } break;

        default:
            break;
    }
}


void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    while (nrf_fstorage_is_busy(p_fstorage))
    {
  
    }
}

ret_code_t ota_flash_init(void){
    ret_code_t ret;
    ret =  nrf_fstorage_init(&fstorage, &nrf_fstorage_sd, NULL);
    return ret;
}

uint8_t*  ota_get_key( void ){
  return __otalabel;
}


uint8_t ota_check_key(void){
    uint8_t* key = (uint8_t*)ota_get_key;
    
    return 0;
}


uint8_t ota_flash_write(uint32_t addr, uint8_t* buffer, uint32_t recv_len){
   uint8_t ret = 0;
    ret = nrf_fstorage_write(&fstorage, addr, buffer, recv_len, NULL);
    wait_for_flash_ready(&fstorage);
    return ret;
}

uint8_t ota_flash_erase(uint32_t start_addr, uint32_t end_addr){
    ret_code_t ret ;//= nrf_fstorage_init(&fstorage, &nrf_fstorage_sd, NULL);
    uint8_t msg = 0;
    uint32_t page_num = ((end_addr - start_addr)/PAGE_SIZE); 

    if(page_num >31){
      page_num =31;
    }

    wait_for_flash_ready(&fstorage);
    ret = nrf_fstorage_erase(&fstorage, start_addr, page_num, &msg);
    wait_for_flash_ready(&fstorage);

    return ret;
}

void ota_label_clear(){
  uint8_t msg = 0;
    ret_code_t ret ;
    ret = nrf_fstorage_erase(&fstorage, label_address, 1, &msg);
    wait_for_flash_ready(&fstorage);
}

void ota_label_write(ota_label_t* label){
    nrf_fstorage_write(&fstorage, label_address, label, sizeof(label), NULL);
    wait_for_flash_ready(&fstorage);
}

void ota_label_read(ota_label_t* label){
    *label = OTA_Label;
}

uint32_t ota_get_version(void){
  uint32_t value = 0;
  value = (0<<24)+((uint32_t)OTA_Label.major_v<<16) + ((uint32_t)OTA_Label.minor_v<<8) +  ((uint32_t)OTA_Label.patch);
  return value;
}

uint8_t ota_is_valid(void){
  return OTA_Label.app_is_valid;
}
