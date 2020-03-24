#include "dfu_transport.h"
#include "dfu.h"
#include <dfu_types.h>

#include <stdlib.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_spi.h"
#include "nrf_delay.h"
#include <string.h>
#include "flash_nrf5x.h"
#include "boards.h"
#include "sdk_common.h"

//For Debug
#include "SEGGER_RTT.h"
#include "bootloader.h"


#define MAX_BUFFERS          4u                                                      /**< Maximum number of buffers that can be received queued without being consumed. */

// Only allow to write application TODO dynamic depending on SD size
#define USER_FLASH_START   0x26000
#define USER_FLASH_END     0xAD000 // Fat Fs start here
#define CODE_PAGE_SIZE     0x1000                      /**< Size of a flash codepage. Used for size of the reserved flash space in the bootloader region. Will be runtime checked against NRF_UICR->CODEPAGESIZE to ensure the region is correct. */

#define FLASH_PAGE_SIZE    4096
#define FLASH_SIZE         (USER_FLASH_END-USER_FLASH_START) // Max flash size

#define FLASH_CACHE_INVALID_ADDR  0xffffffff



static uint32_t _fl_addr = FLASH_CACHE_INVALID_ADDR;
static uint8_t _fl_buf[FLASH_PAGE_SIZE] __attribute__((aligned(4)));

        
        
enum
{
    total_size = (1 << 21), /* 2 MiB */
    start_up_time_us = 5000,
    manufacturer_id = 0xc8,
    memory_type = 0x40,
    capacity = 0x15,
    max_clock_speed_mhz = 104, /* if we need 120 then we can turn on high performance mode */
    quad_enable_bit_mask = 0x02,
    has_sector_protection = false,
    supports_fast_read = true,
    supports_qspi = true,
    supports_qspi_writes = true,
    write_status_register_split = false,
    single_status_byte = false,
}; // GD25Q16C

enum
{
    SFLASH_CMD_READ              = 0x03, // Single Read
    SFLASH_CMD_QUAD_READ         = 0x6B, // 1 line address, 4 line data
    
    SFLASH_CMD_READ_JEDEC_ID     = 0x9f,
    
    SFLASH_CMD_PAGE_PROGRAM      = 0x02,
    SFLASH_CMD_QUAD_PAGE_PROGRAM = 0x32, // 1 line address, 4 line data
    
    SFLASH_CMD_READ_STATUS       = 0x05,
    SFLASH_CMD_READ_STATUS2      = 0x35,
    
    SFLASH_CMD_WRITE_STATUS      = 0x01,
    SFLASH_CMD_WRITE_STATUS2     = 0x31,
    
    SFLASH_CMD_ENABLE_RESET      = 0x66,
    SFLASH_CMD_RESET             = 0x99,
    
    SFLASH_CMD_WRITE_ENABLE      = 0x06,
    SFLASH_CMD_WRITE_DISABLE     = 0x04,
    
    SFLASH_CMD_ERASE_SECTOR      = 0x20,
    SFLASH_CMD_ERASE_BLOCK       = 0xD8,
    SFLASH_CMD_ERASE_CHIP        = 0xC7,
};



void spi_init();
void spi_begin();
void spi_end();
void flash_rom_init();
void waitUntilReady(void);
char transfer(uint8_t data);
uint16_t transfer16(uint16_t data);
bool runCommand(uint8_t command);
bool readMemory(uint32_t addr, uint8_t *data, uint32_t len);
uint32_t readBuffer(uint32_t address, uint8_t *buffer, uint32_t len);
bool readCommand(uint8_t command, uint8_t* response, uint32_t len);
uint8_t readStatus();
void dump_sector(uint32_t sector);

//static inline void wait_for_flash_ready(void);
//void erase_non_volatile_memory(uint32_t address);

uint32_t dfu_transport_ltem_update_start(void);




uint32_t mosi_pin = 20;
uint32_t miso_pin = 21;
uint32_t sck_pin = 19;
uint32_t ss_pin = 24;

// 初期値にNULLを入れて、NULLかどうかを確認してから、malloc(), free()する。
/*
    if (app_copy == NULL) {
        app_copy = malloc(len);
        memset(app_copy, 0, len);
    }
  
    if (app_copy != NULL) {
        free(app_copy);
        app_copy = NULL;
    }
 */
// malloc()で確保した app_copy は配列としてアクセスして良い
uint8_t* app_copy = NULL;
uint32_t app_length = 0;
// Corresponds to the sector size.
const uint32_t PACKET_LENGTH = 512;

///**@cond NO_DOXYGEN */
//static uint32_t                     m_data_received;                                /**< Amount of received data. */
///**@endcond */






void spi_init() {
    NRF_SPI_Type* spi = NRF_SPI0;
    
    spi->ENABLE = 0;
    

    nrf_gpio_pin_set(mosi_pin);
    nrf_gpio_cfg_output(mosi_pin);
    
    nrf_gpio_pin_set(miso_pin);
    nrf_gpio_cfg_input(miso_pin, (nrf_gpio_pin_pull_t)NRF_GPIO_PIN_PULLDOWN);
    
    nrf_gpio_pin_set(sck_pin);
    nrf_gpio_cfg_output(sck_pin);
    
    nrf_gpio_pin_set(ss_pin);
    nrf_gpio_cfg_output(ss_pin);
    
    nrf_spi_pins_set(spi, sck_pin, mosi_pin, miso_pin);
    nrf_spi_configure(spi, NRF_SPI_MODE_0, SPI_CONFIG_ORDER_MsbFirst);
    
//    nrf_spi_frequency_set(spi, max_clock_speed_mhz*1000000UL);
    nrf_spi_frequency_set(spi, 4000000);
}

void spi_begin() {
    nrf_spi_enable(NRF_SPI0);
    nrf_spi_event_clear(NRF_SPI0, NRF_SPI_EVENT_READY);
}

void spi_end() {
    nrf_spi_disable(NRF_SPI0);
}

void flash_rom_init() {
    // SPI_Flash begin()
    // The write in progress bit should be low.
    while ( readStatus() & 0x01 ) {}

    runCommand(SFLASH_CMD_ENABLE_RESET);
    runCommand(SFLASH_CMD_RESET);

    // Wait 30us for the reset
    nrf_delay_us(30);

    runCommand(SFLASH_CMD_WRITE_DISABLE);

    waitUntilReady();
}

void waitUntilReady(void) {
    // both WIP and WREN bit should be clear
    while ( readStatus() & 0x03 ) {}
}


char transfer(uint8_t data) {
    nrf_spi_txd_set(NRF_SPI0, data);
    
    while(!nrf_spi_event_check(NRF_SPI0, NRF_SPI_EVENT_READY));
    
    data = nrf_spi_rxd_get(NRF_SPI0);
    
    nrf_spi_event_clear(NRF_SPI0, NRF_SPI_EVENT_READY);
    
    return data;
}

uint16_t transfer16(uint16_t data) {
    union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;
    
    t.val = data;
    
//    if (_bitOrder == SPI_CONFIG_ORDER_LsbFirst) {
//        t.lsb = transfer(t.lsb);
//        t.msb = transfer(t.msb);
//    } else {
    t.msb = transfer(t.msb);
    t.lsb = transfer(t.lsb);
//    }
    
    return t.val;
}

void getJEDECID() {
    uint8_t jedec_ids[3];
    readCommand(SFLASH_CMD_READ_JEDEC_ID, jedec_ids, 3);
}

bool runCommand(uint8_t command) {
    nrf_gpio_pin_write(ss_pin, 0);
    
    transfer(command);
    
    nrf_gpio_pin_write(ss_pin, 1);
    
    return true;
}

bool readMemory(uint32_t addr, uint8_t *data, uint32_t len) {
    nrf_gpio_pin_write(ss_pin, 0);
    
    transfer(SFLASH_CMD_READ);
    
    // 24-bit address MSB first
    transfer((addr >> 16) & 0xFF);
    transfer((addr >> 8) & 0xFF);
    transfer(addr & 0xFF);
    
    while(len--)
    {
        *data++ = transfer(0xFF);
    }
    
    nrf_gpio_pin_write(ss_pin, 1);
    
    return true;
}

uint32_t readBuffer(uint32_t address, uint8_t *buffer, uint32_t len) {
    waitUntilReady();
    
    SEGGER_RTT_printf(0, "(%u, %u)\n", (address/512), (len/512));
    
    return readMemory(address, buffer, len) ? len : 0;
}

bool readCommand(uint8_t command, uint8_t* response, uint32_t len) {
    nrf_gpio_pin_write(ss_pin, 0);
    
    transfer(command);
    while(len--)
    {
        *response++ = transfer(0xFF);
    }
    
    nrf_gpio_pin_write(ss_pin, 1);
    
    return true;
}

uint8_t readStatus() {
    uint8_t status;
    readCommand(SFLASH_CMD_READ_STATUS, &status, 1);
    return status;
}

void dump_sector(uint32_t sector) {
    uint8_t buf[512];
    readBuffer(sector*512, buf, 512);
    
    for(uint32_t row=0; row<32; row++) {
        if ( row == 0 ) SEGGER_RTT_printf(0, "0");
        if ( row < 16 ) SEGGER_RTT_printf(0, "0");
        SEGGER_RTT_printf(0, "%x", row*16);
        SEGGER_RTT_printf(0, " : ");
        
        for(uint32_t col=0; col<16; col++) {
            uint8_t val = buf[row*16 + col];
            
        if ( val < 16 ) SEGGER_RTT_printf(0, "0");
            SEGGER_RTT_printf(0, "%x", val);
            SEGGER_RTT_printf(0, " ");
        }
        
        SEGGER_RTT_printf(0, "\n");
        nrf_delay_us(10);
    }
}

int get_app_size() {
    // Extract the size of firmware from the sector0 in flash ROM.
    uint8_t sector = 0;
    uint8_t length_data[9];
    readBuffer(sector*512, length_data, 9);
    
    SEGGER_RTT_printf(0, "%02x %02x %02x %02x %02x", length_data[0],length_data[1],length_data[2],length_data[3],length_data[4]);
    
    int len = atoi((const char *)length_data);
    
    SEGGER_RTT_WriteString(0, "\n");
    SEGGER_RTT_printf(0, "%d", len);
    
    return len;
}

void copy_app_to_mcu(uint32_t len) {
  uint8_t buf[512];
  uint32_t sector = 1;
  uint32_t pos = 0;
  
  for (uint32_t i = 0; i < (int)(len/512) + 1; i++) {
    readBuffer(sector*512, buf, 512);
    for (uint32_t j = 0; j < 512; j++) {
      if (pos < len) {
        app_copy[pos] = buf[j];
        pos++;
      } else {
        break;
      }
    }
    sector++;
  }
  
  SEGGER_RTT_WriteString(0, "copy_app_to_mcu(int len)\n");
  SEGGER_RTT_printf(0, "%02x %02x %02x %02x : %02x %02x %02x %02x\n\n", app_copy[0],app_copy[1],app_copy[2],app_copy[3],app_copy[len-4],app_copy[len-3],app_copy[len-2],app_copy[len-1]);
}



uint32_t dfu_transport_ltem_update_start(void) {
    SEGGER_RTT_WriteString(0, "dfu_transport_ltem: dfu_transport_ltem_update_start()\n");
    
    // Open SPI in order to copy the firmware to the heap memory.
    spi_init();
    SEGGER_RTT_WriteString(0, "SPI initialization has done.\n");
    spi_begin();
    SEGGER_RTT_WriteString(0, "SPI has begun.\n");

    flash_rom_init();
    
    SEGGER_RTT_WriteString(0, "\n");
    
    // Get the size of the firmware from the sector0 in flash ROM.
    nrf_delay_us(100);
    waitUntilReady();
    app_length = get_app_size();
    SEGGER_RTT_WriteString(0, "\n");
    
    
    
    // malloc, free で、Flash ROMの中身をコピーするためのメモリを確保して、その先頭アドレスを渡す。
    if (app_copy == NULL) {
        app_copy = malloc(app_length);
        memset(app_copy, 0, app_length);
    }
    if (app_copy == NULL) {
        SEGGER_RTT_WriteString(0, "malloc() error\n");
    }
    
    copy_app_to_mcu(app_length);
    
//    // Print out the sector0 in flash ROM.
//    nrf_delay_us(100);
//    waitUntilReady();
//    dump_sector(0);
//    SEGGER_RTT_WriteString(0, "\n");
//
//    // Print out the sector1 in flash ROM.
//    nrf_delay_us(100);
//    waitUntilReady();
//    dump_sector(1);
//    SEGGER_RTT_WriteString(0, "\n");
//
    
    
    sd_softdevice_disable();
    
    
    uint32_t total_written_app_length = 0;
    
    for (uint32_t i = 0; i < app_length; i += PACKET_LENGTH) {
        uint16_t write_length = PACKET_LENGTH < (app_length-i) ? PACKET_LENGTH : (app_length - i);
        uint32_t newAddr = (USER_FLASH_START + total_written_app_length) & ~(FLASH_PAGE_SIZE - 1);
    
//        erase_non_volatile_memory(USER_FLASH_START);
        
        SEGGER_RTT_printf(0, "total data length: 0x%x, write_length: 0x%x\n", i, write_length);
        SEGGER_RTT_printf(0, "newAddr: 0x%x\n", newAddr);

        if ( newAddr != _fl_addr )
        {
            flash_nrf5x_flush(false);
            _fl_addr = newAddr;
            memcpy(_fl_buf, (void *) newAddr, PACKET_LENGTH);
        }
    
    
        SEGGER_RTT_printf(0, "0x%x, 0x%x, 0x%x, 0x%x\n", _fl_buf, USER_FLASH_START, total_written_app_length, FLASH_PAGE_SIZE);
        SEGGER_RTT_printf(0, "0x%x\n", (_fl_buf + ((USER_FLASH_START + total_written_app_length) & (FLASH_PAGE_SIZE - 1))));
    
    
//        sd_flash_write((uint32_t *)(USER_FLASH_START + total_written_app_length), (const uint32_t*)(&app_copy[i]), write_length);
        nrfx_nvmc_words_write((USER_FLASH_START + total_written_app_length), (const uint32_t*)(&app_copy[i]), write_length);
        
//        memcpy(_fl_buf + ((USER_FLASH_START + total_written_app_length) & (FLASH_PAGE_SIZE - 1)), &app_copy[i], write_length);
        
        total_written_app_length += write_length;
        nrf_delay_us(10);
    }
    
//    sector_number = 1;
//
//    for (uint32_t i = 0; i < FIRMWARE_SIZE; i += WRITE_MAX_SIZE_ONE_TIME) {
//        uint16_t write_length = WRITE_MAX_SIZE_ONE_TIME < (FIRMWARE_SIZE-i) ? WRITE_MAX_SIZE_ONE_TIME : (FIRMWARE_SIZE - i);
//        Serial.printf("sector_number, write_length: %u, %u\n", sector_number, write_length);
//        //    uint16_t write_length = min(WRITE_MAX_SIZE_ONE_TIME, FIRMWARE_SIZE-i);
//        write_sector(sector_number, &FIRMWARE[i], write_length);
//        sector_number++;
//    }
    
    
    
    if (app_copy != NULL) {
        free(app_copy);
        app_copy = NULL;
    }
    
    spi_end();
    SEGGER_RTT_WriteString(0, "SPI has ended.\n\n");
    





    return NRF_SUCCESS;
}

