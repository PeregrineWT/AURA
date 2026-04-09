#ifndef __ESP32HAL_H
#define __ESP32HAL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>

// Pull in the SPI handle from main.c
extern spi_device_handle_t spi_cam_handle;

// Custom inline function to bridge Arducam to ESP-IDF SPI Hardware
static inline uint8_t arducam_spi_transfer(uint8_t val) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.tx_data[0] = val;
    // Transmit over SPI and block until finished
    spi_device_polling_transmit(spi_cam_handle, &t);
    return t.rx_data[0];
}

// Map Arducam's commands directly to ESP-IDF functions
#define arducamSpiBegin()        // Handled securely in main.c
#define arducamSpiTransfer(val)  arducam_spi_transfer(val)
#define arducamSpiCsPinHigh(pin) gpio_set_level(pin, 1)
#define arducamSpiCsPinLow(pin)  gpio_set_level(pin, 0)
#define arducamCsOutputMode(pin) gpio_set_direction(pin, GPIO_MODE_OUTPUT)

#define arducamDelayMs(val)      vTaskDelay(pdMS_TO_TICKS(val)) 
#define arducamDelayUs(val)      ets_delay_us(val) 

#endif /*__ESP32HAL_H*/