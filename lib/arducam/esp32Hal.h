#ifndef __ESP32HAL_H
#define __ESP32HAL_H

// --- MODERN ESP-IDF DELAY FIX ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
// --------------------------------

#include "spi.h"

#define arducamSpiBegin()        spiBegin()
#define arducamSpiTransfer(val)  spiReadWriteByte(val) //  SPI communication sends a byte
#define arducamSpiCsPinHigh(pin) spiCsHigh(pin)        // Set the CS pin of SPI to high level
#define arducamSpiCsPinLow(pin)  spiCsLow(pin)         // Set the CS pin of SPI to low level
#define arducamCsOutputMode(pin) spiCsOutputMode(pin)

// Mapped directly to modern ESP-IDF delay functions
#define arducamDelayMs(val)      vTaskDelay(pdMS_TO_TICKS(val)) 
#define arducamDelayUs(val)      ets_delay_us(val) 

#endif /*__ESP32HAL_H*/