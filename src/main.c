#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h" 

// =========================================================================
// HARDWARE PIN DEFINITIONS
// =========================================================================
#define BLINK_GPIO  47

#define I2C_MASTER_SCL_IO           1      
#define I2C_MASTER_SDA_IO           2      
#define I2C_MASTER_NUM              0      
#define I2C_MASTER_FREQ_HZ          100000 
#define I2C_MASTER_TX_BUF_DISABLE   0      
#define I2C_MASTER_RX_BUF_DISABLE   0      
#define I2C_MASTER_TIMEOUT_MS       1000

#define ADDR_EEPROM_IMU             0x50
#define ADDR_EEPROM_BUZZER          0x51
#define ADDR_EEPROM_WEATHER         0x57

// =========================================================================
// UNIVERSAL I2C WRAPPER FUNCTIONS
// =========================================================================

esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, 
        .scl_pullup_en = GPIO_PULLUP_DISABLE, 
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_ping_device(uint8_t dev_addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret; 
}

esp_err_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_rd, size_t size) {
    if (size == 0) return ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// =========================================================================
// EEPROM LOGIC: BOARD IDENTIFICATION
// =========================================================================

void identify_board(const char* target_name, uint8_t addr) {
    if (i2c_ping_device(addr) == ESP_OK) {
        uint8_t read_data[16] = {0};
        i2c_read_reg(addr, 0x00, read_data, 15);
        printf("[EEPROM] %-7s (0x%02X): Data=\"%s\"\n", target_name, addr, (char*)read_data);
    } else {
        printf("[EEPROM] %-7s (0x%02X): MISSING\n", target_name, addr);
    }
}

// =========================================================================
// MAIN
// =========================================================================
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    printf("\n=================================================\n");
    printf("    --- AURA FULL BUS & EEPROM DIAGNOSTIC ---    \n");
    printf("=================================================\n");

    if (i2c_master_init() == ESP_OK) {
        printf("[SYSTEM] I2C Initialized. Starting Board ID...\n");
        identify_board("IMU",     ADDR_EEPROM_IMU);
        identify_board("Buzzer",  ADDR_EEPROM_BUZZER);
        identify_board("Weather", ADDR_EEPROM_WEATHER);

        printf("\n[SYSTEM] Starting Full Bus Scan for Sensors...\n");
        int found_count = 0;
        for (int i = 1; i < 127; i++) {
            if (i2c_ping_device(i) == ESP_OK) {
                printf("  -> Found 0x%02X", i);
                // Identifying known addresses
                if (i == 0x20) printf(" (Buzzer Slave)");
                if (i == 0x29) printf(" (Light)");
                if (i == 0x48) printf(" (Temp)");
                if (i == 0x60) printf(" (Baro)");
                if (i == 0x68) printf(" (BMI323)");
                if (i == 0x7E) printf(" (I2C Reserved)");
                printf("\n");
                found_count++;
            }
        }
        printf("[SYSTEM] Scan complete. Found %d devices total.\n", found_count);
    }

    printf("=================================================\n\n");
}