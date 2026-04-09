#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h" 

// =========================================================================
// I2C CONFIGURATION
// =========================================================================
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
// UNIVERSAL I2C WRAPPERS
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
// EEPROM METADATA LOGIC
// =========================================================================

// Our standardized 32-byte header
typedef struct __attribute__((packed)) {
    uint8_t unique_id;       
    uint8_t version_major;   
    uint8_t version_minor;   
    char board_name[29];     
} BoardMetadata_t;

void read_and_print_board(const char* expected_type, uint8_t addr) {
    if (i2c_ping_device(addr) == ESP_OK) {
        BoardMetadata_t data;
        // Read 32 bytes starting at memory register 0x00
        esp_err_t err = i2c_read_reg(addr, 0x00, (uint8_t*)&data, sizeof(BoardMetadata_t));
        
        if (err == ESP_OK) {
            // Safety check in case the EEPROM is somehow still blank (0xFF = 255)
            if (data.unique_id == 255) {
                printf("[EEPROM] 0x%02X (%-7s): DETECTED [BLANK / UNFORMATTED]\n", addr, expected_type);
            } else {
                printf("[EEPROM] 0x%02X (%-7s): DETECTED | Name: '%s' | Ver: %d.%d | ID: %d\n", 
                       addr, expected_type, data.board_name, data.version_major, data.version_minor, data.unique_id);
            }
        } else {
            printf("[EEPROM] 0x%02X (%-7s): READ ERROR\n", addr, expected_type);
        }
    } else {
        printf("[EEPROM] 0x%02X (%-7s): MISSING\n", addr, expected_type);
    }
}

// =========================================================================
// MAIN ROUTINE
// =========================================================================
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    printf("\n=================================================\n");
    printf("    --- AURA FLIGHT CONTROLLER BUS SCAN ---      \n");
    printf("=================================================\n");

    if (i2c_master_init() == ESP_OK) {
        printf("[SYSTEM] I2C Initialized (100kHz).\n\n");
        
        printf("--- READING MODULAR IDENTITIES ---\n");
        read_and_print_board("IMU", ADDR_EEPROM_IMU);
        read_and_print_board("Buzzer", ADDR_EEPROM_BUZZER);
        read_and_print_board("Weather", ADDR_EEPROM_WEATHER);

        printf("\n--- FULL BUS SENSOR SCAN ---\n");
        int found_count = 0;
        for (int i = 1; i < 127; i++) {
            if (i2c_ping_device(i) == ESP_OK) {
                printf("  -> Found 0x%02X", i);
                
                // Sensor Identification mapping
                if (i == 0x20) printf(" (Buzzer ESP32-C3 Slave)");
                else if (i == 0x29) printf(" (Light Sensor LTR-329)");
                else if (i == 0x48) printf(" (Temp Sensor P3T1750)");
                else if (i == 0x50) printf(" (IMU EEPROM)");
                else if (i == 0x51) printf(" (Buzzer EEPROM)");
                else if (i == 0x57) printf(" (Weather EEPROM)");
                else if (i == 0x60) printf(" (Barometer MPL3115)");
                else if (i == 0x68) printf(" (IMU BMI323)");
                else if (i == 0x7E) printf(" (I3C Reserved / BMI323 Wake)");
                
                printf("\n");
                found_count++;
            }
        }
        printf("[SYSTEM] Scan complete. Found %d active devices on the bus.\n", found_count);

    } else {
        printf("[ERROR] Failed to initialize I2C bus.\n");
    }

    printf("=================================================\n\n");
}