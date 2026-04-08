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

esp_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data_wr, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// =========================================================================
// EEPROM FORMATTER LOGIC
// =========================================================================

// 1. The Streamlined 32-Byte Structure
typedef struct __attribute__((packed)) {
    uint8_t unique_id;       // 1 byte  (e.g., 1-15)
    uint8_t version_major;   // 1 byte  (e.g., 1)
    uint8_t version_minor;   // 1 byte  (e.g., 0)
    char board_name[29];     // 29 bytes (28 visible chars + '\0')
} BoardMetadata_t;

// 2. Safe Byte-by-Byte Write Function
void format_eeprom(uint8_t i2c_addr, BoardMetadata_t *data) {
    printf("[EEPROM] Formatting board at 0x%02X...\n", i2c_addr);
    
    uint8_t *raw_data = (uint8_t*)data;
    size_t data_size = sizeof(BoardMetadata_t);

    for (int i = 0; i < data_size; i++) {
        i2c_write_reg(i2c_addr, i, &raw_data[i], 1);
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms burn time per byte
    }
    printf("[EEPROM] Formatting complete for 0x%02X.\n", i2c_addr);
}

// 3. Main Routine
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    printf("\n=================================================\n");
    printf("     --- AURA EEPROM FORMATTER UTILITY ---       \n");
    printf("=================================================\n");

    if (i2c_master_init() == ESP_OK) {
        printf("[SYSTEM] I2C Initialized.\n");

        // --- SETUP WEATHER BOARD (ID: 1) ---
        BoardMetadata_t weather_data = {
            .unique_id = 1,
            .version_major = 1,
            .version_minor = 0,
        };
        strncpy(weather_data.board_name, "Weather Board", sizeof(weather_data.board_name));
        format_eeprom(0x57, &weather_data);

        // --- SETUP IMU BOARD (ID: 6) ---
        BoardMetadata_t imu_data = {
            .unique_id = 6,
            .version_major = 1,
            .version_minor = 0,
        };
        strncpy(imu_data.board_name, "IMU Board", sizeof(imu_data.board_name));
        format_eeprom(0x50, &imu_data);

        // --- SETUP BUZZER BOARD (ID: 11) ---
        BoardMetadata_t buzzer_data = {
            .unique_id = 11,
            .version_major = 1,
            .version_minor = 0,
        };
        strncpy(buzzer_data.board_name, "Buzzer Board", sizeof(buzzer_data.board_name));
        format_eeprom(0x51, &buzzer_data);

        printf("\n[SYSTEM] All EEPROMs securely formatted!\n");
    } else {
        printf("[ERROR] Failed to initialize I2C.\n");
    }
    printf("=================================================\n");
}