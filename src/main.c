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

#define ADDR_IMU_BMI323             0x68
#define ADDR_BARO_MPL3115           0x60
#define ADDR_TEMP_P3T1750           0x48
#define ADDR_LIGHT_LTR329           0x29

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
typedef struct __attribute__((packed)) {
    uint8_t unique_id;       
    uint8_t version_major;   
    uint8_t version_minor;
    char board_name[29];     
} BoardMetadata_t;

void read_and_print_board(const char* expected_type, uint8_t addr) {
    if (i2c_ping_device(addr) == ESP_OK) {
        BoardMetadata_t data;
        esp_err_t err = i2c_read_reg(addr, 0x00, (uint8_t*)&data, sizeof(BoardMetadata_t));
        if (err == ESP_OK) {
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
// BMI323 SPECIFIC WRAPPERS
// =========================================================================
esp_err_t bmi323_read_reg(uint8_t reg_addr, uint8_t *data, size_t size) {
    uint8_t buffer[24]; 
    esp_err_t err = i2c_read_reg(ADDR_IMU_BMI323, reg_addr, buffer, size + 2);
    if (err == ESP_OK) {
        memcpy(data, &buffer[2], size);
    }
    return err;
}

esp_err_t bmi323_write_reg(uint8_t reg_addr, uint16_t data_16) {
    uint8_t buffer[2];
    buffer[0] = data_16 & 0xFF;        
    buffer[1] = (data_16 >> 8) & 0xFF; 
    return i2c_write_reg(ADDR_IMU_BMI323, reg_addr, buffer, 2);
}

// =========================================================================
// BMI323 IMU TASK (Drift-Free 5Hz)
// =========================================================================
void imu_task(void *pvParameter) {
    uint16_t chip_id = 0;
    bmi323_read_reg(0x00, (uint8_t*)&chip_id, 2);
    if ((chip_id & 0xFF) != 0x43) {
        printf("[IMU] ERROR: BMI323 not found or invalid Chip ID\n");
        vTaskDelete(NULL);
    }
    printf("[IMU] BMI323 Detected (Chip ID: 0x43). Initializing...\n");

    bmi323_write_reg(0x20, 0x7028); 
    bmi323_write_reg(0x21, 0x7028); 
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t raw_data[12];
    
    // --- DRIFT-FREE TIMING SETUP ---
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 5Hz (200ms)
    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        if (bmi323_read_reg(0x03, raw_data, 12) == ESP_OK) {
            
            int16_t ax_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
            int16_t ay_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
            int16_t az_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);
            int16_t gx_raw = (int16_t)((raw_data[7] << 8) | raw_data[6]);
            int16_t gy_raw = (int16_t)((raw_data[9] << 8) | raw_data[8]);
            int16_t gz_raw = (int16_t)((raw_data[11] << 8) | raw_data[10]);

            float ax = ax_raw / 4096.0f;
            float ay = ay_raw / 4096.0f;
            float az = az_raw / 4096.0f;

            float gx = gx_raw / 16.384f;
            float gy = gy_raw / 16.384f;
            float gz = gz_raw / 16.384f;

            printf("[IMU 5Hz] Accel (g): X:%5.2f Y:%5.2f Z:%5.2f | Gyro (dps): X:%6.1f Y:%6.1f Z:%6.1f\n", 
                   ax, ay, az, gx, gy, gz);
        } else {
            printf("[IMU] Read Error!\n");
        }

        // Wait precisely until exactly 200ms has passed since the last execution
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

// =========================================================================
// WEATHER BOARD TASK (Drift-Free 5Hz)
// =========================================================================
void weather_board_task(void *pvParameter) {
    uint8_t baro_ctrl = 0x01;
    i2c_write_reg(ADDR_BARO_MPL3115, 0x26, &baro_ctrl, 1);

    uint8_t light_ctrl = 0x01;
    i2c_write_reg(ADDR_LIGHT_LTR329, 0x80, &light_ctrl, 1);

    printf("[WEATHER] Sensors Initialized.\n");

    uint8_t raw_temp[2];
    uint8_t raw_light[4];
    uint8_t raw_baro[3];

    // --- DRIFT-FREE TIMING SETUP ---
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 5Hz (200ms)
    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        if (i2c_read_reg(ADDR_TEMP_P3T1750, 0x00, raw_temp, 2) == ESP_OK) {
            int16_t temp_val = (raw_temp[0] << 8) | raw_temp[1];
            float temp_c = (temp_val >> 4) * 0.0625; 
            printf("[WEATHER 5Hz] Temp: %.2f C | ", temp_c);
        }

        if (i2c_read_reg(ADDR_LIGHT_LTR329, 0x88, raw_light, 4) == ESP_OK) {
            uint16_t ir = (raw_light[1] << 8) | raw_light[0];
            uint16_t vis = (raw_light[3] << 8) | raw_light[2];
            printf("Light: %u lux (Vis) %u (IR) | ", vis, ir);
        }

        if (i2c_read_reg(ADDR_BARO_MPL3115, 0x01, raw_baro, 3) == ESP_OK) {
            uint32_t press_raw = (raw_baro[0] << 16) | (raw_baro[1] << 8) | raw_baro[2];
            float pressure_pa = (float)(press_raw >> 4) / 4.0; 
            printf("Baro: %.2f Pa\n", pressure_pa);
        }

        // Wait precisely until exactly 200ms has passed since the last execution
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
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
                printf("  -> Found 0x%02X\n", i);
                found_count++;
            }
        }
        printf("[SYSTEM] Scan complete. Found %d active devices on the bus.\n", found_count);
        printf("=================================================\n\n");

        xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
        xTaskCreate(weather_board_task, "weather_task", 4096, NULL, 5, NULL);

    } else {
        printf("[ERROR] Failed to initialize I2C bus.\n");
    }
}