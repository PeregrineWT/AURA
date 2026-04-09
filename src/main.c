#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h" 
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

// =========================================================================
// HARDWARE PIN DEFINITIONS
// =========================================================================
#define BLINK_GPIO  47
#define PIN_SD_CMD  41  
#define PIN_SD_CLK  40  
#define PIN_SD_D0   39  
#define MOUNT_POINT "/sdcard"

#define I2C_MASTER_SCL_IO           1      
#define I2C_MASTER_SDA_IO           2      
#define I2C_MASTER_NUM              0      
#define I2C_MASTER_FREQ_HZ          100000 
#define I2C_MASTER_TX_BUF_DISABLE   0      
#define I2C_MASTER_RX_BUF_DISABLE   0      
#define I2C_MASTER_TIMEOUT_MS       1000

#define ADDR_IMU_BMI323             0x68
#define ADDR_BARO_MPL3115           0x60
#define ADDR_TEMP_P3T1750           0x48
#define ADDR_LIGHT_LTR329           0x29

// =========================================================================
// CORE DATA STRUCTURES
// =========================================================================
typedef struct {
    char sensor_id;      
    uint32_t timestamp;  
    float values[7];     // Sized for max-case IMU (3 Accel, 3 Gyro)
} LogMessage_t;

QueueHandle_t data_queue;
char current_flight_log[64];

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

esp_err_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_rd, size_t size) {
    if (size == 0) return ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (size > 1) i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data_rd + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// BMI323 Custom Wrappers
esp_err_t bmi323_read_reg(uint8_t reg_addr, uint8_t *data, size_t size) {
    uint8_t buffer[24]; 
    esp_err_t err = i2c_read_reg(ADDR_IMU_BMI323, reg_addr, buffer, size + 2);
    if (err == ESP_OK) memcpy(data, &buffer[2], size);
    return err;
}
esp_err_t bmi323_write_reg(uint8_t reg_addr, uint16_t data_16) {
    uint8_t buffer[2] = { data_16 & 0xFF, (data_16 >> 8) & 0xFF };
    return i2c_write_reg(ADDR_IMU_BMI323, reg_addr, buffer, 2);
}

// =========================================================================
// HELPER: AUTO-INCREMENT LOG GENERATOR 
// =========================================================================
void generate_next_log_filename() {
    int max_num = 0;
    DIR *dir = opendir(MOUNT_POINT);
    if (dir != NULL) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL) {
            int num;
            if (sscanf(ent->d_name, "data_%d.csv", &num) == 1) {
                if (num > max_num) max_num = num;
            }
        }
        closedir(dir);
    }
    snprintf(current_flight_log, sizeof(current_flight_log), MOUNT_POINT "/data_%03d.csv", max_num + 1);
    printf("[SYSTEM] Auto-generated log file: %s\n", current_flight_log);
}

// =========================================================================
// SYSTEM TASK: BACKGROUND HEARTBEAT (1Hz)
// =========================================================================
void heartbeat_task(void *pvParameters) {
    int led_state = 0;
    LogMessage_t msg; 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);

    while (1) {
        led_state = !led_state;
        gpio_set_level(BLINK_GPIO, led_state);

        msg.sensor_id = 'H';
        msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        msg.values[0] = (float)led_state; 
        
        xQueueSend(data_queue, &msg, 0);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =========================================================================
// SENSOR TASK: IMU (Drift-Free 5Hz)
// =========================================================================
void imu_task(void *pvParameter) {
    uint16_t chip_id = 0;
    bmi323_read_reg(0x00, (uint8_t*)&chip_id, 2);
    if ((chip_id & 0xFF) == 0x43) {
        bmi323_write_reg(0x20, 0x7028); 
        bmi323_write_reg(0x21, 0x7028); 
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    LogMessage_t msg;
    uint8_t raw_data[12];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); 

    while(1) {
        if (bmi323_read_reg(0x03, raw_data, 12) == ESP_OK) {
            msg.sensor_id = 'I';
            msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Accel Math
            msg.values[0] = (int16_t)((raw_data[1] << 8) | raw_data[0]) / 4096.0f;
            msg.values[1] = (int16_t)((raw_data[3] << 8) | raw_data[2]) / 4096.0f;
            msg.values[2] = (int16_t)((raw_data[5] << 8) | raw_data[4]) / 4096.0f;
            
            // Gyro Math
            msg.values[3] = (int16_t)((raw_data[7] << 8) | raw_data[6]) / 16.384f;
            msg.values[4] = (int16_t)((raw_data[9] << 8) | raw_data[8]) / 16.384f;
            msg.values[5] = (int16_t)((raw_data[11] << 8) | raw_data[10]) / 16.384f;

            xQueueSend(data_queue, &msg, 0);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

// =========================================================================
// SENSOR TASK: WEATHER BOARD (Drift-Free 5Hz)
// =========================================================================
void weather_board_task(void *pvParameter) {
    uint8_t baro_ctrl = 0x01;
    i2c_write_reg(ADDR_BARO_MPL3115, 0x26, &baro_ctrl, 1);
    uint8_t light_ctrl = 0x01;
    i2c_write_reg(ADDR_LIGHT_LTR329, 0x80, &light_ctrl, 1);

    LogMessage_t msg;
    uint8_t raw_temp[2], raw_light[4], raw_baro[3];
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); 

    while(1) {
        msg.sensor_id = 'W';
        msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Temp
        if (i2c_read_reg(ADDR_TEMP_P3T1750, 0x00, raw_temp, 2) == ESP_OK) {
            msg.values[0] = ((raw_temp[0] << 8) | raw_temp[1] >> 4) * 0.0625f;
        }
        // Light
        if (i2c_read_reg(ADDR_LIGHT_LTR329, 0x88, raw_light, 4) == ESP_OK) {
            msg.values[1] = (float)((raw_light[3] << 8) | raw_light[2]); // Vis
            msg.values[2] = (float)((raw_light[1] << 8) | raw_light[0]); // IR
        }
        // Barometer
        if (i2c_read_reg(ADDR_BARO_MPL3115, 0x01, raw_baro, 3) == ESP_OK) {
            uint32_t press_raw = (raw_baro[0] << 16) | (raw_baro[1] << 8) | raw_baro[2];
            msg.values[3] = (float)(press_raw >> 4) / 4.0f;
        }
        
        xQueueSend(data_queue, &msg, 0);
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

// =========================================================================
// SYSTEM TASK: REAL-TIME SD LOGGER (THE CONSUMER)
// =========================================================================
void sd_logger_task(void *pvParameters) {
    LogMessage_t msg;
    
    FILE *f = fopen(current_flight_log, "a");
    if (f == NULL) {
        printf("[ERROR] Logger task failed to open file.\n");
        vTaskDelete(NULL);
    }
    
    // --- THE MASTER HEADER STRING ---
    fprintf(f, "Time_HB,State_HB,Time_IMU,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Time_WTR,Temp_C,Light_Vis,Light_IR,Baro_Pa,Time_US,US_Front,US_Back,US_Left,US_Right,Time_BTN,Btn_Up,Btn_Down,Btn_Left,Btn_Right\n"); 
    fclose(f); 

    f = fopen(current_flight_log, "a");
    TickType_t last_sync_time = xTaskGetTickCount();
    const TickType_t SYNC_INTERVAL_TICKS = pdMS_TO_TICKS(2500); 

    while (1) {
        if (xQueueReceive(data_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
            if (f != NULL) {
                switch (msg.sensor_id) {
                    
                    case 'H': // Heartbeat (0 commas)
                        fprintf(f, "%lu,%d\n", msg.timestamp, (int)msg.values[0]); 
                        break;
                    
                    case 'I': // IMU (2 commas)
                        fprintf(f, ",,%lu,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f\n", 
                                msg.timestamp, msg.values[0], msg.values[1], msg.values[2], 
                                msg.values[3], msg.values[4], msg.values[5]);
                        break;

                    case 'W': // Weather (9 commas)
                        fprintf(f, ",,,,,,,,,%lu,%.2f,%.0f,%.0f,%.2f\n", 
                                msg.timestamp, msg.values[0], msg.values[1], msg.values[2], msg.values[3]);
                        break;

                    case 'U': // Ultrasonics Future (14 commas)
                        fprintf(f, ",,,,,,,,,,,,,,%lu,%.2f,%.2f,%.2f,%.2f\n", 
                                msg.timestamp, msg.values[0], msg.values[1], msg.values[2], msg.values[3]);
                        break;

                    case 'B': // Buttons Future (19 commas)
                        fprintf(f, ",,,,,,,,,,,,,,,,,,,%lu,%.0f,%.0f,%.0f,%.0f\n", 
                                msg.timestamp, msg.values[0], msg.values[1], msg.values[2], msg.values[3]);
                        break;
                }
            }
        }

        // Flush buffer to SD card safely every 2.5 seconds
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_sync_time) >= SYNC_INTERVAL_TICKS) {
            if (f != NULL) {
                fclose(f);
                f = fopen(current_flight_log, "a"); 
            }
            last_sync_time = current_time; 
        }
    }
}

// =========================================================================
// MAIN APPLICATION ENTRY POINT
// =========================================================================
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(3000)); 
    printf("\n=================================================\n");
    printf("     --- AURA FLIGHT CONTROLLER BOOTING ---      \n");
    printf("=================================================\n");

    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    data_queue = xQueueCreate(1000, sizeof(LogMessage_t));

    // Mount SD Card
    printf("[SYSTEM] Initializing SD Card...\n");
    gpio_reset_pin(PIN_SD_CMD);
    gpio_reset_pin(PIN_SD_CLK);
    gpio_reset_pin(PIN_SD_D0);

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false, 
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT; 
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = PIN_SD_CLK; 
    slot_config.cmd = PIN_SD_CMD; 
    slot_config.d0  = PIN_SD_D0; 
    slot_config.width = 1;

    sdmmc_card_t *card;
    if (esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card) == ESP_OK) {
        printf("[SYSTEM] SD Card mounted successfully!\n");
        generate_next_log_filename();
        xTaskCreate(sd_logger_task, "SD_Logger_Task", 4096, NULL, 4, NULL);
    } else {
        printf("[ERROR] SD Card failed to mount. System will run without logging.\n");
    }

    // Init Sensors
    if (i2c_master_init() == ESP_OK) {
        printf("[SYSTEM] I2C Bus Active. Spawning sensor tasks...\n");
        xTaskCreate(imu_task, "IMU_Task", 4096, NULL, 5, NULL);
        xTaskCreate(weather_board_task, "Weather_Task", 4096, NULL, 5, NULL);
    }

    printf("[SYSTEM] Spawning Heartbeat Task...\n");
    xTaskCreate(heartbeat_task, "Heartbeat_Task", 4096, NULL, 1, NULL);

    printf("[SYSTEM] Initialization Complete. FreeRTOS Scheduler Active.\n");
    printf("=================================================\n\n");
}