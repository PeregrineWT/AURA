#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" 
#include "driver/gpio.h"
#include "driver/i2c.h" 
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_timer.h"
#include "rom/ets_sys.h" 

// The Official Arducam C Library (V3 SDK) & Hardware Abstraction
#include "ArducamCamera.h" 
#include "esp32Hal.h"

// =========================================================================
// HARDWARE PIN DEFINITIONS
// =========================================================================
#define BLINK_GPIO  47
#define PIN_SD_CMD  41  
#define PIN_SD_CLK  40  
#define PIN_SD_D0   39  
#define MOUNT_POINT "/sdcard"

// I2C Pins
#define I2C_MASTER_SCL_IO           1      
#define I2C_MASTER_SDA_IO           2      
#define I2C_MASTER_NUM              0      
#define I2C_MASTER_FREQ_HZ          100000 
#define I2C_MASTER_TX_BUF_DISABLE   0      
#define I2C_MASTER_RX_BUF_DISABLE   0      
#define I2C_MASTER_TIMEOUT_MS       1000

// I2C Sensor Addresses
#define ADDR_IMU_BMI323             0x68
#define ADDR_BARO_MPL3115           0x60
#define ADDR_TEMP_P3T1750           0x48
#define ADDR_LIGHT_LTR329           0x29

// AURA Modular EEPROM Data
#define ADDR_EEPROM_IMU             0x50
#define EXPECTED_ID_IMU             6
#define ADDR_EEPROM_BUZZER          0x51
#define EXPECTED_ID_BUZZER          11
#define ADDR_EEPROM_WEATHER         0x57
#define EXPECTED_ID_WEATHER         1

// Ultrasonic Pins
#define TRIG_A 5
#define TRIG_B 4
#define TRIG_C 7
#define TRIG_D 6
#define ECHO_A 16
#define ECHO_B 15
#define ECHO_C 18
#define ECHO_D 17

// CAMERA SPI PINS
#define CAM_PIN_CS   10
#define CAM_PIN_MOSI 11
#define CAM_PIN_MISO 13
#define CAM_PIN_SCK  12

// =========================================================================
// CORE DATA STRUCTURES & RTOS OBJECTS
// =========================================================================
typedef struct {
    char sensor_id;      
    uint32_t timestamp;  
    float values[7];     
} LogMessage_t;

// Standardized 32-Byte AURA EEPROM Map
typedef struct __attribute__((packed)) {
    uint8_t unique_id;
    uint8_t hw_version_major;
    uint8_t hw_version_minor;
    char board_name[29]; 
} AuraBoardEEPROM_t;

QueueHandle_t data_queue;
SemaphoreHandle_t sd_card_mutex; 
char current_flight_log[64];

spi_device_handle_t spi_cam_handle; // Exported to esp32Hal.h

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
// HELPER: I2C BUS SCANNING & EEPROM CHECK (SEQUENTIAL FORMAT)
// =========================================================================
typedef struct {
    uint8_t address;
    const char *description;
} I2cDeviceDesc_t;

const I2cDeviceDesc_t aura_master_i2c_list[] = {
    {0x10, "Buzzer Board (S3 Command Slave)"}, 
    {0x29, "LTR329 Light Sensor"},
    {0x39, "LTR329 Light Sensor (Alternative)"},
    {0x48, "P3T1750 Temp Sensor"},
    {0x49, "P3T1750 Temp Sensor (Alternative)"},
    {0x50, "IMU Board EEPROM"},
    {0x51, "Buzzer Board EEPROM"},
    {0x57, "Weather Board EEPROM"},
    {0x60, "MPL3115 Barometer"},
    {0x68, "BMI323 IMU"}
};
#define AURA_I2C_LIST_SIZE (sizeof(aura_master_i2c_list) / sizeof(I2cDeviceDesc_t))

const char* get_i2c_description(uint8_t addr) {
    for (int i = 0; i < AURA_I2C_LIST_SIZE; i++) {
        if (aura_master_i2c_list[i].address == addr) {
            return aura_master_i2c_list[i].description;
        }
    }
    return "Unknown Device"; 
}

bool check_i2c_device(uint8_t addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK);
}

void i2c_bus_scan_sequential() {
    uint8_t count = 0;
    printf("\n[SYSTEM] Starting sequential I2C bus scan...\n");
    printf("===========================================\n");

    for (uint8_t i = 1; i < 0x78; i++) { 
        if (check_i2c_device(i)) {
            const char* desc = get_i2c_description(i);
            printf(" -> Found device at 0x%02X (%s)\n", i, desc);
            count++;
        }
    }

    if (count == 0) {
        printf("[SYSTEM] No I2C devices responded.\n");
    } else {
        printf("===========================================\n");
        printf("[SYSTEM] Scan complete. Found %d active devices.\n\n", count);
    }
}

bool read_board_eeprom(uint8_t eeprom_addr, uint8_t expected_id) {
    AuraBoardEEPROM_t board_data;

    esp_err_t err = i2c_read_reg(eeprom_addr, 0x00, (uint8_t*)&board_data, sizeof(AuraBoardEEPROM_t));

    if (err == ESP_OK) {
        board_data.board_name[28] = '\0'; 

        printf(" -> [OK] EEPROM at 0x%02X Responded.\n", eeprom_addr);
        printf("      Board: %s (v%d.%d)\n", board_data.board_name, board_data.hw_version_major, board_data.hw_version_minor);
        printf("      ID:    %d\n", board_data.unique_id);
        
        if (board_data.unique_id == expected_id) {
            printf("      Status: ID Match Confirmed.\n");
            return true;
        } else {
            printf("      Status: [FAIL] ID Mismatch! Expected %d.\n", expected_id);
            return false;
        }
    } else {
        printf(" -> [FAIL] EEPROM at 0x%02X did not respond.\n", eeprom_addr);
        return false;
    }
}

// =========================================================================
// HELPER: ULTRASONIC PING
// =========================================================================
float read_ultrasonic(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    gpio_set_level(trig_pin, 0);
    ets_delay_us(2); 

    gpio_set_level(trig_pin, 1);
    ets_delay_us(10);
    gpio_set_level(trig_pin, 0);

    int64_t timeout_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0) {
        if ((esp_timer_get_time() - timeout_start) > 30000) return -1.0; 
    }
    int64_t pulse_start = esp_timer_get_time();

    while (gpio_get_level(echo_pin) == 1) {
        if ((esp_timer_get_time() - pulse_start) > 30000) return -1.0; 
    }
    int64_t pulse_end = esp_timer_get_time();

    return ((float)(pulse_end - pulse_start) * 0.0343f) / 2.0f;
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
        for(int i = 1; i < 7; i++) { msg.values[i] = 0.0f; }
        
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
            
            msg.values[0] = (int16_t)((raw_data[1] << 8) | raw_data[0]) / 4096.0f;
            msg.values[1] = (int16_t)((raw_data[3] << 8) | raw_data[2]) / 4096.0f;
            msg.values[2] = (int16_t)((raw_data[5] << 8) | raw_data[4]) / 4096.0f;
            
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
        
        if (i2c_read_reg(ADDR_TEMP_P3T1750, 0x00, raw_temp, 2) == ESP_OK) {
            int16_t temp_val = (raw_temp[0] << 8) | raw_temp[1]; 
            msg.values[0] = (temp_val >> 4) * 0.0625f;           
        }
        if (i2c_read_reg(ADDR_LIGHT_LTR329, 0x88, raw_light, 4) == ESP_OK) {
            msg.values[1] = (float)((raw_light[3] << 8) | raw_light[2]); 
            msg.values[2] = (float)((raw_light[1] << 8) | raw_light[0]); 
        }
        if (i2c_read_reg(ADDR_BARO_MPL3115, 0x01, raw_baro, 3) == ESP_OK) {
            uint32_t press_raw = (raw_baro[0] << 16) | (raw_baro[1] << 8) | raw_baro[2];
            msg.values[3] = (float)(press_raw >> 4) / 4.0f;
        }
        
        xQueueSend(data_queue, &msg, 0);
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

// =========================================================================
// SENSOR TASK: ULTRASONIC ARRAY (Drift-Free 5Hz)
// =========================================================================
void ultrasonic_task(void *pvParameter) {
    LogMessage_t msg;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); 

    while (1) {
        msg.sensor_id = 'U';
        msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        msg.values[0] = read_ultrasonic(TRIG_A, ECHO_A); 
        msg.values[1] = read_ultrasonic(TRIG_B, ECHO_B); 
        msg.values[2] = read_ultrasonic(TRIG_C, ECHO_C); 
        msg.values[3] = read_ultrasonic(TRIG_D, ECHO_D); 

        xQueueSend(data_queue, &msg, 0);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =========================================================================
// FEATURE TASK: BUZZER CONTROL (Ground Station Listener)
// =========================================================================
void buzzer_task(void *pvParameter) {
    printf("[BUZZER] Task active. Waiting for Ground Station commands...\n");
    
    while(1) {
        // --- FUTURE ESP-NOW LOGIC GOES HERE ---
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

// =========================================================================
// FEATURE TASK: 1080p CAMERA CAPTURE (3-SECOND INTERVAL)
// =========================================================================
void camera_task(void *pvParameters) {
    printf("[CAMERA] Task Booted. Performing Hardware Pre-Flight Check...\n");

    gpio_set_direction(CAM_PIN_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PIN_CS, 1); 
    vTaskDelay(pdMS_TO_TICKS(10));

    ArducamCamera myCAM = createArducamCamera(CAM_PIN_CS);
    begin(&myCAM);

    if (myCAM.cameraId == 0x00 || myCAM.cameraId == 0xFF) {
        printf("[FATAL] Camera unresponsive. Received ID: 0x%02X\n", myCAM.cameraId);
        vTaskDelete(NULL); 
    }

    printf("[CAMERA] Ping successful. Initializing Arducam Mega...\n");

    int image_counter = 1;
    char image_filename[64];
    
    uint8_t *image_buffer = (uint8_t *)heap_caps_malloc(4096, MALLOC_CAP_DMA);
    if (image_buffer == NULL) {
        printf("[FATAL] Out of DMA memory for camera buffer!\n");
        vTaskDelete(NULL);
    }

    const TickType_t xFrequency = pdMS_TO_TICKS(3000); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        printf("[CAMERA] Capturing Frame %d...\n", image_counter);

        takePicture(&myCAM, CAM_IMAGE_MODE_FHD, CAM_IMAGE_PIX_FMT_JPG);
        uint32_t image_len = myCAM.totalLength; 

        if (image_len > 0) {
            snprintf(image_filename, sizeof(image_filename), MOUNT_POINT "/img_%04d.jpg", image_counter);
            
            if (xSemaphoreTake(sd_card_mutex, portMAX_DELAY) == pdTRUE) {
                FILE *img_file = fopen(image_filename, "wb"); 
                if (img_file != NULL) {
                    uint32_t bytes_read = 0;
                    
                    while (bytes_read < image_len) {
                        uint16_t chunk_size = readBuff(&myCAM, image_buffer, 4096);
                        fwrite(image_buffer, 1, chunk_size, img_file);
                        bytes_read += chunk_size;
                        vTaskDelay(pdMS_TO_TICKS(1)); 
                    }
                    fclose(img_file);
                    printf("[CAMERA] Saved %s (%lu bytes)\n", image_filename, image_len);
                    
                    LogMessage_t cam_msg;
                    cam_msg.sensor_id = 'C';
                    cam_msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    cam_msg.values[0] = (float)image_counter; 
                    xQueueSend(data_queue, &cam_msg, 0);

                    image_counter++;
                } else {
                    printf("[ERROR] Camera failed to open file on SD card.\n");
                }
                
                xSemaphoreGive(sd_card_mutex);
            }
        }
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
    
    fprintf(f, "Time_HB,State_HB,Time_IMU,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Time_WTR,Temp_C,Light_Vis,Light_IR,Baro_Pa,Time_US,US_Front,US_Back,US_Left,US_Right,Time_BTN,Btn_Up,Btn_Down,Btn_Left,Btn_Right,Time_CAM,Cam_Event\n"); 
    fclose(f); 

    f = fopen(current_flight_log, "a");
    TickType_t last_sync_time = xTaskGetTickCount();
    const TickType_t SYNC_INTERVAL_TICKS = pdMS_TO_TICKS(2500); 

    printf("[SD_CARD] Logger task started. Waiting for sensor data...\n");

    while (1) {
        if (xQueueReceive(data_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
            if (f != NULL) {
                if (xSemaphoreTake(sd_card_mutex, portMAX_DELAY) == pdTRUE) {
                    switch (msg.sensor_id) {
                        case 'H': 
                            fprintf(f, "%lu,%d\n", msg.timestamp, (int)msg.values[0]); 
                            break;
                        case 'I': 
                            fprintf(f, ",,%lu,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f\n", 
                                    msg.timestamp, msg.values[0], msg.values[1], msg.values[2], 
                                    msg.values[3], msg.values[4], msg.values[5]);
                            break;
                        case 'W': 
                            fprintf(f, ",,,,,,,,,%lu,%.2f,%.0f,%.0f,%.2f\n", 
                                    msg.timestamp, msg.values[0], msg.values[1], msg.values[2], msg.values[3]);
                            break;
                        case 'U': 
                            fprintf(f, ",,,,,,,,,,,,,,%lu,%.2f,%.2f,%.2f,%.2f\n", 
                                    msg.timestamp, msg.values[0], msg.values[1], msg.values[2], msg.values[3]);
                            break;
                        case 'B': 
                            fprintf(f, ",,,,,,,,,,,,,,,,,,,%lu,%.0f,%.0f,%.0f,%.0f\n", 
                                    msg.timestamp, msg.values[0], msg.values[1], msg.values[2], msg.values[3]);
                            break;
                        case 'C':
                            fprintf(f, ",,,,,,,,,,,,,,,,,,,,,,,,%lu,img_%04d.jpg taken!\n", 
                                    msg.timestamp, (int)msg.values[0]);
                            break;
                    }
                    xSemaphoreGive(sd_card_mutex); 
                }
            }
        }

        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_sync_time) >= SYNC_INTERVAL_TICKS) {
            if (f != NULL) {
                if (xSemaphoreTake(sd_card_mutex, portMAX_DELAY) == pdTRUE) {
                    fclose(f);
                    f = fopen(current_flight_log, "a"); 
                    xSemaphoreGive(sd_card_mutex);
                }
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

    sd_card_mutex = xSemaphoreCreateMutex(); 
    data_queue = xQueueCreate(1000, sizeof(LogMessage_t));

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

    printf("[SYSTEM] Initializing SPI Bus for Camera...\n");
    spi_bus_config_t buscfg = {
        .miso_io_num = CAM_PIN_MISO,
        .mosi_io_num = CAM_PIN_MOSI,
        .sclk_io_num = CAM_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096 
    };
    
    if(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) == ESP_OK) {
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = 4000000,  
            .mode = 0,                 
            .spics_io_num = -1, 
            .queue_size = 7,
        };
        spi_bus_add_device(SPI2_HOST, &devcfg, &spi_cam_handle);
        printf("[SYSTEM] SPI Bus Initialized successfully.\n");
        xTaskCreate(camera_task, "Camera_Task", 8192, NULL, 3, NULL);
    } else {
        printf("[ERROR] Failed to initialize SPI bus.\n");
    }

    // ---------------------------------------------------------------------
    // HARDWARE INIT: I2C SCAN & DYNAMIC TASK SPAWNING
    // ---------------------------------------------------------------------
    if (i2c_master_init() == ESP_OK) {
        printf("[SYSTEM] I2C Bus Active.\n");
        
        // Output the clean sequential I2C bus scan
        i2c_bus_scan_sequential();
        
        // --- CHECK 1: THE IMU BOARD ---
        printf("Checking for IMU Board...\n");
        if (check_i2c_device(ADDR_EEPROM_IMU)) {
            if (read_board_eeprom(ADDR_EEPROM_IMU, EXPECTED_ID_IMU)) {
                printf("[SYSTEM] IMU Board Validated. Spawning task...\n");
                xTaskCreate(imu_task, "IMU_Task", 4096, NULL, 5, NULL);
            }
        } else {
            printf("[INFO] IMU EEPROM not detected. Task disabled.\n");
        }

        // --- CHECK 2: THE WEATHER BOARD ---
        printf("\nChecking for Weather Board...\n");
        if (check_i2c_device(ADDR_EEPROM_WEATHER)) {
            if (read_board_eeprom(ADDR_EEPROM_WEATHER, EXPECTED_ID_WEATHER)) {
                printf("[SYSTEM] Weather Board Validated. Spawning task...\n");
                xTaskCreate(weather_board_task, "Weather_Task", 4096, NULL, 5, NULL);
            }
        } else {
            printf("[INFO] Weather EEPROM not detected. Task disabled.\n");
        }

        // --- CHECK 3: THE BUZZER BOARD (ESP32-C3 Slave) ---
        printf("\nChecking for Buzzer Board...\n");
        if (check_i2c_device(ADDR_EEPROM_BUZZER)) {
            if (read_board_eeprom(ADDR_EEPROM_BUZZER, EXPECTED_ID_BUZZER)) {
                printf("[SYSTEM] Buzzer Board Validated. Spawning task...\n");
                xTaskCreate(buzzer_task, "Buzzer_Task", 4096, NULL, 5, NULL);
            }
        } else {
            printf("[INFO] Buzzer EEPROM not detected. Task disabled.\n");
        }
        
    } else {
        printf("[ERROR] Failed to initialize I2C bus.\n");
    }

    // ---------------------------------------------------------------------
    // HARDWARE INIT: ULTRASONIC ARRAY
    // ---------------------------------------------------------------------
    printf("\n[SYSTEM] Initializing Ultrasonic Array...\n");
    gpio_set_direction(TRIG_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRIG_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRIG_C, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRIG_D, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_A, GPIO_MODE_INPUT);
    gpio_set_direction(ECHO_B, GPIO_MODE_INPUT);
    gpio_set_direction(ECHO_C, GPIO_MODE_INPUT);
    gpio_set_direction(ECHO_D, GPIO_MODE_INPUT); 
    
    printf("[SYSTEM] Spawning Core Tasks...\n");
    xTaskCreate(ultrasonic_task, "Ultra_Task", 4096, NULL, 5, NULL);
    xTaskCreate(heartbeat_task, "Heartbeat_Task", 4096, NULL, 1, NULL);

    printf("\n=================================================\n");
    printf("[SYSTEM] Initialization Complete. Scheduler Active.\n");
    printf("=================================================\n\n");
}