#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h" 
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

// The Official Arducam C Library (V3 SDK) & Our Custom Hardware Abstraction
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

// CAMERA SPI PINS
#define CAM_PIN_CS   10
#define CAM_PIN_MOSI 11
#define CAM_PIN_MISO 13
#define CAM_PIN_SCK  12

// I2C BUS PINS (Matched to KiCad Schematic: SDA=IO2, SCL=IO1)
#define I2C_MASTER_SCL_IO           1      
#define I2C_MASTER_SDA_IO           2      
#define I2C_MASTER_NUM              0      
#define I2C_MASTER_FREQ_HZ          100000 // --- SET TO 100kHz STANDARD MODE ---
#define I2C_MASTER_TX_BUF_DISABLE   0      
#define I2C_MASTER_RX_BUF_DISABLE   0      
#define I2C_MASTER_TIMEOUT_MS       1000

// EEPROM Addresses for Board Detection
#define ADDR_EEPROM_IMU             0x50
#define ADDR_EEPROM_BUZZER          0x51
#define ADDR_EEPROM_WEATHER         0x57

// =========================================================================
// CORE DATA STRUCTURES & RTOS OBJECTS
// =========================================================================
typedef struct {
    char sensor_id;      
    uint32_t timestamp;  
    float values[7];     
} LogMessage_t;

QueueHandle_t data_queue;
SemaphoreHandle_t sd_card_mutex; 
char current_flight_log[64];

// Global flags for board presence
bool has_weather_board = false;
bool has_imu_board = false;
bool has_buzzer_board = false;

// SPI Device Handle (Exported to esp32Hal.h)
spi_device_handle_t spi_cam_handle; 

// =========================================================================
// UNIVERSAL I2C WRAPPER FUNCTIONS
// =========================================================================

esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, 
        .scl_pullup_en = GPIO_PULLUP_DISABLE, 
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
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
// SYSTEM TASK: BACKGROUND HEARTBEAT
// =========================================================================
void heartbeat_task(void *pvParameters) {
    int led_state = 0;
    LogMessage_t msg; 
    printf("[HEARTBEAT] Task started.\n");
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
// FEATURE TASK: 1080p CAMERA CAPTURE (3-SECOND INTERVAL)
// =========================================================================
void camera_task(void *pvParameters) {
    printf("[CAMERA] Task Booted. Performing Hardware Pre-Flight Check...\n");
    gpio_set_level(CAM_PIN_CS, 0);
    uint8_t response = arducam_spi_transfer(0x55); 
    gpio_set_level(CAM_PIN_CS, 1);

    if (response == 0x00 || response == 0xFF) {
        printf("[FATAL] Camera unresponsive (Echo: 0x%02X). Check power/SPI!\n", response);
        vTaskDelete(NULL); 
    }

    ArducamCamera myCAM = createArducamCamera(CAM_PIN_CS);
    begin(&myCAM);
    int image_counter = 1;
    uint8_t image_buffer[4096]; 
    const TickType_t xFrequency = pdMS_TO_TICKS(3000); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        takePicture(&myCAM, CAM_IMAGE_MODE_FHD, CAM_IMAGE_PIX_FMT_JPG);
        uint32_t image_len = myCAM.totalLength; 
        if (image_len > 0) {
            char image_filename[64];
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
                    image_counter++;
                }
                xSemaphoreGive(sd_card_mutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =========================================================================
// SYSTEM TASK: REAL-TIME SD LOGGER
// =========================================================================
void sd_logger_task(void *pvParameters) {
    LogMessage_t msg;
    FILE *f = fopen(current_flight_log, "a");
    if (f == NULL) vTaskDelete(NULL);
    fprintf(f, "Time_HB,State_HB\n"); 
    fclose(f); 
    f = fopen(current_flight_log, "a");
    TickType_t last_sync_time = xTaskGetTickCount();
    const TickType_t SYNC_INTERVAL_TICKS = pdMS_TO_TICKS(2500); 

    while (1) {
        if (xQueueReceive(data_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
            if (f != NULL) {
                if (xSemaphoreTake(sd_card_mutex, portMAX_DELAY) == pdTRUE) {
                    switch (msg.sensor_id) {
                        case 'H': fprintf(f, "%lu,%d\n", msg.timestamp, (int)msg.values[0]); break;
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

    data_queue = xQueueCreate(1000, sizeof(LogMessage_t));
    sd_card_mutex = xSemaphoreCreateMutex(); 

    // ---------------------------------------------------------------------
    // HARDWARE INIT: I2C BUS (100kHz Standard Mode)
    // ---------------------------------------------------------------------
    printf("[SYSTEM] Initializing I2C Bus (100kHz)...\n");
    if (i2c_master_init() == ESP_OK) {
        // Detection Logic
        has_imu_board = (i2c_ping_device(ADDR_EEPROM_IMU) == ESP_OK);
        has_buzzer_board = (i2c_ping_device(ADDR_EEPROM_BUZZER) == ESP_OK);
        has_weather_board = (i2c_ping_device(ADDR_EEPROM_WEATHER) == ESP_OK);

        printf("[I2C] Detection: IMU=%s, Buzzer=%s, Weather=%s\n", 
               has_imu_board ? "YES" : "NO", 
               has_buzzer_board ? "YES" : "NO", 
               has_weather_board ? "YES" : "NO");
    } else {
        printf("[ERROR] Failed to initialize I2C bus.\n");
    }

    // ---------------------------------------------------------------------
    // HARDWARE INIT: SPI BUS (FOR CAMERA)
    // ---------------------------------------------------------------------
    spi_bus_config_t buscfg = {
        .miso_io_num = CAM_PIN_MISO, .mosi_io_num = CAM_PIN_MOSI,
        .sclk_io_num = CAM_PIN_SCK, .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = 4096 
    };
    if(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) == ESP_OK) {
        spi_device_interface_config_t devcfg = {.clock_speed_hz = 8000000, .mode = 0, .spics_io_num = -1, .queue_size = 7};
        spi_bus_add_device(SPI2_HOST, &devcfg, &spi_cam_handle);
        printf("[SYSTEM] SPI Bus Initialized.\n");
    }

    // ---------------------------------------------------------------------
    // HARDWARE INIT: SD CARD
    // ---------------------------------------------------------------------
    gpio_reset_pin(PIN_SD_CMD); gpio_reset_pin(PIN_SD_CLK); gpio_reset_pin(PIN_SD_D0);
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {.format_if_mount_failed = false, .max_files = 5, .allocation_unit_size = 16 * 1024};
    sdmmc_host_t host = SDMMC_HOST_DEFAULT(); host.flags = SDMMC_HOST_FLAG_1BIT; 
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = PIN_SD_CLK; slot_config.cmd = PIN_SD_CMD; slot_config.d0 = PIN_SD_D0; slot_config.width = 1;
    sdmmc_card_t *card;
    if (esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card) == ESP_OK) {
        generate_next_log_filename();
        xTaskCreate(sd_logger_task, "SD_Logger_Task", 4096, NULL, 4, NULL);
    }

    // ---------------------------------------------------------------------
    // SPAWN TASKS
    // ---------------------------------------------------------------------
    xTaskCreate(heartbeat_task, "Heartbeat_Task", 4096, NULL, 1, NULL);
    xTaskCreate(camera_task, "Camera_Task", 8192, NULL, 0, NULL);

    printf("[SYSTEM] Boot Complete. FreeRTOS Active.\n");
    printf("=================================================\n\n");
}