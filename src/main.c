#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

// The Official Arducam C Library (Now linked via your components folder)
#include "ArducamCamera.h" 

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

// =========================================================================
// CORE DATA STRUCTURES & RTOS OBJECTS
// =========================================================================
typedef struct {
    char sensor_id;      
    uint32_t timestamp;  
    float values[7];     
} LogMessage_t;

QueueHandle_t data_queue;
SemaphoreHandle_t sd_card_mutex; // Protects the SD Card pipeline from data collisions
char current_flight_log[64];

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
    printf("[CAMERA] Task Booted. Initializing Arducam Mega...\n");

    ArducamCamera myCAM = createArducamCamera(CAM_PIN_CS);
    begin(&myCAM);
    
    // Set to 1080p (FHD) to guarantee we finish saving within the 3-second cycle
    setResolution(&myCAM, SIZE_1080P); 

    int image_counter = 1;
    char image_filename[64];
    uint8_t image_buffer[4096]; // 4KB chunk buffer to save ESP32 RAM

    // The strict 3-second capture rate
    const TickType_t xFrequency = pdMS_TO_TICKS(3000); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        printf("[CAMERA] Capturing Frame %d...\n", image_counter);

        takePicture(&myCAM, CAM_IMAGE_MODE_FHD, CAM_IMAGE_PIX_FMT_JPG);
        uint32_t image_len = getTotalLength(&myCAM); 

        if (image_len > 0) {
            snprintf(image_filename, sizeof(image_filename), MOUNT_POINT "/img_%04d.jpg", image_counter);
            
            // SECURITY CHECK: Wait for the SD Card Mutex (Block forever if Logger is using it)
            if (xSemaphoreTake(sd_card_mutex, portMAX_DELAY) == pdTRUE) {
                
                FILE *img_file = fopen(image_filename, "wb"); // Write Binary mode
                if (img_file != NULL) {
                    uint32_t bytes_read = 0;
                    
                    // Pull the image from the camera over SPI 4096 bytes at a time
                    while (bytes_read < image_len) {
                        uint16_t chunk_size = readBuff(&myCAM, image_buffer, 4096);
                        fwrite(image_buffer, 1, chunk_size, img_file);
                        bytes_read += chunk_size;
                    }

                    fclose(img_file);
                    printf("[CAMERA] Saved %s (%lu bytes)\n", image_filename, image_len);
                    image_counter++;
                } else {
                    printf("[ERROR] Camera failed to open file on SD card.\n");
                }
                
                // UNLOCK: Hand the SD Card Mutex back so the Logger can write its queued data
                xSemaphoreGive(sd_card_mutex);
            }
        }

        // Sleep deterministically for exactly 3 seconds before taking the next picture
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

    printf("[SD_CARD] Logger task started. Waiting for sensor data...\n");

    while (1) {
        // If data is waiting in the queue...
        if (xQueueReceive(data_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
            if (f != NULL) {
                // Request the Mutex before writing to the SD card
                if (xSemaphoreTake(sd_card_mutex, portMAX_DELAY) == pdTRUE) {
                    
                    switch (msg.sensor_id) {
                        case 'S': 
                            fprintf(f, "%lu,System Event Logged\n", msg.timestamp); 
                            break; 
                        case 'H': 
                            fprintf(f, "%lu,%d\n", msg.timestamp, (int)msg.values[0]); 
                            break;
                    }
                    
                    xSemaphoreGive(sd_card_mutex); // Unlock Mutex instantly after write
                }
            }
        }

        // --- REAL-TIME 2.5-SECOND FLUSH CHECK ---
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_sync_time) >= SYNC_INTERVAL_TICKS) {
            if (f != NULL) {
                // Request Mutex for the Flush operation
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

    // Create RTOS Objects
    data_queue = xQueueCreate(1000, sizeof(LogMessage_t));
    sd_card_mutex = xSemaphoreCreateMutex(); 

    // ---------------------------------------------------------------------
    // HARDWARE INIT: SPI BUS (FOR CAMERA)
    // ---------------------------------------------------------------------
    printf("[SYSTEM] Initializing SPI Bus for Camera...\n");
    spi_bus_config_t buscfg = {
        .miso_io_num = CAM_PIN_MISO,
        .mosi_io_num = CAM_PIN_MOSI,
        .sclk_io_num = CAM_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096 
    };
    
    // Initialize SPI2 Host 
    if(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) == ESP_OK) {
        // We configure the bus, but let the Arducam library handle the CS pin automatically
        printf("[SYSTEM] SPI Bus Initialized successfully.\n");
    } else {
        printf("[ERROR] Failed to initialize SPI bus.\n");
    }

    // ---------------------------------------------------------------------
    // HARDWARE INIT: SD CARD
    // ---------------------------------------------------------------------
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

    // ---------------------------------------------------------------------
    // SPAWN TASKS
    // ---------------------------------------------------------------------
    printf("[SYSTEM] Spawning Tasks...\n");
    xTaskCreate(heartbeat_task, "Heartbeat_Task", 4096, NULL, 1, NULL);
    
    // Spawn the Camera task with a larger stack memory size (8192) to handle image buffers
    xTaskCreate(camera_task, "Camera_Task", 8192, NULL, 3, NULL);

    printf("[SYSTEM] Initialization Complete. FreeRTOS Scheduler Active.\n");
    printf("=================================================\n\n");
}