#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
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

// =========================================================================
// CORE DATA STRUCTURES
// =========================================================================
typedef struct {
    char sensor_id;      // Unique character for the sensor (e.g., 'H' for Heartbeat)
    uint32_t timestamp;  // The RTOS tick time the reading was taken
    float values[7];     // Array sized for max-case IMU (3 Gyro, 3 Accel, 1 Temp)
} LogMessage_t;

QueueHandle_t data_queue;
char current_flight_log[64];

// =========================================================================
// HELPER: AUTO-INCREMENT LOG GENERATOR (8.3 FAT32 Safe)
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
        // Toggle the LED
        led_state = !led_state;
        gpio_set_level(BLINK_GPIO, led_state);

        // Print the current state to serial
        if (led_state) {
            printf("[HEARTBEAT] LED On\n");
        } else {
            printf("[HEARTBEAT] LED Off\n");
        }

        // --- PACKAGE AND SEND TO SD CARD ---
        msg.sensor_id = 'H';
        msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Store the state in the very first slot of the float array
        msg.values[0] = (float)led_state; 
        
        // Zero out the rest of the array just to keep memory pristine
        for(int i = 1; i < 7; i++) {
            msg.values[i] = 0.0f;
        }
        
        xQueueSend(data_queue, &msg, 0);

        // Sleep deterministically
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
    
    // Header remains unchanged for now, as requested.
    fprintf(f, "Time_HB,State_HB\n"); 
    fclose(f); 

    f = fopen(current_flight_log, "a");

    TickType_t last_sync_time = xTaskGetTickCount();
    const TickType_t SYNC_INTERVAL_TICKS = pdMS_TO_TICKS(2500); 

    printf("[SD_CARD] Logger task started. Waiting for sensor data...\n");

    while (1) {
        if (xQueueReceive(data_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
            
            if (f != NULL) {
                switch (msg.sensor_id) {
                    
                    case 'S': 
                        fprintf(f, "%lu,System Event Logged\n", msg.timestamp); // Useless code, no S events 
                        break; 

                    // Heartbeat case now pulls from values[0]
                    case 'H': 
                        fprintf(f, "%lu,%d\n", msg.timestamp, (int)msg.values[0]); 
                        break;
                }
            }
        }

        // --- REAL-TIME 2.5-SECOND FLUSH CHECK ---
        TickType_t current_time = xTaskGetTickCount();
        
        if ((current_time - last_sync_time) >= SYNC_INTERVAL_TICKS) {
            if (f != NULL) {
                fclose(f);
                printf("MicroSD Closed to save\n"); 
                f = fopen(current_flight_log, "a"); 
                printf("MicroSD reopened\n");
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

    // Queue allocation automatically adjusts to the new 36-byte LogMessage_t size
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

    printf("[SYSTEM] Spawning Heartbeat Task...\n");
    xTaskCreate(heartbeat_task, "Heartbeat_Task", 4096, NULL, 1, NULL);

    printf("[SYSTEM] Initialization Complete. FreeRTOS Scheduler Active.\n");
    printf("=================================================\n\n");
}