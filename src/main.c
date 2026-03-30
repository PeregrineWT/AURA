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
// DEFINITIONS & GLOBALS
// =========================================================================
#define BLINK_GPIO  47
#define PIN_SD_CMD  41  
#define PIN_SD_CLK  40  
#define PIN_SD_D0   39  
#define MOUNT_POINT "/sdcard"

// The Data Packet Structure
typedef struct {
    char task_id;
    uint32_t iteration;
    uint32_t timestamp;
} LogMessage_t;

QueueHandle_t data_queue;
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
            if (sscanf(ent->d_name, "flight_%d.csv", &num) == 1) {
                if (num > max_num) max_num = num;
            }
        }
        closedir(dir);
    }
    snprintf(current_flight_log, sizeof(current_flight_log), MOUNT_POINT "/flight_%03d.csv", max_num + 1);
    printf("[SYSTEM] Auto-generated log file: %s\n", current_flight_log);
}

// =========================================================================
// TASK 1: THE BACKGROUND HEARTBEAT
// =========================================================================
void heartbeat_task(void *pvParameters) {
    int counter = 0;
    int led_state = 0;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Exactly 1 Hz

    while (1) {
        led_state = !led_state;
        gpio_set_level(BLINK_GPIO, led_state);
        
        printf("[HEARTBEAT] Tick: %d\n", counter);
        counter++;
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =========================================================================
// THE PRODUCER: GENERIC TASK GENERATOR
// =========================================================================
void generic_rtos_task(char id, int hz) {
    LogMessage_t msg;
    msg.task_id = id;
    msg.iteration = 1;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / hz); 

    while (1) {
        msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        printf("Task %c: %lu, %lu\n", msg.task_id, msg.iteration, msg.timestamp);
        
        // Send to queue (0 block time so we don't stall the timer if full)
        xQueueSend(data_queue, &msg, 0);
        
        msg.iteration++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void task_A(void *pvP) { generic_rtos_task('A', 1); }
void task_B(void *pvP) { generic_rtos_task('B', 5); }
void task_C(void *pvP) { generic_rtos_task('C', 10); }
void task_D(void *pvP) { generic_rtos_task('D', 15); }
void task_E(void *pvP) { generic_rtos_task('E', 25); }

// =========================================================================
// THE CONSUMER: REAL-TIME SD LOGGER TASK
// =========================================================================
void sd_logger_task(void *pvParameters) {
    LogMessage_t msg;
    
    FILE *f = fopen(current_flight_log, "a");
    if (f == NULL) vTaskDelete(NULL);
    fprintf(f, "ID_A,Iter_A,Time_A,ID_B,Iter_B,Time_B,ID_C,Iter_C,Time_C,ID_D,Iter_D,Time_D,ID_E,Iter_E,Time_E\n");
    fclose(f); 

    f = fopen(current_flight_log, "a");

    TickType_t last_sync_time = xTaskGetTickCount();
    const TickType_t SYNC_INTERVAL_TICKS = pdMS_TO_TICKS(5000); // 5 seconds

    while (1) {
        // Wait max 100ms for data, then wake up to check the hardware clock
        if (xQueueReceive(data_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
            
            if (f != NULL) {
                switch (msg.task_id) {
                    case 'A': fprintf(f, "Task A,%lu,%lu,,,,,,,,,,,,\n", msg.iteration, msg.timestamp); break;
                    case 'B': fprintf(f, ",,,Task B,%lu,%lu,,,,,,,,,\n", msg.iteration, msg.timestamp); break;
                    case 'C': fprintf(f, ",,,,,,Task C,%lu,%lu,,,,,,\n", msg.iteration, msg.timestamp); break;
                    case 'D': fprintf(f, ",,,,,,,,,Task D,%lu,%lu,,,\n", msg.iteration, msg.timestamp); break;
                    case 'E': fprintf(f, ",,,,,,,,,,,,Task E,%lu,%lu\n", msg.iteration, msg.timestamp); break;
                }
            }
        }

        // --- REAL-TIME SYNC CHECK ---
        TickType_t current_time = xTaskGetTickCount();
        
        if ((current_time - last_sync_time) >= SYNC_INTERVAL_TICKS) {
            if (f != NULL) {
                fclose(f); 
                f = fopen(current_flight_log, "a"); 
                printf("[SD_CARD] --- Real-Time 5 Second Sync Complete ---\n");
            }
            last_sync_time = current_time; 
        }
    }
}

// =========================================================================
// MAIN APPLICATION
// =========================================================================
void app_main(void) {
    // 3 Second Boot Delay to catch serial monitor connections
    vTaskDelay(pdMS_TO_TICKS(3000)); 
    printf("\n--- Advanced RTOS Architecture Test Booting ---\n");

    // Initialize Heartbeat Hardware
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Create the Queue (1000 slots)
    data_queue = xQueueCreate(1000, sizeof(LogMessage_t));

    // Initialize SD Card Hardware
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
    slot_config.clk = PIN_SD_CLK; slot_config.cmd = PIN_SD_CMD; slot_config.d0 = PIN_SD_D0; slot_config.width = 1;

    sdmmc_card_t *card;
    if (esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card) == ESP_OK) {
        printf("[SYSTEM] SD Card mounted successfully!\n");
        generate_next_log_filename();
        xTaskCreate(sd_logger_task, "SD_Logger", 4096, NULL, 4, NULL);
    } else {
        printf("[ERROR] SD Card failed. Test will run console-only.\n");
    }

    // Spawn the RTOS Tasks
    printf("[SYSTEM] Spawning RTOS Tasks...\n");
    xTaskCreate(heartbeat_task, "Heartbeat", 4096, NULL, 5, NULL);
    xTaskCreate(task_A, "Task_A", 4096, NULL, 5, NULL);
    xTaskCreate(task_B, "Task_B", 4096, NULL, 5, NULL);
    xTaskCreate(task_C, "Task_C", 4096, NULL, 5, NULL);
    xTaskCreate(task_D, "Task_D", 4096, NULL, 5, NULL);
    xTaskCreate(task_E, "Task_E", 4096, NULL, 5, NULL);

    printf("[SYSTEM] Scheduler taking over.\n");
}