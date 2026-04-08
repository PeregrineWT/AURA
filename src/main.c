#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h" 

// =========================================================================
// HARDWARE PIN DEFINITIONS
// =========================================================================
#define BLINK_GPIO  47

// I2C BUS PINS (Matched to KiCad Schematic: SDA=IO2, SCL=IO1)
#define I2C_MASTER_SCL_IO           1      
#define I2C_MASTER_SDA_IO           2      
#define I2C_MASTER_NUM              0      
#define I2C_MASTER_FREQ_HZ          100000 // 100kHz Standard Mode
#define I2C_MASTER_TX_BUF_DISABLE   0      
#define I2C_MASTER_RX_BUF_DISABLE   0      
#define I2C_MASTER_TIMEOUT_MS       1000

// EEPROM Addresses for Board Detection
#define ADDR_EEPROM_IMU             0x50
#define ADDR_EEPROM_BUZZER          0x51
#define ADDR_EEPROM_WEATHER         0x57

// =========================================================================
// CORE DATA STRUCTURES
// =========================================================================
typedef struct {
    char sensor_id;      
    uint32_t timestamp;  
    float values[7];     
} LogMessage_t;

QueueHandle_t data_queue;

// Global flags for board presence
bool has_weather_board = false;
bool has_imu_board = false;
bool has_buzzer_board = false;

// =========================================================================
// UNIVERSAL I2C WRAPPER FUNCTIONS
// =========================================================================

esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, // Relying on external 4.7k PCB resistors
        .scl_pullup_en = GPIO_PULLUP_DISABLE, // Relying on external 4.7k PCB resistors
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
        
        // Simulating data queuing
        msg.sensor_id = 'H';
        msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        msg.values[0] = (float)led_state; 
        xQueueSend(data_queue, &msg, 0);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =========================================================================
// MAIN APPLICATION ENTRY POINT
// =========================================================================
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    printf("\n=================================================\n");
    printf("    --- AURA I2C DIAGNOSTIC MODE BOOTING ---     \n");
    printf("=================================================\n");

    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    data_queue = xQueueCreate(10, sizeof(LogMessage_t));

    // ---------------------------------------------------------------------
    // HARDWARE INIT: I2C BUS (100kHz Standard Mode)
    // ---------------------------------------------------------------------
    printf("[SYSTEM] Initializing I2C Bus (100kHz)...\n");
    if (i2c_master_init() == ESP_OK) {
        printf("[I2C] Bus ready. Scanning for EEPROMs...\n");

        // Detection Logic
        has_imu_board = (i2c_ping_device(ADDR_EEPROM_IMU) == ESP_OK);
        has_buzzer_board = (i2c_ping_device(ADDR_EEPROM_BUZZER) == ESP_OK);
        has_weather_board = (i2c_ping_device(ADDR_EEPROM_WEATHER) == ESP_OK);

        printf("[I2C] Detection Summary:\n");
        printf("      -> IMU Board (0x50):     %s\n", has_imu_board ? "DETECTED" : "MISSING");
        printf("      -> Buzzer Board (0x51):  %s\n", has_buzzer_board ? "DETECTED" : "MISSING");
        printf("      -> Weather Board (0x57): %s\n", has_weather_board ? "DETECTED" : "MISSING");
        
        // Full bus scan for any other sensors
        printf("[I2C] Full Scan of remaining sensors:\n");
        for (int i = 1; i < 127; i++) {
            if (i2c_ping_device(i) == ESP_OK) {
                printf("      -> Device found at: 0x%02X\n", i);
            }
        }
    } else {
        printf("[ERROR] Failed to initialize I2C bus.\n");
    }

    // ---------------------------------------------------------------------
    // SPAWN TASKS
    // ---------------------------------------------------------------------
    xTaskCreate(heartbeat_task, "Heartbeat_Task", 2048, NULL, 1, NULL);

    printf("[SYSTEM] Diagnostic Boot Complete.\n");
    printf("=================================================\n\n");
}