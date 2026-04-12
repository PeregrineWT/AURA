#include <stdio.h>
#include <string.h>
#include <stdarg.h> 
#include <stdbool.h>
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
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/ledc.h"

// --- ESP-IDF 5.x ADC Drivers ---
#include "esp_adc/adc_oneshot.h"

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

// Battery Reading Pin & Calibration
#define BATTERY_PIN 3 // Corresponds to ADC1_CHANNEL_2 on ESP32-S3
#define BATTERY_CALIBRATION_OFFSET 1.0f // Compensates for ESP32 ADC high-end droop (0.1V * 10)

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

// SERVO PINS + DUTY + BOOLEAN
#define SERVO_1_PIN 9
#define SERVO_2_PIN 14
#define SERVO_MIN_DUTY 800   
#define SERVO_MAX_DUTY 1600   

bool servo_open = false;

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

// --- EXCLUSIVE NEW TELEMETRY STRUCTURE ---
typedef enum {
    TLM_CMD = 0,   // Ground -> Drone Commands
    TLM_LOG = 1,   // Drone -> Ground Strings (printf mirror)
    TLM_SENSOR = 2 // Drone -> Ground High-Speed Sensor Data
} tlm_type_t;

typedef struct __attribute__((packed)) {
    uint8_t type; 
    union {
        char command;            
        char log_msg[200];       
        LogMessage_t sensor;     
    } payload;
} telemetry_packet_t;

QueueHandle_t data_queue;
QueueHandle_t buzzer_queue; 
SemaphoreHandle_t sd_card_mutex; 
char current_flight_log[64] = ""; 
spi_device_handle_t spi_cam_handle; 

// ADC Handle Global
adc_oneshot_unit_handle_t adc1_handle;

// UPDATE THIS IF YOUR GROUND MAC CHANGES
uint8_t ground_mac[] = {0x10,0xB4,0x1D,0xD0,0x9C,0x28};

// =========================================================================
// TELEMETRY HELPER FUNCTIONS
// =========================================================================
void send_telemetry_log(const char *format, ...) {
    telemetry_packet_t pkt;
    pkt.type = TLM_LOG;
    
    va_list args;
    va_start(args, format);
    vsnprintf(pkt.payload.log_msg, sizeof(pkt.payload.log_msg), format, args);
    va_end(args);
    
    size_t packet_size = sizeof(uint8_t) + strlen(pkt.payload.log_msg) + 1;
    esp_now_send(ground_mac, (uint8_t*)&pkt, packet_size);
}

#define TLOG(...) { printf(__VA_ARGS__); send_telemetry_log(__VA_ARGS__); }

// =========================================================================
// HARDWARE INITIALIZATIONS
// =========================================================================
void servo_init() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = 50, 
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch1 = {
        .gpio_num = SERVO_1_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config_t ch2 = {
        .gpio_num = SERVO_2_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&ch1);
    ledc_channel_config(&ch2);
}

void set_servos(bool open) {
    uint32_t duty = open ? SERVO_MAX_DUTY : SERVO_MIN_DUTY;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, SERVO_MAX_DUTY + SERVO_MIN_DUTY - duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    
    TLOG("[EVENT] SERVO STATE: %s\n", open ? "OPEN" : "CLOSED");
}

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

// Raw write for Buzzer Slave commands
esp_err_t i2c_write_raw(uint8_t dev_addr, uint8_t *data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
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
// HELPER: I2C BUS SCANNING & EEPROM CHECK
// =========================================================================
typedef struct {
    uint8_t address;
    const char *description;
} I2cDeviceDesc_t;

const I2cDeviceDesc_t aura_master_i2c_list[] = {
    {0x10, "Buzzer Board (S3 Command Slave)"}, 
    {0x20, "Buzzer Board (C3 Audio Slave)"}, 
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
        if (aura_master_i2c_list[i].address == addr) return aura_master_i2c_list[i].description;
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
    TLOG("\n[SYSTEM] Starting sequential I2C bus scan...\n");
    for (uint8_t i = 1; i < 0x78; i++) { 
        if (i == 0x20) continue; 
        if (check_i2c_device(i)) {
            TLOG(" -> Found device at 0x%02X (%s)\n", i, get_i2c_description(i));
            count++;
        }
    }
    TLOG("[SYSTEM] Scan complete. Found %d active devices.\n\n", count);
}

bool read_board_eeprom(uint8_t eeprom_addr, uint8_t expected_id) {
    AuraBoardEEPROM_t board_data;
    esp_err_t err = i2c_read_reg(eeprom_addr, 0x00, (uint8_t*)&board_data, sizeof(AuraBoardEEPROM_t));

    if (err == ESP_OK) {
        board_data.board_name[28] = '\0';
        TLOG(" -> [OK] EEPROM 0x%02X: %s (v%d.%d) | ID: %d\n", 
             eeprom_addr, board_data.board_name, board_data.hw_version_major, board_data.hw_version_minor, board_data.unique_id);
        if (board_data.unique_id == expected_id) {
            return true;
        } else {
            TLOG("      Status: [FAIL] ID Mismatch! Expected %d.\n", expected_id);
            return false;
        }
    } else {
        TLOG(" -> [FAIL] EEPROM at 0x%02X did not respond.\n", eeprom_addr);
        return false;
    }
}

// ================= WIFI INIT & NEW TELEMETRY CALLBACKS =================
void wifi_init() {
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
}

void receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len != sizeof(telemetry_packet_t)) return;

    telemetry_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.type == TLM_CMD && pkt.payload.command != '\0') {
        char cmd_char = pkt.payload.command;
        
        LogMessage_t btn_msg;
        btn_msg.sensor_id = 'B';
        btn_msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        btn_msg.values[0] = (cmd_char == 'f') ? 1.0f : 0.0f; // Btn_Up (Song 1)
        btn_msg.values[1] = (cmd_char == 'm') ? 1.0f : 0.0f; // Btn_Down (Song 2)
        btn_msg.values[2] = (cmd_char == 's') ? 1.0f : 0.0f; // Btn_Left (Stop)
        btn_msg.values[3] = (cmd_char == 'c') ? 1.0f : 0.0f; // Btn_Right (Servo)
        
        xQueueSendFromISR(data_queue, &btn_msg, NULL); 

        switch(cmd_char) {  
            case 's': 
            case 'f': 
            case 'm': 
            case 'i': 
                xQueueSendFromISR(buzzer_queue, &cmd_char, NULL); 
                break;
            case 'c': 
                servo_open = !servo_open;
                set_servos(servo_open);
                break;
        }
    }
}

void send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    // Keep silent
}

// =========================================================================
// HELPER: ULTRASONIC PING & SD GEN
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
    TLOG("[SYSTEM] Auto-generated log file: %s\n", current_flight_log);
}

// =========================================================================
// SENSOR TASKS 
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

void battery_task(void *pvParameter) {
    LogMessage_t msg;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); 

    while(1) {
        int adc_raw;
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw);
        
        float v_pin = (adc_raw / 4095.0f) * 3.3f;
        
        // Multiplier + Calibration Offset applied here!
        float v_batt = (v_pin * 10.0f) + BATTERY_CALIBRATION_OFFSET; 
        
        float percentage = ((v_batt - 19.8f) / (25.2f - 19.8f)) * 100.0f;
        if (percentage > 100.0f) percentage = 100.0f;
        if (percentage < 0.0f) percentage = 0.0f;

        msg.sensor_id = 'E'; 
        msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        msg.values[0] = v_batt;
        msg.values[1] = percentage;
        
        xQueueSend(data_queue, &msg, 0);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

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
    char cmd;
    uint8_t i2c_cmd = 0x00;
    
    while(1) {
        if (xQueueReceive(buzzer_queue, &cmd, portMAX_DELAY) == pdPASS) {
            if (cmd == 's') { 
                i2c_cmd = 0x00; 
                TLOG("[EVENT] BUZZER: Stopping Playback\n"); 
            }
            else if (cmd == 'f') { 
                i2c_cmd = 0x01; 
                TLOG("[EVENT] BUZZER: Playing Song 1\n"); 
            }
            else if (cmd == 'm') { 
                i2c_cmd = 0x02; 
                TLOG("[EVENT] BUZZER: Playing Song 2\n"); 
            }
            else if (cmd == 'i') { 
                i2c_cmd = 0x03; 
                TLOG("[EVENT] BUZZER: Playing Charge (Startup)\n"); 
            }

            int retries = 0;
            esp_err_t err = ESP_FAIL;
            
            while (err != ESP_OK && retries < 3) {
                err = i2c_write_raw(0x20, &i2c_cmd, 1);
                if (err != ESP_OK) {
                    TLOG("[WARNING] Buzzer at 0x20 missed command. Retrying...\n");
                    vTaskDelay(pdMS_TO_TICKS(50));
                    retries++;
                }
            }

            if (err == ESP_OK) {
                TLOG("[EVENT] BUZZER I2C COMMAND SUCCESS: 0x%02X\n", i2c_cmd);
            } else {
                TLOG("[ERROR] BUZZER I2C FAILED AFTER 3 RETRIES\n");
            }
        }
    }
}

// =========================================================================
// FEATURE TASK: CAMERA CAPTURE 
// =========================================================================
void camera_task(void *pvParameters) {
    gpio_set_direction(CAM_PIN_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PIN_CS, 1); 
    vTaskDelay(pdMS_TO_TICKS(10));

    // The overly-aggressive manual SPI probe has been removed here.
    // The hardware MISO pull-up in app_main is sufficient to prevent Watchdog crashes!

    ArducamCamera myCAM = createArducamCamera(CAM_PIN_CS);
    begin(&myCAM);
    
    if (myCAM.cameraId == 0x00 || myCAM.cameraId == 0xFF) {
        TLOG("[FATAL] Camera unresponsive. Received ID: 0x%02X\n", myCAM.cameraId);
        vTaskDelete(NULL); 
    }

    TLOG("[CAMERA] Ping successful. Initializing Arducam Mega...\n");

    int image_counter = 1;
    char image_filename[64];
    uint8_t *image_buffer = (uint8_t *)heap_caps_malloc(4096, MALLOC_CAP_DMA);
    if (image_buffer == NULL) vTaskDelete(NULL);

    const TickType_t xFrequency = pdMS_TO_TICKS(3000); 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
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
                    
                    LogMessage_t cam_msg;
                    cam_msg.sensor_id = 'C';
                    cam_msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    cam_msg.values[0] = (float)image_counter;
                    xQueueSend(data_queue, &cam_msg, 0);

                    TLOG("[EVENT] CAMERA SAVED %s (%lu bytes)\n", image_filename, image_len);
                    image_counter++;
                }
                xSemaphoreGive(sd_card_mutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =========================================================================
// SYSTEM TASK: REAL-TIME SD LOGGER & TELEMETRY BROADCASTER
// =========================================================================
void telemetry_logger_task(void *pvParameters) {
    LogMessage_t msg;
    FILE *f = NULL;
    
    if (strlen(current_flight_log) > 0) {
        f = fopen(current_flight_log, "a");
        if (f != NULL) {
            fprintf(f, "Time_HB,State_HB,Time_IMU,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Time_WTR,Temp_C,Light_Vis,Light_IR,Baro_Pa,Time_US,US_Front,US_Back,US_Left,US_Right,Time_BTN,Btn_Up,Btn_Down,Btn_Left,Btn_Right,Time_CAM,Cam_Event,Time_BAT,Bat_Volt,Bat_Pct\n"); 
            fclose(f); 
            f = fopen(current_flight_log, "a");
        }
    }
    
    TickType_t last_sync_time = xTaskGetTickCount();
    const TickType_t SYNC_INTERVAL_TICKS = pdMS_TO_TICKS(2500); 

    TLOG("[TELEMETRY] Broadcaster active. Waiting for sensor data...\n");
    
    telemetry_packet_t pkt;
    pkt.type = TLM_SENSOR;

    while (1) {
        if (xQueueReceive(data_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
            
            // --- 1. HIGH-SPEED TELEMETRY TRANSMISSION (Unconditional) ---
            memcpy(&pkt.payload.sensor, &msg, sizeof(LogMessage_t));
            esp_now_send(ground_mac, (uint8_t*)&pkt, sizeof(uint8_t) + sizeof(LogMessage_t));

            // --- 2. LOCAL SD FAT32 LOGGING (Conditional on SD Presence) ---
            if (f != NULL) {
                if (xSemaphoreTake(sd_card_mutex, portMAX_DELAY) == pdTRUE) {
                    switch (msg.sensor_id) {
                        case 'H': fprintf(f, "%lu,%d\n", msg.timestamp, (int)msg.values[0]); break;
                        case 'I': fprintf(f, ",,%lu,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f\n", msg.timestamp, msg.values[0], msg.values[1], msg.values[2], msg.values[3], msg.values[4], msg.values[5]); break;
                        case 'W': fprintf(f, ",,,,,,,,,%lu,%.2f,%.0f,%.0f,%.2f\n", msg.timestamp, msg.values[0], msg.values[1], msg.values[2], msg.values[3]); break;
                        case 'U': fprintf(f, ",,,,,,,,,,,,,,%lu,%.2f,%.2f,%.2f,%.2f\n", msg.timestamp, msg.values[0], msg.values[1], msg.values[2], msg.values[3]); break;
                        case 'B': fprintf(f, ",,,,,,,,,,,,,,,,,,,%lu,%.0f,%.0f,%.0f,%.0f\n", msg.timestamp, msg.values[0], msg.values[1], msg.values[2], msg.values[3]); break;
                        case 'C': fprintf(f, ",,,,,,,,,,,,,,,,,,,,,,,,%lu,img_%04d.jpg taken!\n", msg.timestamp, (int)msg.values[0]); break;
                        case 'E': fprintf(f, ",,,,,,,,,,,,,,,,,,,,,,,,,,%lu,%.2f,%.1f\n", msg.timestamp, msg.values[0], msg.values[1]); break;
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

    nvs_flash_init();
    wifi_init();
    esp_now_init();
    esp_now_register_recv_cb(receive_cb);
    esp_now_register_send_cb(send_cb);
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, ground_mac, 6);
    peer.channel = 1;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    TLOG("\n=================================================\n");
    TLOG("     --- AURA FLIGHT CONTROLLER BOOTING ---      \n");
    TLOG("=================================================\n");
    
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    servo_init();
    set_servos(false); 

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, 
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config));

    sd_card_mutex = xSemaphoreCreateMutex(); 
    data_queue = xQueueCreate(1000, sizeof(LogMessage_t));
    buzzer_queue = xQueueCreate(10, sizeof(char));

    TLOG("[SYSTEM] Initializing SD Card...\n");
    gpio_reset_pin(PIN_SD_CMD); gpio_reset_pin(PIN_SD_CLK); gpio_reset_pin(PIN_SD_D0);
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {.format_if_mount_failed = false, .max_files = 5, .allocation_unit_size = 16 * 1024};
    sdmmc_host_t host = SDMMC_HOST_DEFAULT(); host.flags = SDMMC_HOST_FLAG_1BIT; 
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = PIN_SD_CLK; slot_config.cmd = PIN_SD_CMD; slot_config.d0  = PIN_SD_D0; slot_config.width = 1;

    sdmmc_card_t *card;
    if (esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card) == ESP_OK) {
        TLOG("[SYSTEM] SD Card mounted successfully!\n");
        generate_next_log_filename();
    } else {
        TLOG("[WARNING] SD Card failed to mount. Running in Telemetry-Only Mode.\n");
    }
    
    xTaskCreate(telemetry_logger_task, "TLM_Log_Task", 4096, NULL, 4, NULL);

    TLOG("[SYSTEM] Initializing SPI Bus for Camera...\n");
    
    spi_bus_config_t buscfg = {.miso_io_num = CAM_PIN_MISO, .mosi_io_num = CAM_PIN_MOSI, .sclk_io_num = CAM_PIN_SCK, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4096};
    if(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) == ESP_OK) {
        
        // Re-apply MISO Pull-up after spi_bus_initialize to prevent watchdog crash when unplugged
        gpio_set_pull_mode(CAM_PIN_MISO, GPIO_PULLUP_ONLY); 
        
        spi_device_interface_config_t devcfg = {.clock_speed_hz = 4000000, .mode = 0, .spics_io_num = -1, .queue_size = 7};
        spi_bus_add_device(SPI2_HOST, &devcfg, &spi_cam_handle);
        xTaskCreate(camera_task, "Camera_Task", 8192, NULL, 3, NULL);
    } else {
        TLOG("[ERROR] Failed to initialize SPI bus.\n");
    }

    if (i2c_master_init() == ESP_OK) {
        TLOG("[SYSTEM] I2C Bus Active.\n");
        i2c_bus_scan_sequential();
        
        if (check_i2c_device(ADDR_EEPROM_IMU) && read_board_eeprom(ADDR_EEPROM_IMU, EXPECTED_ID_IMU)) {
            TLOG("[SYSTEM] IMU Board Validated. Spawning task...\n");
            xTaskCreate(imu_task, "IMU_Task", 4096, NULL, 5, NULL);
        }

        if (check_i2c_device(ADDR_EEPROM_WEATHER) && read_board_eeprom(ADDR_EEPROM_WEATHER, EXPECTED_ID_WEATHER)) {
            TLOG("[SYSTEM] Weather Board Validated. Spawning task...\n");
            xTaskCreate(weather_board_task, "Weather_Task", 4096, NULL, 5, NULL);
        }

        TLOG("\nChecking for Buzzer Board EEPROM...\n");
        if (check_i2c_device(ADDR_EEPROM_BUZZER)) {
            read_board_eeprom(ADDR_EEPROM_BUZZER, EXPECTED_ID_BUZZER);
        } else {
            TLOG("[INFO] Buzzer EEPROM not ready yet. Proceeding anyway.\n");
        }
        
        TLOG("[SYSTEM] Spawning Buzzer Listener Task...\n");
        xTaskCreate(buzzer_task, "Buzzer_Task", 4096, NULL, 5, NULL);
        
    } else {
        TLOG("[ERROR] Failed to initialize I2C bus.\n");
    }

    TLOG("\n[SYSTEM] Initializing Ultrasonic Array...\n");
    gpio_set_direction(TRIG_A, GPIO_MODE_OUTPUT); gpio_set_direction(TRIG_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRIG_C, GPIO_MODE_OUTPUT); gpio_set_direction(TRIG_D, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_A, GPIO_MODE_INPUT); gpio_set_direction(ECHO_B, GPIO_MODE_INPUT);
    gpio_set_direction(ECHO_C, GPIO_MODE_INPUT); gpio_set_direction(ECHO_D, GPIO_MODE_INPUT); 
    
    xTaskCreate(ultrasonic_task, "Ultra_Task", 4096, NULL, 5, NULL);
    xTaskCreate(battery_task, "Battery_Task", 4096, NULL, 5, NULL); 
    xTaskCreate(heartbeat_task, "Heartbeat_Task", 4096, NULL, 1, NULL);

    TLOG("\n=================================================\n");
    TLOG("[SYSTEM] Initialization Complete. Scheduler Active.\n");
    TLOG("=================================================\n\n");

    // --- PLAY STARTUP SONG ---
    char startup_cmd = 'i'; 
    xQueueSend(buzzer_queue, &startup_cmd, portMAX_DELAY);

    vTaskDelete(NULL);
}