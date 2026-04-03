#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "rom/ets_sys.h" // Required for microsecond delays

// =========================================================================
// HARDWARE PIN DEFINITIONS
// =========================================================================
#define TRIG_A 5
#define TRIG_B 4
#define TRIG_C 7
#define TRIG_D 6

#define ECHO_A 16
#define ECHO_B 15
#define ECHO_C 18
#define ECHO_D 17

// =========================================================================
// ULTRASONIC MEASUREMENT FUNCTION
// =========================================================================
float read_ultrasonic(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    // 1. Ensure trigger is low to start with a clean slate
    gpio_set_level(trig_pin, 0);
    ets_delay_us(2); 

    // 2. Fire the 10-microsecond trigger pulse
    gpio_set_level(trig_pin, 1);
    ets_delay_us(10);
    gpio_set_level(trig_pin, 0);

    // 3. Wait for the Echo pin to go HIGH (Start of the ping)
    // We use a 30ms timeout. If a sensor is unplugged, we don't want the ESP32 to freeze forever.
    int64_t timeout_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0) {
        if ((esp_timer_get_time() - timeout_start) > 30000) {
            return -1.0; // Timeout error
        }
    }
    int64_t pulse_start = esp_timer_get_time();

    // 4. Wait for the Echo pin to go LOW (End of the ping)
    while (gpio_get_level(echo_pin) == 1) {
        if ((esp_timer_get_time() - pulse_start) > 30000) {
            return -1.0; // Timeout error
        }
    }
    int64_t pulse_end = esp_timer_get_time();

    // 5. Calculate the distance
    // Time = pulse_end - pulse_start (in microseconds)
    // Speed of sound = 0.0343 cm/microsecond
    // Distance = (Time * Speed) / 2 (since it travels there and back)
    float time_us = (float)(pulse_end - pulse_start);
    float distance_cm = (time_us * 0.0343f) / 2.0f;

    return distance_cm;
}

// =========================================================================
// MAIN APPLICATION ENTRY POINT
// =========================================================================
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    printf("\n--- STANDALONE ULTRASONIC TEST ---\n");

    // Configure Triggers as Outputs
    gpio_set_direction(TRIG_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRIG_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRIG_C, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRIG_D, GPIO_MODE_OUTPUT);

    // Configure Echos as Inputs
    gpio_set_direction(ECHO_A, GPIO_MODE_INPUT);
    gpio_set_direction(ECHO_B, GPIO_MODE_INPUT);
    gpio_set_direction(ECHO_C, GPIO_MODE_INPUT);
    gpio_set_direction(ECHO_D, GPIO_MODE_INPUT);

    float dist_a, dist_b, dist_c, dist_d;

    while (1) {
        // Read all four sensors sequentially
        dist_a = read_ultrasonic(TRIG_A, ECHO_A);
        dist_b = read_ultrasonic(TRIG_B, ECHO_B);
        dist_c = read_ultrasonic(TRIG_C, ECHO_C);
        dist_d = read_ultrasonic(TRIG_D, ECHO_D);

        // Print the results 
        // Using %.1f to print just one decimal place (e.g., 15.2 cm)
        printf("A: %5.1f cm | B: %5.1f cm | C: %5.1f cm | D: %5.1f cm\n", 
               dist_a, dist_b, dist_c, dist_d);

        // Wait 100ms before pinging again to prevent sound waves from bouncing 
        // around the room and interfering with the next read.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}