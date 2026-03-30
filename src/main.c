#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BLINK_GPIO 47

// The dedicated FreeRTOS Heartbeat Task
void heartbeat_task(void *pvParameters) {
    int led_state = 0;

    // 1. Initialize the xLastWakeTime variable with the current tick count.
    // This provides the baseline time for vTaskDelayUntil to calculate from.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // Define the exact frequency we want to run at (1000ms)
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);

    while (1) {
        // Toggle the LED
        led_state = !led_state;
        gpio_set_level(BLINK_GPIO, led_state);

        // 2. Pause the task until exactly 1000ms have passed since the last wake time.
        // The RTOS automatically updates xLastWakeTime behind the scenes here.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main(void) {
    // 1. Configure the GPIO hardware before spawning the task
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // 2. Spawn the Heartbeat Task
    // xTaskCreate parameters: Function, Name, Stack Size (bytes), Parameter, Priority, Task Handle
    xTaskCreate(heartbeat_task, "Heartbeat_Task", 2048, NULL, 1, NULL);
    
    // app_main naturally returns here, but the FreeRTOS scheduler keeps the heartbeat_task alive forever.
}