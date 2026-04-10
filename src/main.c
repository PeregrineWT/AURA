#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

// Arducam Libraries
#include "ArducamCamera.h" 
#include "esp32Hal.h" 

// --- PINS ---
#define PIN_SD_CMD  41  
#define PIN_SD_CLK  40  
#define PIN_SD_D0   39  
#define MOUNT_POINT "/sdcard"

#define CAM_PIN_CS   10
#define CAM_PIN_MOSI 11
#define CAM_PIN_MISO 13
#define CAM_PIN_SCK  12

spi_device_handle_t spi_cam_handle; 

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    printf("\n=================================================\n");
    printf("     --- ARDUCAM BARE-METAL HARDWARE TEST ---    \n");
    printf("=================================================\n");

    // ---------------------------------------------------------
    // 1. MOUNT SD CARD
    // ---------------------------------------------------------
    printf("Mounting SD Card...\n");
    gpio_set_direction(PIN_SD_CMD, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_SD_CLK, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_SD_D0,  GPIO_MODE_INPUT_OUTPUT);

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT; 
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = PIN_SD_CLK; 
    slot_config.cmd = PIN_SD_CMD; 
    slot_config.d0  = PIN_SD_D0; 
    slot_config.width = 1;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false, 
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_card_t *card;
    if (esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card) != ESP_OK) {
        printf("[ERROR] SD Card Mount Failed! Cannot save images.\n");
        return; 
    }
    printf("SD Card Mounted Successfully.\n");

    // ---------------------------------------------------------
    // 2. INITIALIZE SPI BUS
    // ---------------------------------------------------------
    printf("Initializing SPI Bus...\n");
    spi_bus_config_t buscfg = {
        .miso_io_num = CAM_PIN_MISO,
        .mosi_io_num = CAM_PIN_MOSI,
        .sclk_io_num = CAM_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096 
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4000000, // 4MHz for max stability
        .mode = 0,                 
        .spics_io_num = -1, 
        .queue_size = 7,
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi_cam_handle);

    // ---------------------------------------------------------
    // 3. ARDUCAM INITIALIZATION & TRUE PING
    // ---------------------------------------------------------
    printf("Initializing Arducam Mega Library...\n");
    
    // Explicitly set CS high to start
    gpio_set_direction(CAM_PIN_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PIN_CS, 1); 
    vTaskDelay(pdMS_TO_TICKS(10));

    ArducamCamera myCAM = createArducamCamera(CAM_PIN_CS);
    begin(&myCAM); // This runs the 3-byte protocol under the hood

    // Check if the camera successfully populated its ID struct
    if (myCAM.cameraId == 0x00 || myCAM.cameraId == 0xFF) {
        printf("\n=================================================\n");
        printf("[FATAL] CAMERA DEAD OR UNPLUGGED.\n");
        printf("The 3-byte command failed. Received ID: 0x%02X\n", myCAM.cameraId);
        printf("=================================================\n");
        return; 
    }

    printf("\n[SUCCESS] Camera Alive & Responding!\n");
    printf(" -> Sensor Model ID: 0x%02X\n", myCAM.cameraId);
    printf(" -> Firmware Date: 20%02d-%02d-%02d\n", 
           myCAM.verDateAndNumber[0], myCAM.verDateAndNumber[1], myCAM.verDateAndNumber[2]);
    printf("=================================================\n");

    uint8_t *image_buffer = (uint8_t *)heap_caps_malloc(4096, MALLOC_CAP_DMA);
    if (image_buffer == NULL) {
        printf("[FATAL] Memory allocation failed!\n");
        return;
    }

    // ---------------------------------------------------------
    // 4. BARE MINIMUM 3-SECOND CAPTURE LOOP
    // ---------------------------------------------------------
    int image_counter = 1;
    char image_filename[64];

    printf("\n--- STARTING 3-SECOND CAPTURE LOOP ---\n");

    while (1) {
        printf("Capturing Frame %d...\n", image_counter);

        takePicture(&myCAM, CAM_IMAGE_MODE_FHD, CAM_IMAGE_PIX_FMT_JPG);
        uint32_t image_len = myCAM.totalLength; 

        if (image_len > 0) {
            snprintf(image_filename, sizeof(image_filename), MOUNT_POINT "/test_%d.jpg", image_counter);
            
            FILE *img_file = fopen(image_filename, "wb"); 
            if (img_file != NULL) {
                uint32_t bytes_read = 0;
                while (bytes_read < image_len) {
                    uint16_t chunk_size = readBuff(&myCAM, image_buffer, 4096);
                    fwrite(image_buffer, 1, chunk_size, img_file);
                    bytes_read += chunk_size;
                    
                    vTaskDelay(pdMS_TO_TICKS(1)); // Watchdog breathing room
                }
                fclose(img_file);
                printf(" -> Saved %s (%lu bytes)\n", image_filename, image_len);
                image_counter++;
            } else {
                printf(" -> [ERROR] Failed to create file on SD card.\n");
            }
        } else {
            printf(" -> [ERROR] Camera took a 0-byte image.\n");
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}