// Cubby DeBry - 7/16/2026
// camera.c - functions for using sensor array as magnetic camera

#define BUFFER_SIZE 16

#define SPI_PORT spi0
#define PIN_MISO 0
#define PIN_SCK 2
#define PIN_MOSI 3

// Pico GPIO Chip Selects for ADCs
#define CS1 4
#define CS2 5
#define CS3 6
#define CS4 7
#define CS5 8

// ADC Input Control Bits
#define IN0 (0x0 << 3)
#define IN1 (0x1 << 3)
#define IN2 (0x2 << 3)
#define IN3 (0x3 << 3)
#define IN4 (0x4 << 3)
#define IN5 (0x5 << 3)
#define IN6 (0x6 << 3)
#define IN7 (0x7 << 3)

#include "tasks.h"
#include <stdbool.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "usb.h"
#include "FreeRTOS.h"
#include "task.h"

uint16_t spi_transfer16(uint16_t tx);

static uint adc_buffer[BUFFER_SIZE];
static const uint8_t input_ctrl[8] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN0};
static const uint8_t standard_map[8] = {3, 4, 5, 6, 7, 2, 1, 0};   // Map to physical order
static const uint8_t short_map[8] = {0, 1, 2, 3, 0, 0, 0, 0};      // For last ADC (only 4 inputs used)
static const uint8_t channel_ctrl[5] = {CS1, CS2, CS3, CS4, CS5};

// Get a single frame of 36 sensors
void get_frame(uint16_t* frame){
    const uint8_t (*map)[8];

    for(int i = 0; i < 5; i++){
        map = (i == 4) ? &short_map : &standard_map;
        gpio_put(channel_ctrl[i], 0);   // Activate ADC
        for(int j = 0; j < ((i == 4) ? 4 : 8); j++){
            uint16_t raw = spi_transfer16((uint16_t)input_ctrl[j] << 8);    // Read input, request next, map to physical order
            uint8_t mapped_idx = i*8 + (*map)[j];
            frame[mapped_idx] = raw;
        }
        gpio_put(channel_ctrl[i], 1);   // Deactivate ADC
    }
}

// Get one super sampled frame by averaging n_frames
void super_frame(uint32_t* s_frame, uint8_t n_frames){
    uint32_t frame_total[36] = {0};

    for(int i = 0; i < n_frames; i++){
        uint16_t frame[36];
        get_frame(frame);
        for(int j = 0; j < 36; j++){
            frame_total[j] += (uint32_t)frame[j];
        }
        // vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS/n_frames));
    }

    for(int i = 0; i < 36; i++){
        s_frame[i] = (uint32_t)((((uint64_t)frame_total[i]) << 20) / n_frames);
    }
}

// Magnetic Camera Task
void cam_task(void* params){
    bool toggle = false;
    uint32_t frame[36];
    uint16_t send[36];

    while(1) {
        // get_frame(frame);
        super_frame(frame, 64);
        for(int i = 0; i < 36; i++){
            send[i] = (uint16_t)(frame[i] >> 16);   // Q20 -> Q4
        }
    
        #if (DEBUG_MODE)
            send_frame_csv(send);
        #else   
            send_frame_binary(send);
        #endif

        vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));  // This still needs timing, should be delay FRAME_DELAY_MS minus frame read time
        // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, toggle);
        toggle = !toggle;
    }
}

// Initialize SPI bus for ADC communication
void spi_init_adc_bus(void){
    spi_init(SPI_PORT, 3.2 * 1000 * 1000); // 3.2 MHz
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    for (int i = 0; i < 5; i++){
        gpio_init(channel_ctrl[i]);
        gpio_set_dir(channel_ctrl[i], GPIO_OUT);
        gpio_put(channel_ctrl[i], 1); // Inactive (Active Low)
    }
}

// Read two bytes from SPI, write address of next read
uint16_t spi_transfer16(uint16_t tx){
    uint8_t tx_buf[2] = { (tx >> 8) & 0xFF, tx & 0xFF};
    uint8_t rx_buf[2];
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, 2);
    return ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
}