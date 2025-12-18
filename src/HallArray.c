// HallArray.c 
// Cubby DeBry 2025

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "NCC.h"

#define BUFFER_SIZE 16

#define SPI_PORT spi0
#define PIN_MISO 0
#define PIN_SCK 2
#define PIN_MOSI 3

#define CS1 4
#define CS2 5
#define CS3 6
#define CS4 7
#define CS5 8


#define IN0 (0x0 << 3)
#define IN1 (0x1 << 3)
#define IN2 (0x2 << 3)
#define IN3 (0x3 << 3)
#define IN4 (0x4 << 3)
#define IN5 (0x7 << 3)
#define IN6 (0x6 << 3)
#define IN7 (0x5 << 3)

static uint adc_buffer[BUFFER_SIZE];
static const uint8_t input_ctrl[9] = {IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7, 0}; // Last entry is dummy for easier indexing
static const uint8_t standard_map[8] = {3, 4, 5, 6, 7, 0, 1, 2};   // Map to physical order
static const uint8_t short_map[8] = {0, 1, 2, 3, 0, 0, 0, 0};      // For last ADC (only 4 inputs used)
static const uint8_t channel_ctrl[5] = {CS1, CS2, CS3, CS4, CS5};

void spi_init_adc_bus(void){
    spi_init(SPI_PORT, 1 * 1000 * 1000); // 1 MHz
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

void adc_init(void){

}

uint16_t spi_transfer16(uint16_t tx){
    uint8_t tx_buf[2] = { (tx >> 8) & 0xFF, tx & 0xFF};
    uint8_t rx_buf[2];
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, 2);
    return ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
}

void main_task(__unused void *params) {
    uint16_t buff[2][40];
    uint16_t *framePrev = buff[0];
    uint16_t *frameCurr = buff[0];
    int8_t dx[10];
    int8_t dy[10];
    const uint8_t (*map)[8];
    bool toggle = false;    // Double buffer, probably need semaphore later
    printf("Initializing...\n");
    for(int i = 0; i < 10; i++){
        dx[i] = 0;
        dy[i] = 0;
    }
    while(1) {
        for(int i = 0; i < 5; i++){
            map = (i == 4) ? &short_map : &standard_map;
            gpio_put(channel_ctrl[i], 0);   // Activate ADC
            for(int j = 0; j < 8; j++){
                buff[toggle][i*8 + (*map)[j]] = spi_transfer16((uint16_t)input_ctrl[j + 1] << 8);    // Read input, request next, map to physical order
            }
            gpio_put(channel_ctrl[i], 1);   // Deactivate ADC
        }

        // Test NCC functions
        framePrev = frameCurr;
        frameCurr = buff[toggle];
        float high_score = 0;
        int8_t high_u = 0;
        int8_t high_v = 0;
        overlap_t ov;

        for(int u = -5; u < 6; u++){
            for(int v = -5; v < 6; v++){
                if(ncc_compute_overlap(u, v, &ov)){
                    float score = ncc_score(u, v, framePrev, frameCurr, &ov);
                    // printf("NCC Score at shift (%d,%d): %f\n", u, v, score);
                    if(score > high_score){
                        high_score = score;
                        high_u = u;
                        high_v = v;
                    }
                }
            }
        }

        // for(int i = 0; i < 9; i++){
        //     dx[9 - i] = dx[8 - i];
        //     dy[9 - i] = dy[8 - i];
        // }

        // dx[0] = high_u;
        // dy[0] = high_v;

        // int8_t tot_x = 0;
        // int8_t tot_y = 0;

        // for(int i = 0; i < 10; i++){
        //     tot_x += dx[i];
        //     tot_y += dy[i];
        // }

        // tot_x = (tot_x > 0) ? tot_x : -tot_x;
        // tot_y = (tot_y > 0) ? tot_y : -tot_y;

        // float d = sqrt(tot_x * tot_x + tot_y * tot_y);

        // printf("Average Velocity: %f m/s\n", d * 0.020 / 0.05);  // 20mm per unit, 100ms per frame

        // high_u = 0;
        // high_v = 0;


        // printf("High score at shift (%d,%d): %f\n", high_u, high_v, high_score);

        // if (high_u != 0 || high_v != 0) {
        //     printf("Shift Detected. Max Score: (%d, %d) Score: %f\n", high_u, high_v, high_score);
        // }

        // Simple velocity estimate



        // Print Frame (move to new task later)
        printf("F");
        for(int i = 0; i < 36; i++){
            printf(",%4u", buff[toggle][i]);
        }
        printf("\n");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, toggle);
        toggle = !toggle;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void)
{
    stdio_init_all();
    cyw43_arch_init();
    spi_init_adc_bus();
    TaskHandle_t readtask;
    xTaskCreate(main_task, "Thread",
                configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &readtask);
    printf("Starting scheduler\n\r");
    vTaskStartScheduler();
    return 0;
}
