// HallArray.c 
// Cubby DeBry 2025

#include <stdio.h>
#include "tusb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pico.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "NCC.h"
#include "pico/time.h"

#define BUFFER_SIZE 16
#define FPS 60
#define FRAME_DELAY_MS (1000 / FPS)
#define DEBUG_MODE 0    // 0 = Binary output, 1 = CSV debug output
#define ARRAY_MODE 1    // 0 = Camera Mode, 1 = Motion Tracker Mode, 2 = Ensemble Mode
#define ALPHA 5         // Baseline update factor, ALPHA/1024
#define HISTORY_LENGTH 2 // Number of historical frames for frame shifting
#define VELOCITY_FRAMES 5   // Averaging period for velocity

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

static uint adc_buffer[BUFFER_SIZE];
static const uint8_t input_ctrl[8] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN0};
static const uint8_t standard_map[8] = {3, 4, 5, 6, 7, 2, 1, 0};   // Map to physical order
static const uint8_t short_map[8] = {0, 1, 2, 3, 0, 0, 0, 0};      // For last ADC (only 4 inputs used)
static const uint8_t channel_ctrl[5] = {CS1, CS2, CS3, CS4, CS5};

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

// Send a fram of data over USB CDC in binary format
void send_frame_binary(uint16_t* frame) {
    static const uint8_t sync[4] = {0xAA, 0x55, 0xAA, 0x55};
    if (!tud_cdc_connected()) return; // optional

    tud_cdc_write(sync, 4);
    tud_cdc_write(frame, 36 * sizeof(uint16_t));
    tud_cdc_write_flush();
}

void tud_printf(const char* format, ...){
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    while(tud_cdc_write_available() < strlen(buffer)){
        tud_cdc_write_flush();
        vTaskDelay(1);
    }

    tud_cdc_write_str(buffer);
    tud_cdc_write_flush();
}

// Send a frame over CDC in CSV format for debugging
void send_frame_csv(uint16_t* frame){
    tud_printf("F");
    for(int i = 0; i < 36; i++){
        tud_printf(",%4u", frame[i]);
    }
    tud_printf("\n");
}

// Read two bytes from SPI, write address of next read
uint16_t spi_transfer16(uint16_t tx){
    uint8_t tx_buf[2] = { (tx >> 8) & 0xFF, tx & 0xFF};
    uint8_t rx_buf[2];
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, 2);
    return ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
}

// Get a single frame of 36 readings
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

void tud_update_task(void* params){
    while(1){
        tud_task();
        vTaskDelay(1);
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
            send[i] = (uint16_t)(frame[i] >> 16);
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

// Normalized Cross-Correlation Velocity Task
void vel_task(__unused void *params) {
    uint32_t frameBase[36];
    uint32_t frameCurr[36];
    int32_t frameDevCurr[36];
    int32_t frameDevPrev[36];
    int32_t frameHistory[HISTORY_LENGTH * 36]; // replace devCurr and devPrev

    const uint32_t BASE_MAX = (uint32_t)4095 << 20;

    int8_t dx[VELOCITY_FRAMES];
    int8_t dy[VELOCITY_FRAMES];
    bool toggle = false;

    for(int i = 0; i < VELOCITY_FRAMES; i++){
        dx[i] = 0;
        dy[i] = 0;
    }

    for(int i = 0; i < 36; i++){
        frameBase[i] = 2048 << 20;
    }

    // Initialize frameHistory to zeros
    for(int i = 0; i < HISTORY_LENGTH * 36; i++){
        frameHistory[i] = 0;
    }

    while(1) {
        super_frame(frameCurr, 40); // Update frame

        for(int i = 0; i < 36; i++){
            int64_t dev = (int64_t)frameCurr[i] - (int64_t)frameBase[i];
            if (dev > INT32_MAX) dev = INT32_MAX;   // Clamp to int32 range
            if (dev < INT32_MIN) dev = INT32_MIN;
            // tud_printf("%lld, ", dev >> 20);
            frameHistory[0 * 36 + i] = (int32_t)dev;
        }
        // tud_printf("\n");
        
        uint64_t high_score = 0;
        uint64_t zz_score = 0;
        int8_t high_frame = 0;
        int8_t high_u = 0;
        int8_t high_v = 0;
        overlap_t ov;

        for(int i = 1; i < HISTORY_LENGTH; i++){
            for(int u = -3; u < 4; u++){
                for(int v = -3; v < 4; v++){
                    if(ncc_compute_overlap(u, v, &ov)){
                        uint64_t score = ncc_score(u, v, &frameHistory[i * 36], &frameHistory[0 * 36], &ov);
                        // tud_printf("NCC Score at frame: %d, shift (%d,%d): %f\n", i, u, v, (float)score / (float)(1 << 20));
                        if (u == 0 && v == 0 && i == 1){
                            zz_score = score;
                        }
                        if(score > high_score){
                            high_score = score;
                            high_frame = i;
                            high_u = u;
                            high_v = v;
                        }
                    }
                }
            }
        }

        for(int i = VELOCITY_FRAMES - 1; i > 0; i--){
            dx[i] = dx[i - 1];
            dy[i] = dy[i - 1];
        }

        dx[0] = high_u;
        dy[0] = high_v;

        int8_t tot_x = 0;
        int8_t tot_y = 0;

        for(int i = 0; i < VELOCITY_FRAMES; i++){
            tot_x += (dx[i] > 0) ? dx[i] : -dx[i];
            tot_y += (dy[i] > 0) ? dy[i] : -dy[i];
        }

        // uint64_t d = sqrt(tot_x * tot_x + tot_y * tot_y);

        // uint64_t vel = d * 20 / (10 * FRAME_DELAY_MS);  // mm/ms -> m/s
        // tud_printf("High score at shift (%d,%d): %f\n", high_u, high_v, high_score);

        if (high_u != 0 || high_v != 0) {
            tud_printf("Shift Detected. Frame: n-%d Max Score: (%d, %d) Score: %lld, Average Velocity: m/s\n", high_frame, high_u, high_v, high_score);
            // tud_printf("(0, 0) Score: %lld, Shift won by %d percent\n", zz_score, (uint8_t)(((float)high_score / (float)zz_score) - 1) * 100.0f);
        }

        // Baseline update
        for(int i = 0; i < 36; i++){
            int64_t base = (int64_t)frameBase[i];
            int64_t del = (int64_t)frameCurr[i] - base;

            int64_t step = (del * ALPHA) >> 10;
            int64_t base_new = base + step;

            if (base_new < 0) base_new = 0;
            if (base_new > (int64_t)BASE_MAX) base_new = (int64_t)BASE_MAX;

            frameBase[i] = (uint32_t)base_new;
            // tud_printf("%lu, ", frameBase[0 * 36 + i] >> 20);
        }
        // tud_printf("\n");

        for(int i = HISTORY_LENGTH - 1; i > 0; i--){
            for(int j = 0; j < 36; j++){
                frameHistory[(i * 36) + j] = frameHistory[((i - 1) * 36) + j];
            }
        }

        // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, toggle);
        toggle = !toggle;
        vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
    }
}

// Magnetic Array Ensemble Averaging
void avg_task(void* params){
    uint32_t raw;
    uint32_t spatial;
    uint32_t temporal;
    uint32_t combined;
    uint32_t comb_avg;
    uint64_t temp = 0;

    uint16_t frame[36];
    uint32_t s_frame[36];

    while(1){
        // Update readings
        get_frame(frame);           // Raw frame (12 bit)
        super_frame(s_frame, 64);   // Super sampled frame (32 bit)
        raw = ((uint32_t)frame[15]) << 20;

        // Compute spatial average
        for(int i = 0; i < 36; i++){
            temp += (uint64_t)(((uint32_t)frame[i]) << 20);
        }
        spatial = (uint32_t)(temp/36);

        // Read temporal average
        temporal = s_frame[15];

        // Sum combined average
        temp = 0;
        for(int i = 0; i < 36; i++){
            temp += (uint64_t)s_frame[i];
        }
        combined = (uint32_t)(temp/36);

        temp = 0;

        // Send results
        tud_printf("%u, %u, %u, %u\n", raw, spatial, temporal, combined);
        vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
    }
}

int main(void)
{
    stdio_init_all();
    // cyw43_arch_init();
    tusb_init();
    spi_init_adc_bus();

    TaskHandle_t readtask;
    TaskHandle_t tudtask;

    xTaskCreate(tud_update_task, "tud",
            configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &tudtask);

    #if (ARRAY_MODE == 0)
        xTaskCreate(cam_task, "Thread",
                    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &readtask);
    #elif (ARRAY_MODE == 1)
        xTaskCreate(vel_task, "Thread",
                    2048 * 10, NULL, tskIDLE_PRIORITY + 2, &readtask);
    #elif (ARRAY_MODE == 2)
        xTaskCreate(avg_task, "Thread",
                    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &readtask);
    #endif

    vTaskStartScheduler();
    return 0;
}
