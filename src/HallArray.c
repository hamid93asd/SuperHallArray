// HallArray.c 
// Cubby DeBry 2025
#include "HallArray.h"

#include "usb.h"

#define BUFFER_SIZE 16
#define FPS 60
#define FRAME_DELAY_MS (1000 / FPS)
#define DEBUG_MODE 0    // 0 = Binary output, 1 = CSV debug output
#define MODE_CAMERA 0
#define MODE_NCC_VEL 1
#define MODE_AVG 2
#define MODE_SIMPLE_VEL 3
#define ARRAY_MODE MODE_AVG
#define ALPHA 256         // Baseline update factor, ALPHA/1024
#define ALPHA_V 0.01     // Velocity Avg baseline update
#define HISTORY_LENGTH 2 // Number of historical frames for frame shifting
#define VELOCITY_FRAMES 5   // Averaging period for velocity
#define EPS 0.001   // Minimum Denominator

#define GRID_PITCH_MM 20.0f
#define SIMPLE_VEL_SUPER_FRAMES 16
#define SIMPLE_VEL_BASE_ALPHA 64
#define SIMPLE_VEL_PEAK_THRESH 60
#define SIMPLE_VEL_ALPHA 0.25f
#define SIMPLE_VEL_MIN_EVENT_DT_US 5000
#define SIMPLE_VEL_MAX_EVENT_DT_US 500000
#define SIMPLE_VEL_MAX_COL_STEP 1

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

// Barebones 1D velocity from column-to-column peak timing
void simple_vel_task(__unused void *params){
    uint32_t frame[36];
    int32_t col_raw[6];
    int32_t col_base[6];
    int32_t col_dev[6];
    int32_t col_dev_prev[6] = {0};
    int32_t col_deriv_prev[6] = {0};

    bool base_init = false;
    bool have_last_peak = false;
    uint8_t last_peak_col = 0;
    uint64_t last_peak_us = 0;

    float v_inst = 0.0f;
    float v_ema = 0.0f;

    // tud_printf("simple_vel,start\n");

    while(1){
        uint64_t loop_start = time_us_64();
        super_frame(frame, SIMPLE_VEL_SUPER_FRAMES);

        for(int col = 0; col < 6; col++){
            int32_t sum = 0;
            for(int row = 0; row < 6; row++){
                sum += (int32_t)(frame[col * 6 + row] >> 20);
            }
            col_raw[col] = sum;
            if(!base_init){
                col_base[col] = sum;
            }
        }
        base_init = true;

        int32_t mean = 0;
        for(int col = 0; col < 6; col++){
            int32_t del = col_raw[col] - col_base[col];
            col_base[col] += (del * SIMPLE_VEL_BASE_ALPHA) >> 10;
            col_dev[col] = del;
            mean += del;
        }
        mean /= 6;
        for(int col = 0; col < 6; col++){
            col_dev[col] -= mean;
        }

        int32_t best_amp = SIMPLE_VEL_PEAK_THRESH;
        int8_t best_col = -1;
        for(int col = 0; col < 6; col++){
            int32_t deriv = col_dev[col] - col_dev_prev[col];
            bool sign_flip = (col_deriv_prev[col] > 0) && (deriv <= 0);
            bool strong = col_dev[col] > SIMPLE_VEL_PEAK_THRESH;
            bool left_ok = (col == 0) || (col_dev[col] >= col_dev[col - 1]);
            bool right_ok = (col == 5) || (col_dev[col] >= col_dev[col + 1]);
            bool spatial_peak = left_ok && right_ok;

            if(sign_flip && strong && spatial_peak && (col_dev[col] > best_amp)){
                best_amp = col_dev[col];
                best_col = (int8_t)col;
            }

            col_deriv_prev[col] = deriv;
        }

        uint64_t now_us = time_us_64();
        if(best_col >= 0){
            if(!have_last_peak){
                have_last_peak = true;
                last_peak_col = (uint8_t)best_col;
                last_peak_us = now_us;
            } else if((uint8_t)best_col != last_peak_col){
                uint64_t dt_us = now_us - last_peak_us;
                bool accepted_event = false;
                if((dt_us >= SIMPLE_VEL_MIN_EVENT_DT_US) && (dt_us <= SIMPLE_VEL_MAX_EVENT_DT_US)){
                    int32_t dcol = (int32_t)best_col - (int32_t)last_peak_col;
                    if((dcol <= SIMPLE_VEL_MAX_COL_STEP) && (dcol >= -SIMPLE_VEL_MAX_COL_STEP)){
                        v_inst = ((float)dcol * GRID_PITCH_MM * 1000000.0f) / (float)dt_us; // mm/s
                        v_ema = ((1.0f - SIMPLE_VEL_ALPHA) * v_ema) + (SIMPLE_VEL_ALPHA * v_inst);
                        // tud_printf("%llu,%d,%7.2f,%7.2f\n", now_us, (int)best_col, v_inst, v_ema);
                        tud_printf("\n%12llu, %7.3f, 0", now_us, v_inst);  // Data collection csv format: time, vx, vy
                        accepted_event = true;
                    }
                }
                if(accepted_event || (dt_us > SIMPLE_VEL_MAX_EVENT_DT_US)){
                    last_peak_col = (uint8_t)best_col;
                    last_peak_us = now_us;
                }
            }
        }

        for(int col = 0; col < 6; col++){
            col_dev_prev[col] = col_dev[col];
        }

        uint64_t frame_time_ms = (time_us_64() - loop_start) / 1000;
        if(frame_time_ms < FRAME_DELAY_MS){
            vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS - frame_time_ms));
        }
    }
}

// Normalized Cross-Correlation Velocity Task
void vel_task(__unused void *params) {
    uint32_t frameBase[36];
    uint32_t frameCurr[36];
    uint32_t frameNext[36];
    int32_t frameDevCurr[36];
    int32_t frameDevPrev[36];
    int32_t frameHistory[HISTORY_LENGTH * 36]; // replace devCurr and devPrev
    uint64_t start = time_us_64();
    uint64_t finish;
    uint64_t frame_time;
    uint64_t prev_sample = time_us_64();

    const uint32_t BASE_MAX = (uint32_t)4095 << 20;
    const uint32_t e_threshold = 200;

    float dx[VELOCITY_FRAMES];
    float dy[VELOCITY_FRAMES];
    float dt[VELOCITY_FRAMES];
    bool toggle = false;

    float vx_avg = 0;
    float vy_avg = 0;
    float win_vx = 0;
    float win_vy = 0;
    float vx = 0;
    float vy = 0;

    for(int i = 0; i < VELOCITY_FRAMES; i++){
        dx[i] = 0;
        dy[i] = 0;
        dt[i] = FRAME_DELAY_MS;
    }

    for(int i = 0; i < 36; i++){
        frameBase[i] = 2048 << 20;
    }

    // Initialize frameHistory to zeros
    for(int i = 0; i < HISTORY_LENGTH * 36; i++){
        frameHistory[i] = 0;
    }

    // Terminal Setup
    tud_printf("\x1b[0m\x1b[?25l\x1b[3J\x1b[2J\x1b[H");

    while(1) {
        prev_sample = start;
        start = time_us_64();
        super_frame(frameCurr, 40); // Update frame

        int64_t sum = 0;
        for (int i = 0; i < 36; i++){
            sum += (int64_t)frameCurr[i] - (int64_t)frameBase[i];
        }
        int64_t mean = sum / 36;
        // tud_printf("Mean deviation: %llu\n", mean);

        for (int i = 0; i < 36; i++){
            int64_t x = ((int64_t)frameCurr[i] - (int64_t)frameBase[i]) - (int64_t)mean;
            
            if (x > INT32_MAX) x = INT32_MAX;
            if (x < INT32_MIN) x = INT32_MIN;
            frameHistory[0 * 36 + i] = (int32_t)x;
        }

        // Check RMS energy
        uint32_t e_curr = 0;
        for (int i = 0; i < 36; i++){
            e_curr += (uint32_t)((frameHistory[0 * 36 + i] >> 20) * (frameHistory[0 * 36 + i] >> 20));
        }
        
        uint64_t score[HISTORY_LENGTH * 7 * 7] = {0};
        uint16_t high_idx = 24; // Center index (0 shift, 1st frame)
        int8_t high_frame = 0;
        int8_t high_u = 0;
        int8_t high_v = 0;
        overlap_t ov;

        if(e_curr > e_threshold){
            for(int i = 1; i < HISTORY_LENGTH; i++){
                for(int u = -3; u < 4; u++){
                    for(int v = -3; v < 4; v++){
                        if(ncc_compute_overlap(u, v, &ov)){
                            uint16_t idx = (i - 1) * 49 + (u + 3) * 7 + (v + 3);
                            score[idx] = 1024 * f_score(u, v, &frameHistory[i * 36], &frameHistory[0 * 36], &ov);
                            score[idx] = score[idx] / (i);    // Penalize older frames

                            if(score[idx] > score[high_idx]){
                                high_idx = idx;
                                high_frame = i - 1;
                                high_u = u;
                                high_v = v;
                            }
                        }
                    }
                }
            }

            // Quadratic Peak Interpolation
            float sub_x = 0;
            float sub_y = 0;

            if(-3 < high_u && high_u < 3){
                float den = (2.0f * ((float)score[high_idx - 7] - (2.0f * (float)score[high_idx]) + (float)score[high_idx + 7]));
                den = (den < -EPS) ? den : 0;   // Check for concave
                sub_x = (den == 0) ? 0 : ((float)score[high_idx - 7] - (float)score[high_idx + 7]) / den;
            }

            if(-3 < high_v && high_v < 3){
                float den = (2.0f * ((float)score[high_idx - 1] - (2.0f * (float)score[high_idx]) + (float)score[high_idx + 1]));
                den = (den < -EPS) ? den : 0;   // Check for concave
                sub_y = (den == 0) ? 0 : ((float)score[high_idx - 1] - (float)score[high_idx + 1]) / den;
            }

            // Clamp Sub-pixel Refinement
            sub_x = (sub_x > 0.5) ? 0.5 : sub_x;
            sub_x = (sub_x < -0.5) ? -0.5 : sub_x;
            sub_y = (sub_y > 0.5) ? 0.5 : sub_y;
            sub_y = (sub_y < -0.5) ? -0.5 : sub_y;

            for(int i = VELOCITY_FRAMES - 1; i > 0; i--){
                dx[i] = dx[i - 1];
                dy[i] = dy[i - 1];
                dt[i] = dt[i - 1];
            }

            dx[0] = (float)high_u + sub_x;
            dy[0] = (float)high_v + sub_y;
            dt[0] = (float)(start - prev_sample) * 0.001f; // us -> ms

            float tot_x = 0;
            float tot_y = 0;
            float tot_t = 0;

            for(int i = 0; i < VELOCITY_FRAMES; i++){
                tot_x += dx[i];
                tot_y += dy[i];
                tot_t += dt[i];
            }

            // Instantaneous Velocity
            vx = dx[0] * 20.0 / (dt[0] * (HISTORY_LENGTH - 1));
            vy = dy[0] * 20.0 / (dt[0] * (HISTORY_LENGTH - 1));

            // Windowed Average
            win_vx = tot_x * 20 / tot_t;
            win_vy = tot_y * 20 / tot_t;

            // Exponential Moving average
            vx_avg = vx_avg * (1.0f - ALPHA_V) + vx * ALPHA_V;
            vy_avg = vy_avg * (1.0f - ALPHA_V) + vy * ALPHA_V;

        } else {
            vx_avg = vx_avg * 0.5;
            vy_avg = vy_avg * 0.5;
            win_vx = 0;
            win_vy = 0;
            vx = 0;
            vy = 0;
            dx[0] = dy[0] = 0;
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
        }

        for(int i = HISTORY_LENGTH - 1; i > 0; i--){
            for(int j = 0; j < 36; j++){
                frameHistory[(i * 36) + j] = frameHistory[((i - 1) * 36) + j];
            }
        }

        finish = time_us_64();
        frame_time = (finish - start) / 1000;    // Include print duration estimate

        // Terminal Setup
        // tud_printf("\x1b[?25l\x1b[H\x1b[2E\x1b[32;49m\x1b[3mSuper\x1b[23m\x1b[39;49m Hall Array Velocity Monitor - Cubby DeBry, Jan 2026");
        // tud_printf("\x1b[2E");
        // 
        // if(frame_time < FRAME_DELAY_MS)
            // tud_printf("Shift: (%7.3f, %7.3f) \tScore: %8lld \tCompute time: \x1b[32m%4llu\x1b[39m", dx[0], dy[0], score[high_idx], frame_time);
        // else
            // tud_printf("Shift: (%7.3f, %7.3f) \tScore: %8lld \tCompute time: \x1b[31m%4llu\x1b[39m", dx[0], dy[0], score[high_idx], frame_time);
        // tud_printf("\n%7.3f, %7.3f, %7.3f, %7.3f", vx, vy, win_vx, win_vy);

        tud_printf("Shift: (%7.3f, %7.3f) Score: %7.3f\n", dx[0], dy[0], (float)score[high_idx] * 0.001f);


        if(frame_time < FRAME_DELAY_MS){
            vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS - frame_time));
        }
    }
}

// Magnetic Array Ensemble Averaging
void avg_task(void* params){
    uint16_t oversample = 64;           // Oversamples per frame
    uint8_t sensors[4] = {2, 8, 3, 9};  // 0 index, NOT board index
    uint8_t n = sizeof(sensors);
    uint32_t spatial;
    uint32_t combined;
    uint64_t start;
    uint32_t frame[36] = {0};
    uint32_t s_frame[36] = {0};

    while(1){
        // Update readings
        memset(s_frame, 0, sizeof(s_frame[0]));
        start = time_us_64();

        uint16_t raw[36] = {0};
        for (int i = 0; i < oversample; i++){
            get_frame(raw);
            for (int j = 0; j < 36; j++){
                s_frame[j] += (uint32_t)raw[j];
                if (i == 0) frame[j] = (uint32_t)raw[j] << 20;   // Q20
            }
        }

        // Per Sensor Time Average
        for (int i = 0; i < 36; i++){
            s_frame[i] = (uint32_t)(((uint64_t)s_frame[i] / (uint64_t)oversample) << 20);   // Q20
        }

        // n Sensor Spatial Average
        uint64_t temp = 0;
        for (int i = 0; i < n; i++){
            temp += (uint64_t)((uint32_t)frame[sensors[i]]);
        }
        spatial = (uint32_t)(temp / n);

        // n Sensor Combined Average
        temp = 0;
        for (int i = 0; i < n; i++){
            temp += (uint64_t)((uint32_t)s_frame[sensors[i]]);
        }
        combined = (uint32_t)(temp / n);

        // Print results
        for (int i = 0; i < n; i++){    // Raw
            tud_printf("%lu, ", frame[sensors[i]]);
        }

        for (int i = 0; i < n; i++){    // Temporal
            tud_printf("%lu, ", s_frame[sensors[i]]);
        }

        tud_printf("%lu, ", spatial);   // Spatial
        
        tud_printf("%lu, ", combined);  // Combined
        tud_printf("%llu\n", start);    // Timestamp    
        taskYIELD();
    }
}

int main(void)
{
    stdio_init_all();
    usb_init();
    spi_init_adc_bus();

    TaskHandle_t readtask;
    TaskHandle_t tudtask;

    xTaskCreate(tud_update_task, "tud",
            configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &tudtask);

    #if (ARRAY_MODE == MODE_CAMERA)
        xTaskCreate(cam_task, "Thread",
                    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &readtask);
    #elif (ARRAY_MODE == MODE_NCC_VEL)
        xTaskCreate(vel_task, "Thread",
                    2048 * 10, NULL, tskIDLE_PRIORITY + 2, &readtask);
    #elif (ARRAY_MODE == MODE_AVG)
        xTaskCreate(avg_task, "Thread",
                    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &readtask);
    #elif (ARRAY_MODE == MODE_SIMPLE_VEL)
        xTaskCreate(simple_vel_task, "Thread",
                    2048 * 4, NULL, tskIDLE_PRIORITY + 2, &readtask);
    #endif

    vTaskStartScheduler();
    return 0;
}
