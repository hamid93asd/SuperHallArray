#include <Dog.h>

void get_frame(uint16_t* frame);

// Get frame

// Do Processing
//      -deviation
//      -mean removal

// Integer NCC

// Quadratic peak fit

// Quadrant / subgrid gradient check

void velocity(void* params){
    uint8_t gain = 1;
    uint8_t h_len = 20;     // Number of subframes
    uint8_t fps = 60;
    uint16_t alpha = 256;

    uint16_t raw_frame[36];
    uint32_t raw_acc[36] = {0};
    int16_t dev_curr[36];
    int16_t dev_prev[36] = {0};
    int64_t dev_acc[36] = {0};
    int32_t mean_acc = 0;
    uint32_t frame_base[36] = {0};
    uint64_t score[49 * h_len];
    uint64_t super_score[49] = {0};
    uint64_t score_acc[49] = {0};

    int32_t mean = 0x00FF;

    // Timing
    uint64_t loop_start;
    uint64_t loop_time;
    uint64_t collect_time;
    uint64_t ncc_time;
    uint64_t timer;
    uint64_t timer_b;
    uint64_t ncc_iteration;
    

    // Compute overlap table for grid size
    overlap_t ov[49];

    for(int u = 0; u < 7; u++){
        for(int v = 0; v < 7; v++){
            ncc_compute_overlap(u - 3, v - 3, &ov[(u * 7) + v]);
        }
    }

    while(1){
        for(int f = 0; f < h_len; f++){
            loop_start = time_us_64();
            // New frame
            timer = time_us_64();
            get_frame(raw_frame);
            collect_time = time_us_64() - timer;

            // Updates dev mean
            int64_t sum = 0;
            for(int i = 0; i < 36; i++){
                sum += (int64_t)raw_frame[i] - (int64_t)frame_base[i];
                if(sum > INT32_MAX) tud_printf("Sum Overflow\n");
            }
            mean_acc += (int32_t)(sum / 36);

            // Compute deviation, update raw_acc
            for(int i = 0; i < 36; i++){
                int64_t x = (int64_t)raw_frame[i] - (int64_t)frame_base[i] - mean;
                x = x * gain;
                if(x < INT16_MIN) x = INT16_MIN;
                if(x > INT16_MAX) x = INT16_MAX;
                dev_curr[i] = (int16_t)x;
                dev_acc[i] += x;
                raw_acc[i] += raw_frame[i];
            }

            // Score each shift
            timer = time_us_64();
            for(int u = 0; u < 7; u++){
                for(int v = 0; v < 7; v++){
                    uint64_t t = fast_ncc(u - 3, v - 3, dev_prev, dev_curr, &ov[(u * 7) + v]);
                    score_acc[(u * 7) + v] += t;
                    score[(u * 7) + v] = t;
                }
            }
            ncc_time = time_us_64() - timer;

            // Update Prev
            for(int i = 0; i < 36; i++){
                dev_prev[i] = dev_curr[i];
            }
            loop_time = time_us_64() - loop_start;
        }

        // Update mean
        mean = mean_acc / h_len;
        mean_acc = 0;

        // Update baseline
        for(int i = 0; i < 36; i++){
            int64_t raw_mean = (int64_t)raw_acc[i] / h_len;
            raw_acc[i] = 0;
            int64_t del = raw_mean - (int64_t)frame_base[i];
            int64_t step = (del * alpha) >> 10;
            int64_t base_new = (int64_t)frame_base[i] + step;
            if(base_new < 0) base_new = 0;
            if(base_new > UINT32_MAX) base_new = UINT32_MAX;
            frame_base[i] = (uint32_t)base_new;
        }

        // Update dev
        for(int i = 0; i < 36; i++){
            int64_t new_dev = dev_acc[i] / h_len;
            dev_acc[i] = 0; // Reset
            if(new_dev < INT16_MIN) new_dev = INT16_MIN;
            if(new_dev > INT16_MAX) new_dev = INT16_MAX;
            dev_curr[i] = (int16_t)new_dev;
        }

        // Average scores, find best
        int8_t high_shift = 24;    // Default center, (u + 3) * 7 + (v + 3)
        uint64_t high_score = 0;

        for(int i = 0; i < 49; i++){
            super_score[i] = score_acc[i] / h_len;
            score_acc[i] = 0;   // Reset
            if(super_score[i] > high_score){
                high_score = super_score[i];
                high_shift = i;
            }
        }

        tud_printf("Best Shift (%5d, %5d) \tBest Score: %10llu\tLoop Time: %8llu", 
            (high_shift/7) - 3, (high_shift%7) - 3, high_score, loop_time);

        tud_printf("\tCollection Time:%8llu\tNCC Time:%8llu\tNCC Iteration:%8llu\n", collect_time, ncc_time, ncc_iteration);
        vTaskDelay(100);
    }
}
