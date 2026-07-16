// Cubby DeBry - 7/16/2026
// tasks.h - header for for velocity and camera tasks (separate if gets large)

#define FPS 60
#define FRAME_DELAY_MS (1000 / FPS)

#include <sys/cdefs.h>
#include <stdint.h>

void cam_task(void* params);
void spi_init_adc_bus(void);
void get_frame(uint16_t* frame);
void super_frame(uint32_t* s_frame, uint8_t n_frames);

void avg_task(void* params);
void vel_task(__unused void *params);
