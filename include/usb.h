// Cubby DeBry - 7/16/2026
// usb.h - USB interface functions
#include <stdint.h>

void usb_init();
void tud_update_task(void* params);
void tud_printf(const char* format, ...);
void send_frame_binary(uint16_t* frame);
void send_frame_csv(uint16_t* frame);