// Cubby DeBry - 7/16/2026
// usb.c - contains usb communication setup and helper functions

#include "usb.h"
#include "tusb.h"
#include <stdarg.h>

void usb_init(){
    tusb_init();
}

void tud_update_task(void* params){
    while(1){
        tud_task();
        vTaskDelay(1);
    }
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

// Send a fram of data over USB CDC in binary format
void send_frame_binary(uint16_t* frame) {
    static const uint8_t sync[4] = {0xAA, 0x55, 0xAA, 0x55};
    if (!tud_cdc_connected()) return; // optional

    tud_cdc_write(sync, 4);
    tud_cdc_write(frame, 36 * sizeof(uint16_t));
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
