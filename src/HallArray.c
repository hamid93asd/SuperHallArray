// HallArray.c 
// Cubby DeBry 2025

#define DEBUG_MODE 0    // 0 = Binary output, 1 = CSV debug output

#define MODE_CAMERA 0
#define MODE_NCC_VEL 1
#define MODE_AVG 2
#define MODE_SIMPLE_VEL 3
#define ARRAY_MODE MODE_AVG

#include "HallArray.h"
#include "usb.h"
#include "tasks.h"

int main(void){
    
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
