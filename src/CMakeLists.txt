# This is the main binary. List your C files here.
add_executable(hall_array
    HallArray.c
    NCC.c
    usb_descriptors.c
    )

pico_set_program_name(hall_array "test")
pico_set_program_version(hall_array "0.1")

# Modify the below lines to enable/disable output over UART/USB.
# These are mutually exclusive, only set one.
pico_enable_stdio_uart(hall_array 0)
pico_enable_stdio_usb(hall_array 0)

# Add the libraries to the build
target_link_libraries(hall_array PRIVATE
  pico_stdlib
  pico_multicore
  pico_stdio_usb_headers
  tinyusb_device
  pico_async_context_freertos
  FreeRTOS-Kernel-Heap4
  hardware_spi
)

# The CYW43 is the Wifi/Bluetooth module. If the board is set to pico_w, this
# variable will be set and the wireless libraries added.
if(PICO_CYW43_SUPPORTED)
    # For led support on pico_w
    target_link_libraries(hall_array PRIVATE
        pico_cyw43_arch_none
    )
endif()

# Another pico macro which generates the uf2 file in addition to the elf
pico_add_extra_outputs(hall_array)

find_program(OPENOCD openocd)
find_program(PICOTOOL picotool)

add_custom_target(flash ${PICOTOOL} load -f $<TARGET_FILE:hall_array> DEPENDS hall_array)

add_custom_target(openocd_flash ${OPENOCD} -s ${OPENOCD_PATH} -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c 'adapter speed 5000; program "$<TARGET_FILE:hall_array>" verify reset exit' DEPENDS hall_array)
