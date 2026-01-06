// Cubby DeBry 2025 - TinyUSB Configuration Header
// Heavily based on Anthropic AI Generated Content

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

// Board Config
#define CFG_TUSB_RHPORT0_MODE       (OPT_MODE_DEVICE)
#define CFG_TUSB_OS                 OPT_OS_FREERTOS

// 
#ifndef CFG_USB_DEBUG
#define CFG_USB_DEBUG               0
#endif

// Device Class Drivers
#define CFG_TUD_CDC                 1
#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0

// CDC FIFO size
#define CFG_TUD_CDC_RX_BUFSIZE      64
#define CFG_TUD_CDC_TX_BUFSIZE      256

#ifdef __cplusplus
}
#endif

#endif