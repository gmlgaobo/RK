#ifndef USB_HID_MOUSE_H
#define USB_HID_MOUSE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// USB HID mouse report structure
// Relative movement mouse: buttons + dx + dy + scroll wheel
typedef struct {
    uint8_t buttons;  // Buttons bitmask: bit0=left, bit1=right, bit2=middle
    int8_t dx;        // X movement relative (-127 to +127)
    int8_t dy;        // Y movement relative (-127 to +127)
    int8_t scroll;    // Scroll wheel
} hid_mouse_report_t;

// Initialize USB HID mouse gadget
// Returns 0 on success, -1 on error
int usb_hid_mouse_init(void);

// Cleanup and exit
void usb_hid_mouse_exit(void);

// Send mouse movement report
// dx, dy: relative movement in pixels (-127 to +127)
// buttons: button bitmask (0 = no buttons pressed)
int usb_hid_mouse_move(int8_t dx, int8_t dy, uint8_t buttons);

// Check if device is initialized and ready
bool usb_hid_mouse_ready(void);

#ifdef __cplusplus
}
#endif

#endif
