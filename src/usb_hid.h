#ifndef USB_HID_H
#define USB_HID_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ========== 鼠标 ==========

// USB HID mouse report structure
typedef struct {
    uint8_t buttons;  // Buttons bitmask: bit0=left, bit1=right, bit2=middle
    int8_t dx;        // X movement relative (-127 to +127)
    int8_t dy;        // Y movement relative (-127 to +127)
    int8_t scroll;    // Scroll wheel
} hid_mouse_report_t;

// Initialize USB HID mouse gadget
int usb_hid_mouse_init(void);

// Cleanup
void usb_hid_mouse_exit(void);

// Send mouse report
int usb_hid_mouse_send(const hid_mouse_report_t* report);

// Send mouse movement (shortcut)
int usb_hid_mouse_move(int8_t dx, int8_t dy, uint8_t buttons);

// Check if mouse is ready
bool usb_hid_mouse_ready(void);

// ========== 键盘 ==========

// USB HID keyboard report structure
typedef struct {
    uint8_t modifiers;  // 修饰键
    uint8_t reserved;   // 保留
    uint8_t keycodes[6]; // 按键码
} hid_keyboard_report_t;

// Initialize USB HID keyboard gadget
int usb_hid_keyboard_init(void);

// Cleanup
void usb_hid_keyboard_exit(void);

// Send keyboard report
int usb_hid_keyboard_send(const hid_keyboard_report_t* report);

// Check if keyboard is ready
bool usb_hid_keyboard_ready(void);

// ========== 通用 ==========

// Check if USB OTG is connected to a host
bool usb_hid_otg_connected(void);

// Check if connection state changed (returns true if changed)
// *prev_state: input - previous state, output - new state
bool usb_hid_otg_state_changed(bool* prev_state);

#ifdef __cplusplus
}
#endif

#endif
