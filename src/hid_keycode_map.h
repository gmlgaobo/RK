#ifndef HID_KEYCODE_MAP_H
#define HID_KEYCODE_MAP_H

#include <stdint.h>
#include <linux/input.h>

#ifdef __cplusplus
extern "C" {
#endif

// USB HID Usage ID (Keyboard/Keypad Page - 0x07)
// 参考: HID Usage Tables 1.12, Section 10

// HID 键盘码定义
#define HID_KEY_A               0x04
#define HID_KEY_B               0x05
#define HID_KEY_C               0x06
#define HID_KEY_D               0x07
#define HID_KEY_E               0x08
#define HID_KEY_F               0x09
#define HID_KEY_G               0x0A
#define HID_KEY_H               0x0B
#define HID_KEY_I               0x0C
#define HID_KEY_J               0x0D
#define HID_KEY_K               0x0E
#define HID_KEY_L               0x0F
#define HID_KEY_M               0x10
#define HID_KEY_N               0x11
#define HID_KEY_O               0x12
#define HID_KEY_P               0x13
#define HID_KEY_Q               0x14
#define HID_KEY_R               0x15
#define HID_KEY_S               0x16
#define HID_KEY_T               0x17
#define HID_KEY_U               0x18
#define HID_KEY_V               0x19
#define HID_KEY_W               0x1A
#define HID_KEY_X               0x1B
#define HID_KEY_Y               0x1C
#define HID_KEY_Z               0x1D

#define HID_KEY_1               0x1E
#define HID_KEY_2               0x1F
#define HID_KEY_3               0x20
#define HID_KEY_4               0x21
#define HID_KEY_5               0x22
#define HID_KEY_6               0x23
#define HID_KEY_7               0x24
#define HID_KEY_8               0x25
#define HID_KEY_9               0x26
#define HID_KEY_0               0x27

#define HID_KEY_ENTER           0x28
#define HID_KEY_ESCAPE          0x29
#define HID_KEY_BACKSPACE       0x2A
#define HID_KEY_TAB             0x2B
#define HID_KEY_SPACE           0x2C
#define HID_KEY_MINUS           0x2D
#define HID_KEY_EQUAL           0x2E
#define HID_KEY_LEFTBRACE       0x2F
#define HID_KEY_RIGHTBRACE      0x30
#define HID_KEY_BACKSLASH       0x31
#define HID_KEY_SEMICOLON       0x33
#define HID_KEY_APOSTROPHE      0x34
#define HID_KEY_GRAVE           0x35
#define HID_KEY_COMMA           0x36
#define HID_KEY_DOT             0x37
#define HID_KEY_SLASH           0x38

#define HID_KEY_F1              0x3A
#define HID_KEY_F2              0x3B
#define HID_KEY_F3              0x3C
#define HID_KEY_F4              0x3D
#define HID_KEY_F5              0x3E
#define HID_KEY_F6              0x3F
#define HID_KEY_F7              0x40
#define HID_KEY_F8              0x41
#define HID_KEY_F9              0x42
#define HID_KEY_F10             0x43
#define HID_KEY_F11             0x44
#define HID_KEY_F12             0x45

#define HID_KEY_INSERT          0x49
#define HID_KEY_HOME            0x4A
#define HID_KEY_PAGEUP          0x4B
#define HID_KEY_DELETE          0x4C
#define HID_KEY_END             0x4D
#define HID_KEY_PAGEDOWN        0x4E
#define HID_KEY_RIGHT           0x4F
#define HID_KEY_LEFT            0x50
#define HID_KEY_DOWN            0x51
#define HID_KEY_UP              0x52

#define HID_KEY_KP_NUMLOCK      0x53
#define HID_KEY_KP_DIVIDE       0x54
#define HID_KEY_KP_MULTIPLY     0x55
#define HID_KEY_KP_MINUS        0x56
#define HID_KEY_KP_PLUS         0x57
#define HID_KEY_KP_ENTER        0x58
#define HID_KEY_KP_1            0x59
#define HID_KEY_KP_2            0x5A
#define HID_KEY_KP_3            0x5B
#define HID_KEY_KP_4            0x5C
#define HID_KEY_KP_5            0x5D
#define HID_KEY_KP_6            0x5E
#define HID_KEY_KP_7            0x5F
#define HID_KEY_KP_8            0x60
#define HID_KEY_KP_9            0x61
#define HID_KEY_KP_0            0x62
#define HID_KEY_KP_DOT          0x63

// Linux input event code 到 USB HID code 的映射
uint8_t linux_keycode_to_hid(uint16_t linux_code);

#ifdef __cplusplus
}
#endif

#endif
