#include "hid_keycode_map.h"
#include <string.h>

// Linux input event code 到 USB HID usage code 的映射表
// 参考: linux/input-event-codes.h 和 HID Usage Tables

static const struct {
    uint16_t linux_code;
    uint8_t hid_code;
} keycode_map[] = {
    // 字母键
    {KEY_A, HID_KEY_A},
    {KEY_B, HID_KEY_B},
    {KEY_C, HID_KEY_C},
    {KEY_D, HID_KEY_D},
    {KEY_E, HID_KEY_E},
    {KEY_F, HID_KEY_F},
    {KEY_G, HID_KEY_G},
    {KEY_H, HID_KEY_H},
    {KEY_I, HID_KEY_I},
    {KEY_J, HID_KEY_J},
    {KEY_K, HID_KEY_K},
    {KEY_L, HID_KEY_L},
    {KEY_M, HID_KEY_M},
    {KEY_N, HID_KEY_N},
    {KEY_O, HID_KEY_O},
    {KEY_P, HID_KEY_P},
    {KEY_Q, HID_KEY_Q},
    {KEY_R, HID_KEY_R},
    {KEY_S, HID_KEY_S},
    {KEY_T, HID_KEY_T},
    {KEY_U, HID_KEY_U},
    {KEY_V, HID_KEY_V},
    {KEY_W, HID_KEY_W},
    {KEY_X, HID_KEY_X},
    {KEY_Y, HID_KEY_Y},
    {KEY_Z, HID_KEY_Z},
    
    // 数字键
    {KEY_1, HID_KEY_1},
    {KEY_2, HID_KEY_2},
    {KEY_3, HID_KEY_3},
    {KEY_4, HID_KEY_4},
    {KEY_5, HID_KEY_5},
    {KEY_6, HID_KEY_6},
    {KEY_7, HID_KEY_7},
    {KEY_8, HID_KEY_8},
    {KEY_9, HID_KEY_9},
    {KEY_0, HID_KEY_0},
    
    // 功能键
    {KEY_F1, HID_KEY_F1},
    {KEY_F2, HID_KEY_F2},
    {KEY_F3, HID_KEY_F3},
    {KEY_F4, HID_KEY_F4},
    {KEY_F5, HID_KEY_F5},
    {KEY_F6, HID_KEY_F6},
    {KEY_F7, HID_KEY_F7},
    {KEY_F8, HID_KEY_F8},
    {KEY_F9, HID_KEY_F9},
    {KEY_F10, HID_KEY_F10},
    {KEY_F11, HID_KEY_F11},
    {KEY_F12, HID_KEY_F12},
    
    // 特殊键
    {KEY_ENTER, HID_KEY_ENTER},
    {KEY_ESC, HID_KEY_ESCAPE},
    {KEY_BACKSPACE, HID_KEY_BACKSPACE},
    {KEY_TAB, HID_KEY_TAB},
    {KEY_SPACE, HID_KEY_SPACE},
    {KEY_MINUS, HID_KEY_MINUS},
    {KEY_EQUAL, HID_KEY_EQUAL},
    {KEY_LEFTBRACE, HID_KEY_LEFTBRACE},
    {KEY_RIGHTBRACE, HID_KEY_RIGHTBRACE},
    {KEY_BACKSLASH, HID_KEY_BACKSLASH},
    {KEY_SEMICOLON, HID_KEY_SEMICOLON},
    {KEY_APOSTROPHE, HID_KEY_APOSTROPHE},
    {KEY_GRAVE, HID_KEY_GRAVE},
    {KEY_COMMA, HID_KEY_COMMA},
    {KEY_DOT, HID_KEY_DOT},
    {KEY_SLASH, HID_KEY_SLASH},
    
    // 方向键和导航键
    {KEY_INSERT, HID_KEY_INSERT},
    {KEY_HOME, HID_KEY_HOME},
    {KEY_PAGEUP, HID_KEY_PAGEUP},
    {KEY_DELETE, HID_KEY_DELETE},
    {KEY_END, HID_KEY_END},
    {KEY_PAGEDOWN, HID_KEY_PAGEDOWN},
    {KEY_RIGHT, HID_KEY_RIGHT},
    {KEY_LEFT, HID_KEY_LEFT},
    {KEY_DOWN, HID_KEY_DOWN},
    {KEY_UP, HID_KEY_UP},
    
    // 小键盘
    {KEY_NUMLOCK, HID_KEY_KP_NUMLOCK},
    {KEY_KPSLASH, HID_KEY_KP_DIVIDE},
    {KEY_KPASTERISK, HID_KEY_KP_MULTIPLY},
    {KEY_KPMINUS, HID_KEY_KP_MINUS},
    {KEY_KPPLUS, HID_KEY_KP_PLUS},
    {KEY_KPENTER, HID_KEY_KP_ENTER},
    {KEY_KP1, HID_KEY_KP_1},
    {KEY_KP2, HID_KEY_KP_2},
    {KEY_KP3, HID_KEY_KP_3},
    {KEY_KP4, HID_KEY_KP_4},
    {KEY_KP5, HID_KEY_KP_5},
    {KEY_KP6, HID_KEY_KP_6},
    {KEY_KP7, HID_KEY_KP_7},
    {KEY_KP8, HID_KEY_KP_8},
    {KEY_KP9, HID_KEY_KP_9},
    {KEY_KP0, HID_KEY_KP_0},
    {KEY_KPDOT, HID_KEY_KP_DOT},
};

#define KEYCODE_MAP_SIZE (sizeof(keycode_map) / sizeof(keycode_map[0]))

uint8_t linux_keycode_to_hid(uint16_t linux_code) {
    // 修饰键直接映射（bit 位）
    // LeftCtrl=0, LeftShift=1, LeftAlt=2, LeftGUI=3
    // RightCtrl=4, RightShift=5, RightAlt=6, RightGUI=7
    // 这些在 hid_relay.cpp 中单独处理
    
    // 查找映射表
    for (size_t i = 0; i < KEYCODE_MAP_SIZE; i++) {
        if (keycode_map[i].linux_code == linux_code) {
            return keycode_map[i].hid_code;
        }
    }
    
    // 未找到映射，返回 0（无按键）
    return 0;
}
