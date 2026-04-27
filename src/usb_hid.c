/*
 * RK3588 USB HID Gadget (键盘+鼠标)
 */

#include "usb_hid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// ========== 鼠标 ==========

static int hidg0_fd = -1;
static bool mouse_initialized = false;

int usb_hid_mouse_init(void) {
    hidg0_fd = open("/dev/hidg0", O_WRONLY);
    if (hidg0_fd < 0) {
        perror("open /dev/hidg0");
        return -1;
    }
    mouse_initialized = true;
    printf("USB HID 鼠标初始化成功: /dev/hidg0\n");
    return 0;
}

void usb_hid_mouse_exit(void) {
    if (hidg0_fd >= 0) {
        close(hidg0_fd);
        hidg0_fd = -1;
    }
    mouse_initialized = false;
}

int usb_hid_mouse_send(const hid_mouse_report_t* report) {
    if (!mouse_initialized || hidg0_fd < 0) {
        return -1;
    }
    
    // 检查 OTG 连接状态
    static bool last_otg_state = false;
    bool current_otg_state = usb_hid_otg_connected();
    
    // 如果 OTG 状态变化，立即重连
    if (current_otg_state != last_otg_state) {
        printf("[HID] OTG 状态变化: %s -> %s\n", 
               last_otg_state ? "已连接" : "未连接",
               current_otg_state ? "已连接" : "未连接");
        
        if (current_otg_state) {
            // 重新连接
            close(hidg0_fd);
            hidg0_fd = open("/dev/hidg0", O_WRONLY | O_NONBLOCK);
            if (hidg0_fd >= 0) {
                printf("[HID] 重新连接鼠标设备\n");
            }
        }
        last_otg_state = current_otg_state;
    }
    
    // 如果未连接，不发送数据
    if (!current_otg_state) {
        return -1;
    }
    
    ssize_t n = write(hidg0_fd, report, sizeof(*report));
    if (n != sizeof(*report)) {
        // 立即重连（不等待100次失败）
        perror("[HID] mouse write failed");
        
        // 立即重连
        close(hidg0_fd);
        hidg0_fd = open("/dev/hidg0", O_WRONLY | O_NONBLOCK);
        if (hidg0_fd >= 0) {
            printf("[HID] 立即重连鼠标设备\n");
            // 重试发送
            n = write(hidg0_fd, report, sizeof(*report));
            if (n == sizeof(*report)) return 0;
        }
        return -1;
    }
    return 0;
}

int usb_hid_mouse_move(int8_t dx, int8_t dy, uint8_t buttons) {
    hid_mouse_report_t report;
    report.buttons = buttons;
    report.dx = dx;
    report.dy = dy;
    report.scroll = 0;
    return usb_hid_mouse_send(&report);
}

bool usb_hid_mouse_ready(void) {
    return mouse_initialized && (hidg0_fd >= 0);
}

// ========== 键盘 ==========

static int hidg1_fd = -1;
static bool keyboard_initialized = false;

int usb_hid_keyboard_init(void) {
    hidg1_fd = open("/dev/hidg1", O_WRONLY);
    if (hidg1_fd < 0) {
        perror("open /dev/hidg1");
        return -1;
    }
    keyboard_initialized = true;
    printf("USB HID 键盘初始化成功: /dev/hidg1\n");
    return 0;
}

void usb_hid_keyboard_exit(void) {
    if (hidg1_fd >= 0) {
        close(hidg1_fd);
        hidg1_fd = -1;
    }
    keyboard_initialized = false;
}

int usb_hid_keyboard_send(const hid_keyboard_report_t* report) {
    if (!keyboard_initialized || hidg1_fd < 0) {
        return -1;
    }
    
    // 检查 OTG 连接状态
    static bool last_otg_state = false;
    bool current_otg_state = usb_hid_otg_connected();
    
    // 如果 OTG 状态变化，立即重连
    if (current_otg_state != last_otg_state) {
        printf("[HID] OTG 状态变化: %s -> %s\n", 
               last_otg_state ? "已连接" : "未连接",
               current_otg_state ? "已连接" : "未连接");
        
        if (current_otg_state) {
            // 重新连接
            close(hidg1_fd);
            hidg1_fd = open("/dev/hidg1", O_WRONLY | O_NONBLOCK);
            if (hidg1_fd >= 0) {
                printf("[HID] 重新连接键盘设备\n");
            }
        }
        last_otg_state = current_otg_state;
    }
    
    // 如果未连接，不发送数据
    if (!current_otg_state) {
        return -1;
    }
    
    ssize_t n = write(hidg1_fd, report, sizeof(*report));
    if (n != sizeof(*report)) {
        // 立即重连（不等待100次失败）
        perror("[HID] keyboard write failed");
        
        // 立即重连
        close(hidg1_fd);
        hidg1_fd = open("/dev/hidg1", O_WRONLY | O_NONBLOCK);
        if (hidg1_fd >= 0) {
            printf("[HID] 立即重连键盘设备\n");
            // 重试发送
            n = write(hidg1_fd, report, sizeof(*report));
            if (n == sizeof(*report)) return 0;
        }
        return -1;
    }
    return 0;
}

bool usb_hid_keyboard_ready(void) {
    return keyboard_initialized && (hidg1_fd >= 0);
}

// ========== 通用 ==========

static bool get_otg_state(void) {
    // RK3588 的 UDC 是 fc000000.usb，直接硬编码更可靠
    const char* udc_name = "fc000000.usb";
    
    // 读取 UDC 状态
    char state_path[128];
    snprintf(state_path, sizeof(state_path), 
             "/sys/class/udc/%s/state", udc_name);
    
    FILE* f = fopen(state_path, "r");
    if (!f) {
        return false;
    }
    
    char state[64];
    bool connected = false;
    if (fscanf(f, "%63s", state) == 1) {
        // "configured" 或 "addressed" 表示已连接到主机
        connected = (strcmp(state, "configured") == 0) ||
                    (strcmp(state, "addressed") == 0);
    }
    fclose(f);
    
    return connected;
}

bool usb_hid_otg_connected(void) {
    return get_otg_state();
}

bool usb_hid_otg_state_changed(bool* prev_state) {
    if (!prev_state) return false;
    
    bool new_state = get_otg_state();
    if (new_state != *prev_state) {
        *prev_state = new_state;
        return true;
    }
    return false;
}
