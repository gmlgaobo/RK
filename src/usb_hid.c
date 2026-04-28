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
#include <sys/stat.h>
#include <sys/types.h>

// ========== 设备检查与创建 ==========

static bool check_device_exists(const char* device) {
    struct stat st;
    return (stat(device, &st) == 0);
}

static bool create_hid_device(const char* device, int major, int minor) {
    // 检查设备是否已存在
    if (check_device_exists(device)) {
        return true;
    }
    
    // 创建设备节点
    if (mknod(device, S_IFCHR | 0666, (major << 8) | minor) != 0) {
        if (errno != EEXIST) {
            perror("mknod");
            return false;
        }
    }
    
    printf("[USB-HID] 已创建设备节点: %s\n", device);
    return true;
}

static bool get_device_numbers(int* major0, int* minor0, int* major1, int* minor1) {
    const char* gadget_path = "/sys/kernel/config/usb_gadget/rk3588-aimbot";
    
    // 读取设备号
    char dev_path[256];
    FILE* f;
    
    // hidg0
    snprintf(dev_path, sizeof(dev_path), "%s/functions/hid.usb0/dev", gadget_path);
    f = fopen(dev_path, "r");
    if (!f) {
        return false;
    }
    if (fscanf(f, "%d:%d", major0, minor0) != 2) {
        fclose(f);
        return false;
    }
    fclose(f);
    
    // hidg1
    snprintf(dev_path, sizeof(dev_path), "%s/functions/hid.usb1/dev", gadget_path);
    f = fopen(dev_path, "r");
    if (!f) {
        return false;
    }
    if (fscanf(f, "%d:%d", major1, minor1) != 2) {
        fclose(f);
        return false;
    }
    fclose(f);
    
    return true;
}

static bool ensure_device_exists(const char* device) {
    if (check_device_exists(device)) {
        return true;
    }
    
    // 设备不存在，尝试创建
    printf("[USB-HID] 设备节点不存在: %s\n", device);
    printf("[USB-HID] 尝试自动创建...\n");
    
    int major0, minor0, major1, minor1;
    if (!get_device_numbers(&major0, &minor0, &major1, &minor1)) {
        printf("[USB-HID] 错误: 无法获取设备号\n");
        printf("[USB-HID] 请运行: sudo bash /home/ztl/github/RK/scripts/startup-check.sh\n");
        return false;
    }
    
    // 创建两个设备
    if (strcmp(device, "/dev/hidg0") == 0) {
        return create_hid_device("/dev/hidg0", major0, minor0);
    } else if (strcmp(device, "/dev/hidg1") == 0) {
        return create_hid_device("/dev/hidg1", major1, minor1);
    }
    
    return false;
}

// ========== 鼠标 ==========

static int hidg0_fd = -1;
static bool mouse_initialized = false;

int usb_hid_mouse_init(void) {
    // 检查并创建设备节点
    if (!ensure_device_exists("/dev/hidg0")) {
        fprintf(stderr, "[USB-HID] 错误: 无法创建 /dev/hidg0\n");
        fprintf(stderr, "[USB-HID] 请运行: sudo bash /home/ztl/github/RK/scripts/startup-check.sh\n");
        return -1;
    }
    
    hidg0_fd = open("/dev/hidg0", O_WRONLY);
    if (hidg0_fd < 0) {
        perror("open /dev/hidg0");
        return -1;
    }
    mouse_initialized = true;
    printf("[USB-HID] 鼠标初始化成功: /dev/hidg0\n");
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
    
    // 直接发送数据，不检查OTG状态
    ssize_t n = write(hidg0_fd, report, sizeof(*report));
    if (n != sizeof(*report)) {
        // 简单重连
        close(hidg0_fd);
        hidg0_fd = open("/dev/hidg0", O_WRONLY);
        if (hidg0_fd >= 0) {
            // 重试
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
    // 检查并创建设备节点
    if (!ensure_device_exists("/dev/hidg1")) {
        fprintf(stderr, "[USB-HID] 错误: 无法创建 /dev/hidg1\n");
        fprintf(stderr, "[USB-HID] 请运行: sudo bash /home/ztl/github/RK/scripts/startup-check.sh\n");
        return -1;
    }
    
    hidg1_fd = open("/dev/hidg1", O_WRONLY);
    if (hidg1_fd < 0) {
        perror("open /dev/hidg1");
        return -1;
    }
    keyboard_initialized = true;
    printf("[USB-HID] 键盘初始化成功: /dev/hidg1\n");
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
    
    // 直接发送数据，不检查OTG状态
    ssize_t n = write(hidg1_fd, report, sizeof(*report));
    if (n != sizeof(*report)) {
        // 简单重连
        close(hidg1_fd);
        hidg1_fd = open("/dev/hidg1", O_WRONLY);
        if (hidg1_fd >= 0) {
            // 重试
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
