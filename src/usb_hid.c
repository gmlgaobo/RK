/*
 * RK3588 USB HID Virtual Mouse
 * Uses kernel libcomposite USB gadget to create a HID mouse device
 * After config, send reports to /dev/hidg0
 */

#include "usb_hid.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// HID mouse Report Descriptor
// Standard relative mouse report descriptor
static const uint8_t mouse_report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (1)
    0x29, 0x03,        //     Usage Maximum (3)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x03,        //     Input (Const,Var,Abs) - 5 bit padding
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x06,        //     Input (Data,Var,Rel) - relative movement
    0x09, 0x38,        //     Usage (Scroll Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0xC0               // End Collection
};

// Our report struct matches exactly what the descriptor says:
// 3 buttons (3 bits) + 5 bits padding + 8bit dx + 8bit dy + 8bit scroll = 4 bytes total

static int hidg_fd = -1;
static bool initialized = false;

// HID mouse gadget needs to be configured via configfs
// This just opens /dev/hidg0 for sending reports
int usb_hid_mouse_init(void) {
    // Open the HID gadget device
    hidg_fd = open("/dev/hidg0", O_WRONLY);
    if (hidg_fd < 0) {
        perror("open /dev/hidg0");
        fprintf(stderr, "\n");
        fprintf(stderr, "To create HID gadget device, you need to configure libcomposite:\n");
        fprintf(stderr, "1. Enable USB OTG mode: echo otg > /sys/devices/platform/fd5d0000.syscon/fd5d0000.syscon:usb2-phy@0/otg_mode\n");
        fprintf(stderr, "2. Load kernel modules: modprobe libcomposite\n");
        fprintf(stderr, "3. Create gadget via configfs (see README for full script)\n");
        return -1;
    }

    initialized = true;
    printf("USB HID mouse initialized: /dev/hidg0\n");
    return 0;
}

void usb_hid_mouse_exit(void) {
    if (hidg_fd >= 0) {
        close(hidg_fd);
        hidg_fd = -1;
    }
    initialized = false;
}

bool usb_hid_mouse_ready(void) {
    return initialized;
}

int usb_hid_mouse_move(int8_t dx, int8_t dy, uint8_t buttons) {
    if (!initialized || hidg_fd < 0) {
        return -1;
    }

    hid_mouse_report_t report;
    report.buttons = buttons;
    report.dx = dx;
    report.dy = dy;
    report.scroll = 0;

    // Send report to /dev/hidg0
    ssize_t n = write(hidg_fd, &report, sizeof(report));
    if (n != sizeof(report)) {
        perror("write /dev/hidg0");
        return -1;
    }

    return 0;
}
