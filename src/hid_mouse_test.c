/*
 * USB HID Mouse Test
 * Moves the cursor in a square pattern to test the virtual mouse
 */

#include "usb_hid.h"
#include <stdio.h>
#include <unistd.h>

int main(int argc, char** argv) {
    printf("USB HID Virtual Mouse Test\n");

    if (usb_hid_mouse_init() != 0) {
        fprintf(stderr, "Failed to initialize USB HID mouse\n");
        fprintf(stderr, "\nMake sure you have:\n");
        fprintf(stderr, "1. Enabled OTG mode: echo otg > /sys/devices/platform/fd5d0000.syscon/fd5d0000.syscon:usb2-phy@0/otg_mode\n");
        fprintf(stderr, "2. Run configure script as root: sudo ../scripts/configure-hid-gadget.sh\n");
        return 1;
    }

    printf("Mouse initialized, moving cursor in square pattern...\n");
    printf("Press Ctrl+C to exit\n");

    // Move in a square pattern
    while (1) {
        // Move right
        for (int i = 0; i < 10; i++) {
            usb_hid_mouse_move(10, 0, 0);
            usleep(100000);
        }
        // Move down
        for (int i = 0; i < 10; i++) {
            usb_hid_mouse_move(0, 10, 0);
            usleep(100000);
        }
        // Move left
        for (int i = 0; i < 10; i++) {
            usb_hid_mouse_move(-10, 0, 0);
            usleep(100000);
        }
        // Move up
        for (int i = 0; i < 10; i++) {
            usb_hid_mouse_move(0, -10, 0);
            usleep(100000);
        }
    }

    usb_hid_mouse_exit();
    return 0;
}
