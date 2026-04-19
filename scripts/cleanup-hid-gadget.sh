#!/bin/bash
# Cleanup USB HID gadget configuration

set -e

# Check if we're root
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run as root" 1>&2
    exit 1
fi

# configfs doesn't allow removing non-empty directories, need to clean in correct order
if [ -d /sys/kernel/config/usb_gadget/rk3588-aimbot ]; then
    cd /sys/kernel/config/usb_gadget/rk3588-aimbot

    # Unbind from UDC
    echo "" > UDC 2>/dev/null || true

    # Remove symlink from config to function
    rm -f configs/c.1/hid.usb0 2>/dev/null || true

    # Cleanup config
    rm -rf configs/c.1/strings/0x409 2>/dev/null || true
    rmdir configs/c.1 2>/dev/null || true

    # Cleanup function
    rmdir functions/hid.usb0 2>/dev/null || true

    # Cleanup strings
    rm -rf strings/0x409 2>/dev/null || true

    cd ..
    rmdir rk3588-aimbot 2>/dev/null || true
fi

echo "Cleanup done"
