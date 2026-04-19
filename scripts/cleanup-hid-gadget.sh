#!/bin/bash
# Cleanup USB HID gadget configuration

set -e

# Check if we're root
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run as root" 1>&2
    exit 1
fi

cd /sys/kernel/config/usb_gadget/rk3588-aimbot

# Unbind
echo "" > UDC

# Remove symlink
rm -f configs/c.1/hid.usb0

# Remove directories
rm -rf configs/c.1/strings/0x409
rmdir configs/c.1
rm -rf functions/hid.usb0
rm -rf strings/0x409

cd ..
rmdir rk3588-aimbot

echo "Cleanup done"
