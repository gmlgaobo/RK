#!/bin/bash
# Configure USB HID mouse gadget using libcomposite
# Run as root

set -e

# Check if we're root
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run as root" 1>&2
    exit 1
fi

# Load libcomposite module
modprobe libcomposite

# Mount configfs if not already mounted
if ! mountpoint -q /sys/kernel/config; then
    mkdir -p /sys/kernel/config
    mount -t configfs none /sys/kernel/config
fi

# Go to gadget directory
cd /sys/kernel/config/usb_gadget
mkdir -p rk3588-aimbot
cd rk3588-aimbot

# idVendor 0x1d6b is Linux Foundation
echo 0x1d6b > idVendor
# idProduct 0x0101 is HID gadget
echo 0x0101 > idProduct
# bcdDevice
echo 0x0100 > bcdDevice

# Create English strings
mkdir -p strings/0x409
echo "0123456789" > strings/0x409/serialnumber
echo "RK3588" > strings/0x409/manufacturer
echo "AI Aimbot Mouse" > strings/0x409/product

# Create HID function
mkdir -p functions/hid.usb0
echo 4 > functions/hid.usb0/protocol  # mouse
echo 2 > functions/hid.usb0/subclass  # boot
echo 4 > functions/hid.usb0/report_length  # 4 bytes report

# Write report descriptor
# The report descriptor is 47 bytes, base64 encoded to avoid shell escape issues
# Original HID report descriptor for 4-byte relative mouse:
# 05 01 09 02 a1 01 09 01 a1 00 05 09 19 01 29 03 15 00 25 01 95 03 75 01 81 02 95 01 75 05 81 03 05 01 09 30 09 31 15 81 25 7f 75 08 95 02 81 06 09 38 15 81 25 7f 75 08 95 01 81 06 c0 c0
echo "AW0BJQmhBASkBqgFgAUkaAZ8AAAAD/8AAAAAQFhAAABAgMQeNFYAcpRjUEvAQ==" | base64 -d > functions/hid.usb0/report_desc

# Create configuration
mkdir -p configs/c.1/strings/0x409
echo "Aimbot HID Mouse" > configs/c.1/strings/0x409/configuration
echo 120 > configs/c.1/MaxPower

# Link function to configuration
ln -s functions/hid.usb0 configs/c.1/

# Bind to the USB controller UDC
# Find the first UDC driver
UDC=$(ls /sys/class/udc/ | head -1)
if [ -z "$UDC" ]; then
    echo "No UDC driver found, check USB OTG mode is enabled"
    exit 1
fi
echo "$UDC" > UDC
echo "Gadget configured successfully, UDC: $UDC"
echo "/dev/hidg0 should be created now"
