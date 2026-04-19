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
echo -n \
\\x05\\x01\
\\x09\\x02\
\\xa1\\x01\
\\x09\\x01\
\\xa1\\x00\
\\x05\\x09\
\\x19\\x01\
\\x29\\x03\
\\x15\\x00\
\\x25\\x01\
\\x95\\x03\
\\x75\\x01\
\\x81\\x02\
\\x95\\x01\
\\x75\\x05\
\\x81\\x03\
\\x05\\x01\
\\x09\\x30\
\\x09\\x31\
\\x15\\x81\
\\x25\\x7f\
\\x75\\x08\
\\x95\\x02\
\\x81\\x06\
\\x09\\x38\
\\x15\\x81\
\\x25\\x7f\
\\x75\\x08\
\\x95\\x01\
\\x81\\x06\
\\xc0\
\\xc0 \
> functions/hid.usb0/report_desc

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
