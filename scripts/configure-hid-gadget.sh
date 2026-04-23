#!/bin/bash
# Configure USB HID mouse + keyboard gadget using libcomposite
# Run as root

set -e

# Check if we're root
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run as root" 1>&2
    exit 1
fi

# Unbind any existing gadget first (silently fail if none)
if [ -d /sys/kernel/config/usb_gadget/rk3588-aimbot ]; then
    cd /sys/kernel/config/usb_gadget/rk3588-aimbot
    echo > UDC 2>/dev/null || true
    cd ..
    rmdir rk3588-aimbot/configs/c.1/hid.usb0 2>/dev/null || true
    rmdir rk3588-aimbot/configs/c.1/hid.usb1 2>/dev/null || true
    rmdir rk3588-aimbot/configs/c.1/strings/0x409 2>/dev/null || true
    rmdir rk3588-aimbot/configs/c.1 2>/dev/null || true
    rmdir rk3588-aimbot/functions/hid.usb0 2>/dev/null || true
    rmdir rk3588-aimbot/functions/hid.usb1 2>/dev/null || true
    rmdir rk3588-aimbot/strings/0x409 2>/dev/null || true
    rmdir rk3588-aimbot 2>/dev/null || true
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
echo "AI Aimbot Mouse+Keyboard" > strings/0x409/product

# Create HID mouse function (hid.usb0)
mkdir -p functions/hid.usb0
echo 4 > functions/hid.usb0/protocol  # mouse
echo 2 > functions/hid.usb0/subclass  # boot
echo 4 > functions/hid.usb0/report_length  # 4 bytes report

# Write mouse report descriptor
printf "\x05\x01\x09\x02\xa1\x01\x09\x01\xa1\x00\x05\x09\x19\x01\x29\x03\x15\x00\x25\x01\x95\x03\x75\x01\x81\x02\x95\x01\x75\x05\x81\x03\x05\x01\x09\x30\x09\x31\x15\x81\x25\x7f\x75\x08\x95\x02\x81\x06\x09\x38\x15\x81\x25\x7f\x75\x08\x95\x01\x81\x06\xc0\xc0" > functions/hid.usb0/report_desc

# Create HID keyboard function (hid.usb1)
mkdir -p functions/hid.usb1
echo 1 > functions/hid.usb1/protocol  # keyboard
echo 2 > functions/hid.usb1/subclass  # boot
echo 8 > functions/hid.usb1/report_length  # 8 bytes report

# Write keyboard report descriptor (standard boot keyboard)
printf "\x05\x01\x09\x06\xa1\x01\x05\x07\x19\xe0\x29\xe7\x15\x00\x25\x01\x75\x01\x95\x08\x81\x02\x75\x08\x95\x01\x81\x03\x75\x01\x95\x05\x05\x08\x19\x01\x29\x05\x91\x02\x75\x03\x95\x01\x91\x03\x75\x08\x95\x06\x05\x07\x19\x00\x29\xff\x81\x00\xc0" > functions/hid.usb1/report_desc

# Create configuration
mkdir -p configs/c.1/strings/0x409
echo "Aimbot HID Mouse+Keyboard" > configs/c.1/strings/0x409/configuration
echo 120 > configs/c.1/MaxPower

# Link functions to configuration
ln -s functions/hid.usb0 configs/c.1/
ln -s functions/hid.usb1 configs/c.1/

# Bind to the USB controller UDC
# Find the first UDC driver
UDC=$(ls /sys/class/udc/ | head -1)
if [ -z "$UDC" ]; then
    echo "No UDC driver found, check USB OTG mode is enabled"
    exit 1
fi
echo "$UDC" > UDC

# Wait for device nodes to appear
sleep 0.5

# Fix permissions so non-root users can access it
OK=true
if [ -e /dev/hidg0 ]; then
    chmod 666 /dev/hidg0
    echo "/dev/hidg0 (mouse) created with permissions 666"
else
    echo "Warning: /dev/hidg0 not found after configuration"
    OK=false
fi

if [ -e /dev/hidg1 ]; then
    chmod 666 /dev/hidg1
    echo "/dev/hidg1 (keyboard) created with permissions 666"
else
    echo "Warning: /dev/hidg1 not found after configuration"
    OK=false
fi

if [ "$OK" = true ]; then
    echo "Gadget configured successfully, UDC: $UDC"
fi
