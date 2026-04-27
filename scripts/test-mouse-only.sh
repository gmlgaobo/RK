#!/bin/bash
# 仅配置鼠标，最小化测试

set -e

if [ "$(id -u)" != "0" ]; then
    echo "请用 sudo 运行"
    exit 1
fi

echo "=== 仅配置 USB HID 鼠标测试 ==="
echo ""

# 先清理旧 gadget
if [ -d /sys/kernel/config/usb_gadget/rk3588-aimbot ]; then
    cd /sys/kernel/config/usb_gadget/rk3588-aimbot
    echo > UDC 2>/dev/null || true
    sleep 0.3
    cd ..
    rmdir rk3588-aimbot/configs/c.1/hid.usb0 2>/dev/null || true
    rmdir rk3588-aimbot/configs/c.1/strings/0x409 2>/dev/null || true
    rmdir rk3588-aimbot/configs/c.1 2>/dev/null || true
    rmdir rk3588-aimbot/functions/hid.usb0 2>/dev/null || true
    rmdir rk3588-aimbot/strings/0x409 2>/dev/null || true
    rmdir rk3588-aimbot 2>/dev/null || true
fi
sleep 1

# 挂载 configfs
if ! mountpoint -q /sys/kernel/config; then
    mkdir -p /sys/kernel/config
    mount -t configfs none /sys/kernel/config
fi

cd /sys/kernel/config/usb_gadget
mkdir -p rk3588-aimbot
cd rk3588-aimbot

echo 0x1d6b > idVendor
echo 0x0101 > idProduct
echo 0x0100 > bcdDevice
echo 0x0200 > bcdUSB

mkdir -p strings/0x409
echo "0123456789" > strings/0x409/serialnumber
echo "RK3588" > strings/0x409/manufacturer
echo "AI Aimbot Mouse" > strings/0x409/product

mkdir -p functions/hid.usb0
echo 4 > functions/hid.usb0/protocol
echo 2 > functions/hid.usb0/subclass
echo 4 > functions/hid.usb0/report_length
printf "\x05\x01\x09\x02\xa1\x01\x09\x01\xa1\x00\x05\x09\x19\x01\x29\x03\x15\x00\x25\x01\x95\x03\x75\x01\x81\x02\x95\x01\x75\x05\x81\x03\x05\x01\x09\x30\x09\x31\x15\x81\x25\x7f\x75\x08\x95\x02\x81\x06\x09\x38\x15\x81\x25\x7f\x75\x08\x95\x01\x81\x06\xc0\xc0" > functions/hid.usb0/report_desc

mkdir -p configs/c.1/strings/0x409
echo "Aimbot HID Mouse" > configs/c.1/strings/0x409/configuration
echo 120 > configs/c.1/MaxPower

ln -s functions/hid.usb0 configs/c.1/

UDC=$(ls /sys/class/udc/ | head -1)
echo "绑定 UDC: $UDC"
echo "$UDC" > UDC
sleep 1

if [ -e /dev/hidg0 ]; then
    chmod 666 /dev/hidg0
    echo "/dev/hidg0 配置成功"
    STATE=$(cat /sys/class/udc/$UDC/state 2>/dev/null || echo "unknown")
    echo "UDC 状态: $STATE"
fi
echo ""
echo "请重新插拔 USB OTG 线，然后测试！"
