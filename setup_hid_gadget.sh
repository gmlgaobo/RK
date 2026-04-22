#!/bin/bash

# RK3588 USB HID Gadget 配置脚本
# 使用方法: sudo bash setup_hid_gadget.sh

set -e

# 检查是否为 root 用户
if [ "$EUID" -ne 0 ]; then 
    echo "请使用 sudo 运行此脚本"
    echo "用法: sudo bash $0"
    exit 1
fi

# 配置参数
GADGET_NAME="rk3588_mouse"
USB_DIR="/sys/kernel/config/usb_gadget/$GADGET_NAME"

echo "=== RK3588 USB HID Gadget 配置 ==="

# 卸载旧配置（如果存在）
if [ -d "$USB_DIR" ]; then
    echo "清理旧配置..."
    # 尝试移除绑定
    if [ -e "$USB_DIR/UDC" ]; then
        echo "" > "$USB_DIR/UDC" 2>/dev/null || true
    fi
    # 移除配置
    if [ -d "$USB_DIR/configs/c.1/strings/0x409" ]; then
        rmdir "$USB_DIR/configs/c.1/strings/0x409" 2>/dev/null || true
    fi
    # 移除函数链接
    if [ -L "$USB_DIR/configs/c.1/hid.usb0" ]; then
        rm "$USB_DIR/configs/c.1/hid.usb0" 2>/dev/null || true
    fi
    # 移除配置目录
    if [ -d "$USB_DIR/configs/c.1" ]; then
        rmdir "$USB_DIR/configs/c.1" 2>/dev/null || true
    fi
    # 移除函数目录
    if [ -d "$USB_DIR/functions/hid.usb0" ]; then
        rmdir "$USB_DIR/functions/hid.usb0" 2>/dev/null || true
    fi
    # 移除字符串目录
    if [ -d "$USB_DIR/strings/0x409" ]; then
        rmdir "$USB_DIR/strings/0x409" 2>/dev/null || true
    fi
    # 移除 gadget 目录
    if [ -d "$USB_DIR" ]; then
        rmdir "$USB_DIR" 2>/dev/null || true
    fi
fi

# 创建 gadget 目录
echo "创建 USB gadget..."
mkdir -p "$USB_DIR"

# 设置 Vendor ID 和 Product ID (使用标准 Linux USB Gadget ID)
echo 0x1d6b > "$USB_DIR/idVendor"   # Linux Foundation
echo 0x0104 > "$USB_DIR/idProduct"  # Multi-function Composite Gadget
echo 0x0100 > "$USB_DIR/bcdDevice"  # Device version 1.0
echo 0x0200 > "$USB_DIR/bcdUSB"     # USB 2.0

# 配置字符串描述符
mkdir -p "$USB_DIR/strings/0x409"
echo "Radxa" > "$USB_DIR/strings/0x409/manufacturer"
echo "RK3588 HID Mouse" > "$USB_DIR/strings/0x409/product"
echo "00000001" > "$USB_DIR/strings/0x409/serialnumber"

# 创建 HID 函数
echo "创建 HID 函数..."
mkdir -p "$USB_DIR/functions/hid.usb0"

# 写入 HID Report Descriptor (相对移动鼠标)
# Report Descriptor 结构:
# - Buttons (3 bits) + Padding (5 bits)
# - X Movement (8 bits, relative)
# - Y Movement (8 bits, relative)
# - Scroll Wheel (8 bits, relative)
echo -ne "\x05\x01\x09\x02\xa1\x01\x09\x01\xa1\x00\x05\x09\x19\x01\x29\x03\x15\x00\x25\x01\x95\x03\x75\x01\x81\x02\x95\x01\x75\x05\x81\x03\x05\x01\x09\x30\x09\x31\x15\x81\x25\x7f\x75\x08\x95\x02\x81\x06\x09\x38\x15\x81\x25\x7f\x75\x08\x95\x01\x81\x06\xc0\xc0" > "$USB_DIR/functions/hid.usb0/report_desc"

# 设置 HID 参数
echo 4 > "$USB_DIR/functions/hid.usb0/report_length"  # Report 长度 = 4 bytes
echo 1 > "$USB_DIR/functions/hid.usb0/subclass"       # Boot Interface Subclass
echo 1 > "$USB_DIR/functions/hid.usb0/protocol"       # Mouse Protocol

# 创建配置
echo "创建配置..."
mkdir -p "$USB_DIR/configs/c.1/strings/0x409"
echo "HID Mouse" > "$USB_DIR/configs/c.1/strings/0x409/configuration"
echo 250 > "$USB_DIR/configs/c.1/MaxPower"  # 250mA

# 绑定函数到配置
echo "绑定函数..."
ln -sf "$USB_DIR/functions/hid.usb0" "$USB_DIR/configs/c.1/"

# 查找并绑定 UDC
echo "绑定 UDC..."
UDC_DIR="/sys/class/udc"
if [ -d "$UDC_DIR" ]; then
    UDC_DEVICES=$(ls "$UDC_DIR")
    if [ -n "$UDC_DEVICES" ]; then
        # 选择第一个 UDC 设备
        FIRST_UDC=$(echo "$UDC_DEVICES" | head -1)
        echo "找到 UDC: $FIRST_UDC"
        echo "$FIRST_UDC" > "$USB_DIR/UDC"
    else
        echo "警告: 未找到 UDC 设备"
        echo "请检查 USB OTG 模式是否正确设置"
    fi
else
    echo "警告: UDC 目录不存在"
fi

# 等待设备创建
echo "等待 /dev/hidg0 创建..."
sleep 2

# 检查设备是否创建成功
if [ -e "/dev/hidg0" ]; then
    echo "✅ /dev/hidg0 创建成功!"
    
    # 设置权限
    echo "设置权限..."
    chmod 666 /dev/hidg0
    echo "✅ /dev/hidg0 权限设置为 666"
    
    # 创建 udev 规则（可选，永久生效）
    UDEV_RULE="/etc/udev/rules.d/99-hidg.rules"
    if [ ! -e "$UDEV_RULE" ]; then
        echo "创建 udev 规则..."
        echo 'SUBSYSTEM=="misc", KERNEL=="hidg*", MODE="0666"' > "$UDEV_RULE"
        udevadm control --reload-rules
        udevadm trigger
        echo "✅ udev 规则已创建"
    fi
    
    echo ""
    echo "=== 配置完成 ==="
    echo "现在可以运行 pose_demo 了"
    echo "按 'a' 键开启吸附辅助"
    echo ""
else
    echo "❌ 失败: /dev/hidg0 未创建"
    echo ""
    echo "可能的原因:"
    echo "1. USB OTG 模式未设置"
    echo "2. 内核模块未加载"
    echo ""
    echo "尝试的补救措施:"
    echo "1. 检查 OTG 模式: cat /sys/class/udc/*/state"
    echo "2. 加载内核模块: modprobe libcomposite"
    echo ""
    exit 1
fi
