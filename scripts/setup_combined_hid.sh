#!/bin/bash

# RK3588 组合 HID Gadget 配置脚本（键盘+鼠标）
# 使用方法: sudo bash setup_combined_hid.sh

set -e

# 检查是否为 root 用户
if [ "$EUID" -ne 0 ]; then 
    echo "请使用 sudo 运行此脚本"
    echo "用法: sudo bash $0"
    exit 1
fi

# 配置参数
GADGET_NAME="rk3588_combo"
USB_DIR="/sys/kernel/config/usb_gadget/$GADGET_NAME"

echo "=== RK3588 组合 HID Gadget 配置 (键盘+鼠标) ==="

# 函数：清理单个 gadget
cleanup_gadget() {
    local gadget_dir=$1
    if [ ! -d "$gadget_dir" ]; then
        return
    fi
    
    echo "清理 gadget: $gadget_dir"
    
    # 1. 解绑 UDC
    if [ -e "$gadget_dir/UDC" ]; then
        echo "" > "$gadget_dir/UDC" 2>/dev/null || true
        sleep 0.5
    fi
    
    # 2. 移除所有配置下的函数链接
    for config_dir in "$gadget_dir"/configs/*; do
        if [ -d "$config_dir" ]; then
            for func_link in "$config_dir"/*.*; do
                if [ -L "$func_link" ]; then
                    rm "$func_link" 2>/dev/null || true
                fi
            done
            # 移除配置的字符串
            if [ -d "$config_dir/strings" ]; then
                for lang_dir in "$config_dir/strings"/*; do
                    if [ -d "$lang_dir" ]; then
                        rmdir "$lang_dir" 2>/dev/null || true
                    fi
                done
                rmdir "$config_dir/strings" 2>/dev/null || true
            fi
            # 移除配置目录
            rmdir "$config_dir" 2>/dev/null || true
        fi
    done
    
    # 3. 移除所有函数
    for func_dir in "$gadget_dir"/functions/*.*; do
        if [ -d "$func_dir" ]; then
            rmdir "$func_dir" 2>/dev/null || true
        fi
    done
    
    # 4. 移除 gadget 的字符串
    if [ -d "$gadget_dir/strings" ]; then
        for lang_dir in "$gadget_dir/strings"/*; do
            if [ -d "$lang_dir" ]; then
                rmdir "$lang_dir" 2>/dev/null || true
            fi
        done
        rmdir "$gadget_dir/strings" 2>/dev/null || true
    fi
    
    # 5. 移除 gadget 目录
    rmdir "$gadget_dir" 2>/dev/null || true
}

# 清理所有旧 gadget
echo "清理所有旧配置..."
for gadget in /sys/kernel/config/usb_gadget/*; do
    if [ -d "$gadget" ] && [ "$(basename "$gadget")" != "." ] && [ "$(basename "$gadget")" != ".." ]; then
        cleanup_gadget "$gadget"
    fi
done

# 等待一下
sleep 0.5

# 创建 gadget 目录
echo "创建 USB gadget..."
mkdir -p "$USB_DIR"

# 设置 Vendor ID 和 Product ID
echo 0x1d6b > "$USB_DIR/idVendor"   # Linux Foundation
echo 0x0104 > "$USB_DIR/idProduct"  # Multi-function Composite Gadget
echo 0x0100 > "$USB_DIR/bcdDevice"  # Device version 1.0
echo 0x0200 > "$USB_DIR/bcdUSB"     # USB 2.0

# 配置字符串描述符
mkdir -p "$USB_DIR/strings/0x409"
echo "Radxa" > "$USB_DIR/strings/0x409/manufacturer"
echo "RK3588 Combo HID" > "$USB_DIR/strings/0x409/product"
echo "00000001" > "$USB_DIR/strings/0x409/serialnumber"

# ========== 创建 HID 函数 0: 鼠标 ==========
echo "创建鼠标 HID 函数..."
mkdir -p "$USB_DIR/functions/hid.usb0"

# 鼠标 Report Descriptor
echo -ne "\x05\x01\x09\x02\xa1\x01\x09\x01\xa1\x00\x05\x09\x19\x01\x29\x03\x15\x00\x25\x01\x95\x03\x75\x01\x81\x02\x95\x01\x75\x05\x81\x03\x05\x01\x09\x30\x09\x31\x15\x81\x25\x7f\x75\x08\x95\x02\x81\x06\x09\x38\x15\x81\x25\x7f\x75\x08\x95\x01\x81\x06\xc0\xc0" > "$USB_DIR/functions/hid.usb0/report_desc"
echo 4 > "$USB_DIR/functions/hid.usb0/report_length"
echo 1 > "$USB_DIR/functions/hid.usb0/subclass"
echo 1 > "$USB_DIR/functions/hid.usb0/protocol"

# ========== 创建 HID 函数 1: 键盘 ==========
echo "创建键盘 HID 函数..."
mkdir -p "$USB_DIR/functions/hid.usb1"

# 键盘 Report Descriptor
echo -ne "\x05\x01\x09\x06\xa1\x01\x05\x07\x19\xe0\x29\xe7\x15\x00\x25\x01\x75\x01\x95\x08\x81\x02\x95\x01\x75\x08\x81\x03\x95\x06\x75\x08\x15\x00\x25\x65\x05\x07\x19\x00\x29\x65\x81\x00\xc0" > "$USB_DIR/functions/hid.usb1/report_desc"
echo 8 > "$USB_DIR/functions/hid.usb1/report_length"
echo 1 > "$USB_DIR/functions/hid.usb1/subclass"
echo 1 > "$USB_DIR/functions/hid.usb1/protocol"

# 创建配置
echo "创建配置..."
mkdir -p "$USB_DIR/configs/c.1/strings/0x409"
echo "Combo HID" > "$USB_DIR/configs/c.1/strings/0x409/configuration"
echo 250 > "$USB_DIR/configs/c.1/MaxPower"

# 绑定函数到配置
echo "绑定函数..."
ln -sf "$USB_DIR/functions/hid.usb0" "$USB_DIR/configs/c.1/"
ln -sf "$USB_DIR/functions/hid.usb1" "$USB_DIR/configs/c.1/"

# 查找并绑定 UDC
echo "绑定 UDC..."
UDC_DIR="/sys/class/udc"
if [ -d "$UDC_DIR" ]; then
    UDC_DEVICES=$(ls "$UDC_DIR")
    if [ -n "$UDC_DEVICES" ]; then
        # 尝试绑定每个 UDC，直到成功
        for UDC in $UDC_DEVICES; do
            echo "尝试绑定 UDC: $UDC"
            if echo "$UDC" > "$USB_DIR/UDC" 2>/dev/null; then
                echo "✅ 成功绑定到 UDC: $UDC"
                break
            else
                echo "   绑定失败，尝试下一个..."
            fi
        done
    else
        echo "警告: 未找到 UDC 设备"
    fi
else
    echo "警告: UDC 目录不存在"
fi

# 等待设备创建
echo "等待设备创建..."
sleep 2

# 检查设备
HIDG0="/dev/hidg0"
HIDG1="/dev/hidg1"

# 如果设备不存在，手动创建
if [ ! -e "$HIDG0" ]; then
    echo "手动创建鼠标设备节点..."
    mknod "$HIDG0" c 511 0 2>/dev/null || echo "警告: 无法创建 $HIDG0 (可能需要root权限)"
fi
if [ ! -e "$HIDG1" ]; then
    echo "手动创建键盘设备节点..."
    mknod "$HIDG1" c 511 1 2>/dev/null || echo "警告: 无法创建 $HIDG1 (可能需要root权限)"
fi

if [ -e "$HIDG0" ] && [ -e "$HIDG1" ]; then
    echo "✅ 设备创建成功!"
    echo "   - 鼠标: $HIDG0"
    echo "   - 键盘: $HIDG1"
    
    # 设置权限
    echo "设置权限..."
    chmod 666 "$HIDG0" 2>/dev/null || echo "警告: 无法设置 $HIDG0 权限"
    chmod 666 "$HIDG1" 2>/dev/null || echo "警告: 无法设置 $HIDG1 权限"
    echo "✅ 权限设置完成"
    
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
    echo "/dev/hidg0 = 鼠标"
    echo "/dev/hidg1 = 键盘"
    echo ""
else
    echo "❌ 失败: 设备未创建"
    echo ""
    echo "可能的原因:"
    echo "1. USB OTG 模式未设置"
    echo "2. 内核模块未加载"
    echo ""
    echo "尝试的补救措施:"
    echo "1. 检查 OTG 模式: cat /sys/class/udc/*/state"
    echo "2. 加载内核模块: modprobe libcomposite"
    echo "3. 重启后重试"
    echo ""
    exit 1
fi
