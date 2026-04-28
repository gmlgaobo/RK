#!/bin/bash

echo "==================================="
echo "RK3588 AimBot USB Gadget 重置"
echo "==================================="
echo ""

# 检查 root 权限
if [ "$(id -u)" != "0" ]; then
    echo "需要 root 权限，正在使用 sudo..."
    exec sudo "$0" "$@"
fi

# 1. 卸载 UDC
echo "1. 卸载 UDC..."
UDC=$(cat /sys/kernel/config/usb_gadget/rk3588-aimbot/UDC 2>/dev/null)
if [ -n "$UDC" ]; then
    echo "" > /sys/kernel/config/usb_gadget/rk3588-aimbot/UDC 2>/dev/null || true
    echo "  ✓ UDC 已卸载"
else
    echo "  - UDC 未绑定"
fi

# 2. 删除设备节点
echo ""
echo "2. 删除设备节点..."
rm -f /dev/hidg0 /dev/hidg1 2>/dev/null || true
echo "  ✓ 设备节点已删除"

# 3. 删除 gadget 配置
echo ""
echo "3. 删除 gadget 配置..."
GADGET_PATH="/sys/kernel/config/usb_gadget/rk3588-aimbot"
if [ -d "$GADGET_PATH" ]; then
    # 删除符号链接
    rm -f "$GADGET_PATH/configs/c.1/hid.usb0" 2>/dev/null || true
    rm -f "$GADGET_PATH/configs/c.1/hid.usb1" 2>/dev/null || true
    
    # 删除目录
    rmdir "$GADGET_PATH/functions/hid.usb0" 2>/dev/null || true
    rmdir "$GADGET_PATH/functions/hid.usb1" 2>/dev/null || true
    rmdir "$GADGET_PATH/configs/c.1/strings/0x409" 2>/dev/null || true
    rmdir "$GADGET_PATH/configs/c.1" 2>/dev/null || true
    rmdir "$GADGET_PATH/strings/0x409" 2>/dev/null || true
    rmdir "$GADGET_PATH" 2>/dev/null || true
    
    echo "  ✓ Gadget 配置已删除"
else
    echo "  - Gadget 未配置"
fi

echo ""
echo "==================================="
echo "重置完成"
echo "==================================="
echo ""
echo "现在可以运行："
echo "  sudo bash /home/ztl/github/RK/scripts/startup-check.sh"
echo ""
