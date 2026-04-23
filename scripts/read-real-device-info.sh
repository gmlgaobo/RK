#!/bin/bash
# 读取真实连接的键盘和鼠标硬件信息

echo "=========================================="
echo "  真实USB设备信息读取工具"
echo "=========================================="
echo ""

# 检查是否root
if [ "$EUID" -ne 0 ]; then 
    echo "请使用 root 权限运行: sudo $0"
    exit 1
fi

# 1. 列出所有USB设备
echo "=== 所有连接的USB设备 ==="
lsusb
echo ""

# 2. 列出输入设备
echo "=== 输入设备列表 (/dev/input/) ==="
ls -l /dev/input/
echo ""

echo "=== 输入设备详细信息 ==="
for dev in /dev/input/event*; do
    if [ -e "$dev" ]; then
        echo "--- $dev ---"
        # 尝试获取设备名称
        DEV_NAME=$(udevadm info -q name -n $dev 2>/dev/null || echo "unknown")
        echo "  名称: $DEV_NAME"
        
        # 尝试读取VID/PID（如果是USB设备）
        SYS_PATH=$(udevadm info -q path -n $dev 2>/dev/null)
        if [ -n "$SYS_PATH" ]; then
            # 向上查找USB设备
            USB_PATH="/sys$SYS_PATH"
            while [ "$USB_PATH" != "/sys" ]; do
                if [ -f "$USB_PATH/idVendor" ] && [ -f "$USB_PATH/idProduct" ]; then
                    VID=$(cat "$USB_PATH/idVendor")
                    PID=$(cat "$USB_PATH/idProduct")
                    echo "  VID:PID = $VID:$PID"
                    if [ -f "$USB_PATH/manufacturer" ]; then
                        echo "  厂商: $(cat "$USB_PATH/manufacturer")"
                    fi
                    if [ -f "$USB_PATH/product" ]; then
                        echo "  产品: $(cat "$USB_PATH/product")"
                    fi
                    break
                fi
                USB_PATH=$(dirname "$USB_PATH")
            done
        fi
        
        # 尝试用evtest获取信息
        if command -v evtest &> /dev/null; then
            echo "  功能键(使用 evtest 查看更多): evtest $dev"
        fi
    fi
done
echo ""

# 3. 尝试挂载debugfs读取HID report descriptor
echo "=== HID Report Descriptor (需要debugfs) ==="
if [ ! -d /sys/kernel/debug/hid ]; then
    echo "挂载 debugfs..."
    mount -t debugfs none /sys/kernel/debug
fi

if [ -d /sys/kernel/debug/hid ]; then
    for hid_dev in /sys/kernel/debug/hid/*; do
        if [ -d "$hid_dev" ] && [ -f "$hid_dev/rdesc" ]; then
            echo "--- $(basename $hid_dev) ---"
            echo "  Report Descriptor (hex):"
            hexdump -C "$hid_dev/rdesc" | head -20
            echo ""
        fi
    done
else
    echo "  无法访问 /sys/kernel/debug/hid"
fi

echo ""
echo "=========================================="
echo "  信息收集完成！"
echo "=========================================="
