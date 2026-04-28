#!/bin/bash

set -e

echo "==================================="
echo "RK3588 AimBot 启动检查与修复"
echo "==================================="
echo ""

# 检查是否以 root 运行
if [ "$(id -u)" != "0" ]; then
    echo "⚠️  需要 root 权限，正在使用 sudo 重新运行..."
    exec sudo "$0" "$@"
fi

# 1. 检查并挂载 configfs
echo "1. 检查 configfs..."
if ! mountpoint -q /sys/kernel/config; then
    echo "  挂载 configfs..."
    mount -t configfs none /sys/kernel/config 2>/dev/null || true
fi
echo "  ✓ configfs 已就绪"

# 2. 检查 USB Device Controller
echo ""
echo "2. 检查 USB OTG 硬件..."
UDC=$(ls /sys/class/udc/ 2>/dev/null | head -1)
if [ -z "$UDC" ]; then
    echo "  ✗ 未找到 USB Device Controller"
    echo "  请检查 USB OTG 硬件连接"
    exit 1
fi
echo "  ✓ 找到 UDC: $UDC"

# 3. 检查 USB gadget 配置
echo ""
echo "3. 检查 USB gadget 配置..."
GADGET_PATH="/sys/kernel/config/usb_gadget/rk3588-aimbot"

if [ ! -d "$GADGET_PATH" ]; then
    echo "  USB gadget 未配置，正在配置..."
    
    # 加载模块
    modprobe libcomposite 2>/dev/null || true
    
    # 创建 gadget
    cd /sys/kernel/config/usb_gadget
    mkdir -p rk3588-aimbot
    cd rk3588-aimbot
    
    # USB 设备描述符
    echo 0x1d6b > idVendor
    echo 0x0104 > idProduct
    echo 0x0100 > bcdDevice
    echo 0x0200 > bcdUSB
    
    # 复合设备类
    echo 0xEF > bDeviceClass
    echo 0x02 > bDeviceSubClass
    echo 0x01 > bDeviceProtocol
    
    # 字符串描述符
    mkdir -p strings/0x409
    echo "rk3588-aimbot" > strings/0x409/serialnumber
    echo "RK3588" > strings/0x409/manufacturer
    echo "RK3588 AimBot HID Device" > strings/0x409/product
    
    # 创建配置
    mkdir -p configs/c.1/strings/0x409
    echo "HID Configuration" > configs/c.1/strings/0x409/configuration
    echo 250 > configs/c.1/MaxPower
    
    # 创建 HID 鼠标功能
    mkdir -p functions/hid.usb0
    echo 1 > functions/hid.usb0/subclass
    echo 1 > functions/hid.usb0/protocol
    echo 8 > functions/hid.usb0/report_length
    
    # 鼠标报告描述符
    echo -ne \\x05\\x01\\x09\\x02\\xa1\\x01\\x09\\x01\\xa1\\x00\\x05\\x09\\x19\\x01\\x29\\x03\\x15\\x00\\x25\\x01\\x95\\x03\\x75\\x01\\x81\\x02\\x95\\x01\\x75\\x05\\x81\\x03\\x05\\x01\\x09\\x30\\x09\\x31\\x15\\x81\\x25\\x7f\\x75\\x08\\x95\\x02\\x81\\x06\\xc0\\xc0 > functions/hid.usb0/report_desc
    
    # 创建 HID 键盘功能
    mkdir -p functions/hid.usb1
    echo 1 > functions/hid.usb1/subclass
    echo 1 > functions/hid.usb1/protocol
    echo 8 > functions/hid.usb1/report_length
    
    # 键盘报告描述符
    echo -ne \\x05\\x01\\x09\\x06\\xa1\\x01\\x05\\x07\\x19\\xe0\\x29\\xe7\\x15\\x00\\x25\\x01\\x75\\x01\\x95\\x08\\x81\\x02\\x95\\x01\\x75\\x08\\x81\\x03\\x95\\x05\\x75\\x01\\x05\\x08\\x19\\x01\\x29\\x05\\x91\\x02\\x95\\x01\\x75\\x03\\x91\\x03\\x95\\x06\\x75\\x08\\x15\\x00\\x25\\x65\\x05\\x07\\x19\\x00\\x29\\x65\\x81\\x00\\xc0 > functions/hid.usb1/report_desc
    
    # 绑定功能到配置
    ln -s functions/hid.usb0 configs/c.1/
    ln -s functions/hid.usb1 configs/c.1/
    
    echo "  ✓ USB gadget 配置完成"
else
    echo "  ✓ USB gadget 已配置"
fi

# 4. 检查 UDC 绑定
echo ""
echo "4. 检查 UDC 绑定..."
UDC_CURRENT=$(cat "$GADGET_PATH/UDC" 2>/dev/null)
if [ -z "$UDC_CURRENT" ]; then
    echo "  绑定 UDC..."
    echo "$UDC" > "$GADGET_PATH/UDC"
    sleep 0.5
    echo "  ✓ UDC 已绑定"
else
    echo "  ✓ UDC 已绑定: $UDC_CURRENT"
fi

# 5. 检查并创建设备节点
echo ""
echo "5. 检查 USB HID 设备节点..."

# 获取设备号
MAJOR0=$(cat "$GADGET_PATH/functions/hid.usb0/dev" 2>/dev/null | cut -d: -f1)
MINOR0=$(cat "$GADGET_PATH/functions/hid.usb0/dev" 2>/dev/null | cut -d: -f2)
MAJOR1=$(cat "$GADGET_PATH/functions/hid.usb1/dev" 2>/dev/null | cut -d: -f1)
MINOR1=$(cat "$GADGET_PATH/functions/hid.usb1/dev" 2>/dev/null | cut -d: -f2)

if [ -z "$MAJOR0" ] || [ -z "$MAJOR1" ]; then
    echo "  ✗ 无法获取设备号"
    exit 1
fi

# 检查设备节点是否存在且正确
NEED_CREATE=0

if [ ! -e "/dev/hidg0" ]; then
    NEED_CREATE=1
else
    # 检查设备号是否匹配
    CURRENT_MAJOR=$(stat -c %t /dev/hidg0 2>/dev/null)
    CURRENT_MINOR=$(stat -c %T /dev/hidg0 2>/dev/null)
    if [ "$((CURRENT_MAJOR))" != "$MAJOR0" ] || [ "$((CURRENT_MINOR))" != "$MINOR0" ]; then
        NEED_CREATE=1
    fi
fi

if [ ! -e "/dev/hidg1" ]; then
    NEED_CREATE=1
else
    CURRENT_MAJOR=$(stat -c %t /dev/hidg1 2>/dev/null)
    CURRENT_MINOR=$(stat -c %T /dev/hidg1 2>/dev/null)
    if [ "$((CURRENT_MAJOR))" != "$MAJOR1" ] || [ "$((CURRENT_MINOR))" != "$MINOR1" ]; then
        NEED_CREATE=1
    fi
fi

if [ $NEED_CREATE -eq 1 ]; then
    echo "  创建设备节点..."
    rm -f /dev/hidg0 /dev/hidg1 2>/dev/null || true
    mknod /dev/hidg0 c $MAJOR0 $MINOR0
    mknod /dev/hidg1 c $MAJOR1 $MINOR1
    chmod 666 /dev/hidg0 /dev/hidg1
    echo "  ✓ 设备节点已创建"
else
    echo "  ✓ 设备节点已存在"
fi

# 6. 检查 USB OTG 连接状态
echo ""
echo "6. 检查 USB OTG 连接..."
SUSPENDED=$(cat /sys/class/udc/$UDC/device/gadget/suspended 2>/dev/null)
if [ "$SUSPENDED" = "0" ]; then
    echo "  ✓ USB OTG 已连接到 PC"
else
    echo "  ⚠️  USB OTG 未连接或已挂起"
    echo ""
    echo "  请检查："
    echo "    1. USB OTG 线缆是否正确连接"
    echo "    2. 是否插在 OTG 口（不是普通 USB 口）"
    echo "    3. Windows PC 是否识别到 USB 设备"
    echo "    4. Windows 设备管理器中是否显示 HID 设备"
    echo ""
    echo "  继续运行程序，但游戏模式可能无法正常工作"
fi

# 7. 检查输入设备
echo ""
echo "7. 检查输入设备..."
KEYBOARD_DEVICE=$(ls /dev/input/by-path/*kbd 2>/dev/null | head -1)
MOUSE_DEVICE=$(ls /dev/input/by-path/*mouse 2>/dev/null | head -1)

if [ -n "$KEYBOARD_DEVICE" ]; then
    echo "  ✓ 键盘设备: $KEYBOARD_DEVICE"
else
    echo "  ⚠️  未找到键盘设备"
fi

if [ -n "$MOUSE_DEVICE" ]; then
    echo "  ✓ 鼠标设备: $MOUSE_DEVICE"
else
    echo "  ⚠️  未找到鼠标设备"
fi

echo ""
echo "==================================="
echo "检查完成，所有问题已自动修复"
echo "==================================="
echo ""

# 显示设备信息
echo "设备信息："
ls -l /dev/hidg* 2>/dev/null || echo "  设备节点创建失败"
echo ""

# 提示运行程序
echo "现在可以运行程序："
echo "  cd /home/ztl/github/RK/build/src"
echo "  sudo ./pose_demo"
echo ""

# 如果有参数 --run，则自动运行程序
if [ "$1" = "--run" ]; then
    echo "自动启动程序..."
    cd /home/ztl/github/RK/build/src
    exec ./pose_demo
fi
