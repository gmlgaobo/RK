#!/bin/bash
# 一键重新连接 USB Gadget（用于重新插拔 USB OTG 线后）
# 2026-04-25

set -e

if [ "$(id -u)" != "0" ]; then
    echo "请用 sudo 运行: sudo $0"
    exit 1
fi

echo "=== 开始重新连接 USB Gadget ==="
cd /sys/kernel/config/usb_gadget

GADGET_NAME="rk3588-aimbot"

# 1. 先解绑 UDC（如果已绑定）
if [ -d "$GADGET_NAME" ]; then
    echo "[1/3] 解绑旧的 UDC..."
    cd "$GADGET_NAME"
    echo > UDC 2>/dev/null || true
    cd ..
    sleep 0.5
fi

# 2. 重新配置 gadget（用原始脚本）
echo "[2/3] 重新运行配置脚本..."
SCRIPT_DIR=$(dirname "$0")
cd "$SCRIPT_DIR/.."
sudo ./scripts/configure-hid-gadget.sh
sleep 1

# 3. 检查状态
echo "[3/3] 检查连接状态..."
UDC_STATE=$(cat /sys/class/udc/fc000000.usb/state)
echo "    UDC 状态: $UDC_STATE"

echo "---"
echo "请重新插拔 USB OTG 线！"
echo "等待 5 秒后，状态应该变为 configured"
echo "---"

# 等待 10 秒，每 2 秒检查一次
for i in 1 2 3 4 5; do
    sleep 2
    CURRENT_STATE=$(cat /sys/class/udc/fc000000.usb/state)
    echo "  [检查 $i/5] 状态: $CURRENT_STATE"
    if [ "$CURRENT_STATE" = "configured" ] || [ "$CURRENT_STATE" = "addressed" ]; then
        echo ""
        echo "✅ 成功！USB Gadget 已连接！"
        exit 0
    fi
done

echo ""
echo "⚠️ 请手动重新插拔 USB OTG 线！"
echo "然后运行: cat /sys/class/udc/fc000000.usb/state"
echo ""
