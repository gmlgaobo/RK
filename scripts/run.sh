#!/bin/bash

echo "==================================="
echo "RK3588 AimBot 快速启动"
echo "==================================="
echo ""

# 运行启动检查
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$SCRIPT_DIR/startup-check.sh"

# 检查是否成功
if [ $? -ne 0 ]; then
    echo ""
    echo "✗ 启动检查失败，请检查错误信息"
    exit 1
fi

echo ""
echo "启动程序..."
echo ""

# 运行程序
cd /home/ztl/github/RK/build/src
exec ./pose_demo
