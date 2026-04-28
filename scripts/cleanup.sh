#!/bin/bash

echo "==================================="
echo "RK3588 AimBot 进程清理脚本"
echo "==================================="
echo ""

# 查找并杀死 pose_demo 进程
echo "查找 pose_demo 进程..."
PIDS=$(ps aux | grep -E "pose_demo|hdmi_preview" | grep -v grep | awk '{print $2}')

if [ -z "$PIDS" ]; then
    echo "✓ 没有找到运行中的进程"
else
    echo "找到进程: $PIDS"
    echo "正在终止进程..."
    
    for PID in $PIDS; do
        if kill -0 $PID 2>/dev/null; then
            kill -9 $PID 2>/dev/null
            if [ $? -eq 0 ]; then
                echo "✓ 进程 $PID 已终止"
            else
                echo "✗ 无法终止进程 $PID (需要 root 权限)"
                echo "  请运行: sudo kill -9 $PID"
            fi
        fi
    done
fi

echo ""
echo "检查视频设备..."
if [ -e /dev/video40 ]; then
    echo "✓ /dev/video40 存在"
else
    echo "✗ /dev/video40 不存在"
    echo ""
    echo "尝试重新加载 HDMI-IN 驱动..."
    
    # 卸载驱动
    sudo modprobe -r rk_hdmirx 2>/dev/null
    sleep 1
    
    # 重新加载驱动
    sudo modprobe rk_hdmirx 2>/dev/null
    sleep 2
    
    # 检查设备
    if [ -e /dev/video40 ]; then
        echo "✓ /dev/video40 已恢复"
    else
        echo "✗ 无法恢复 /dev/video40"
        echo "  请检查 HDMI-IN 连接"
    fi
fi

echo ""
echo "==================================="
echo "清理完成"
echo "==================================="
echo ""
echo "现在可以运行: sudo ./pose_demo"
echo ""
