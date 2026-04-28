#!/bin/bash

echo "==================================="
echo "RK3588 AimBot 测试工具"
echo "==================================="
echo ""

# 检查参数
if [ $# -eq 0 ]; then
    echo "用法: $0 <测试项>"
    echo ""
    echo "可用测试项："
    echo "  hid        - 测试 USB HID 设备"
    echo "  keyboard   - 测试键盘输入"
    echo "  mouse      - 测试鼠标输入"
    echo "  priority   - 测试线程优先级"
    echo "  all        - 运行所有测试"
    echo ""
    echo "示例："
    echo "  $0 hid"
    echo "  $0 all"
    exit 1
fi

TEST_TYPE=$1

# 测试 USB HID 设备
test_hid() {
    echo "测试 USB HID 设备..."
    echo ""
    
    # 检查设备节点
    if [ -e "/dev/hidg0" ] && [ -e "/dev/hidg1" ]; then
        echo "✓ 设备节点存在"
        ls -l /dev/hidg*
    else
        echo "✗ 设备节点不存在"
        return 1
    fi
    
    echo ""
    
    # 检查 UDC 绑定
    UDC=$(cat /sys/kernel/config/usb_gadget/rk3588-aimbot/UDC 2>/dev/null)
    if [ -n "$UDC" ]; then
        echo "✓ UDC 已绑定: $UDC"
    else
        echo "✗ UDC 未绑定"
        return 1
    fi
    
    echo ""
    
    # 检查连接状态
    SUSPENDED=$(cat /sys/class/udc/fc000000.usb/device/gadget/suspended 2>/dev/null)
    if [ "$SUSPENDED" = "0" ]; then
        echo "✓ USB OTG 已连接"
    else
        echo "✗ USB OTG 未连接"
        return 1
    fi
    
    echo ""
    echo "✓ USB HID 测试通过"
    return 0
}

# 测试键盘输入
test_keyboard() {
    echo "测试键盘输入..."
    echo ""
    
    # 检查键盘设备
    KEYBOARD_DEVICE=$(ls /dev/input/by-path/*kbd 2>/dev/null | head -1)
    if [ -n "$KEYBOARD_DEVICE" ]; then
        echo "✓ 键盘设备: $KEYBOARD_DEVICE"
    else
        echo "✗ 未找到键盘设备"
        return 1
    fi
    
    echo ""
    
    # 测试读取
    echo "尝试读取键盘事件（按任意键测试）..."
    timeout 5s sudo cat "$KEYBOARD_DEVICE" > /dev/null 2>&1
    if [ $? -eq 124 ]; then
        echo "✓ 键盘设备可读"
    else
        echo "✗ 键盘设备读取失败"
        return 1
    fi
    
    echo ""
    echo "✓ 键盘测试通过"
    return 0
}

# 测试鼠标输入
test_mouse() {
    echo "测试鼠标输入..."
    echo ""
    
    # 检查鼠标设备
    MOUSE_DEVICE=$(ls /dev/input/by-path/*mouse 2>/dev/null | head -1)
    if [ -n "$MOUSE_DEVICE" ]; then
        echo "✓ 鼠标设备: $MOUSE_DEVICE"
    else
        echo "✗ 未找到鼠标设备"
        return 1
    fi
    
    echo ""
    
    # 测试读取
    echo "尝试读取鼠标事件（移动鼠标测试）..."
    timeout 5s sudo cat "$MOUSE_DEVICE" > /dev/null 2>&1
    if [ $? -eq 124 ]; then
        echo "✓ 鼠标设备可读"
    else
        echo "✗ 鼠标设备读取失败"
        return 1
    fi
    
    echo ""
    echo "✓ 鼠标测试通过"
    return 0
}

# 测试线程优先级
test_priority() {
    echo "测试线程优先级..."
    echo ""
    
    PID=$(pgrep -x pose_demo)
    if [ -z "$PID" ]; then
        echo "✗ pose_demo 未运行"
        echo "请先运行程序："
        echo "  sudo bash /home/ztl/github/RK/scripts/startup-check.sh --run"
        return 1
    fi
    
    echo "✓ 找到 pose_demo 进程 (PID: $PID)"
    echo ""
    
    # 检查实时线程
    REALTIME_COUNT=$(ps -T -p $PID -o rtprio | grep -v "-" | grep -v "RTPRIO" | wc -l)
    echo "实时线程数: $REALTIME_COUNT"
    
    if [ "$REALTIME_COUNT" -ge 3 ]; then
        echo "✓ 至少 3 个线程已设置为实时优先级"
    else
        echo "✗ 实时线程数不足"
        return 1
    fi
    
    echo ""
    
    # 检查优先级
    MAX_RTPRIO=$(ps -T -p $PID -o rtprio | grep -v "-" | grep -v "RTPRIO" | sort -n | tail -1)
    if [ "$MAX_RTPRIO" = "99" ]; then
        echo "✓ 最高实时优先级为 99"
    else
        echo "✗ 最高实时优先级为 $MAX_RTPRIO (应为 99)"
        return 1
    fi
    
    echo ""
    
    # 显示线程信息
    echo "线程列表："
    ps -T -p $PID -o tid,rtprio,sched,psr,comm
    
    echo ""
    echo "✓ 线程优先级测试通过"
    return 0
}

# 运行所有测试
test_all() {
    echo "运行所有测试..."
    echo ""
    
    FAILED=0
    
    test_hid || FAILED=1
    echo ""
    echo "----------------------------------------"
    echo ""
    
    test_keyboard || FAILED=1
    echo ""
    echo "----------------------------------------"
    echo ""
    
    test_mouse || FAILED=1
    echo ""
    echo "----------------------------------------"
    echo ""
    
    test_priority || FAILED=1
    echo ""
    
    if [ $FAILED -eq 0 ]; then
        echo "==================================="
        echo "✓ 所有测试通过"
        echo "==================================="
    else
        echo "==================================="
        echo "✗ 部分测试失败"
        echo "==================================="
    fi
    
    return $FAILED
}

# 执行测试
case $TEST_TYPE in
    hid)
        test_hid
        ;;
    keyboard)
        test_keyboard
        ;;
    mouse)
        test_mouse
        ;;
    priority)
        test_priority
        ;;
    all)
        test_all
        ;;
    *)
        echo "✗ 未知的测试项: $TEST_TYPE"
        exit 1
        ;;
esac
