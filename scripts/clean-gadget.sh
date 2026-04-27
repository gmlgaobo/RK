#!/bin/bash
# 彻底清理 gadget

if [ "$(id -u)" != "0" ]; then
    echo "请用 sudo 运行"
    exit 1
fi

echo "=== 彻底清理 Gadget ==="
cd /sys/kernel/config/usb_gadget

for GADGET in *; do
    if [ -d "$GADGET" ] && [ "$GADGET" != "." ] && [ "$GADGET" != ".." ]; then
        echo "清理: $GADGET"
        cd "$GADGET"
        
        # 解绑 UDC
        if [ -f "UDC" ]; then
            echo > UDC 2>/dev/null || true
            sleep 0.5
        fi
        
        # 移除函数链接
        for CONFIG in configs/*; do
            if [ -d "$CONFIG" ]; then
                for LINK in "$CONFIG"/*.*; do
                    if [ -L "$LINK" ]; then
                        rm "$LINK" 2>/dev/null || true
                    fi
                done
                for S in "$CONFIG"/strings/*; do
                    [ -d "$S" ] && rmdir "$S" 2>/dev/null || true
                done
                rmdir "$CONFIG" 2>/dev/null || true
            fi
        done
        
        # 移除函数
        for FUNC in functions/*.*; do
            [ -d "$FUNC" ] && rmdir "$FUNC" 2>/dev/null || true
        done
        
        # 移除字符串
        for S in strings/*; do
            [ -d "$S" ] && rmdir "$S" 2>/dev/null || true
        done
        
        cd ..
        
        # 删除 gadget，多试几次
        for i in 1 2 3; do
            rmdir "$GADGET" 2>/dev/null && break || true
            sleep 0.3
        done
    fi
done

sleep 1
echo "完成!"
