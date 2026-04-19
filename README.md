# RK3588 自动瞄准闭环系统

硬件连接：游戏主机 HDMI → RK3588 HDMI-IN 采集 → NPU AI推理 → USB-HID虚拟鼠标 OTG → 游戏主机 USB口

## 项目目标

MVP 阶段：实现 HDMI 采集 → YOLOv8n-pose 姿态检测 → 鼠标移动闭环控制

## 组件

- `src/hdmi_capture.c` - V4L2 HDMI-IN 多平面格式采集 (rk_hdmirx 驱动适配)
- `src/usb_hid.c` - USB HID 虚拟鼠标 gadget 配置
- `src/yolo_inference.cpp` - RKNN YOLOv8n-pose 推理
- `src/aimbot.cpp` - 主闭环控制

## 环境

- 开发板：定昌 RK3588
- 固件：GB-RK3588-ubuntu22.04-20260321-165139-gnome-v10-ALL-A-CM-EN.img
- 内核：Linux 5.10.226 Rockchip LTS
- 推理：rknn-toolkit2

## HDMI-IN 采集

默认设备节点：`/dev/video40`

检查信号状态：
```bash
v4l2-ctl -d /dev/video40 --list-ctrls
```
`power_present=1` 表示有输入信号

测试帧率：
```bash
v4l2-ctl -d /dev/video40 --stream-mmap=4 --stream-skip=10 --stream-count=100 --stream-poll
```

## USB-OTG 配置

切换到从设备模式（模拟鼠标需要）：
```bash
echo otg > /sys/devices/platform/fd5d0000.syscon/fd5d0000.syscon\:usb2-phy\@0/otg_mode
```
