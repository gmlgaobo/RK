# RK3588 自动瞄准闭环系统

硬件连接：游戏主机 HDMI → RK3588 HDMI-IN 采集 → NPU AI推理 → USB-HID虚拟鼠标 OTG → 游戏主机 USB口

## 项目目标

MVP 阶段：实现 HDMI 采集 → YOLOv8n-pose 姿态检测 → 鼠标移动闭环控制

## 组件

- `src/hdmi_capture.[c/h]` - V4L2 HDMI-IN 多平面格式采集 (rk_hdmirx 驱动适配)
- `src/hdmi_preview.cpp` - HDMI 实时预览 (OpenCV)
- `src/usb_hid.[c/h]` - USB HID 虚拟鼠标 gadget
- `src/hid_mouse_test.c` - 鼠标测试程序，画方块测试移动
- `scripts/configure-hid-gadget.sh` - 配置 USB HID gadget 脚本
- `scripts/cleanup-hid-gadget.sh` - 清理配置脚本
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

运行预览：
```bash
./build/src/hdmi_preview          # 默认 /dev/video40 1920x1080
./build/src/hdmi_preview /dev/video40 1280 720  # 自定义分辨率
```
窗口会显示实时画面，左上角显示 FPS，按 q 或 ESC 退出。

## USB-OTG 和 HID 虚拟鼠标

### 1. 切换 OTG 从设备模式
必须先切换 USB 到从设备模式才能模拟鼠标：
```bash
echo otg > /sys/devices/platform/fd5d0000.syscon/fd5d0000.syscon\:usb2-phy\@0/otg_mode
```
验证：
```bash
cat /sys/devices/platform/fd5d0000.syscon/fd5d0000.syscon\:usb2-phy\@0/otg_mode
# should output "otg"
```

### 2. 配置 HID gadget (必须 root 运行)
```bash
sudo ./scripts/configure-hid-gadget.sh
```
这会：
- 加载 `libcomposite` 内核模块
- 挂载 configfs
- 创建 HID 鼠标 gadget
- 绑定到 USB UDC 控制器

完成后会创建 `/dev/hidg0` 设备节点。

### 3. 测试鼠标
```bash
./build/src/hid_mouse_test
```
程序会让光标自动走正方形，如果能看到光标移动说明成功。按 Ctrl+C 退出。

### 4. 清理配置 (如果需要重新配置)
```bash
sudo ./scripts/cleanup-hid-gadget.sh
```

### 自动开机启动
可以把 `configure-hid-gadget.sh` 添加到 systemd 服务，开机自动配置。

## 第三步：YOLOv8n-pose 姿态检测

RK3588 NPU 推理，检测人体姿态关键点，用于自动瞄准。

### 模型准备

需要先把 YOLOv8n-pose 导出为 ONNX，然后用 rknn-toolkit2 转换为 `.rknn` 模型：

1. 从 ultralytics 导出 `yolov8n-pose.onnx`
2. 用 `rknn-toolkit2` 转换为 `yolov8n-pose.rknn`
3. 将 `yolov8n-pose.rknn` 放在项目根目录

### 运行检测demo

```bash
./build/src/pose_demo /dev/video40 1920 1080 ./yolov8n-pose.rknn
```

会显示实时画面，画出检测框、关键点和骨架。

## 第四步：闭环自动瞄准

TODO: 主闭环程序，检测到胸部位置，计算鼠标移动，发送到 USB HID
