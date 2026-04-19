# USB-HID 虚拟鼠标 - 逐行注释

## src/usb_hid.h

```c
 1 #ifndef USB_HID_MOUSE_H
 2 #define USB_HID_MOUSE_H
 3 // 头文件保护，防止重复包含
 4 
 5 #include <stdint.h>   // 标准整数类型 uint8_t int8_t
 6 #include <stdbool.h>  // bool 类型
 7 
 8 // USB HID mouse report structure
 9 // Relative movement mouse: buttons + dx + dy + scroll wheel
10 // USB HID 鼠标报告结构体：相对坐标鼠标格式
11 typedef struct {
12     uint8_t buttons;  // Buttons bitmask: bit0=left, bit1=right, bit2=middle
13     // 按钮位图：bit0 左键，bit1 右键，bit2 中键，0 松开 1 按下
14     int8_t dx;        // X movement relative (-127 to +127)
15     // X 轴相对移动，范围 -127 到 +127 像素
16     int8_t dy;        // Y movement relative (-127 to +127)
17     // Y 轴相对移动，范围 -127 到 +127 像素
18     int8_t scroll;    // Scroll wheel
19     // 滚轮滚动，正负表示方向
20 } hid_mouse_report_t;
21 // 总共 4 字节，正好匹配 HID 报告描述符定义
22 
23 // Initialize USB HID mouse gadget
24 // Returns 0 on success, -1 on error
25 // 初始化 USB HID 鼠标，打开 /dev/hidg0
26 int usb_hid_mouse_init(void);
27 
28 // Cleanup and exit
29 // 清理，关闭文件描述符
30 void usb_hid_mouse_exit(void);
31 
32 // Send mouse movement report
33 // dx, dy: relative movement in pixels (-127 to +127)
34 // buttons: button bitmask (0 = no buttons pressed)
35 // 发送鼠标移动报告：dx dy 是相对移动，buttons 是按钮位图
36 int usb_hid_mouse_move(int8_t dx, int8_t dy, uint8_t buttons);
37 
38 // Check if device is initialized and ready
39 // 检查设备是否就绪
40 bool usb_hid_mouse_ready(void);
41 
42 #endif
```

---

## src/usb_hid.c

```c
 1 /*
 2  * RK3588 USB HID Virtual Mouse
 3  * Uses kernel libcomposite USB gadget to create a HID mouse device
 4  * After config, send reports to /dev/hidg0
 5  * 使用内核 libcomposite USB gadget 框架创建 HID 鼠标设备
 6  * 配置完成后，直接向 /dev/hidg0 写报告就能移动鼠标
 7  */
 8 
 9 #include "usb_hid.h"
10 // 包含自定义头文件
11 
12 #include <stdio.h>
13 // 标准输入输出 printf perror
14 #include <stdlib.h>
15 // 标准库
16 #include <string.h>
17 // 字符串操作
18 #include <unistd.h>
19 // close write 等系统调用
20 #include <fcntl.h>
21 // open 文件标志
22 #include <errno.h>
23 // 错误码
24 
25 // HID mouse Report Descriptor
26 // Standard relative mouse report descriptor
27 // 标准相对鼠标 HID 报告描述符，这里定义了报告格式
28 // 虽然 C 代码不用它（配置在脚本里做了），保留在这里方便参考
29 static const uint8_t mouse_report_desc[] = {
30     0x05, 0x01,        // Usage Page (Generic Desktop) 告诉USB这是通用桌面设备
31     0x09, 0x02,        // Usage (Mouse) 设备类型是鼠标
32     0xA1, 0x01,        // Collection (Application) 开始一个应用集合
33     0x09, 0x01,        //   Usage (Pointer) 指针
34     0xA1, 0x00,        //   Collection (Physical) 开始物理集合
35     0x05, 0x09,        //     Usage Page (Button) 按钮页
36     0x19, 0x01,        //     Usage Minimum (1) 按钮最小编号 1
37     0x29, 0x03,        //     Usage Maximum (3) 按钮最大编号 3（左右中三键）
38     0x15, 0x00,        //     Logical Minimum (0) 逻辑最小值 0
39     0x25, 0x01,        //     Logical Maximum (1) 逻辑最大值 1
40     0x95, 0x03,        //     Report Count (3) 3个按钮
41     0x75, 0x01,        //     Report Size (1) 每个按钮占1 bit
42     0x81, 0x02,        //     Input (Data,Var,Abs) 输入，数据，变量，绝对值
43     0x95, 0x01,        //     Report Count (1) 接下来一项
44     0x75, 0x05,        //     Report Size (5) 占5 bit（填充对齐到整字节）
45     0x81, 0x03,        //     Input (Const,Var,Abs) 常量填充，不传递数据
46     0x05, 0x01,        //     Usage Page (Generic Desktop) 回到通用桌面
47     0x09, 0x30,        //     Usage (X) X 坐标
48     0x09, 0x31,        //     Usage (Y) Y 坐标
49     0x15, 0x81,        //     Logical Minimum (-127) 最小 -127
50     0x25, 0x7F,        //     Logical Maximum (127) 最大 +127
51     0x75, 0x08,        //     Report Size (8) 每个坐标占 1 字节
52     0x95, 0x02,        //     Report Count (2) X+Y 两个
53     0x81, 0x06,        //     Input (Data,Var,Rel) 相对坐标（移动增量）
54     0x09, 0x38,        //     Usage (Scroll Wheel) 滚轮
55     0x15, 0x81,        //     Logical Minimum (-127)
56     0x25, 0x7F,        //     Logical Maximum (127)
57     0x75, 0x08,        //     Report Size (8)
58     0x95, 0x01,        //     Report Count (1)
59     0x81, 0x06,        //     Input (Data,Var,Rel)
60     0xC0,              //   End Collection 结束物理集合
61     0xC0               // End Collection 结束应用集合
62 };
63 // 总共 47 字节，正好对应 4 字节报告：
64 // 3 bits buttons + 5 bits padding = 1 byte
65 // 1 byte X + 1 byte Y + 1 byte wheel = 3 bytes
66 // 总共 4 bytes
67 
68 // Global variable: file descriptor for /dev/hidg0
69 // 全局变量：/dev/hidg0 文件描述符
70 static int hidg_fd = -1;
71 static bool initialized = false;
72 // initialized 标记是否初始化成功
73 
74 // HID mouse gadget needs to be configured via configfs
75 // This just opens /dev/hidg0 for sending reports
76 // HID gadget 需要通过 configfs 配置（脚本做），这里只需要打开设备发送报告
77 int usb_hid_mouse_init(void) {
78     // Open the HID gadget device
79     hidg_fd = open("/dev/hidg0", O_WRONLY);
80     // 只读？不，我们只写，所以 O_WRONLY
81     if (hidg_fd < 0) {
82         perror("open /dev/hidg0");
83         fprintf(stderr, "\n");
84         fprintf(stderr, "To create HID gadget device, you need to configure libcomposite:\n");
85         fprintf(stderr, "1. Enable USB OTG mode: echo otg > /sys/devices/platform/fd5d0000.syscon/fd5d0000.syscon:usb2-phy@0/otg_mode\n");
86         fprintf(stderr, "2. Load kernel modules: modprobe libcomposite\n");
87         fprintf(stderr, "3. Create gadget via configfs (see README for full script)\n");
88         // 打开失败打印帮助信息
89         return -1;
90     }
91 
92     initialized = true;
93     printf("USB HID mouse initialized: /dev/hidg0\n");
94     return 0;
95 }
96 
97 void usb_hid_mouse_exit(void) {
98 // 退出清理
99     if (hidg_fd >= 0) {
100         close(hidg_fd);
101         hidg_fd = -1;
102     }
103     // 关闭文件描述符
104     initialized = false;
105 }
106 
107 bool usb_hid_mouse_ready(void) {
108 // 返回是否就绪
109     return initialized;
110 }
111 
112 int usb_hid_mouse_move(int8_t dx, int8_t dy, uint8_t buttons) {
113 // 发送鼠标移动报告：dx dy 相对移动，buttons 按钮位图
114     if (!initialized || hidg_fd < 0) {
115         return -1;
116     }
117     // 没初始化直接返回错误
118 
119     hid_mouse_report_t report;
120     report.buttons = buttons;
121     report.dx = dx;
122     report.dy = dy;
123     report.scroll = 0;
124     // 填充报告，滚轮默认 0
125 
126     // Send report to /dev/hidg0
127     ssize_t n = write(hidg_fd, &report, sizeof(report));
128     // 直接写到 /dev/hidg0，内核会把报告发给 USB 主机
129     if (n != sizeof(report)) {
130         perror("write /dev/hidg0");
131         return -1;
132     }
133     // 检查写入长度是否正确
134 
135     return 0;
136 }
```

---

## src/hid_mouse_test.c

```c
 1 /*
 2  * USB HID Mouse Test
 3  * Moves the cursor in a square pattern to test the virtual mouse
 4  * 测试 USB HID 虚拟鼠标，让光标走正方形
 5  */
 6 
 7 #include "usb_hid.h"
 8 // 包含 USB HID 头文件
 9 #include <stdio.h>
10 #include <unistd.h>
11 
12 int main(int argc, char** argv) {
13     printf("USB HID Virtual Mouse Test\n");
14 
15     if (usb_hid_mouse_init() != 0) {
16         fprintf(stderr, "Failed to initialize USB HID mouse\n");
17         fprintf(stderr, "\nMake sure you have:\n");
18         fprintf(stderr, "1. Enabled OTG mode: echo otg > /sys/devices/platform/fd5d0000.syscon/fd5d0000.syscon:usb2-phy@0/otg_mode\n");
19         fprintf(stderr, "2. Run configure script as root: sudo ../scripts/configure-hid-gadget.sh\n");
20         return 1;
21     }
22     // 初始化失败打印帮助
23 
24     printf("Mouse initialized, moving cursor in square pattern...\n");
25     printf("Press Ctrl+C to exit\n");
26 
27     // Move in a square pattern
28     // 循环画正方形
29     while (true) {
30         // Move right
31         // 向右移动
32         for (int i = 0; i < 10; i++) {
33             usb_hid_mouse_move(10, 0, 0);
34             usleep(100000);
35         }
36         // 每次移动 10 像素，间隔 100ms
37         // Move down
38         // 向下移动
39         for (int i = 0; i < 10; i++) {
40             usb_hid_mouse_move(0, 10, 0);
41             usleep(100000);
42         }
43         // Move left
44         // 向左移动
45         for (int i = 0; i < 10; i++) {
46             usb_hid_mouse_move(-10, 0, 0);
47             usleep(100000);
48         }
49         // Move up
50         // 向上移动
51         for (int i = 0; i < 10; i++) {
52             usb_hid_mouse_move(0, -10, 0);
53             usleep(100000);
54         }
55     }
56     // 正方形四个边，循环走
57 
58     usb_hid_mouse_exit();
59     return 0;
60 }
```

---

## scripts/configure-hid-gadget.sh

```bash
 1 #!/bin/bash
 2 # Configure USB HID mouse gadget using libcomposite
 3 # Run as root
 4 # 使用 libcomposite 配置 USB HID 鼠标 gadget，必须 root 运行
 5 
 6 set -e
 7 # 任何命令失败脚本立即退出
 8 
 9 # Check if we're root
10 if [ "$(id -u)" != "0" ]; then
11     echo "This script must be run as root" 1>&2
12     exit 1
13 fi
14 # 检查 root 权限，不是 root 直接退出
15 
16 # Load libcomposite module
17 modprobe libcomposite
18 # 加载 libcomposite 内核模块
19 
20 # Mount configfs if not already mounted
21 if ! mountpoint -q /sys/kernel/config; then
22     mkdir -p /sys/kernel/config
23     mount -t configfs none /sys/kernel/config
24 fi
25 # 如果 configfs 没挂载，挂载它，所有 USB gadget 配置都在这里
26 
27 # Go to gadget directory
28 cd /sys/kernel/config/usb_gadget
29 mkdir -p rk3588-aimbot
30 cd rk3588-aimbot
31 # 创建我们的 gadget 目录
32 
33 # idVendor 0x1d6b is Linux Foundation
34 echo 0x1d6b > idVendor
35 # idProduct 0x0101 is HID gadget
36 echo 0x0101 > idProduct
37 # bcdDevice
38 echo 0x0100 > bcdDevice
39 # 设置 VID/PID，0x1d6b 是 Linux 基金会的合法 VID
40 
41 # Create English strings
42 mkdir -p strings/0x409
43 echo "0123456789" > strings/0x409/serialnumber
44 echo "RK3588" > strings/0x409/manufacturer
45 echo "AI Aimbot Mouse" > strings/0x409/product
46 # 创建英文（0x409）字符串，设备信息
47 
48 # Create HID function
49 mkdir -p functions/hid.usb0
50 echo 4 > functions/hid.usb0/protocol  # mouse 协议编号 4 是鼠标
51 echo 2 > functions/hid.usb0/subclass  # boot 子类 2
52 echo 4 > functions/hid.usb0/report_length  # 4 bytes 报告长度
53 # 创建 HID 功能，设置协议、子类、报告长度（我们是 4 字节）
54 
55 # Write report descriptor
56 # Use printf to write each byte directly, avoids any escape issues
57 # 使用 printf 直接写二进制报告描述符，避免 shell 转义问题
58 printf "\x05\x01\x09\x02\xa1\x01\x09\x01\xa1\x00\x05\x09\x19\x01\x29\x03\x15\x00\x25\x01\x95\x03\x75\x01\x81\x02\x95\x01\x75\x05\x81\x03\x05\x01\x09\x30\x09\x31\x15\x81\x25\x7f\x75\x08\x95\x02\x81\x06\x09\x38\x15\x81\x25\x7f\x75\x08\x95\x01\x81\x06\xc0\xc0" > functions/hid.usb0/report_desc
59 # 写入 HID 报告描述符，二进制正确
60 
61 # Create configuration
62 mkdir -p configs/c.1/strings/0x409
63 echo "Aimbot HID Mouse" > configs/c.1/strings/0x409/configuration
64 echo 120 > configs/c.1/MaxPower
65 # 创建配置，最大电流 120mA
66 
67 # Link function to configuration
68 ln -s functions/hid.usb0 configs/c.1/
69 # 把 HID 功能链接到配置
70 
71 # Bind to the USB controller UDC
72 # Find the first UDC driver
73 # 绑定到第一个找到的 UDC（USB device controller）
74 UDC=$(ls /sys/class/udc/ | head -1)
75 if [ -z "$UDC" ]; then
76     echo "No UDC driver found, check USB OTG mode is enabled"
77     exit 1
78 fi
79 # 自动找第一个 UDC，RK3588 通常是 fc000000.usb
80 echo "$UDC" > UDC
81 # 绑定完成
82 echo "Gadget configured successfully, UDC: $UDC"
83 echo "/dev/hidg0 should be created now"
```

---

## scripts/cleanup-hid-gadget.sh

```bash
 1 #!/bin/bash
 2 # Cleanup USB HID gadget configuration
 3 # 清理 USB HID gadget 配置
 4 
 5 set -e
 6 
 7 # Check if we're root
 8 if [ "$(id -u)" != "0" ]; then
 9     echo "This script must be run as root" 1>&2
10     exit 1
11 fi
12 # 检查 root
13 
14 # configfs doesn't allow removing non-empty directories, need to clean in correct order
15 # configfs 不允许删除非空目录，必须按正确顺序清理
16 if [ -d /sys/kernel/config/usb_gadget/rk3588-aimbot ]; then
17     cd /sys/kernel/config/usb_gadget/rk3588-aimbot
18 
19     # Unbind from UDC
20     echo "" > UDC 2>/dev/null || true
21     # 解除 UDC 绑定，写空字符串
22 
23     # Remove symlink from config to function
24     rm -f configs/c.1/hid.usb0 2>/dev/null || true
25     # 删除配置到功能的符号链接
26 
27     # Cleanup config
28     rm -rf configs/c.1/strings/0x409 2>/dev/null || true
29     rmdir configs/c.1 2>/dev/null || true
30     # 清理配置目录
31 
32     # Cleanup function
33     rmdir functions/hid.usb0 2>/dev/null || true
34     # 删除功能目录
35 
36     # Cleanup strings
37     rm -rf strings/0x409 2>/dev/null || true
38     # 删除字符串目录
39 
40     cd ..
41     rmdir rk3588-aimbot 2>/dev/null || true
42     # 删除 gadget 根目录
43 fi
44 
45 echo "Cleanup done"
```

就是这样，所有源文件逐行注释完成。
