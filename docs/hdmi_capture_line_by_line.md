# RK3588 HDMI 采集代码 - 逐行注释解释

## src/hdmi_capture.c

```c
 1 /*
 2  * RK3588 HDMI-IN Capture
 3  * Adapts to rk_hdmirx driver with V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
 4  * Supports 1920x1080 BGR24 format
 5  * 2026-04-19 10:36 再次更新 - 逐行注释 + 修复 buf.length
 6  */
 // 文件头注释：说明这个文件用途、适配的驱动、支持的格式、更新记录
 7 
 8 #include "hdmi_capture.h"   // 包含自定义头文件，函数声明和结构体定义
 9 
10 #include <stdio.h>         // 标准输入输出，printf/perror用
11 #include <stdlib.h>        // 标准库函数，malloc/memset/exit等
12 #include <string.h>        // 字符串处理，memset等
13 #include <unistd.h>        // 系统调用，close/usleep等
14 #include <fcntl.h>         // 文件控制，open函数的flag定义
15 #include <errno.h>         // 错误码，perror打印错误信息
16 #include <sys/ioctl.h>     // IO控制，V4L2的ioctl命令都是通过这个
17 #include <sys/mman.h>      // 内存映射，把内核缓冲区映射到用户空间
18 #include <linux/videodev2.h>// V4L2驱动的结构体和常量定义，来自内核头文件
19 
20 // Helper: ioctl wrapper, retry on EINTR
21 // When system call is interrupted by signal, retry it
22 static int xioctl(int fd, int req, void* arg) {
23 // IOCTL封装：处理被信号中断的系统调用，自动重试，标准V4L2编程模式
24     int r;
25     do {
26         r = ioctl(fd, req, arg);
27     } while (r == -1 && errno == EINTR);
28     // 只要不是被信号中断，就返回结果，EINTR说明调用被信号打断，需要重试
29     return r;
30 }
31 
32 int hdmi_open(hdmi_capture_t* cap, const char* device, int width, int height) {
33 // 打开HDMI捕获设备，初始化V4L2，映射缓冲区
34 // cap: 输出的捕获结构体
35 // device: 设备路径，比如 "/dev/video40"
36 // width/height: 请求的分辨率
37 // 返回 0 成功，-1 失败
38 
39     // Zero-initialize the capture struct
40     memset(cap, 0, sizeof(*cap));
41     // 把整个结构体清零，避免垃圾数据
42     cap->fd = -1;
43     cap->width = width;
44     cap->height = height;
45     cap->nbufs = 0;
46     cap->is_mplane = true; // Rockchip HDMI driver uses multi-planar API
47     // Rockchip HDMI驱动使用多平面API，标记为true
48 
49     // Open device file: O_RDWR for V4L2, O_NONBLOCK for non-blocking mode
50     cap->fd = open(device, O_RDWR | O_NONBLOCK, 0);
51     // 打开设备：读写权限，非阻塞模式（获取帧不会一直卡住）
52     if (cap->fd == -1) {
53         perror("open");
54         return -1;
55     }
56     // 打开失败打印错误，返回
57 
58     // Query device capabilities
59     struct v4l2_capability cap_info;
60     if (xioctl(cap->fd, VIDIOC_QUERYCAP, &cap_info) < 0) {
61         perror("VIDIOC_QUERYCAP");
62         goto error;
63     }
64     // 查询驱动能力，获取驱动名称、设备信息等
65 
66     // Check if device supports multi-planar capture
67     if (!(cap_info.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
68         fprintf(stderr, "Device does not support multi-planar capture\n");
69         goto error;
70     }
71     // 检查设备是否支持多平面捕获，不支持就报错退出
72 
73     printf("Driver: %s\nCard: %s\nBus: %s\n", cap_info.driver, cap_info.card, cap_info.bus_info);
74     // 打印驱动信息，方便调试
75 
76     // Check HDMI signal presence
77     struct v4l2_control ctrl;
78     ctrl.id = 0x00a00964; // power_present control
79     if (xioctl(cap->fd, VIDIOC_G_CTRL, &ctrl) >= 0) {
80         if (ctrl.value != 1) {
81             fprintf(stderr, "\nWARNING: No HDMI signal detected!\n");
82             fprintf(stderr, "power_present = %d\n", ctrl.value);
83             fprintf(stderr, "Check HDMI cable and signal source.\n\n");
84         } else {
85             printf("HDMI signal detected: OK\n");
86         }
87     }
88     // 新增功能：检查HDMI是否有信号，control ID 0x00a00964 是 rk_hdmirx 定义的 power_present
89     // value=1 有信号，value=0 没信号，没信号打印警告
90 
91     // Set pixel format
92     struct v4l2_format fmt;
93     memset(&fmt, 0, sizeof(fmt));
94     fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
95     fmt.fmt.pix_mp.width = width;
96     fmt.fmt.pix_mp.height = height;
97     fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_BGR24;
98     fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
99     // For packed BGR24, we only need one plane
100     fmt.fmt.pix_mp.num_planes = 1;
101     // 设置格式：多平面类型，分辨率，BGR24像素格式，不指定场（逐行），一个平面
102 
103     // Ask driver to set the format
104     if (xioctl(cap->fd, VIDIOC_S_FMT, &fmt) < 0) {
105         perror("VIDIOC_S_FMT");
106         goto error;
107     }
108     // 告诉驱动我们想要的格式
109 
110     // Read back the actual format that driver accepted
111     if (xioctl(cap->fd, VIDIOC_G_FMT, &fmt) < 0) {
112         perror("VIDIOC_G_FMT");
113         goto error;
114     }
115     // 读回驱动实际设置的格式，驱动可能不支持我们请求的，会调整，所以要读回来
116 
117     // Store actual parameters
118     cap->width = fmt.fmt.pix_mp.width;
119     cap->height = fmt.fmt.pix_mp.height;
120     cap->pixelformat = fmt.fmt.pix_mp.pixelformat;
121     // 保存驱动实际给我们的分辨率和像素格式
122 
123     // Print the actual format for debugging
124     printf("Actual format: %dx%d\n", cap->width, cap->height);
125     printf("Fourcc: %c%c%c%c (0x%08x)\n",
126            (cap->pixelformat & 0xff),
127            (cap->pixelformat >> 8) & 0xff,
128            (cap->pixelformat >> 16) & 0xff,
129            (cap->pixelformat >> 24) & 0xff,
130            cap->pixelformat);
131     printf("Number of planes: %d\n", fmt.fmt.pix_mp.num_planes);
132 
133     for (int i = 0; i < fmt.fmt.pix_mp.num_planes; i++) {
134         printf("  Plane %d: size=%d, bytesperline=%d\n",
135                i, fmt.fmt.pix_mp.plane_fmt[i].sizeimage,
136                fmt.fmt.pix_mp.plane_fmt[i].bytesperline);
137     }
138     // 打印实际格式信息，方便调试，能看到驱动给了我们什么格式
139 
140     // Request buffers from the driver
141     struct v4l2_requestbuffers req;
142     memset(&req, 0, sizeof(req));
143     req.count = MAX_BUFFERS;
144     req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
145     req.memory = V4L2_MEMORY_MMAP; // Use memory-mapped buffers
146     // 请求缓冲区：请求 MAX_BUFFERS(4) 个缓冲，多平面类型，内存映射方式
147 
148     if (xioctl(cap->fd, VIDIOC_REQBUFS, &req) < 0) {
149         perror("VIDIOC_REQBUFS");
150         goto error;
151     }
152     // 向内核申请缓冲区
153 
154     if (req.count < 2) {
155         fprintf(stderr, "Insufficient buffer memory from driver\n");
156         goto error;
157     }
158     // 至少需要两个缓冲才能流水线，太少了不行
159 
160     // Map each buffer into our process address space
161     for (int i = 0; i < req.count; i++) {
162         struct v4l2_buffer buf;
163         struct v4l2_plane planes[MAX_BUFFERS];
164         memset(&buf, 0, sizeof(buf));
165         memset(planes, 0, sizeof(planes));
166         buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
167         buf.memory = V4L2_MEMORY_MMAP;
168         buf.index = i;
169         buf.m.planes = planes;
170         buf.length = fmt.fmt.pix_mp.num_planes; // length = number of planes
171         // 循环查询每个缓冲区：对于多平面API，buf.length = 平面个数，这是要求
172 
173         // Query buffer information
174         if (xioctl(cap->fd, VIDIOC_QUERYBUF, &buf) < 0) {
175             perror("VIDIOC_QUERYBUF");
176             goto error;
177         }
178         // 查询这个缓冲的地址和长度信息
179 
180         // We only use the first plane for packed BGR24
181         cap->lengths[i] = planes[0].length;
182         cap->buffers[i] = mmap(NULL, planes[0].length,
183                               PROT_READ | PROT_WRITE,
184                               MAP_SHARED, cap->fd, planes[0].m.mem_offset);
185         // 把内核缓冲区映射到用户空间地址，这样我们可以直接读写
186         // PROT_READ | PROT_WRITE 可读可写，MAP_SHARED 共享和内核同步
187 
188         // Map the buffer from kernel space to user space
189         if (cap->buffers[i] == MAP_FAILED) {
190             perror("mmap");
191             goto error;
192         }
193         // 映射失败处理
194 
195         // Queue the buffer so driver can fill it with frames
196         if (xioctl(cap->fd, VIDIOC_QBUF, &buf) < 0) {
197             perror("VIDIOC_QBUF");
198             goto error;
199         }
200         // 把这个空缓冲放入队列，让驱动可以往里面填帧
201 
202         cap->nbufs++;
203         // 计数成功映射的缓冲区个数
204     }
205 
206     printf("Opened %s: %dx%d, %d buffers mapped\n", device, cap->width, cap->height, cap->nbufs);
207     return 0;
208     // 成功返回
209 
210 error:
211     hdmi_close(cap);
212     return -1;
213     // 任何一步出错，清理已经分配的资源，返回错误
214 }
215 
216 int hdmi_start(hdmi_capture_t* cap) {
217 // 开始流式传输
218     enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
219     if (xioctl(cap->fd, VIDIOC_STREAMON, &type) < 0) {
220         perror("VIDIOC_STREAMON");
221         return -1;
222     }
223     // 告诉驱动开始抓帧
224     return 0;
225 }
226 
227 void hdmi_stop(hdmi_capture_t* cap) {
228 // 停止流式传输
229     enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
230     xioctl(cap->fd, VIDIOC_STREAMOFF, &type);
231     // 告诉驱动停止抓帧
232 }
233 
234 void hdmi_close(hdmi_capture_t* cap) {
235 // 关闭设备，取消映射，释放资源
236     if (cap->fd != -1) {
237         for (int i = 0; i < cap->nbufs; i++) {
238             if (cap->buffers[i] && cap->buffers[i] != MAP_FAILED) {
239                 munmap(cap->buffers[i], cap->lengths[i]);
240             }
241         }
242         close(cap->fd);
243         cap->fd = -1;
244     }
245     // 遍历所有缓冲区，取消映射，关闭文件描述符
246 }
247 
248 int hdmi_get_frame(hdmi_capture_t* cap, uint8_t** out_data, int* out_width, int* out_height) {
249 // 从队列取出一个已经填充好的帧
250 // out_data: 输出指针指向BGR数据
251 // out_width/out_height: 输出宽高
252 // 返回 0 成功，-1 没有帧或错误
253     struct v4l2_buffer buf;
254     struct v4l2_plane planes[MAX_BUFFERS];
255     memset(&buf, 0, sizeof(buf));
256     memset(planes, 0, sizeof(planes));
257     buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
258     buf.memory = V4L2_MEMORY_MMAP;
259     buf.length = 1;
260     buf.m.planes = planes;
261     // 初始化结构体，长度=平面个数(1)
262 
263     // When dequeuing, we don't set index - kernel gives us the buffer
264     // bytesused will be filled by kernel, so we don't need to set it
265     // 出队列时，不需要设置index，内核返回哪个缓冲就用哪个
266     // bytesused由内核填充，我们不用管
267 
268     // Dequeue a filled buffer from the driver
269     if (xioctl(cap->fd, VIDIOC_DQBUF, &buf) < 0) {
270         if (errno == EAGAIN) {
271             return -1; // No frame available yet, try again later
272         }
273         perror("VIDIOC_DQBUF");
274         return -1;
275     }
276     // 从驱动出队列一个已经填满的帧，如果EAGAIN说明现在没帧，重试
277 
278     // Store current buffer index
279     cap->current_buf = buf.index;
280     // Return pointer to the mapped data
281     *out_data = (uint8_t*)cap->buffers[buf.index];
282     *out_width = cap->width;
283     *out_height = cap->height;
284     // 保存当前缓冲索引，返回数据指针给用户
285     return 0;
286 }
287 
288 void hdmi_release_frame(hdmi_capture_t* cap) {
289 // 用完帧后，放回队列给驱动重新使用
290     struct v4l2_buffer buf;
291     struct v4l2_plane planes[MAX_BUFFERS];
292     memset(&buf, 0, sizeof(buf));
293     memset(planes, 0, sizeof(planes));
294     buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
295     buf.memory = V4L2_MEMORY_MMAP;
296     buf.index = cap->current_buf;
297     buf.length = 1;
298     buf.m.planes = planes;
299 
300     // When queueing, bytesused is 0 for capture - driver will fill it when captured
301     planes[0].bytesused = 0;
302     // 对于捕获方向，入队列时bytesused设为0，表示整个缓冲区都可以用来放数据
303     // 内核捕获完会设置正确的bytesused
304 
305     // Re-queue the buffer back to driver after we're done with it
306     if (xioctl(cap->fd, VIDIOC_QBUF, &buf) < 0) {
307         perror("VIDIOC_QBUF");
308     }
309     // 把缓冲重新放入队列，循环使用
310 }
```

---

## src/hdmi_capture.h

```c
 1 #ifndef HDMI_CAPTURE_H
 2 #define HDMI_CAPTURE_H
 3 // 头文件保护，防止重复包含
 4 
 5 #include <stdint.h>   // 标准整数类型 uint8_t
 6 #include <stdbool.h>  // bool类型
 7 
 8 #define MAX_BUFFERS 4
 9 // 最多使用4个缓冲区，满足流水线需要
10 
11 typedef struct {
12     int fd;             // Device file descriptor 设备文件描述符
13     int width;          // Frame width 帧宽度
14     int height;         // Frame height 帧高度
15     int pixelformat;    // V4L2 pixel format fourcc 像素格式fourcc
16     int nbufs;          // Number of mapped buffers 映射的缓冲区个数
17     void* buffers[MAX_BUFFERS];  // Mapped buffer pointers 缓冲区指针数组
18     int lengths[MAX_BUFFERS];    // Buffer lengths in bytes 每个缓冲区长度
19     int current_buf;    // Current dequeued buffer index 当前出队列的缓冲索引
20     bool is_mplane;     // Is multi-planar format 是否多平面格式
21 } hdmi_capture_t;
22 // 捕获句柄结构体，保存所有状态信息
23 
24 #ifdef __cplusplus
25 extern "C" {
26 #endif
27 // 如果被C++代码包含，用extern "C"声明，保证C++能链接C函数
28 
29 // Open HDMI capture device
30 // device: /dev/videoX path 设备路径
31 // width/height: requested frame size 请求的帧大小
32 // Returns 0 on success, -1 on error 返回0成功，-1失败
33 int hdmi_open(hdmi_capture_t* cap, const char* device, int width, int height);
34 
35 // Start streaming 开始流传输
36 // Returns 0 on success, -1 on error
37 int hdmi_start(hdmi_capture_t* cap);
38 
39 // Stop streaming 停止流传输
40 void hdmi_stop(hdmi_capture_t* cap);
41 
42 // Close device and free resources 关闭设备释放资源
43 void hdmi_close(hdmi_capture_t* cap);
44 
45 // Get next frame, returns pointer to BGR data 获取下一帧，返回BGR数据指针
46 // Returns 0 on success, -1 if no frame available or error
47 int hdmi_get_frame(hdmi_capture_t* cap, uint8_t** out_data, int* out_width, int* out_height);
48 
49 // Release frame back to driver for re-use 释放帧回驱动重用
50 void hdmi_release_frame(hdmi_capture_t* cap);
51 
52 #ifdef __cplusplus
53 }
54 #endif
55 // extern "C"块结束
56 
57 #endif
```

---

## src/hdmi_preview.cpp

```cpp
 1 /*
 2  * HDMI Preview with OpenCV
 3  * Displays live capture from HDMI-IN in a window
 4  */
 5 // 文件头注释：这是带OpenCV预览的主程序
 6 
 7 #include "hdmi_capture.h"    // 包含我们的HDMI采集头文件
 8 #include <opencv2/opencv.hpp> // OpenCV头文件，用于显示窗口和图像处理
 9 #include <sys/time.h>        // 计时gettimeofday
10 #include <unistd.h>          // usleep
11 
12 static double get_time_ms() {
13 // 获取当前时间，单位毫秒，用于计算FPS
14     struct timeval tv;
15     gettimeofday(&tv, NULL);
16     return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
17     // 秒转毫秒 + 微秒转毫秒
18 }
19 
20 int main(int argc, char** argv) {
21 // 主函数入口
22     const char* device = "/dev/video40"; // 默认设备节点
23     int width = 1920;                    // 默认宽度
24     int height = 1080;                   // 默认高度
25 
26     if (argc >= 2) {
27         device = argv[1];
28     }
29     if (argc >= 4) {
30         width = atoi(argv[2]);
31         height = atoi(argv[3]);
32     }
33     // 命令行参数解析：
34     // argv[1] = 设备路径 (可选)
35     // argv[2] argv[3] = 宽 高 (可选)
36 
37     printf("RK3588 HDMI Capture Preview\n");
38     printf("Usage: %s [device] [width] [height]\n", argv[0]);
39     printf("Device: %s\nRequested resolution: %dx%d\n", device, width, height);
40     // 打印欢迎信息和用法
41 
42     hdmi_capture_t cap;
43     if (hdmi_open(&cap, device, width, height) != 0) {
44         fprintf(stderr, "Failed to open %s\n", device);
45         return 1;
46     }
47     // 打开HDMI设备，如果失败退出
48 
49     if (hdmi_start(&cap) != 0) {
50         fprintf(stderr, "Failed to start streaming\n");
51         hdmi_close(&cap);
52         return 1;
53     }
54     // 开始抓流，失败清理退出
55 
56     printf("Streaming started, opening preview window...\n");
57     printf("Press 'q' or ESC to quit\n");
58 
59     cv::namedWindow("HDMI Preview", cv::WINDOW_NORMAL);
60     cv::resizeWindow("HDMI Preview", 960, 540);
61     // 创建OpenCV窗口，缩放到一半大小(960x540)方便显示
62 
63     int frame_count = 0;
64     double start_time = get_time_ms();
65     double fps = 0;
66     // FPS统计变量
67 
68     while (true) {
69     // 主循环
70         uint8_t* data;
71         int w, h;
72 
73         if (hdmi_get_frame(&cap, &data, &w, &h) != 0) {
74             // No frame yet, retry
75             usleep(1000);
76             continue;
77         }
78         // 获取下一帧，没得到就睡1ms再试
79 
80         // Wrap the buffer directly into OpenCV Mat (no copy)
81         cv::Mat frame(h, w, CV_8UC3, data);
82         // 直接用我们映射好的数据创建OpenCV Mat，不需要内存拷贝！零拷贝复用缓冲区
83 
84         // Calculate FPS
85         frame_count++;
86         double elapsed = get_time_ms() - start_time;
87         if (elapsed >= 1000.0) {
88             fps = frame_count * 1000.0 / elapsed;
89             printf("FPS: %.1f\n", fps);
90             frame_count = 0;
91             start_time = get_time_ms();
92         }
93         // 每秒钟计算一次FPS，打印出来
94 
95         // Draw FPS on frame
96         char text[32];
97         snprintf(text, sizeof(text), "FPS: %.1f", fps);
98         cv::putText(frame, text, cv::Point(10, 30),
99                     cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
100         // 在画面左上角画绿色FPS文字
101 
102         cv::imshow("HDMI Preview", frame);
103         // 显示帧
104 
105         hdmi_release_frame(&cap);
106         // 释放帧，放回队列给驱动
107 
108         // Check for exit
109         int key = cv::waitKey(1);
110         if (key == 'q' || key == 27) { // q or ESC
111             break;
112         }
113         // 等待按键，按下q或ESC退出循环
114     }
115 
116     hdmi_stop(&cap);
117     hdmi_close(&cap);
118     cv::destroyAllWindows();
119     // 停止抓流，关闭设备，销毁窗口
120 
121     printf("Done\n");
122     return 0;
123 }
```
