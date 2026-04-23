# YOLOv8 Pose 检测 + 瞄准辅助 - 逐行注释

本文档详细注释了 RK3588 YOLOv8n-pose 姿态检测与瞄准辅助系统的核心代码。
- pose_demo.cpp - 主程序

- 多线程流水线架构
- ThreadQueue 线程安全队列
- 配置解析 (config.ini)
- 按键映射 (F1-F12)
- OpenCV 绘制 (边界框、关键点、骨架、瞄准十字)
- yolo_inference.h/cpp - YOLO 推理

- RKNN API 接口
- NMS 非极大值抑制
- Letterbox 预处理
- 坐标映射
- aim_engine.h - 瞄准引擎

- TargetSelector (目标选择)
- SpringAim (弹簧-阻尼物理)
- StrengthCurve (强度曲线)
- LEGIT/SEMI_RAGE/RAGE 预设
- rga_preprocess.h/c - RGA 硬件加速

- 图像缩放、颜色转换、填充
- hid_controller.h - HID 控制

- AimSystem 系统集成
- 灵敏度/平滑/人类化
- 架构图和配置文件格式

---

## src/pose_demo.cpp

```c
/*
 * HDMI + YOLOv8n-pose Pose Detection Demo - Multi-threaded Pipeline
 * 
 * Pipeline architecture:
 *   Thread 1 (main): Capture frame -> push to preprocess queue
 *   Thread 2: Preprocess frame -> push to NPU queue
 *   Thread 3: NPU inference -> push to postprocess queue
 *   Thread 4: Postprocess + Draw -> display
 * 
 * This allows parallel execution: while NPU processes frame N,
 * CPU preprocesses frame N+1, and captures frame N+2.
 */
// 多线程流水线架构说明
// 主线程负责采集，预处理、NPU推理、后处理分别在独立线程执行
// 实现并行：NPU处理第N帧时，CPU预处理第N+1帧，采集第N+2帧

#include "hdmi_capture.h"      // HDMI 采集
#include "yolo_inference.h"    // YOLO 推理
#include "hid_controller.h"   // HID 控制
#include "hid_relay.h"         // HID 转发
#include "usb_hid.h"          // USB HID 鼠标
#include <opencv2/opencv.hpp> // OpenCV 图像处理
#include <sys/time.h>         // 时间获取
#include <unistd.h>           // usleep
#include <chrono>             // C++ 时间库
#include <string.h>           // 字符串
#include <thread>             // 线程
#include <mutex>              // 互斥锁
#include <condition_variable> // 条件变量
#include <queue>              // 队列
#include <atomic>             // 原子变量
#include <getopt.h>           // 命令行参数解析
// 包含所需的头文件

using namespace std::chrono;
// 使用 chrono 命名空间简化代码

static double get_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}
// 获取当前时间（毫秒）

const int skeleton[14][2] = {
    {0, 1}, {0, 2}, {1, 3}, {2, 4},
    {5, 6}, {5, 11}, {6, 12}, {11, 12},
    {11, 13}, {12, 14}, {0, 5}, {0, 6}, {5, 7}, {6, 8}
};
// 骨架连接定义（COCO 17个关键点的连线索引）
// 每行表示两个关键点之间的连接关系

// Pipeline frame structure
// 流水线帧结构体
struct PipelineFrame {
    cv::Mat frame;  // Original frame (BGR from HDMI)
    // 原始帧（HDMI采集的BGR格式）
    cv::Mat preprocessed;  // Preprocessed frame (RGB 640x640)
    // 预处理后的帧（RGB 640x640）
    std::vector<PoseDetection> detections;
    // 检测结果数组
    float scale = 0;
    // 缩放比例（letterbox用）
    float pad_left = 0;
    // 左侧填充像素
    float pad_top = 0;
    // 顶部填充像素
    int frame_id = 0;
    // 帧ID
    
    // Timing
    // 性能计时
    double capture_time = 0;
    // 采集耗时
    double preprocess_time = 0;
    // 预处理耗时
    double npu_time = 0;
    // NPU推理耗时
    double postprocess_time = 0;
    // 后处理耗时
};

// Thread-safe queue with capacity limit and frame dropping
// 线程安全的队列（带容量限制和丢帧策略）
template<typename T>
class ThreadQueue {
public:
    explicit ThreadQueue(size_t max_size = 3) : max_size_(max_size), dropped_frames_(0) {}
    // 构造函数，设置最大队列长度
    
    // Push with drop policy: if queue is full, drop oldest frame
    // 入队（队列满时丢旧帧）
    bool push(T item, bool drop_if_full = true) {
        std::unique_lock<std::mutex> lock(mutex_);
        // 加锁
        if (queue_.size() >= max_size_) {
            // 队列已满
            if (drop_if_full) {
                // 丢旧帧策略：丢弃最老的帧防止内存爆炸
                queue_.pop();
                dropped_frames_++;
            } else {
                return false; // Queue full
            }
        }
        queue_.push(std::move(item));
        // 入队
        cond_.notify_one();
        // 通知等待线程
        return true;
    }
    
    // 出队
    bool pop(T& item, int timeout_ms = 100) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cond_.wait_for(lock, milliseconds(timeout_ms), [this] { return !queue_.empty() || stop_; })) {
            return false;
        }
        if (queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }
    
    // 获取队列大小
    size_t size() {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    // 获取丢帧数
    size_t dropped() {
        std::unique_lock<std::mutex> lock(mutex_);
        return dropped_frames_;
    }
    
    // 重置丢帧计数
    void reset_dropped() {
        std::unique_lock<std::mutex> lock(mutex_);
        dropped_frames_ = 0;
    }
    
    // 停止队列
    void stop() {
        std::unique_lock<std::mutex> lock(mutex_);
        stop_ = true;
        cond_.notify_all();
    }
    
private:
    std::queue<T> queue_;
    // 内部队列
    std::mutex mutex_;
    // 互斥锁
    std::condition_variable cond_;
    // 条件变量
    bool stop_ = false;
    // 停止标志
    size_t max_size_;
    // 最大容量
    size_t dropped_frames_;
    // 丢帧计数
};

// Global queues with limited capacity to prevent memory explosion
// 全局流水线队列（容量限制防止内存爆炸）
// Capacity = 2 means at most 2 frames buffered, old frames dropped when full
// 容量为2表示最多缓冲2帧，满时丢旧帧
ThreadQueue<PipelineFrame> preprocess_queue(2);
ThreadQueue<PipelineFrame> npu_queue(2);
ThreadQueue<PipelineFrame> display_queue(2);
std::atomic<bool> running{true};
// 运行标志
std::atomic<int> frame_counter{0};
// 帧计数器
std::atomic<size_t> total_dropped_frames{0};
// 总丢帧数

// Aim system (integrates aim engine + HID controller)
// 瞄准系统（整合瞄准引擎和HID控制器）
hid::AimSystem* g_aim_system = nullptr;
hid::HIDRelay* g_hid_relay = nullptr;
bool g_aim_enabled = true;   // 默认开启吸附
bool g_auto_fire = false;    // F4 控制自动开枪模式
aim::AimPreset g_aim_preset = aim::AimPreset::LEGIT;
std::vector<PoseDetection> g_latest_detections; // 保存最新的检测结果

// 按键名称解析函数（配置文件中F1-F12转换为Linux按键码）
static int parse_key_name(const char* name) {
    if (!name) return 0;
    if (strcmp(name, "F1") == 0) return 59;
    if (strcmp(name, "F2") == 0) return 60;
    if (strcmp(name, "F3") == 0) return 61;
    if (strcmp(name, "F4") == 0) return 62;
    if (strcmp(name, "F5") == 0) return 63;
    if (strcmp(name, "F6") == 0) return 64;
    if (strcmp(name, "F7") == 0) return 65;
    if (strcmp(name, "F8") == 0) return 66;
    if (strcmp(name, "F9") == 0) return 67;
    if (strcmp(name, "F10") == 0) return 68;
    if (strcmp(name, "F11") == 0) return 87;
    if (strcmp(name, "F12") == 0) return 88;
    return 0;
 }
// Linux input event 按键码：F1-F12 对应 59-68, 87, 88

// 字符串去空格辅助函数
 static std::string trim(const std::string& s) {
     size_t start = s.find_first_not_of(" \t\r\n");
     if (start == std::string::npos) return "";
     size_t end = s.find_last_not_of(" \t\r\n");
     return s.substr(start, end - start + 1);
 }

 // 按键码转名称（用于显示）
 static const char* key_code_to_name(int code) {
     switch(code) {
         case 59: return "F1";
         case 60: return "F2";
         case 61: return "F3";
         case 62: return "F4";
         case 63: return "F5";
         case 64: return "F6";
         case 65: return "F7";
         case 66: return "F8";
         case 67: return "F9";
         case 68: return "F10";
         case 87: return "F11";
         case 88: return "F12";
         default: return "?";
     }
 }

// 模式 B: 完整转发配置（设置为空表示禁用转发）
const char* g_keyboard_device = nullptr;  // "/dev/input/event0"
const char* g_mouse_device = nullptr;     // "/dev/input/event1"

// 默认按键映射
int g_key_legit = 59;        // F1
int g_key_semirage = 60;     // F2
int g_key_rage = 61;         // F3
int g_key_autofire = 62;     // F4
int g_key_mode = 63;         // F5
int g_key_sens_up = 64;      // F6
int g_key_sens_down = 65;     // F7
int g_key_game_mode = 67;    // F9
// Linux 按键码对应关系

// Preprocess thread: BGR -> RGB letterbox using fast_letterbox (with RGA support)
// 预处理线程：BGR -> RGB letterbox（支持RGA硬件加速）
void preprocess_thread(int input_w, int input_h) {
    // Pre-allocate buffer for preprocessing
    // 预分配预处理缓冲区
    cv::Mat preprocessed(input_h, input_w, CV_8UC3);
    
    while (running) {
        // 从预处理队列获取帧
        PipelineFrame frame;
        if (!preprocess_queue.pop(frame, 50)) continue;
        
        auto t0 = high_resolution_clock::now();
        // 记录开始时间
        
        float scale, pad_left, pad_top;
        
        // Use fast_letterbox from yolo_inference.cpp (supports RGA hardware acceleration)
        // 使用 yolo_inference.cpp 的 fast_letterbox（支持RGA硬件加速）
        // This ensures consistent preprocessing between pipeline and direct inference
        // 确保流水线模式和直接推理的预处理一致
        fast_letterbox(frame.frame, preprocessed.data, input_w, input_h, scale, pad_left, pad_top);
        
        frame.preprocessed = preprocessed.clone();  // Clone to own the data
        // 克隆数据拥有所有权
        frame.scale = scale;
        frame.pad_left = pad_left;
        frame.pad_top = pad_top;
        
        auto t1 = high_resolution_clock::now();
        frame.preprocess_time = duration<double>(t1 - t0).count() * 1000.0;
        // 计算预处理耗时
        
        // Push to NPU queue, drop if full (old frames discarded)
        // 推入NPU队列，满时丢帧
        if (!npu_queue.push(std::move(frame), true)) {
            // Frame dropped
        }
    }
}

// NPU inference thread
// NPU推理线程
void npu_thread(YoloPoseInference* inference) {
    while (running) {
        // 从NPU队列获取帧
        PipelineFrame frame;
        if (!npu_queue.pop(frame, 50)) continue;
        
        auto t0 = high_resolution_clock::now();
        // 记录开始时间
        
        // Run NPU inference directly with preprocessed buffer
        // 直接使用预处理缓冲区进行NPU推理
        // Use lower confidence threshold (0.25) to detect more distant/small targets
        // 使用较低置信度阈值(0.25)检测更远/更小的目标
        frame.detections = inference->detect_raw(
            frame.preprocessed.data,
            frame.scale,
            frame.pad_left,
            frame.pad_top,
            0.25f
        );
        
        auto t1 = high_resolution_clock::now();
        frame.npu_time = duration<double>(t1 - t0).count() * 1000.0;
        // 计算NPU推理耗时
        
        // Push to display queue, drop if full
        // 推入显示队列，满时丢帧
        if (!display_queue.push(std::move(frame), true)) {
            // Frame dropped
        }
    }
}

int main(int argc, char** argv) {
    // 主函数：程序入口
    const char* device = "/dev/video40";
    // 默认HDMI设备
    int width = 1920;
    int height = 1080;
    // 默认分辨率

    // 获取可执行文件所在目录
    std::string exe_dir = "";
    if (argc > 0 && argv[0]) {
        std::string exe_path = argv[0];
        size_t pos = exe_path.rfind('/');
        if (pos != std::string::npos) {
            exe_dir = exe_path.substr(0, pos + 1);
        }
    }

    // 模型路径列表（按优先级尝试加载）
    std::vector<std::string> model_paths = {
        "/home/ztl/github/RK/src/yolov8n-pose.rknn",
        exe_dir + "yolov8n-pose.rknn",
        "yolov8n-pose.rknn",
    };
    const char* model_path = model_paths[0].c_str();

    // 首先解析 --config 选项
    const char* config_file = nullptr;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_file = argv[++i];
            break;
        }
    }

    // 如果指定了配置文件，加载它
    if (config_file) {
        FILE* fp = fopen(config_file, "r");
        if (fp) {
            fclose(fp);
            char line[256];
            fp = fopen(config_file, "r");
            while (fgets(line, sizeof(line), fp)) {
                // 解析模型路径
                if (strstr(line, "model = ")) {
                    char* eq = strstr(line, "=");
                    if (eq) {
                        char* path = eq + 1;
                        while (*path == ' ') path++;
                        path[strlen(path)-1] = '\0';
                        model_paths.insert(model_paths.begin(), path);
                    }
                }
                // 解析设备路径
                if (strstr(line, "device = ") && !strstr(line, "keyboard") && !strstr(line, "mouse")) {
                    char* eq = strstr(line, "=");
                    if (eq) {
                        char* path = eq + 1;
                        while (*path == ' ') path++;
                        path[strlen(path)-1] = '\0';
                        device = strdup(path);
                    }
                }
                // 解析按键映射
                if (strstr(line, "key_legit = ")) {
                    char* eq = strstr(line, "=");
                    if (eq) { g_key_legit = parse_key_name(trim(eq+1).c_str()); }
                }
                if (strstr(line, "key_semirage = ")) {
                    char* eq = strstr(line, "=");
                    if (eq) { g_key_semirage = parse_key_name(trim(eq+1).c_str()); }
                }
                if (strstr(line, "key_rage = ")) {
                    char* eq = strstr(line, "=");
                    if (eq) { g_key_rage = parse_key_name(trim(eq+1).c_str()); }
                }
                if (strstr(line, "key_autofire = ")) {
                    char* eq = strstr(line, "=");
                    if (eq) { g_key_autofire = parse_key_name(trim(eq+1).c_str()); }
                }
                if (strstr(line, "key_mode = ")) {
                    char* eq = strstr(line, "=");
                    if (eq) { g_key_mode = parse_key_name(trim(eq+1).c_str()); }
                }
                if (strstr(line, "key_sens_up = ")) {
                    char* eq = strstr(line, "=");
                    if (eq) { g_key_sens_up = parse_key_name(trim(eq+1).c_str()); }
                }
                if (strstr(line, "key_sens_down = ")) {
                    char* eq = strstr(line, "=");
                    if (eq) { g_key_sens_down = parse_key_name(trim(eq+1).c_str()); }
                }
                if (strstr(line, "key_game_mode = ")) {
                    char* eq = strstr(line, "=");
                    if (eq) { g_key_game_mode = parse_key_name(trim(eq+1).c_str()); }
                }
            }
            fclose(fp);
            printf("Loaded config from: %s\n", config_file);
        } else {
            fprintf(stderr, "Warning: Cannot open config file: %s\n", config_file);
        }
    }

    // 解析位置参数
    std::vector<char*> positional_args;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0) {
            i++; // skip value
        } else if (argv[i][0] != '-') {
            positional_args.push_back(argv[i]);
        }
    }
    
    // device [width height [model]]
    if (positional_args.size() >= 1) {
        device = positional_args[0];
    }
    if (positional_args.size() == 3) {
        width = atoi(positional_args[1]);
        height = atoi(positional_args[2]);
    } else if (positional_args.size() == 2) {
        if (isdigit(positional_args[0][0])) {
            width = atoi(positional_args[0]);
            height = atoi(positional_args[1]);
        }
    } else if (positional_args.size() >= 4) {
        width = atoi(positional_args[1]);
        height = atoi(positional_args[2]);
        model_path = positional_args[3];
    }
    
    // 解析可选的键盘鼠标设备参数（模式B）
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--keyboard") == 0 && i + 1 < argc) {
            g_keyboard_device = argv[++i];
        } else if (strcmp(argv[i], "--mouse") == 0 && i + 1 < argc) {
            g_mouse_device = argv[++i];
        }
    }

    // 打印配置信息
    printf("\n");
    printf("=======================================\n");
    printf("RK3588 YOLOv8n-pose Pipeline Demo\n");
    printf("=======================================\n");
    printf("Device: %s\n", device);
    printf("Resolution: %dx%d\n", width, height);
    printf("Model: %s\n", model_path);
    if (g_keyboard_device) printf("Keyboard: %s\n", g_keyboard_device);
    if (g_mouse_device) printf("Mouse: %s\n", g_mouse_device);
    printf("=======================================\n");
    printf("\n");

    // 打开HDMI采集设备
    hdmi_capture_t cap;
    if (hdmi_open(&cap, device, width, height) != 0) {
        fprintf(stderr, "Failed to open %s\n", device);
        return 1;
    }

    // 开始视频流
    if (hdmi_start(&cap) != 0) {
        fprintf(stderr, "Failed to start streaming\n");
        hdmi_close(&cap);
        return 1;
    }

    // 初始化YOLO推理
    YoloPoseInference inference;
    const char* loaded_model = nullptr;
    for (const auto& path : model_paths) {
        if (inference.init(path.c_str()) == 0) {
            loaded_model = path.c_str();
            model_path = loaded_model;
            break;
        }
    }
    if (!loaded_model) {
        fprintf(stderr, "Failed to initialize YOLO model: %s\n", model_path);
        hdmi_stop(&cap);
        hdmi_close(&cap);
        return 1;
    }

    // 获取模型输入尺寸
    int input_w = inference.get_input_width();
    int input_h = inference.get_input_height();

    // 打印帮助信息
    printf("\n");
    printf("Model input resolution: %dx%d\n", input_w, input_h);
    printf("Streaming started, opening preview window...\n");
    printf("Controls:\n");
    printf("  'q' or ESC - Quit\n");
    printf("\n");
    printf("=== 模式 B: 键盘鼠标转发 (按 %s 开启游戏模式) ===\n", key_code_to_name(g_key_game_mode));
    printf("  %s - Legit预设\n", key_code_to_name(g_key_legit));
    printf("  %s - Semi-rage预设\n", key_code_to_name(g_key_semirage));
    printf("  %s - Rage预设\n", key_code_to_name(g_key_rage));
    printf("  %s - 自动开枪模式开关\n", key_code_to_name(g_key_autofire));
    printf("  %s - 切换绝对/相对模式\n", key_code_to_name(g_key_mode));
    printf("  %s - 增加灵敏度\n", key_code_to_name(g_key_sens_up));
    printf("  %s - 降低灵敏度\n", key_code_to_name(g_key_sens_down));
    printf("\n");
    
    // 初始化瞄准系统（引擎 + HID控制器）
    g_aim_system = new hid::AimSystem();
    if (!g_aim_system->init(g_aim_preset)) {
        fprintf(stderr, "Warning: Failed to initialize HID controller. Aim assist will be visual only.\n");
        fprintf(stderr, "To enable HID control, run as root or configure udev rules.\n");
    } else {
        g_aim_system->setEnabled(g_aim_enabled);
        printf("Aim system initialized. Aim is: %s (Auto-fire: %s)\n", 
               g_aim_enabled ? "ON" : "OFF", g_auto_fire ? "ON" : "OFF");
        printf("Press %s to toggle auto-fire mode (requires %s game mode).\n",
               key_code_to_name(g_key_autofire), key_code_to_name(g_key_game_mode));
    }
    
    // 初始化HID转发器（模式B：完整键盘鼠标转发）
    if (g_keyboard_device || g_mouse_device) {
        printf("\n=== 模式 B: 完整键盘鼠标转发 ===\n");
        hid::RelayConfig relay_cfg;
        relay_cfg.keyboard_device = g_keyboard_device;
        relay_cfg.mouse_device = g_mouse_device;
        relay_cfg.preset_legit_key = g_key_legit;
        relay_cfg.preset_semirage_key = g_key_semirage;
        relay_cfg.preset_rage_key = g_key_rage;
        relay_cfg.toggle_aim_key = g_key_autofire;
        relay_cfg.toggle_mode_key = g_key_mode;
        relay_cfg.sens_up_key = g_key_sens_up;
        relay_cfg.sens_down_key = g_key_sens_down;
        relay_cfg.toggle_game_mode_key = g_key_game_mode;
        
        g_hid_relay = new hid::HIDRelay(relay_cfg);
        if (g_hid_relay->init(g_aim_system)) {
            g_hid_relay->start();
            printf("HID relay started.\n");
        } else {
            fprintf(stderr, "Warning: Failed to initialize HID relay.\n");
            delete g_hid_relay;
            g_hid_relay = nullptr;
        }
    }

    // 创建OpenCV预览窗口
    cv::namedWindow("YOLOv8n-pose Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("YOLOv8n-pose Detection", 960, 540);

    // 启动工作线程
    std::thread preprocess_t(preprocess_thread, input_w, input_h);
    std::thread npu_t(npu_thread, &inference);

    // 主循环变量
    int frame_count = 0;
    double start_time = get_time_ms();
    
    float total_capture = 0;
    float total_preprocess = 0;
    float total_npu = 0;
    float total_draw = 0;
    float total_all = 0;

    while (true) {
        // 主循环：采集 -> 显示
        auto t0 = high_resolution_clock::now();
        
        uint8_t* data;
        int w, h;

        // 获取一帧
        if (hdmi_get_frame(&cap, &data, &w, &h) != 0) {
            usleep(500);
            continue;
        }
        
        auto t1 = high_resolution_clock::now();
        float capture_ms = duration<float>(t1 - t0).count() * 1000.0f;

        // 创建帧并推入预处理队列（满时丢帧防止卡顿）
        PipelineFrame frame;
        frame.frame = cv::Mat(h, w, CV_8UC3, data).clone();  // Clone to avoid buffer issues
        // 克隆数据避免缓冲区问题
        frame.frame_id = frame_counter++;
        frame.capture_time = capture_ms;
        
        if (!preprocess_queue.push(std::move(frame), true)) {
            // Frame dropped due to full queue
        }
        
        // 尝试从显示队列获取处理好的帧（非阻塞）
        PipelineFrame display_frame;
        while (display_queue.pop(display_frame, 0)) {
            // 保存最新检测结果用于瞄准系统
            g_latest_detections = display_frame.detections;
            
            auto t2 = high_resolution_clock::now();
            
            // 绘制检测结果
            cv::Mat& display_img = display_frame.frame;
            
            for (const auto& det : display_frame.detections) {
                // 绘制边界框
                int x1 = static_cast<int>(det.x - det.w / 2.0f);
                int y1 = static_cast<int>(det.y - det.h / 2.0f);
                int x2 = static_cast<int>(det.x + det.w / 2.0f);
                int y2 = static_cast<int>(det.y + det.h / 2.0f);
                
                cv::rectangle(display_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
                // 绿色边界框

                // 绘制关键点
                for (int k = 0; k < NUM_KP; k++) {
                    if (det.kp_score[k] > 0.5f) {
                        cv::circle(display_img, cv::Point(static_cast<int>(det.kp[k][0]), static_cast<int>(det.kp[k][1])), 3, cv::Scalar(0, 0, 255), -1);
                    }
                }
                // 红色关键点

                // 绘制骨架
                for (int i = 0; i < 14; i++) {
                    int kp1 = skeleton[i][0];
                    int kp2 = skeleton[i][1];
                    if (det.kp_score[kp1] > 0.5f && det.kp_score[kp2] > 0.5f) {
                        cv::line(display_img,
                            cv::Point(static_cast<int>(det.kp[kp1][0]), static_cast<int>(det.kp[kp1][1])),
                            cv::Point(static_cast<int>(det.kp[kp2][0]), static_cast<int>(det.kp[kp2][1])),
                            cv::Scalar(255, 0, 255), 1);
                    }
                }
                // 紫色骨架线

                // 绘制瞄准目标（红色十字）
                if (g_aim_system && g_aim_system->hasTarget()) {
                    auto target = g_aim_system->getTarget();
                    int tx = static_cast<int>(target.pos.x);
                    int ty = static_cast<int>(target.pos.y);
                    cv::line(display_img, cv::Point(tx - 10, ty), cv::Point(tx + 10, ty), cv::Scalar(0, 0, 255), 2);
                    cv::line(display_img, cv::Point(tx, ty - 10), cv::Point(tx, ty + 10), cv::Scalar(0, 0, 255), 2);
                }
            }
            
            cv::imshow("YOLOv8n-pose Detection", display_img);
            // 显示图像
            
            auto t3 = high_resolution_clock::now();
            float draw_ms = duration<float>(t3 - t2).count() * 1000.0f;
            
            total_draw += draw_ms;
        }
        
        // 处理按键
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            // 'q' 或 ESC 退出
            break;
        }
    }
    
    running = false;
    
    // 等待线程结束
    preprocess_t.join();
    npu_t.join();
    
    // 清理资源
    hdmi_stop(&cap);
    hdmi_close(&cap);
    
    if (g_hid_relay) {
        delete g_hid_relay;
    }
    
    if (g_aim_system) {
        delete g_aim_system;
    }
    
    cv::destroyAllWindows();
    
    printf("Done\n");
    return 0;
}
```

---

## src/yolo_inference.h

```cpp
#ifndef YOLO_POSE_INFERENCE_H
#define YOLO_POSE_INFERENCE_H

#include "rknn_api.h"         // RKNN API 头文件
#include <stdint.h>           // 标准整数类型
#include <vector>             // 向量容器
#include <opencv2/opencv.hpp> // OpenCV

const int NUM_KP = 17;
// COCO 姿态检测的关键点数量

// 暴露给流水线的预处理函数
// 快速 letterbox 缩放（支持RGA硬件加速）
void fast_letterbox(const cv::Mat& image, uint8_t* output_buf, 
                    int target_w, int target_h,
                    float& scale, float& pad_left, float& pad_top);
// image: 输入图像
// output_buf: 预分配的输出缓冲区
// target_w, target_h: 目标尺寸
// scale: 输出缩放比例
// pad_left, pad_top: 输出填充偏移

// 检测结果结构体
struct PoseDetection {
    float x, y, w, h;         // 边界框 (x,y 为中心点)
    float score;              // 置信度
    float kp[NUM_KP][2];      // 关键点坐标
    float kp_score[NUM_KP];   // 关键点置信度
};

class YoloPoseInference {
public:
    YoloPoseInference();                  // 构造函数
    ~YoloPoseInference();                 // 析构函数

    // 初始化模型
    // model_path: 模型文件路径
    // input_width, input_height: 输入尺寸（0表示从模型获取）
    int init(const char* model_path, int input_width = 0, int input_height = 0);
    void release();                       // 释放资源

    // 检测接口（带完整预处理）
    std::vector<PoseDetection> detect(const cv::Mat& bgr_img, float conf_threshold = 0.25f);
    
    // 原始检测接口（使用预处理好的缓冲区，流水线模式用）
    std::vector<PoseDetection> detect_raw(uint8_t* preprocessed_buf, float scale, float pad_left, float pad_top, float conf_threshold = 0.25f);
    
    bool is_initialized() const { return initialized_; }  // 是否已初始化
    
    // 性能统计
    struct TimingStats {
        float preprocess_ms;   // 预处理耗时
        float inference_ms;     // 推理耗时
        float postprocess_ms;   // 后处理耗时
    };
    TimingStats get_last_timing() const { return last_timing_; }
    
    // 获取模型输入尺寸
    int get_input_width() const { return input_width_; }
    int get_input_height() const { return input_height_; }

private:
    rknn_context rknn_ctx_;    // RKNN 上下文句柄
    bool initialized_;          // 初始化标志
    int input_width_;          // 输入宽度
    int input_height_;         // 输入高度
    int channel_;              // 通道数
    bool output_quantized_;     // 输出是否量化
    float output_scale_;       // 输出量化 scale
    int32_t output_zero_point_; // 输出量化零点
    int output_type_;          // 输出数据类型
    int output_fmt_;           // 输出格式 (NCHW/NHWC)
    int output_dims_[4];       // 输出维度
    
    uint8_t* input_buf_;       // 预分配的输入缓冲区
    size_t input_buf_size_;    // 输入缓冲区大小
    
    TimingStats last_timing_;   // 最后一次推理的性能统计
};

#endif
```

---

## src/yolo_inference.cpp

```cpp
/*
 * YOLOv8n-pose Inference on RK3588 NPU via RKNN
 * Optimized for performance - with detailed timing
 */
// YOLOv8n-pose 在 RK3588 NPU 上的推理实现
// 优化性能，包含详细计时

#include "yolo_inference.h"
// 包含 YOLO 推理头文件

#include <stdio.h>      // 标准输入输出
#include <stdlib.h>     // 标准库
#include <string.h>     // 字符串处理
#include <algorithm>    // 算法库
#include <cmath>        // 数学函数
#include <chrono>       // 时间库
#include <vector>       // 向量

#include "rknn_api.h"   // RKNN API
#include "opencv2/opencv.hpp" // OpenCV

#ifdef USE_RGA_PREPROCESS
#include "rga_preprocess.h" // RGA 硬件加速
#endif

using namespace std::chrono;
// 使用 chrono 命名空间

// NMS 非极大值抑制
// 过滤重叠的检测框
static void nms(std::vector<PoseDetection>& input, std::vector<PoseDetection>& output, float iou_threshold) {
    // 按置信度降序排序
    sort(input.begin(), input.end(), [](const PoseDetection& a, const PoseDetection& b) {
        return a.score > b.score;
    });

    while (!input.empty()) {
        // 保留置信度最高的
        output.push_back(input[0]);
        if (input.size() == 1) {
            break;
        }

        // 计算与最高置信度框的 IOU
        std::vector<PoseDetection> remaining;
        float x1a = input[0].x - input[0].w / 2.0f;
        float y1a = input[0].y - input[0].h / 2.0f;
        float x2a = input[0].x + input[0].w / 2.0f;
        float y2a = input[0].y + input[0].h / 2.0f;
        float areaa = input[0].w * input[0].h;

        for (size_t i = 1; i < input.size(); i++) {
            float x1b = input[i].x - input[i].w / 2.0f;
            float y1b = input[i].y - input[i].h / 2.0f;
            float x2b = input[i].x + input[i].w / 2.0f;
            float y2b = input[i].y + input[i].h / 2.0f;
            float areab = input[i].w * input[i].h;

            // 计算交集
            float xx1 = std::max(x1a, x1b);
            float yy1 = std::max(y1a, y1b);
            float xx2 = std::min(x2a, x2b);
            float yy2 = std::min(y2a, y2b);

            float w = std::max(0.0f, xx2 - xx1);
            float h = std::max(0.0f, yy2 - yy1);
            float intersection = w * h;
            float union_area = areaa + areab - intersection;
            float iou = intersection / union_area;

            // IOU 小于阈值的保留
            if (iou <= iou_threshold) {
                remaining.push_back(input[i]);
            }
        }
        input = remaining;
    }
}

YoloPoseInference::YoloPoseInference() {
    // 构造函数：初始化成员变量
    rknn_ctx_ = 0;
    initialized_ = false;
    input_width_ = 640;
    input_height_ = 640;
    channel_ = 3;
    output_quantized_ = false;
    output_scale_ = 0.0f;
    output_zero_point_ = 0;
    output_type_ = 0;
    input_buf_ = nullptr;
    input_buf_size_ = 0;
    memset(&last_timing_, 0, sizeof(last_timing_));
}

YoloPoseInference::~YoloPoseInference() {
    release();
}
// 析构函数：释放资源

int YoloPoseInference::init(const char* model_path, int input_width, int input_height) {
    // 加载 RKNN 模型
    FILE* fp = fopen(model_path, "rb");
    if (!fp) {
        fprintf(stderr, "Failed to open model: %s\n", model_path);
        return -1;
    }

    // 获取文件大小
    fseek(fp, 0, SEEK_END);
    size_t model_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    // 分配内存并读取模型
    void* model_buffer = malloc(model_size);
    if (!model_buffer) {
        fprintf(stderr, "Failed to allocate memory\n");
        fclose(fp);
        return -1;
    }

    if (fread(model_buffer, 1, model_size, fp) != model_size) {
        fprintf(stderr, "Warning: Failed to read full model file\n");
    }
    fclose(fp);

    // 初始化 RKNN 上下文
    rknn_context ctx;
    int ret = rknn_init(&ctx, model_buffer, model_size, 0, 0);
    free(model_buffer);

    if (ret != 0) {
        fprintf(stderr, "rknn_init failed: %d\n", ret);
        return -1;
    }

    rknn_ctx_ = ctx;
    initialized_ = true;

#ifdef USE_RGA_PREPROCESS
    // 初始化 RGA 硬件加速
    rga_preprocess_init();
#endif

    // 查询输入输出数量
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != 0) {
        fprintf(stderr, "rknn_query io num failed: %d\n", ret);
        release();
        return -1;
    }

    printf("YOLOv8n-pose model loaded: %d inputs, %d outputs\n", io_num.n_input, io_num.n_output);

    // 查询输入张量属性
    rknn_tensor_attr input_attr;
    memset(&input_attr, 0, sizeof(input_attr));
    input_attr.index = 0;
    ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &input_attr, sizeof(input_attr));
    if (ret == 0) {
        printf("Input tensor info:\n");
        printf("  type: %d\n", input_attr.type);
        printf("  qnt_type: %d\n", input_attr.qnt_type);
        printf("  fmt: %d\n", input_attr.fmt);
        printf("  dims: [%d, %d, %d, %d]\n",
               input_attr.dims[0], input_attr.dims[1],
               input_attr.dims[2], input_attr.dims[3]);
        
        // 解析输入维度
        if (input_attr.n_dims == 4) {
            if (input_attr.dims[1] == 3) {
                channel_ = input_attr.dims[1];
                input_height_ = input_attr.dims[2];
                input_width_ = input_attr.dims[3];
            } else {
                input_height_ = input_attr.dims[1];
                input_width_ = input_attr.dims[2];
                channel_ = input_attr.dims[3];
            }
        }
        printf("Input size: %dx%d %d channels\n", input_width_, input_height_, channel_);
        if (input_attr.qnt_type != RKNN_TENSOR_QNT_NONE) {
            printf("  Input quantized: scale=%f, zp=%d\n", input_attr.scale, input_attr.zp);
        }
    }

    // 查询输出张量属性
    if (io_num.n_output >= 1) {
        rknn_tensor_attr output_attr;
        memset(&output_attr, 0, sizeof(output_attr));
        output_attr.index = 0;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &output_attr, sizeof(output_attr));
        if (ret == 0) {
            printf("Output tensor info:\n");
            printf("  type: %d\n", output_attr.type);
            printf("  qnt_type: %d\n", output_attr.qnt_type);
            printf("  fmt: %d\n", output_attr.fmt);
            printf("  dims: [%d, %d, %d, %d]\n",
                   output_attr.dims[0], output_attr.dims[1],
                   output_attr.dims[2], output_attr.dims[3]);
            
            output_type_ = output_attr.type;
            output_fmt_ = output_attr.fmt;
            memcpy(output_dims_, output_attr.dims, sizeof(output_dims_));
            
            // 处理量化
            if (output_attr.qnt_type != RKNN_TENSOR_QNT_NONE) {
                output_quantized_ = true;
                output_scale_ = output_attr.scale;
                output_zero_point_ = output_attr.zp;
                printf("  Quantized: scale=%.6f, zp=%d\n", output_scale_, output_zero_point_);
            } else {
                output_quantized_ = false;
                printf("  Not quantized\n");
            }
            
            // 判断输出布局
            // YOLOv8-pose 输出: [1, 56, 8400] (NCHW) 或 [1, 8400, 56] (NHWC)
            if (output_attr.n_dims >= 3) {
                if (output_attr.dims[1] == 56 && output_attr.dims[2] == 8400) {
                    printf("  Output layout: NCHW [1, 56, 8400]\n");
                } else if (output_attr.dims[1] == 8400 && output_attr.dims[2] == 56) {
                    printf("  Output layout: NHWC [1, 8400, 56]\n");
                }
            }
        }
    }
    
    // 预分配输入缓冲区
    input_buf_size_ = input_width_ * input_height_ * channel_;
    input_buf_ = static_cast<uint8_t*>(malloc(input_buf_size_));
    printf("Pre-allocated input buffer: %zu bytes\n", input_buf_size_);

    return 0;
}

void YoloPoseInference::release() {
    // 释放资源
    if (input_buf_) {
        free(input_buf_);
        input_buf_ = nullptr;
    }
    if (initialized_) {
        rknn_destroy(rknn_ctx_);
    }
    initialized_ = false;
}

// OpenCV 版本的 letterbox 预处理
static void fast_letterbox_opencv(const cv::Mat& image, uint8_t* output_buf, 
                        int target_w, int target_h,
                        float& scale, float& pad_left, float& pad_top) {
    int img_w = image.cols;
    int img_h = image.rows;
    
    // 计算缩放比例（保持宽高比）
    float scale_x = static_cast<float>(target_w) / img_w;
    float scale_y = static_cast<float>(target_h) / img_h;
    scale = std::min(scale_x, scale_y);
    
    // 计算缩放后的尺寸
    int new_w = static_cast<int>(img_w * scale);
    int new_h = static_cast<int>(img_h * scale);
    
    // 计算填充偏移（居中）
    pad_left = (target_w - new_w) / 2.0f;
    pad_top = (target_h - new_h) / 2.0f;
    
    // 创建灰色填充的目标图像
    cv::Mat padded(target_h, target_w, CV_8UC3, cv::Scalar(114, 114, 114));
    
    int x_offset = static_cast<int>(pad_left);
    int y_offset = static_cast<int>(pad_top);
    
    // 缩放到中间位置
    cv::Mat roi = padded(cv::Rect(x_offset, y_offset, new_w, new_h));
    
    // 缩放 + 颜色转换一步到位
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    
    // HDMI 采集的是 BGR3 但实际数据是 RGB
    // 所以不需要颜色转换，直接拷贝
    resized.copyTo(roi);
    
    // 拷贝到输出缓冲区
    memcpy(output_buf, padded.data, target_w * target_h * 3);
}

// 暴露给流水线的预处理函数
void fast_letterbox(const cv::Mat& image, uint8_t* output_buf,
                        int target_w, int target_h,
                        float& scale, float& pad_left, float& pad_top) {
    // 优先使用 RGA 硬件加速
#ifndef FORCE_OPENCV_PREPROCESS
#ifdef USE_RGA_PREPROCESS
    if (rga_preprocess_available()) {
        int ret = rga_letterbox(image.data, image.cols, image.rows,
                                output_buf, target_w, target_h,
                                &scale, &pad_left, &pad_top);
        if (ret == 0) {
            return;
        }
        // RGA 失败，回退到 OpenCV
    }
#endif
#endif
    // 回退到 OpenCV 实现
    fast_letterbox_opencv(image, output_buf, target_w, target_h, scale, pad_left, pad_top);
}

// 内部函数：运行 NPU 推理并进行后处理
static std::vector<PoseDetection> run_inference_postprocess(
    rknn_context ctx,            // RKNN 上下文
    uint8_t* input_buf,         // 输入缓冲区
    size_t input_buf_size,      // 输入缓冲区大小
    int input_width,            // 输入宽度
    int input_height,           // 输入高度
    int* output_dims,           // 输出维度
    int output_type,            // 输出类型
    bool output_quantized,      // 是否量化
    float output_scale,         // 量化 scale
    int32_t output_zero_point,  // 量化零点
    float scale,                // letterbox 缩放比例
    float pad_left,             // 左侧填充
    float pad_top,              // 顶部填充
    float conf_threshold,       // 置信度阈值
    YoloPoseInference::TimingStats* timing) // 性能统计
{
    std::vector<PoseDetection> result;
    
    auto t1 = high_resolution_clock::now();

    // 准备输入
    rknn_input input;
    memset(&input, 0, sizeof(input));
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.size = input_buf_size;
    input.fmt = RKNN_TENSOR_NHWC;
    input.buf = input_buf;

    // 设置输入
    int ret = rknn_inputs_set(ctx, 1, &input);
    if (ret != 0) {
        fprintf(stderr, "rknn_inputs_set failed: %d\n", ret);
        return result;
    }

    // 运行推理
    ret = rknn_run(ctx, nullptr);
    if (ret != 0) {
        fprintf(stderr, "rknn_run failed: %d\n", ret);
        return result;
    }

    // 获取输出
    rknn_output output;
    memset(&output, 0, sizeof(output));
    output.index = 0;
    output.want_float = 1;  // RKNN 自动反量化为 float
    ret = rknn_outputs_get(ctx, 1, &output, nullptr);
    if (ret != 0) {
        fprintf(stderr, "rknn_outputs_get failed: %d\n", ret);
        return result;
    }
    
    auto t2 = high_resolution_clock::now();
    if (timing) {
        timing->inference_ms = duration<float>(t2 - t1).count() * 1000.0f;
    }

    // RKNN with want_float=1 返回 float 缓冲区
    float* float_buf = static_cast<float*>(output.buf);
    
    // YOLO 输出格式：8400 个检测结果，每个 56 个值
    // 前4个: cx, cy, w, h
    // 第5个: 置信度
    // 后51个: 17个关键点 * 3 (x, y, conf)
    int num_detections = 8400;
    int num_classes = 56;
    std::vector<PoseDetection> candidates;
    candidates.reserve(20);
    
    // 判断是 NCHW 还是 NHWC
    bool is_nchw = (output_dims[1] == 56 && output_dims[2] == 8400);
    
    // 遍历所有检测结果
    for (int i = 0; i < num_detections; i++) {
        float cx, cy, w, h, score;
        float kp_data[51];
        
        if (is_nchw) {
            // NCHW 格式: [batch, channels, num_detections]
            cx = get_value(i);
            cy = get_value(num_detections + i);
            w  = get_value(2 * num_detections + i);
            h  = get_value(3 * num_detections + i);
            score = get_value(4 * num_detections + i);
            
            // 读取17个关键点
            for (int k = 0; k < NUM_KP; k++) {
                kp_data[k * 3 + 0] = get_value((5 + k * 3 + 0) * num_detections + i);
                kp_data[k * 3 + 1] = get_value((5 + k * 3 + 1) * num_detections + i);
                kp_data[k * 3 + 2] = get_value((5 + k * 3 + 2) * num_detections + i);
            }
        } else {
            // NHWC 格式: [batch, num_detections, channels]
            cx = get_value(i * num_classes + 0);
            cy = get_value(i * num_classes + 1);
            w  = get_value(i * num_classes + 2);
            h  = get_value(i * num_classes + 3);
            score = get_value(i * num_classes + 4);
            
            for (int k = 0; k < NUM_KP; k++) {
                kp_data[k * 3 + 0] = get_value(i * num_classes + 5 + k * 3 + 0);
                kp_data[k * 3 + 1] = get_value(i * num_classes + 5 + k * 3 + 1);
                kp_data[k * 3 + 2] = get_value(i * num_classes + 5 + k * 3 + 2);
            }
        }
        
        // 过滤低置信度
        if (score < conf_threshold) {
            continue;
        }

        // 转换坐标到原始图像空间
        PoseDetection det;
        det.score = score;
        
        det.x = (cx - pad_left) / scale;
        det.y = (cy - pad_top) / scale;
        det.w = w / scale;
        det.h = h / scale;

        // 转换关键点坐标
        for (int k = 0; k < NUM_KP; k++) {
            det.kp[k][0] = (kp_data[k * 3 + 0] - pad_left) / scale;
            det.kp[k][1] = (kp_data[k * 3 + 1] - pad_top) / scale;
            det.kp_score[k] = kp_data[k * 3 + 2];
        }

        candidates.push_back(det);
    }
    
    // NMS 非极大值抑制
    std::vector<PoseDetection> nms_result;
    if (!candidates.empty()) {
        nms(candidates, nms_result, 0.45f);
    }
    
    auto t3 = high_resolution_clock::now();
    if (timing) {
        timing->postprocess_ms = duration<float>(t3 - t2).count() * 1000.0f;
    }

    // 释放输出缓冲区
    rknn_outputs_release(ctx, 1, &output);

    return nms_result;
}

std::vector<PoseDetection> YoloPoseInference::detect(const cv::Mat& bgr_img, float conf_threshold) {
    // 完整的检测接口（包含预处理）
    std::vector<PoseDetection> result;
    if (!initialized_) {
        return result;
    }
    
    auto t0 = high_resolution_clock::now();

    float scale, pad_left, pad_top;
    fast_letterbox(bgr_img, input_buf_, input_width_, input_height_, scale, pad_left, pad_top);
    
    auto t1 = high_resolution_clock::now();
    last_timing_.preprocess_ms = duration<float>(t1 - t0).count() * 1000.0f;

    // 运行推理和后处理
    result = run_inference_postprocess(
        rknn_ctx_, input_buf_, input_buf_size_,
        input_width_, input_height_, output_dims_,
        output_type_, output_quantized_, output_scale_, output_zero_point_,
        scale, pad_left, pad_top, conf_threshold, &last_timing_);

    return result;
}

std::vector<PoseDetection> YoloPoseInference::detect_raw(
    uint8_t* preprocessed_buf,  // 预处理好的缓冲区
    float scale,                // 缩放比例
    float pad_left,             // 左侧填充
    float pad_top,              // 顶部填充
    float conf_threshold)       // 置信度阈值
{
    // 流水线模式使用的原始检测接口
    // 直接使用预处理好的缓冲区（零拷贝）
    std::vector<PoseDetection> result;
    if (!initialized_) {
        return result;
    }

    result = run_inference_postprocess(
        rknn_ctx_, preprocessed_buf, input_buf_size_,
        input_width_, input_height_, output_dims_,
        output_type_, output_quantized_, output_scale_, output_zero_point_,
        scale, pad_left, pad_top, conf_threshold, nullptr);

    return result;
}
```

---

## src/aim_engine.h (核心部分)

```cpp
#ifndef AIM_ENGINE_H
#define AIM_ENGINE_H

#include <opencv2/opencv.hpp>  // OpenCV
#include <vector>               // 向量
#include <deque>                // 双端队列
#include <random>               // 随机数
#include "yolo_inference.h"    // YOLO 检测结果

namespace aim {

// 目标部位枚举
enum BodyPart {
    HEAD = 0,    // 头部（鼻子）
    NECK = 1,    // 颈部
    CHEST = 2,   // 胸口
    BELLY = 3,   // 腹部
    BODY_PART_COUNT
};

// 吸附目标点
struct AimTarget {
    cv::Point2f pos;           // 屏幕坐标
    float confidence;          // 综合置信度
    float hit_probability;     // 预估命中率
    BodyPart body_part;        // 目标部位
    float distance_estimate;   // 估算距离（米）
};

// 目标选择器配置
struct TargetSelectorConfig {
    // 部位权重（不同部位优先级不同）
    float head_weight = 3.0f;    // 头部权重最高
    float neck_weight = 2.2f;    // 颈部
    float chest_weight = 1.5f;    // 胸口
    float belly_weight = 1.0f;    // 腹部
    
    // 距离衰减
    float distance_penalty = 0.8f;
    
    // 动态调整（根据距离自动选择瞄准部位）
    bool adaptive_body_part = true;
    
    // 距离阈值（米）
    float close_distance = 10.0f;
    float mid_distance = 30.0f;
};

// 目标选择器：根据配置选择最佳瞄准部位
class TargetSelector {
public:
    explicit TargetSelector(const TargetSelectorConfig& cfg = TargetSelectorConfig());
    
    // 选择目标点
    AimTarget select(const PoseDetection& det, float screen_height);
    
private:
    TargetSelectorConfig cfg_;
    
    // 不同部位的目标点计算
    AimTarget getHeadTarget(const PoseDetection& det);
    AimTarget getNeckTarget(const PoseDetection& det, float head_y);
    AimTarget getChestTarget(const PoseDetection& det);
    AimTarget getBellyTarget(const PoseDetection& det);
    
    // 根据边界框高度估算距离
    float estimateDistance(float bbox_h, float screen_height);
};

// 弹簧-阻尼系统配置
// 模拟人类瞄准的物理特性
struct SpringConfig {
    float stiffness = 8.0f;          // 刚度（越大响应越快）
    float damping = 2.5f;            // 阻尼（越大越稳定）
    float mass = 1.0f;               // 质量
    
    float max_speed = 120.0f;        // 最大速度（像素/秒）
    float max_accel = 800.0f;        // 最大加速度
    
    float micro_jitter = 0.8f;       // 微抖动幅度（模拟人手抖）
    float reaction_delay_ms = 80.0f; // 反应延迟（毫秒）
    float overshoot_ratio = 0.06f;   // 过冲比例
};

// 弹簧-阻尼物理引擎
// 使用弹簧-阻尼系统模拟人类瞄准动作
class SpringAim {
public:
    explicit SpringAim(const SpringConfig& cfg = SpringConfig());
    
    // 每帧调用，dt 为时间差（秒）
    // 返回需要移动的位移
    cv::Point2f update(cv::Point2f target, float dt);
    
    void reset(cv::Point2f start);  // 重置状态
    
    cv::Point2f getCurrentPos() const { return current_pos_; }
    cv::Point2f getVelocity() const { return velocity_; }
    
private:
    SpringConfig cfg_;
    cv::Point2f current_pos_;       // 当前位置
    cv::Point2f velocity_;          // 当前速度
    std::deque<cv::Point2f> target_history_; // 目标历史（用于延迟）
    std::mt19937 rng_;              // 随机数生成器
    std::uniform_real_distribution<float> jitter_dist_; // 抖动分布
    
    size_t reactionBufferSize() const;
    bool approachingTarget(cv::Point2f displacement, cv::Point2f velocity);
};

// 吸附强度曲线配置
struct StrengthCurveConfig {
    float close_strength = 1.0f;     // 近距离强度（1.0 = 100%吸附）
    float mid_strength = 0.6f;       // 中距离强度
    float far_strength = 0.25f;      // 远距离强度
    
    float close_distance = 10.0f;    // 近距离阈值
    float mid_distance = 30.0f;      // 中距离阈值
    float far_distance = 50.0f;      // 远距离阈值
};

// 吸附强度曲线
// 根据距离调整吸附强度
class StrengthCurve {
public:
    explicit StrengthCurve(const StrengthCurveConfig& cfg = StrengthCurveConfig());
    
    float getStrength(float distance) const;
    
private:
    StrengthCurveConfig cfg_;
};

// 完整吸附配置
struct AimConfig {
    TargetSelectorConfig selector;   // 目标选择配置
    SpringConfig spring;             // 弹簧配置
    StrengthCurveConfig strength;    // 强度曲线配置
    
    // 全局开关
    bool enabled = true;
    float trigger_threshold = 0.15f;  // 触发阈值
    int fire_delay_frames = 3;       // 开火延迟帧数
};

// 预设配置
enum class AimPreset {
    LEGIT,      // Legit 模式（最像人类，最保守）
    SEMI_RAGE,  // 半激进模式
    RAGE        // 激进模式（最快最准，但也最明显）
};

// 获取预设配置
AimConfig getPreset(AimPreset preset);

// 完整吸附引擎
class AimEngine {
public:
    explicit AimEngine(const AimConfig& cfg = getPreset(AimPreset::LEGIT));
    
    // 更新目标（每帧调用）
    void updateTarget(const std::vector<PoseDetection>& detections, 
                      float screen_width, float screen_height);
    
    // 获取当前吸附位移（相对移动）
    cv::Point2f getAimDelta(float dt);
    
    // 获取从屏幕中心到目标的绝对位移
    cv::Point2f getTargetOffsetFromCenter(float screen_width, float screen_height) const;
    
    // 是否允许开火
    bool shouldFire() const;
    
    // 获取当前目标信息
    bool hasTarget() const { return has_target_; }
    AimTarget getCurrentTarget() const { return current_target_; }
    
    // 配置
    void setConfig(const AimConfig& cfg) { cfg_ = cfg; }
    AimConfig getConfig() const { return cfg_; }
    
    // 重置
    void reset();
    
private:
    AimConfig cfg_;                   // 当前配置
    TargetSelector selector_;        // 目标选择器
    SpringAim spring_;                // 弹簧系统
    StrengthCurve strength_curve_;   // 强度曲线
    
    bool has_target_;                 // 是否有目标
    AimTarget current_target_;        // 当前目标
    int fire_delay_counter_;         // 开火延迟计数器
    int frames_locked_;               // 锁定帧数
    
    std::mt19937 rng_;               // 随机数生成器
    std::uniform_int_distribution<int> reaction_delay_dist_; // 随机延迟
    
    float screen_width_ = 0;
    float screen_height_ = 0;
};

} // namespace aim

#endif // AIM_ENGINE_H
```

---

## src/rga_preprocess.h

```c
/*
 * RGA Hardware Accelerated Preprocessing for RK3588
 * Replaces OpenCV CPU-based letterbox with RGA2/RGA3 hardware
 */
// RGA 硬件加速预处理
// 使用 RK3588 的 RGA2/RGA3 硬件进行图像缩放、颜色转换、填充

#ifndef RGA_PREPROCESS_H
#define RGA_PREPROCESS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 RGA 硬件
int rga_preprocess_init(void);

// 清理 RGA
void rga_preprocess_deinit(void);

// 检查 RGA 是否可用
bool rga_preprocess_available(void);

// RGA 加速的 letterbox: 缩放 + 颜色转换 + 填充
// src: 源图像缓冲区 (BGR, 连续)
// src_w, src_h: 源图像尺寸
// dst_buf: 预分配的目标缓冲区 (RGB, 连续)
// dst_w, dst_h: 目标尺寸 (如 640x640)
// scale, pad_left, pad_top: 输出参数，用于坐标映射
// 返回 0 表示成功
int rga_letterbox(const uint8_t* src, int src_w, int src_h,
                  uint8_t* dst_buf, int dst_w, int dst_h,
                  float* scale, float* pad_left, float* pad_top);

#ifdef __cplusplus
}
#endif

#endif // RGA_PREPROCESS_H
```

---

## src/rga_preprocess.c

```c
/*
 * RGA Hardware Accelerated Preprocessing Implementation
 * Uses Rockchip RGA2/RGA3 for resize + BGR->RGB + padding
 * Uses C API (im*_t functions) for C file compatibility
 */
// RGA 硬件加速预处理实现
// 使用 Rockchip RGA2/RGA3 进行 resize + BGR->RGB + padding

#include "rga_preprocess.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <rga/im2d_type.h>
#include <rga/im2d_single.h>
#include <rga/im2d_buffer.h>
#include <rga/rga.h>

static bool rga_initialized = false;
// RGA 初始化标志

int rga_preprocess_init(void) {
    // 初始化 RGA
    rga_initialized = true;
    printf("RGA hardware preprocessing initialized\n");
    return 0;
}

void rga_preprocess_deinit(void) {
    // 清理 RGA
    rga_initialized = false;
}

bool rga_preprocess_available(void) {
    // 检查 RGA 是否可用
    return rga_initialized;
}

int rga_letterbox(const uint8_t* src, int src_w, int src_h,
                  uint8_t* dst_buf, int dst_w, int dst_h,
                  float* scale, float* pad_left, float* pad_top) {
    // RGA 加速的 letterbox
    if (!rga_initialized) {
        return -1;
    }

    // 计算缩放比例（与 OpenCV 版本相同）
    float scale_x = (float)dst_w / src_w;
    float scale_y = (float)dst_h / src_h;
    float s = fminf(scale_x, scale_y);
    
    // 计算缩放后的尺寸
    int new_w = (int)(src_w * s);
    int new_h = (int)(src_h * s);
    
    // 计算居中填充偏移
    float x_offset = (dst_w - new_w) / 2.0f;
    float y_offset = (dst_h - new_h) / 2.0f;
    
    if (scale) *scale = s;
    if (pad_left) *pad_left = x_offset;
    if (pad_top) *pad_top = y_offset;

    // 用灰色 (114, 114, 114) 填充背景
    memset(dst_buf, 114, dst_w * dst_h * 3);

    // 使用 RGA C API 创建缓冲区
    // 源: BGR888 (OpenCV 默认)
    rga_buffer_t src_buf = wrapbuffer_virtualaddr_t((void*)src, src_w, src_h, src_w, src_h, RK_FORMAT_BGR_888);
    
    // 创建临时缓冲区用于 BGR 缩放
    uint8_t* temp_bgr_buf = (uint8_t*)malloc(new_w * new_h * 3);
    if (!temp_bgr_buf) {
        return -1;
    }
    
    // 步骤 1: RGA 缩放 BGR -> BGR
    rga_buffer_t temp_bgr_rga = wrapbuffer_virtualaddr_t(temp_bgr_buf, new_w, new_h, new_w, new_h, RK_FORMAT_BGR_888);
    IM_STATUS status = imresize_t(src_buf, temp_bgr_rga, 0, 0, IM_INTERP_LINEAR, 1);
    if (status != IM_STATUS_SUCCESS) {
        fprintf(stderr, "RGA resize failed: %d\n", status);
        free(temp_bgr_buf);
        return -1;
    }
    
    // 步骤 2: 手动颜色转换 BGR -> RGB 并拷贝到目标区域
    // BGR -> RGB: 交换 R 和 B 通道
    int x_off = (int)x_offset;
    int y_off = (int)y_offset;
    
    for (int y = 0; y < new_h; y++) {
        uint8_t* dst_row = dst_buf + ((y_off + y) * dst_w + x_off) * 3;
        uint8_t* src_row = temp_bgr_buf + y * new_w * 3;
        for (int x = 0; x < new_w; x++) {
            // BGR -> RGB: 交换 B 和 R
            dst_row[x * 3 + 0] = src_row[x * 3 + 2]; // R
            dst_row[x * 3 + 1] = src_row[x * 3 + 1]; // G
            dst_row[x * 3 + 2] = src_row[x * 3 + 0]; // B
        }
    }
    
    free(temp_bgr_buf);
    return 0;
}
```

---

## src/hid_controller.h (核心部分)

```cpp
#ifndef HID_CONTROLLER_H
#define HID_CONTROLLER_H

#include <opencv2/opencv.hpp>
#include "aim_engine.h"

namespace hid {

// HID 控制器配置
struct HIDConfig {
    float sensitivity = 1.0f;        // 灵敏度倍数
    float smoothing = 0.3f;          // 平滑系数 (0-1)
    int max_delta = 127;             // 最大单次移动像素
    int update_rate_hz = 60;         // 更新频率
    bool humanize = true;            // 是否启用人类化
};

// HID 鼠标控制器
class HIDController {
public:
    explicit HIDController(const HIDConfig& cfg = HIDConfig());
    ~HIDController();
    
    // 初始化 HID 设备
    bool init();
    void shutdown();
    bool isReady() const;
    
    // 发送鼠标移动（相对位移）
    bool move(int dx, int dy, bool left_button = false);
    
    // 根据吸附引擎输出移动鼠标
    bool aim(const cv::Point2f& delta, float strength = 1.0f);
    
    // 应用人类化曲线
    cv::Point2f humanizeDelta(cv::Point2f delta);
    
    // 限制 delta 范围
    cv::Point2f clampDelta(cv::Point2f delta);
    
    // 设置配置
    void setConfig(const HIDConfig& cfg) { cfg_ = cfg; }
    HIDConfig getConfig() const { return cfg_; }
    
private:
    HIDConfig cfg_;
    bool initialized_;
    cv::Point2f last_delta_;
};

// 集成吸附引擎和 HID 控制的完整系统
class AimSystem {
public:
    AimSystem();
    ~AimSystem();
    
    // 初始化
    bool init(aim::AimPreset preset = aim::AimPreset::LEGIT);
    void shutdown();
    bool isReady() const;
    
    // 更新目标（每帧调用）
    void update(const std::vector<PoseDetection>& detections, 
                float screen_width, float screen_height);
    
    // 执行吸附（返回实际移动的距离）
    cv::Point2f execute(float dt);
    
    // 直接移动到目标位置（绝对定位）
    bool moveToTarget(float screen_width, float screen_height);
    
    // 是否允许开火
    bool shouldFire() const;
    
    // 获取当前目标
    bool hasTarget() const;
    aim::AimTarget getTarget() const;
    
    // 配置
    void setPreset(aim::AimPreset preset);
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }
    
    // 设置灵敏度
    void setSensitivity(float sens) { hid_config_.sensitivity = sens; }
    float getSensitivity() const { return hid_config_.sensitivity; }
    
    // 设置吸附模式
    void setAbsoluteMode(bool absolute) { absolute_mode_ = absolute; }
    bool isAbsoluteMode() const { return absolute_mode_; }
    
private:
    aim::AimEngine* aim_engine_;    // 吸附引擎
    HIDController* hid_controller_; // HID 控制器
    aim::AimConfig aim_config_;     // 吸附配置
    HIDConfig hid_config_;          // HID 配置
    bool enabled_;                  // 吸附开关
    bool initialized_;              // 初始化标志
    bool absolute_mode_ = true;     // 默认绝对定位模式
};

} // namespace hid

#endif // HID_CONTROLLER_H
```

---

## 文件关系图

```
pose_demo.cpp (主程序)
    │
    ├── hdmi_capture.c/h (HDMI 采集)
    │       └── V4L2 驱动接口
    │
    ├── yolo_inference.cpp/h (YOLO 推理)
    │       ├── rga_preprocess.c/h (RGA 硬件加速)
    │       └── rknn_api.h (RKNN NPU 接口)
    │
    ├── hid_controller.cpp/h (瞄准系统)
    │       ├── aim_engine.cpp/h (吸附引擎)
    │       │       ├── TargetSelector (目标选择)
    │       │       ├── SpringAim (弹簧阻尼)
    │       │       └── StrengthCurve (强度曲线)
    │       └── usb_hid.c/h (USB HID 鼠标)
    │
    └── hid_relay.cpp/h (HID 转发)
            ├── usb_hid.c/h (USB HID 鼠标/键盘)
            └── input.h (Linux input 事件)
```

---

## 流水线时序图

```
时间 →

主线程:    [采集] → [显示] → [采集] → [显示] → ...
               ↓            ↑
预处理:        [预处理] → [预处理] → ...
               ↓
NPU:                  [推理] → [推理] → ...
               ↓
显示:                    [后处理] → [后处理] → ...

每个阶段独立并行，实现高吞吐量
```

---

## 配置文件格式 (config.ini)

```ini
# 配置文件示例

[video]
device = /dev/video40
model = yolov8n-pose.rknn

[keys]
key_legit = F1         # Legit 预设
key_semirage = F2      # Semi-rage 预设
key_rage = F3          # Rage 预设
key_autofire = F4      # 自动开枪开关
key_mode = F5          # 切换绝对/相对模式
key_sens_up = F6       # 增加灵敏度
key_sens_down = F7     # 降低灵敏度
key_game_mode = F9     # 游戏模式开关
```

文档完成！
```

> 文档内容较长，已生成至 `/home/ztl/github/RK/docs/Pose_line_by_line.md`
