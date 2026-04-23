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

#include "hdmi_capture.h"
#include "yolo_inference.h"
#include "hid_controller.h"
#include "hid_relay.h"
#include "usb_hid.h"
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <getopt.h>

using namespace std::chrono;

static double get_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

const int skeleton[14][2] = {
    {0, 1}, {0, 2}, {1, 3}, {2, 4},
    {5, 6}, {5, 11}, {6, 12}, {11, 12},
    {11, 13}, {12, 14}, {0, 5}, {0, 6}, {5, 7}, {6, 8}
};

// Pipeline frame structure
struct PipelineFrame {
    cv::Mat frame;  // Original frame (BGR from HDMI)
    cv::Mat preprocessed;  // Preprocessed frame (RGB 640x640)
    std::vector<PoseDetection> detections;
    float scale = 0;
    float pad_left = 0;
    float pad_top = 0;
    int frame_id = 0;
    
    // Timing
    double capture_time = 0;
    double preprocess_time = 0;
    double npu_time = 0;
    double postprocess_time = 0;
};

// Thread-safe queue with capacity limit and frame dropping
template<typename T>
class ThreadQueue {
public:
    explicit ThreadQueue(size_t max_size = 3) : max_size_(max_size), dropped_frames_(0) {}
    
    // Push with drop policy: if queue is full, drop oldest frame
    bool push(T item, bool drop_if_full = true) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_) {
            if (drop_if_full) {
                // Drop oldest frame to prevent memory explosion
                queue_.pop();
                dropped_frames_++;
            } else {
                return false; // Queue full
            }
        }
        queue_.push(std::move(item));
        cond_.notify_one();
        return true;
    }
    
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
    
    size_t size() {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    size_t dropped() {
        std::unique_lock<std::mutex> lock(mutex_);
        return dropped_frames_;
    }
    
    void reset_dropped() {
        std::unique_lock<std::mutex> lock(mutex_);
        dropped_frames_ = 0;
    }
    
    void stop() {
        std::unique_lock<std::mutex> lock(mutex_);
        stop_ = true;
        cond_.notify_all();
    }
    
private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    bool stop_ = false;
    size_t max_size_;
    size_t dropped_frames_;
};

// Global queues with limited capacity to prevent memory explosion
// Capacity = 2 means at most 2 frames buffered, old frames dropped when full
ThreadQueue<PipelineFrame> preprocess_queue(2);
ThreadQueue<PipelineFrame> npu_queue(2);
ThreadQueue<PipelineFrame> display_queue(2);
std::atomic<bool> running{true};
std::atomic<int> frame_counter{0};
std::atomic<size_t> total_dropped_frames{0};

// Aim system (integrates aim engine + HID controller)
hid::AimSystem* g_aim_system = nullptr;
hid::HIDRelay* g_hid_relay = nullptr;
bool g_aim_enabled = true;   // 默认开启吸附
bool g_auto_fire = false;    // F4 控制自动开枪模式
aim::AimPreset g_aim_preset = aim::AimPreset::LEGIT;
std::vector<PoseDetection> g_latest_detections; // 保存最新的检测结果

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

 static std::string trim(const std::string& s) {
     size_t start = s.find_first_not_of(" \t\r\n");
     if (start == std::string::npos) return "";
     size_t end = s.find_last_not_of(" \t\r\n");
     return s.substr(start, end - start + 1);
 }

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

int g_key_legit = 59;        // F1
int g_key_semirage = 60;     // F2
int g_key_rage = 61;         // F3
int g_key_autofire = 62;     // F4
int g_key_mode = 63;         // F5
int g_key_sens_up = 64;      // F6
int g_key_sens_down = 65;     // F7
int g_key_game_mode = 67;    // F9

// Preprocess thread: BGR -> RGB letterbox using fast_letterbox (with RGA support)
void preprocess_thread(int input_w, int input_h) {
    // Pre-allocate buffer for preprocessing
    cv::Mat preprocessed(input_h, input_w, CV_8UC3);
    
    while (running) {
        PipelineFrame frame;
        if (!preprocess_queue.pop(frame, 50)) continue;
        
        auto t0 = high_resolution_clock::now();
        
        float scale, pad_left, pad_top;
        
        // Use fast_letterbox from yolo_inference.cpp (supports RGA hardware acceleration)
        // This ensures consistent preprocessing between pipeline and direct inference
        fast_letterbox(frame.frame, preprocessed.data, input_w, input_h, scale, pad_left, pad_top);
        
        // DEBUG: Save preprocessed image to check if preprocessing is correct (disabled)
        // static int debug_count = 0;
        // if (debug_count < 5) {
        //     cv::Mat debug_img(input_h, input_w, CV_8UC3, preprocessed.data);
        //     char debug_path[256];
        //     snprintf(debug_path, sizeof(debug_path), "/tmp/preprocess_debug_%d.jpg", debug_count++);
        //     cv::imwrite(debug_path, debug_img);
        //     printf("[DEBUG] Saved preprocessed image to %s\n", debug_path);
        // }
        
        frame.preprocessed = preprocessed.clone();  // Clone to own the data
        frame.scale = scale;
        frame.pad_left = pad_left;
        frame.pad_top = pad_top;
        
        auto t1 = high_resolution_clock::now();
        frame.preprocess_time = duration<double>(t1 - t0).count() * 1000.0;
        
        // Push to NPU queue, drop if full (old frames discarded)
        if (!npu_queue.push(std::move(frame), true)) {
            // Frame dropped
        }
    }
}

// NPU inference thread
void npu_thread(YoloPoseInference* inference) {
    while (running) {
        PipelineFrame frame;
        if (!npu_queue.pop(frame, 50)) continue;
        
        auto t0 = high_resolution_clock::now();
        
        // Run NPU inference directly with preprocessed buffer
        // Use lower confidence threshold (0.25) to detect more distant/small targets
        frame.detections = inference->detect_raw(
            frame.preprocessed.data,
            frame.scale,
            frame.pad_left,
            frame.pad_top,
            0.25f
        );
        
        auto t1 = high_resolution_clock::now();
        frame.npu_time = duration<double>(t1 - t0).count() * 1000.0;
        
        // Push to display queue, drop if full
        if (!display_queue.push(std::move(frame), true)) {
            // Frame dropped
        }
    }
}

int main(int argc, char** argv) {
    const char* device = "/dev/video40";
    int width = 1920;
    int height = 1080;

    std::string exe_dir = "";
    if (argc > 0 && argv[0]) {
        std::string exe_path = argv[0];
        size_t pos = exe_path.rfind('/');
        if (pos != std::string::npos) {
            exe_dir = exe_path.substr(0, pos + 1);
        }
    }

    std::vector<std::string> model_paths = {
        "/home/ztl/github/RK/src/yolov8n-pose.rknn",
        exe_dir + "yolov8n-pose.rknn",
        "yolov8n-pose.rknn",
    };
    const char* model_path = model_paths[0].c_str();

    // Parse --config option first
    const char* config_file = nullptr;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_file = argv[++i];
            break;
        }
    }

    // If config file specified, load it
    if (config_file) {
        FILE* fp = fopen(config_file, "r");
        if (fp) {
            fclose(fp);
            char line[256];
            fp = fopen(config_file, "r");
            while (fgets(line, sizeof(line), fp)) {
                if (strstr(line, "model = ")) {
                    char* eq = strstr(line, "=");
                    if (eq) {
                        char* path = eq + 1;
                        while (*path == ' ') path++;
                        path[strlen(path)-1] = '\0';
                        model_paths.insert(model_paths.begin(), path);
                    }
                }
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

    // Parse positional arguments
    std::vector<char*> positional_args;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0) {
            i++; // skip value
        } else if (argv[i][0] != '-') {
            positional_args.push_back(argv[i]);
        }
    }
    
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
    
    // Parse optional keyboard/mouse device arguments (mode B)
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--keyboard") == 0 && i + 1 < argc) {
            g_keyboard_device = argv[++i];
        } else if (strcmp(argv[i], "--mouse") == 0 && i + 1 < argc) {
            g_mouse_device = argv[++i];
        }
    }

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

    hdmi_capture_t cap;
    if (hdmi_open(&cap, device, width, height) != 0) {
        fprintf(stderr, "Failed to open %s\n", device);
        return 1;
    }

    if (hdmi_start(&cap) != 0) {
        fprintf(stderr, "Failed to start streaming\n");
        hdmi_close(&cap);
        return 1;
    }

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

    int input_w = inference.get_input_width();
    int input_h = inference.get_input_height();

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
    
    // Initialize aim system (engine + HID controller)
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
    
    // Initialize HID relay (mode B: full keyboard/mouse forwarding)
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

    cv::namedWindow("YOLOv8n-pose Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("YOLOv8n-pose Detection", 960, 540);

    // Start worker threads
    std::thread preprocess_t(preprocess_thread, input_w, input_h);
    std::thread npu_t(npu_thread, &inference);

    int frame_count = 0;
    double start_time = get_time_ms();
    
    float total_capture = 0;
    float total_preprocess = 0;
    float total_npu = 0;
    float total_draw = 0;
    float total_all = 0;

    while (true) {
        auto t0 = high_resolution_clock::now();
        
        uint8_t* data;
        int w, h;

        if (hdmi_get_frame(&cap, &data, &w, &h) != 0) {
            usleep(500);
            continue;
        }
        
        auto t1 = high_resolution_clock::now();
        float capture_ms = duration<float>(t1 - t0).count() * 1000.0f;

        // Create frame and push to preprocess queue (drop if full to prevent lag)
        PipelineFrame frame;
        frame.frame = cv::Mat(h, w, CV_8UC3, data).clone();  // Clone to avoid buffer issues
        frame.frame_id = frame_counter++;
        frame.capture_time = capture_ms;
        
        if (!preprocess_queue.push(std::move(frame), true)) {
            // Frame dropped due to full queue
        }
        
        // Try to get processed frames from display queue (non-blocking)
        PipelineFrame display_frame;
        while (display_queue.pop(display_frame, 0)) {
            // Save latest detections for aim system
            g_latest_detections = display_frame.detections;
            
            auto t2 = high_resolution_clock::now();
            
            // Draw detections
            cv::Mat& display_img = display_frame.frame;
            
            for (const auto& det : display_frame.detections) {
                int x1 = static_cast<int>(det.x - det.w / 2.0f);
                int y1 = static_cast<int>(det.y - det.h / 2.0f);
                int x2 = static_cast<int>(det.x + det.w / 2.0f);
                int y2 = static_cast<int>(det.y + det.h / 2.0f);
                
                cv::rectangle(display_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

                for (int k = 0; k < NUM_KP; k++) {
                    if (det.kp_score[k] > 0.5f) {
                        cv::circle(display_img, cv::Point(static_cast<int>(det.kp[k][0]), static_cast<int>(det.kp[k][1])), 3, cv::Scalar(0, 0, 255), -1);
                    }
                }

                for (int i = 0; i < 14; i++) {
                    int kp1 = skeleton[i][0];
                    int kp2 = skeleton[i][1];
                    if (det.kp_score[kp1] > 0.5f && det.kp_score[kp2] > 0.5f) {
                        cv::line(display_img,
                                cv::Point(static_cast<int>(det.kp[kp1][0]), static_cast<int>(det.kp[kp1][1])),
                                cv::Point(static_cast<int>(det.kp[kp2][0]), static_cast<int>(det.kp[kp2][1])),
                                cv::Scalar(255, 0, 0), 2);
                    }
                }
            }

            // Draw aim target (always show when has target, regardless of aim_enabled)
            static int draw_count = 0;
            if (g_aim_system && g_aim_system->hasTarget()) {
                auto target = g_aim_system->getTarget();
                cv::Point2f target_pos = target.pos;

                // Debug: print target position occasionally
                if (++draw_count % 60 == 0) {
                    printf("[DRAW] Target at (%.0f, %.0f), img size: %dx%d\n",
                           target_pos.x, target_pos.y, display_img.cols, display_img.rows);
                }

                // Ensure target is within image bounds
                if (target_pos.x >= 0 && target_pos.x < display_img.cols &&
                    target_pos.y >= 0 && target_pos.y < display_img.rows) {
                    // Draw RED crosshair (more visible)
                    cv::Scalar crosshair_color(0, 0, 255);  // Red in BGR
                    cv::circle(display_img, target_pos, 20, crosshair_color, 5);
                    cv::line(display_img, cv::Point(target_pos.x - 25, target_pos.y),
                            cv::Point(target_pos.x + 25, target_pos.y), crosshair_color, 5);
                    cv::line(display_img, cv::Point(target_pos.x, target_pos.y - 25),
                            cv::Point(target_pos.x, target_pos.y + 25), crosshair_color, 5);

                    // Draw target info with red color
                    char aim_text[128];
                    const char* part_name = (target.body_part == aim::HEAD) ? "HEAD" :
                                           (target.body_part == aim::NECK) ? "NECK" :
                                           (target.body_part == aim::CHEST) ? "CHEST" : "BELLY";
                    snprintf(aim_text, sizeof(aim_text), "AIM: %s (%.0f,%.0f) %.0f%%",
                            part_name, target_pos.x, target_pos.y, target.hit_probability * 100);
                    cv::putText(display_img, aim_text, cv::Point(10, 60),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, crosshair_color, 3, cv::LINE_AA);
                }
            }

            char info_text[256];
            float total_ms_so_far = duration<float>(high_resolution_clock::now() - t0).count() * 1000.0f;
            snprintf(info_text, sizeof(info_text), 
                    "Cap:%.0f Pre:%.0f NPU:%.0f | %.0fms %.0fFPS %s",
                    display_frame.capture_time,
                    display_frame.preprocess_time,
                    display_frame.npu_time,
                    total_ms_so_far, 1000.0f / total_ms_so_far,
                    g_aim_enabled ? "[AIM ON]" : "");
            cv::putText(display_img, info_text, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            
            auto t3 = high_resolution_clock::now();
            float draw_ms = duration<float>(t3 - t2).count() * 1000.0f;

            cv::imshow("YOLOv8n-pose Detection", display_img);
            
            // Timing stats
            total_capture += display_frame.capture_time;
            total_preprocess += display_frame.preprocess_time;
            total_npu += display_frame.npu_time;
            total_draw += draw_ms;
            
            auto t4 = high_resolution_clock::now();
            float total_ms = duration<float>(t4 - t0).count() * 1000.0f;
            total_all += total_ms;
            frame_count++;
        }
        
        hdmi_release_frame(&cap);

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
        
        // Update aim system with current detections（仅用于检测）
        if (g_aim_system && !g_latest_detections.empty()) {
            g_aim_system->update(g_latest_detections, width, height);
        }
        
        // Print stats every 10 seconds (disabled for clean output)
        double elapsed = get_time_ms() - start_time;
        if (elapsed >= 10000.0 && frame_count > 0) {
            // printf("FPS: %.1f\n", 1000.0f * frame_count / elapsed);
            
            frame_count = 0;
            total_capture = 0;
            total_preprocess = 0;
            total_npu = 0;
            total_draw = 0;
            total_all = 0;
            start_time = get_time_ms();
        }
    }

    running = false;
    preprocess_queue.stop();
    npu_queue.stop();
    display_queue.stop();
    
    preprocess_t.join();
    npu_t.join();

    hdmi_stop(&cap);
    hdmi_close(&cap);
    cv::destroyAllWindows();
    
    // Cleanup aim system
    if (g_hid_relay) {
        g_hid_relay->stop();
        delete g_hid_relay;
        g_hid_relay = nullptr;
    }
    if (g_aim_system) {
        delete g_aim_system;
        g_aim_system = nullptr;
    }

    printf("Done\n");
    return 0;
}
