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
        
        // DEBUG: Save preprocessed image to check if preprocessing is correct
        static int debug_count = 0;
        if (debug_count < 5) {
            cv::Mat debug_img(input_h, input_w, CV_8UC3, preprocessed.data);
            char debug_path[256];
            snprintf(debug_path, sizeof(debug_path), "/tmp/preprocess_debug_%d.jpg", debug_count++);
            cv::imwrite(debug_path, debug_img);
            printf("[DEBUG] Saved preprocessed image to %s\n", debug_path);
            
            // Print first 10 pixels of preprocessed image to verify data
            printf("[DEBUG] First 10 pixels (RGB): ");
            for (int i = 0; i < 10 && i < input_w * input_h; i++) {
                int y = i / input_w;
                int x = i % input_w;
                uint8_t* pixel = preprocessed.data + (y * input_w + x) * 3;
                printf("(%d,%d,%d) ", pixel[0], pixel[1], pixel[2]);
            }
            printf("\n");
            
            // Print pixel at center (should be part of the image, not gray border)
            int center_y = input_h / 2;
            int center_x = input_w / 2;
            uint8_t* center_pixel = preprocessed.data + (center_y * input_w + center_x) * 3;
            printf("[DEBUG] Center pixel at (%d,%d): RGB=(%d,%d,%d)\n", 
                   center_x, center_y, center_pixel[0], center_pixel[1], center_pixel[2]);
        }
        
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
    const char* model_path = "yolov8n-pose.rknn";

    if (argc >= 2) {
        device = argv[1];
    }
    if (argc == 3) {
        model_path = argv[2];
    } else if (argc >= 4) {
        width = atoi(argv[2]);
        height = atoi(argv[3]);
        model_path = argv[4];
    }

    printf("\n");
    printf("=======================================\n");
    printf("RK3588 YOLOv8n-pose Pipeline Demo\n");
    printf("=======================================\n");
    printf("Device: %s\n", device);
    printf("Resolution: %dx%d\n", width, height);
    printf("Model: %s\n", model_path);
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
    if (inference.init(model_path) != 0) {
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
    printf("Press 'q' or ESC to quit\n");
    printf("\n");

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

            char info_text[256];
            float total_ms_so_far = duration<float>(high_resolution_clock::now() - t0).count() * 1000.0f;
            snprintf(info_text, sizeof(info_text), 
                    "Cap:%.0f Pre:%.0f NPU:%.0f | %.0fms %.0fFPS",
                    display_frame.capture_time,
                    display_frame.preprocess_time,
                    display_frame.npu_time,
                    total_ms_so_far, 1000.0f / total_ms_so_far);
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
        
        // Print stats every 2 seconds
        double elapsed = get_time_ms() - start_time;
        if (elapsed >= 2000.0 && frame_count > 0) {
            printf("=== Pipeline Avg (last %d frames) ===\n", frame_count);
            printf("Capture:  %.1fms\n", total_capture / frame_count);
            printf("Preproc:  %.1fms (parallel)\n", total_preprocess / frame_count);
            printf("NPU:      %.1fms (parallel)\n", total_npu / frame_count);
            printf("Draw:     %.1fms\n", total_draw / frame_count);
            printf("Total:    %.1fms (%.1f FPS)\n", 
                   total_all / frame_count, 
                   1000.0f * frame_count / elapsed);
            printf("Queue: pre=%zu npu=%zu disp=%zu dropped=%zu\n",
                   preprocess_queue.size(), npu_queue.size(), display_queue.size(),
                   preprocess_queue.dropped() + npu_queue.dropped() + display_queue.dropped());
            printf("=====================================\n\n");
            
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

    printf("Done\n");
    return 0;
}
