#ifndef YOLO_POSE_INFERENCE_H
#define YOLO_POSE_INFERENCE_H

#include "rknn_api.h"
#include <stdint.h>
#include <vector>
#include <opencv2/opencv.hpp>

const int NUM_KP = 17;

// Exposed preprocessing function for pipeline mode
void fast_letterbox(const cv::Mat& image, uint8_t* output_buf, 
                    int target_w, int target_h,
                    float& scale, float& pad_left, float& pad_top);

struct PoseDetection {
    float x, y, w, h;
    float score;
    float kp[NUM_KP][2];
    float kp_score[NUM_KP];
};

class YoloPoseInference {
public:
    YoloPoseInference();
    ~YoloPoseInference();

    int init(const char* model_path, int input_width = 0, int input_height = 0);
    void release();
    std::vector<PoseDetection> detect(const cv::Mat& bgr_img, float conf_threshold = 0.5f);
    // Raw detect with preprocessed buffer (for pipeline mode)
    std::vector<PoseDetection> detect_raw(uint8_t* preprocessed_buf, float scale, float pad_left, float pad_top, float conf_threshold = 0.5f);
    bool is_initialized() const { return initialized_; }
    
    struct TimingStats {
        float preprocess_ms;
        float inference_ms;
        float postprocess_ms;
    };
    TimingStats get_last_timing() const { return last_timing_; }
    
    int get_input_width() const { return input_width_; }
    int get_input_height() const { return input_height_; }

private:
    rknn_context rknn_ctx_;
    bool initialized_;
    int input_width_;
    int input_height_;
    int channel_;
    bool output_quantized_;
    float output_scale_;
    int32_t output_zero_point_;
    int output_type_;
    int output_fmt_;  // NCHW or NHWC
    int output_dims_[4];  // Store output dimensions
    
    uint8_t* input_buf_;
    size_t input_buf_size_;
    
    TimingStats last_timing_;
};

#endif
