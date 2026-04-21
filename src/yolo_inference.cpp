/*
 * YOLOv8n-pose Inference on RK3588 NPU via RKNN
 * Optimized for performance - with detailed timing
 */

#include "yolo_inference.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <vector>

#include "rknn_api.h"
#include "opencv2/opencv.hpp"

#ifdef USE_RGA_PREPROCESS
#include "rga_preprocess.h"
#endif

using namespace std::chrono;

static void nms(std::vector<PoseDetection>& input, std::vector<PoseDetection>& output, float iou_threshold) {
    sort(input.begin(), input.end(), [](const PoseDetection& a, const PoseDetection& b) {
        return a.score > b.score;
    });

    while (!input.empty()) {
        output.push_back(input[0]);
        if (input.size() == 1) {
            break;
        }

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

            float xx1 = std::max(x1a, x1b);
            float yy1 = std::max(y1a, y1b);
            float xx2 = std::min(x2a, x2b);
            float yy2 = std::min(y2a, y2b);

            float w = std::max(0.0f, xx2 - xx1);
            float h = std::max(0.0f, yy2 - yy1);
            float intersection = w * h;
            float union_area = areaa + areab - intersection;
            float iou = intersection / union_area;

            if (iou <= iou_threshold) {
                remaining.push_back(input[i]);
            }
        }
        input = remaining;
    }
}

YoloPoseInference::YoloPoseInference() {
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

int YoloPoseInference::init(const char* model_path, int input_width, int input_height) {
    FILE* fp = fopen(model_path, "rb");
    if (!fp) {
        fprintf(stderr, "Failed to open model: %s\n", model_path);
        return -1;
    }

    fseek(fp, 0, SEEK_END);
    size_t model_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    void* model_buffer = malloc(model_size);
    if (!model_buffer) {
        fprintf(stderr, "Failed to allocate memory\n");
        fclose(fp);
        return -1;
    }

    (void)fread(model_buffer, 1, model_size, fp);
    fclose(fp);

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
    // Initialize RGA hardware for preprocessing
    rga_preprocess_init();
#endif

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != 0) {
        fprintf(stderr, "rknn_query io num failed: %d\n", ret);
        release();
        return -1;
    }

    printf("YOLOv8n-pose model loaded: %d inputs, %d outputs\n", io_num.n_input, io_num.n_output);

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
            
            if (output_attr.qnt_type != RKNN_TENSOR_QNT_NONE) {
                output_quantized_ = true;
                output_scale_ = output_attr.scale;
                output_zero_point_ = output_attr.zp;
                printf("  Quantized: scale=%.6f, zp=%d\n", output_scale_, output_zero_point_);
            } else {
                output_quantized_ = false;
                printf("  Not quantized\n");
            }
            
            // Determine output layout
            // YOLOv8-pose output: could be [1, 56, 8400] (NCHW) or [1, 8400, 56] (NHWC)
            if (output_attr.n_dims >= 3) {
                if (output_attr.dims[1] == 56 && output_attr.dims[2] == 8400) {
                    printf("  Output layout: NCHW [1, 56, 8400]\n");
                } else if (output_attr.dims[1] == 8400 && output_attr.dims[2] == 56) {
                    printf("  Output layout: NHWC [1, 8400, 56]\n");
                } else {
                    printf("  Output layout: unknown dims[%d, %d, %d, %d]\n",
                           output_attr.dims[0], output_attr.dims[1],
                           output_attr.dims[2], output_attr.dims[3]);
                }
            }
        }
    }
    
    input_buf_size_ = input_width_ * input_height_ * channel_;
    input_buf_ = static_cast<uint8_t*>(malloc(input_buf_size_));
    printf("Pre-allocated input buffer: %zu bytes\n", input_buf_size_);

    return 0;
}

void YoloPoseInference::release() {
    if (input_buf_) {
        free(input_buf_);
        input_buf_ = nullptr;
    }
    if (initialized_) {
        rknn_destroy(rknn_ctx_);
    }
    initialized_ = false;
}

static void fast_letterbox_opencv(const cv::Mat& image, uint8_t* output_buf, 
                        int target_w, int target_h,
                        float& scale, float& pad_left, float& pad_top) {
    int img_w = image.cols;
    int img_h = image.rows;
    
    float scale_x = static_cast<float>(target_w) / img_w;
    float scale_y = static_cast<float>(target_h) / img_h;
    scale = std::min(scale_x, scale_y);
    
    int new_w = static_cast<int>(img_w * scale);
    int new_h = static_cast<int>(img_h * scale);
    
    pad_left = (target_w - new_w) / 2.0f;
    pad_top = (target_h - new_h) / 2.0f;
    
    // 直接操作预分配的 buffer，减少中间步骤
    cv::Mat padded(target_h, target_w, CV_8UC3, cv::Scalar(114, 114, 114));
    
    int x_offset = static_cast<int>(pad_left);
    int y_offset = static_cast<int>(pad_top);
    
    // 缩放到中间位置
    cv::Mat roi = padded(cv::Rect(x_offset, y_offset, new_w, new_h));
    
    // 缩放 + 颜色转换一步到位
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    
    // HDMI reports BGR3 but actual data is RGB
    // So we don't need color conversion, just copy directly
    resized.copyTo(roi);
    
    // 直接拷贝
    memcpy(output_buf, padded.data, target_w * target_h * 3);
}

// Exposed for pipeline mode (pose_demo.cpp)
void fast_letterbox(const cv::Mat& image, uint8_t* output_buf,
                        int target_w, int target_h,
                        float& scale, float& pad_left, float& pad_top) {
    // TEMPORARILY DISABLE RGA to test with OpenCV preprocessing
    // This helps verify if the model is working correctly
#ifndef FORCE_OPENCV_PREPROCESS
#ifdef USE_RGA_PREPROCESS
    if (rga_preprocess_available()) {
        int ret = rga_letterbox(image.data, image.cols, image.rows,
                                output_buf, target_w, target_h,
                                &scale, &pad_left, &pad_top);
        if (ret == 0) {
            return;
        }
        // RGA failed, fallback to OpenCV
    }
#endif
#endif
    fast_letterbox_opencv(image, output_buf, target_w, target_h, scale, pad_left, pad_top);
}

// Internal: run NPU inference and postprocess
static std::vector<PoseDetection> run_inference_postprocess(
    rknn_context ctx,
    uint8_t* input_buf,
    size_t input_buf_size,
    int input_width,
    int input_height,
    int* output_dims,
    int output_type,
    bool output_quantized,
    float output_scale,
    int32_t output_zero_point,
    float scale,
    float pad_left,
    float pad_top,
    float conf_threshold,
    YoloPoseInference::TimingStats* timing)
{
    std::vector<PoseDetection> result;
    
    auto t1 = high_resolution_clock::now();

    rknn_input input;
    memset(&input, 0, sizeof(input));
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.size = input_buf_size;
    input.fmt = RKNN_TENSOR_NHWC;
    input.buf = input_buf;

    int ret = rknn_inputs_set(ctx, 1, &input);
    if (ret != 0) {
        fprintf(stderr, "rknn_inputs_set failed: %d\n", ret);
        return result;
    }

    ret = rknn_run(ctx, nullptr);
    if (ret != 0) {
        fprintf(stderr, "rknn_run failed: %d\n", ret);
        return result;
    }

    rknn_output output;
    memset(&output, 0, sizeof(output));
    output.index = 0;
    output.want_float = 1;  // RKNN automatically dequantizes to float
    ret = rknn_outputs_get(ctx, 1, &output, nullptr);
    if (ret != 0) {
        fprintf(stderr, "rknn_outputs_get failed: %d\n", ret);
        return result;
    }
    
    auto t2 = high_resolution_clock::now();
    if (timing) {
        timing->inference_ms = duration<float>(t2 - t1).count() * 1000.0f;
    }

    // RKNN with want_float=1 returns float buffer
    float* float_buf = static_cast<float*>(output.buf);
    
    // Helper lambda to get value at index
    auto get_value = [&](int idx) -> float {
        return float_buf[idx];
    };
    
    int num_detections = 8400;
    int num_classes = 56;
    std::vector<PoseDetection> candidates;
    candidates.reserve(20);
    
    bool is_nchw = (output_dims[1] == 56 && output_dims[2] == 8400);
    
    // Debug: print first few raw output values
    printf("[DEBUG] Output type=%d, is_nchw=%d, output.size=%u\n", output_type, is_nchw, output.size);
    printf("[DEBUG] Channel 0-7 values at index 0: ");
    for (int c = 0; c < 8; c++) {
        printf("ch%d=%.3f ", c, get_value(c * num_detections + 0));
    }
    printf("\n");
    printf("[DEBUG] First 10 values (channel 0, cx): ");
    for (int i = 0; i < 10 && i < num_detections; i++) {
        printf("%.3f ", get_value(i));
    }
    printf("\n");
    printf("[DEBUG] First 10 values (channel 4, score): ");
    for (int i = 0; i < 10 && i < num_detections; i++) {
        printf("%.3f ", get_value(4 * num_detections + i));
    }
    printf("\n");
    printf("[DEBUG] First 10 values (channel 5, keypoint x): ");
    for (int i = 0; i < 10 && i < num_detections; i++) {
        printf("%.3f ", get_value(5 * num_detections + i));
    }
    printf("\n");
    
    // Find max score across ALL detections
    float max_score = 0;
    int max_idx = -1;
    for (int i = 0; i < num_detections; i++) {
        float score = get_value(4 * num_detections + i);
        if (score > max_score) {
            max_score = score;
            max_idx = i;
        }
    }
    printf("[DEBUG] Max score: %.3f at index %d\n", max_score, max_idx);
    
    // Print some statistics about the scores
    int count_0 = 0, count_1 = 0, count_10 = 0, count_50 = 0, count_100 = 0;
    for (int i = 0; i < num_detections; i++) {
        float score = get_value(4 * num_detections + i);
        if (score < 0.001f) count_0++;
        else if (score < 1.0f) count_1++;
        else if (score < 10.0f) count_10++;
        else if (score < 50.0f) count_50++;
        else count_100++;
    }
    printf("[DEBUG] Score stats: <0.001=%d, <1=%d, <10=%d, <50=%d, >=50=%d\n", count_0, count_1, count_10, count_50, count_100);
    
    for (int i = 0; i < num_detections; i++) {
        float cx, cy, w, h, score;
        float kp_data[51];
        
        if (is_nchw) {
            // NCHW format: [batch, channels, num_detections]
            // Each channel contains all detections for that attribute
            cx = get_value(i);
            cy = get_value(num_detections + i);
            w  = get_value(2 * num_detections + i);
            h  = get_value(3 * num_detections + i);
            score = get_value(4 * num_detections + i);
            
            for (int k = 0; k < NUM_KP; k++) {
                kp_data[k * 3 + 0] = get_value((5 + k * 3 + 0) * num_detections + i);
                kp_data[k * 3 + 1] = get_value((5 + k * 3 + 1) * num_detections + i);
                kp_data[k * 3 + 2] = get_value((5 + k * 3 + 2) * num_detections + i);
            }
        } else {
            // NHWC format: [batch, num_detections, channels]
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
        
        if (score < conf_threshold) {
            continue;
        }

        PoseDetection det;
        det.score = score;
        
        det.x = (cx - pad_left) / scale;
        det.y = (cy - pad_top) / scale;
        det.w = w / scale;
        det.h = h / scale;

        for (int k = 0; k < NUM_KP; k++) {
            det.kp[k][0] = (kp_data[k * 3 + 0] - pad_left) / scale;
            det.kp[k][1] = (kp_data[k * 3 + 1] - pad_top) / scale;
            det.kp_score[k] = kp_data[k * 3 + 2];
        }

        candidates.push_back(det);
    }
    
    // Debug: print first detection coordinates
    if (!candidates.empty()) {
        const auto& det = candidates[0];
        printf("[DEBUG] Detection: bbox=(%.1f,%.1f,%.1f,%.1f) score=%.3f\n", 
               det.x, det.y, det.w, det.h, det.score);
        printf("[DEBUG] Keypoints: nose=(%.1f,%.1f,%.3f) left_eye=(%.1f,%.1f,%.3f) right_eye=(%.1f,%.1f,%.3f)\n",
               det.kp[0][0], det.kp[0][1], det.kp_score[0],
               det.kp[1][0], det.kp[1][1], det.kp_score[1],
               det.kp[2][0], det.kp[2][1], det.kp_score[2]);
        printf("[DEBUG] Keypoints: left_shoulder=(%.1f,%.1f,%.3f) right_shoulder=(%.1f,%.1f,%.3f)\n",
               det.kp[5][0], det.kp[5][1], det.kp_score[5],
               det.kp[6][0], det.kp[6][1], det.kp_score[6]);
    }

    // Count score distribution
    int count_09 = 0, count_05 = 0, count_01 = 0, count_001 = 0;
    for (const auto& c : candidates) {
        if (c.score > 0.9f) count_09++;
        if (c.score > 0.5f) count_05++;
        if (c.score > 0.1f) count_01++;
        if (c.score > 0.01f) count_001++;
    }
    printf("[DEBUG] Score distribution: >0.9=%d, >0.5=%d, >0.1=%d, >0.01=%d\n", count_09, count_05, count_01, count_001);
    
    std::vector<PoseDetection> nms_result;
    if (!candidates.empty()) {
        // Lower NMS threshold to detect more overlapping objects (e.g., multiple people close together)
        nms(candidates, nms_result, 0.45f);
    }
    
    printf("[DEBUG] Candidates: %zu, After NMS: %zu\n", candidates.size(), nms_result.size());
    
    auto t3 = high_resolution_clock::now();
    if (timing) {
        timing->postprocess_ms = duration<float>(t3 - t2).count() * 1000.0f;
    }

    rknn_outputs_release(ctx, 1, &output);

    return nms_result;
}

std::vector<PoseDetection> YoloPoseInference::detect(const cv::Mat& bgr_img, float conf_threshold) {
    std::vector<PoseDetection> result;
    if (!initialized_) {
        return result;
    }
    
    auto t0 = high_resolution_clock::now();

    float scale, pad_left, pad_top;
    fast_letterbox(bgr_img, input_buf_, input_width_, input_height_, scale, pad_left, pad_top);
    
    auto t1 = high_resolution_clock::now();
    last_timing_.preprocess_ms = duration<float>(t1 - t0).count() * 1000.0f;

    result = run_inference_postprocess(
        rknn_ctx_, input_buf_, input_buf_size_,
        input_width_, input_height_, output_dims_,
        output_type_, output_quantized_, output_scale_, output_zero_point_,
        scale, pad_left, pad_top, conf_threshold, &last_timing_);

    return result;
}

std::vector<PoseDetection> YoloPoseInference::detect_raw(
    uint8_t* preprocessed_buf,
    float scale,
    float pad_left,
    float pad_top,
    float conf_threshold)
{
    std::vector<PoseDetection> result;
    if (!initialized_) {
        return result;
    }

    // Use the provided preprocessed buffer directly (ZeroCopy)
    result = run_inference_postprocess(
        rknn_ctx_, preprocessed_buf, input_buf_size_,
        input_width_, input_height_, output_dims_,
        output_type_, output_quantized_, output_scale_, output_zero_point_,
        scale, pad_left, pad_top, conf_threshold, nullptr);

    return result;
}