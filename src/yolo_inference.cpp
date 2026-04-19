/*
 * YOLOv8n-pose Inference on RK3588 NPU via RKNN
 * Takes BGR image from HDMI capture, runs detection, returns pose keypoints
 */

#include "yolo_inference.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <cmath>

#include <rockchip/rknn_api.h>
#include "opencv2/opencv.hpp"

// YOLOv8n-pose output shape: 1x(1 + 17)*8400 = 18x8400 = 151200
// box cx,cy,w,h + 17 kp * 2 = 4 + 34 = 38 values per box
// So output is 1 x 38 x 8400 = 1 x 8400 x 38 for NCHW or NHWC depending on model export

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

// NMS non-maximum suppression to remove overlapping boxes
static void nms(std::vector<PoseDetection>& input, std::vector<PoseDetection>& output, float iou_threshold) {
    sort(input.begin(), input.end(), [](const PoseDetection& a, const PoseDetection& b) {
        return a.score > b.score;
    });

    while (!input.empty()) {
        // Take highest score box
        output.push_back(input[0]);
        if (input.size() == 1) {
            break;
        }

        // Remove overlapping boxes with IoU > threshold
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
    input_width_ = 640;  // YOLOv8 default input size
    input_height_ = 640;
    channel_ = 3;
}

YoloPoseInference::~YoloPoseInference() {
    release();
}

int YoloPoseInference::init(const char* model_path, bool use_npu) {
    FILE* fp = fopen(model_path, "rb");
    if (!fp) {
        fprintf(stderr, "Failed to open model: %s\n", model_path);
        return -1;
    }

    // Get model size
    fseek(fp, 0, SEEK_END);
    size_t model_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    void* model_buffer = malloc(model_size);
    if (!model_buffer) {
        fprintf(stderr, "Failed to allocate memory for model\n");
        fclose(fp);
        return -1;
    }

    (void)fread(model_buffer, 1, model_size, fp);
    fclose(fp);

    // Init RKNN
    rknn_context ctx;
    int ret = rknn_init(&ctx, model_buffer, model_size, 0, 0);
    free(model_buffer); // RKNN copied it, we can free our copy

    if (ret != 0) {
        fprintf(stderr, "rknn_init failed: %d\n", ret);
        return -1;
    }

    rknn_ctx_ = ctx;
    initialized_ = true;

    // Get input tensor info
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != 0) {
        fprintf(stderr, "rknn_query io num failed: %d\n", ret);
        release();
        return -1;
    }

    printf("YOLOv8n-pose model loaded: %d inputs, %d outputs\n", io_num.n_input, io_num.n_output);

    // Get input dimensions
    rknn_tensor_attr input_attr;
    memset(&input_attr, 0, sizeof(input_attr));
    input_attr.index = 0;
    ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &input_attr, sizeof(input_attr));
    if (ret == 0) {
        if (input_attr.n_dims == 4) {
            // NHWC: batch,height,width,channel
            if (input_attr.dims[1] == 3) {
                // NCHW
                channel_ = input_attr.dims[1];
                input_height_ = input_attr.dims[2];
                input_width_ = input_attr.dims[3];
            } else {
                // NHWC
                input_height_ = input_attr.dims[1];
                input_width_ = input_attr.dims[2];
                channel_ = input_attr.dims[3];
            }
        }
        printf("Input size: %dx%d %d channels\n", input_width_, input_height_, channel_);
    }

    return 0;
}

void YoloPoseInference::release() {
    if (initialized_) {
        rknn_destroy(rknn_ctx_);
    }
    initialized_ = false;
}

std::vector<PoseDetection> YoloPoseInference::detect(const cv::Mat& bgr_img, float conf_threshold) {
    std::vector<PoseDetection> result;
    if (!initialized_) {
        return result;
    }

    // Preprocess: resize to model input size, convert to NHWC RGB 0-1 float
    cv::Mat resized;
    cv::resize(bgr_img, resized, cv::Size(input_width_, input_height_));
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

    // Prepare RKNN input
    rknn_input input;
    memset(&input, 0, sizeof(input));
    input.index = 0;
    input.type = RKNN_TENSOR_FLOAT32;
    input.size = input_width_ * input_height_ * channel_ * sizeof(float);
    input.buf = malloc(input.size);

    // Normalize to 0-1
    float* fbuf = (float*)input.buf;
    unsigned char* data = resized.data;
    for (int i = 0; i < input_width_ * input_height_; i++) {
        for (int c = 0; c < 3; c++) {
            fbuf[i * 3 + c] = data[i * 3 + c] / 255.0f;
        }
    }

    int ret = rknn_inputs_set(rknn_ctx_, 1, &input);
    free(input.buf);
    if (ret != 0) {
        fprintf(stderr, "rknn_inputs_set failed: %d\n", ret);
        return result;
    }

    // Run inference
    ret = rknn_run(rknn_ctx_, nullptr);
    if (ret != 0) {
        fprintf(stderr, "rknn_run failed: %d\n", ret);
        return result;
    }

    // Get output
    rknn_output output;
    memset(&output, 0, sizeof(output));
    output.index = 0;
    ret = rknn_outputs_get(rknn_ctx_, 1, &output, nullptr);
    if (ret != 0) {
        fprintf(stderr, "rknn_outputs_get failed: %d\n", ret);
        return result;
    }

    // Process output
    // YOLOv8n-pose output: (38, 8400) where 38 = 4box + 17*2kp + 17 score = 4 + 17*3 = 38
    float* out = (float*)output.buf;
    int num_detections = 8400;
    float scale_x = (float)bgr_img.cols / (float)input_width_;
    float scale_y = (float)bgr_img.rows / (float)input_height_;

    std::vector<PoseDetection> candidates;

    for (int i = 0; i < num_detections; i++) {
        // Output is [cx, cy, w, h, kp1x, kp1y, kp1s, kp2x, ... ]
        // Wait - actually for ultralytics export, it's 38 x 8400 -> each column is one detection
        float cx = out[i * 38 + 0];
        float cy = out[i * 38 + 1];
        float w = out[i * 38 + 2];
        float h = out[i * 38 + 3];
        float box_score = out[i * 38 + 4]; // actually objectness is already included in score
        float score = sigmoid(box_score);

        if (score < conf_threshold) {
            continue;
        }

        PoseDetection det;
        det.score = score;
        // cx cy w h -> center in image coordinates
        det.x = cx * scale_x;
        det.y = cy * scale_y;
        det.w = w * scale_x;
        det.h = h * scale_y;

        // Read 17 keypoints
        for (int k = 0; k < NUM_KP; k++) {
            det.kp[k][0] = out[i * 38 + 5 + k * 3 + 0] * scale_x;
            det.kp[k][1] = out[i * 38 + 5 + k * 3 + 1] * scale_y;
            det.kp_score[k] = sigmoid(out[i * 38 + 5 + k * 3 + 2]);
        }

        candidates.push_back(det);
    }

    // NMS
    std::vector<PoseDetection> nms_result;
    nms(candidates, nms_result, 0.7f);

    rknn_outputs_release(rknn_ctx_, 1, &output);

    return nms_result;
}
