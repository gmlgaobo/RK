#ifndef YOLO_POSE_INFERENCE_H
#define YOLO_POSE_INFERENCE_H

#include <stdint.h>
#include <vector>
#include <opencv2/opencv.hpp>

// 人体关键点数量 - YOLOv8n-pose 是 17 个关键点
const int NUM_KP = 17;

// 单个检测结果： bounding box + 置信度 + 17 个关键点
struct PoseDetection {
    float x, y, w, h;       // bounding box center x, y, width, height
    float score;            // detection confidence
    float kp[NUM_KP][2];   // keypoints x, y (image coordinates)
    float kp_score[NUM_KP];// keypoint confidence
};

class YoloPoseInference {
public:
    YoloPoseInference();
    ~YoloPoseInference();

    // 初始化 RKNN 模型，加载 .rknn 文件
    // model_path: path to .rknn model file
    // use_npu: true to use NPU, false to use CPU for debug
    int init(const char* model_path, bool use_npu = true);

    // 释放资源
    void release();

    // 运行推理，输入 BGR 图像，返回检测结果
    // confidence_threshold: 过滤低置信度检测
    std::vector<PoseDetection> detect(const cv::Mat& bgr_img, float confidence_threshold = 0.5f);

    // 是否已初始化
    bool is_initialized() const { return initialized_; }

private:
    rknn_context rknn_ctx_;
    bool initialized_;
    int input_width_;
    int input_height_;
    int channel_;
};

#endif
