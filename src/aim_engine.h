#ifndef AIM_ENGINE_H
#define AIM_ENGINE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <random>
#include "yolo_inference.h"

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
    // 部位权重
    float head_weight = 3.0f;
    float neck_weight = 2.2f;
    float chest_weight = 1.5f;
    float belly_weight = 1.0f;
    
    // 距离衰减
    float distance_penalty = 0.8f;
    
    // 动态调整
    bool adaptive_body_part = true;
    
    // 距离阈值（米）
    float close_distance = 10.0f;
    float mid_distance = 30.0f;
};

// 目标选择器
class TargetSelector {
public:
    explicit TargetSelector(const TargetSelectorConfig& cfg = TargetSelectorConfig());
    
    AimTarget select(const PoseDetection& det, float screen_height);
    
private:
    TargetSelectorConfig cfg_;
    
    AimTarget getHeadTarget(const PoseDetection& det);
    AimTarget getNeckTarget(const PoseDetection& det, float head_y);
    AimTarget getChestTarget(const PoseDetection& det);
    AimTarget getBellyTarget(const PoseDetection& det);
    
    float estimateDistance(float bbox_h, float screen_height);
};

// 弹簧-阻尼系统配置
struct SpringConfig {
    float stiffness = 8.0f;          // 刚度
    float damping = 2.5f;            // 阻尼
    float mass = 1.0f;               // 质量
    
    float max_speed = 120.0f;        // 最大速度（像素/秒）
    float max_accel = 800.0f;        // 最大加速度
    
    float micro_jitter = 0.8f;       // 微抖动幅度
    float reaction_delay_ms = 80.0f; // 反应延迟
    float overshoot_ratio = 0.06f;   // 过冲比例
};

// 弹簧-阻尼物理引擎
class SpringAim {
public:
    explicit SpringAim(const SpringConfig& cfg = SpringConfig());
    
    // 每帧调用，dt 为时间差（秒）
    cv::Point2f update(cv::Point2f target, float dt);
    
    void reset(cv::Point2f start);
    
    cv::Point2f getCurrentPos() const { return current_pos_; }
    cv::Point2f getVelocity() const { return velocity_; }
    
private:
    SpringConfig cfg_;
    cv::Point2f current_pos_;
    cv::Point2f velocity_;
    std::deque<cv::Point2f> target_history_;
    std::mt19937 rng_;
    std::uniform_real_distribution<float> jitter_dist_;
    
    size_t reactionBufferSize() const;
    bool approachingTarget(cv::Point2f displacement, cv::Point2f velocity);
};

// 吸附强度曲线配置
struct StrengthCurveConfig {
    float close_strength = 1.0f;     // 近距离强度
    float mid_strength = 0.6f;       // 中距离强度
    float far_strength = 0.25f;      // 远距离强度
    
    float close_distance = 10.0f;    // 近距离阈值
    float mid_distance = 30.0f;      // 中距离阈值
    float far_distance = 50.0f;      // 远距离阈值
};

// 吸附强度曲线
class StrengthCurve {
public:
    explicit StrengthCurve(const StrengthCurveConfig& cfg = StrengthCurveConfig());
    
    float getStrength(float distance) const;
    
private:
    StrengthCurveConfig cfg_;
};

// 完整吸附配置
struct AimConfig {
    TargetSelectorConfig selector;
    SpringConfig spring;
    StrengthCurveConfig strength;
    
    // 全局开关
    bool enabled = true;
    float trigger_threshold = 0.15f;  // 触发阈值（降低以更容易触发）
    int fire_delay_frames = 3;       // 开火延迟帧数
};

// 预设配置
enum class AimPreset {
    LEGIT,      //  legit 模式（最像人类）
    SEMI_RAGE,  // 半激进
    RAGE        // 激进（高风险）
};

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
    AimConfig cfg_;
    TargetSelector selector_;
    SpringAim spring_;
    StrengthCurve strength_curve_;
    
    bool has_target_;
    AimTarget current_target_;
    int fire_delay_counter_;
    int frames_locked_;
    
    std::mt19937 rng_;
    std::uniform_int_distribution<int> reaction_delay_dist_;
    
    float screen_width_ = 0;
    float screen_height_ = 0;
};

} // namespace aim

#endif // AIM_ENGINE_H
