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
    aim::AimEngine* aim_engine_;
    HIDController* hid_controller_;
    aim::AimConfig aim_config_;
    HIDConfig hid_config_;
    bool enabled_;
    bool initialized_;
    bool absolute_mode_ = true;  // 默认使用绝对定位模式
};

} // namespace hid

#endif // HID_CONTROLLER_H
