#ifndef HID_CONTROLLER_H
#define HID_CONTROLLER_H

#include <opencv2/opencv.hpp>
#include <string>
#include "aim_engine.h"

namespace hid {

// HID 控制器配置
struct HIDConfig {
    float sensitivity = 1.0f;
    float smoothing = 0.3f;
    int max_delta = 127;
    int update_rate_hz = 60;
    bool humanize = true;
};

// HID 鼠标控制器
class HIDController {
public:
    explicit HIDController(const HIDConfig& cfg = HIDConfig());
    ~HIDController();
    
    bool init();
    void shutdown();
    bool isReady() const;
    
    bool move(int dx, int dy, bool left_button = false);
    
    bool aim(const cv::Point2f& delta, float strength = 1.0f);
    
    cv::Point2f humanizeDelta(cv::Point2f delta);
    
    cv::Point2f clampDelta(cv::Point2f delta);
    
    void setConfig(const HIDConfig& cfg) { cfg_ = cfg; }
    HIDConfig getConfig() const { return cfg_; }
    
    cv::Point2f getAccumulatedDelta() const { return accumulated_delta_; }
    void resetAccumulatedDelta() { accumulated_delta_ = cv::Point2f(0, 0); }
    
private:
    HIDConfig cfg_;
    bool initialized_;
    cv::Point2f last_delta_;
    cv::Point2f accumulated_delta_;
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
    
    // 更新准星位置（跟踪用户手动移动）
    void updateCrosshairPosition(float dx, float dy);
    
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
    
    void setConfigFile(const std::string& path) { config_file_ = path; }
    std::string getConfigFile() const { return config_file_; }
    
    void setSensitivity(float sens) { hid_config_.sensitivity = sens; }
    float getSensitivity() const { return hid_config_.sensitivity; }
    
    void setAbsoluteMode(bool absolute) { absolute_mode_ = absolute; }
    bool isAbsoluteMode() const { return absolute_mode_; }
    
    aim::AimPreset getCurrentPreset() const { return current_preset_; }

private:
    aim::AimEngine* aim_engine_;
    HIDController* hid_controller_;
    aim::AimConfig aim_config_;
    HIDConfig hid_config_;
    bool enabled_;
    bool initialized_;
    bool absolute_mode_ = true;
    std::string config_file_;
    aim::AimPreset current_preset_ = aim::AimPreset::LEGIT;
    
    bool loadPresetFromFile(aim::AimPreset preset, aim::AimConfig& cfg);
};

} // namespace hid

#endif // HID_CONTROLLER_H
