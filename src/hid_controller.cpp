#include "hid_controller.h"
#include "usb_hid.h"
#include <cmath>
#include <algorithm>

namespace hid {

// ========== HIDController ==========

HIDController::HIDController(const HIDConfig& cfg) 
    : cfg_(cfg), initialized_(false), last_delta_(0, 0) {
}

HIDController::~HIDController() {
    shutdown();
}

bool HIDController::init() {
    if (initialized_) return true;
    
    if (usb_hid_mouse_init() != 0) {
        return false;
    }
    
    initialized_ = true;
    return true;
}

void HIDController::shutdown() {
    if (initialized_) {
        usb_hid_mouse_exit();
        initialized_ = false;
    }
}

bool HIDController::isReady() const {
    return initialized_ && usb_hid_mouse_ready();
}

bool HIDController::move(int dx, int dy, bool left_button) {
    if (!isReady()) return false;
    
    uint8_t buttons = left_button ? 0x01 : 0x00;
    
    // 限制范围
    dx = std::clamp(dx, -127, 127);
    dy = std::clamp(dy, -127, 127);
    
    return usb_hid_mouse_move(static_cast<int8_t>(dx), static_cast<int8_t>(dy), buttons) == 0;
}

cv::Point2f HIDController::clampDelta(cv::Point2f delta) {
    delta.x = std::clamp(delta.x, static_cast<float>(-cfg_.max_delta), static_cast<float>(cfg_.max_delta));
    delta.y = std::clamp(delta.y, static_cast<float>(-cfg_.max_delta), static_cast<float>(cfg_.max_delta));
    return delta;
}

cv::Point2f HIDController::humanizeDelta(cv::Point2f delta) {
    if (!cfg_.humanize) return delta;
    
    // 应用平滑
    delta = last_delta_ * cfg_.smoothing + delta * (1.0f - cfg_.smoothing);
    last_delta_ = delta;
    
    // 添加微小随机偏移（模拟手抖）
    float jitter_x = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.5f;
    float jitter_y = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.5f;
    delta.x += jitter_x;
    delta.y += jitter_y;
    
    return delta;
}

bool HIDController::aim(const cv::Point2f& delta, float strength) {
    if (!isReady()) return false;
    
    // 应用灵敏度
    cv::Point2f scaled_delta = delta * cfg_.sensitivity * strength;
    
    // 人类化
    scaled_delta = humanizeDelta(scaled_delta);
    
    // 限制范围
    scaled_delta = clampDelta(scaled_delta);
    
    // 发送移动命令
    return move(static_cast<int>(scaled_delta.x), static_cast<int>(scaled_delta.y));
}

// ========== AimSystem ==========

AimSystem::AimSystem() 
    : aim_engine_(nullptr), hid_controller_(nullptr), 
      enabled_(false), initialized_(false) {
}

AimSystem::~AimSystem() {
    shutdown();
}

bool AimSystem::init(aim::AimPreset preset) {
    if (initialized_) return true;
    
    // 初始化吸附引擎（始终初始化，即使没有 HID 权限）
    aim_config_ = aim::getPreset(preset);
    aim_engine_ = new aim::AimEngine(aim_config_);
    
    // 初始化 HID 控制器
    hid_controller_ = new HIDController(hid_config_);
    if (!hid_controller_->init()) {
        // HID 初始化失败，仅使用视觉辅助模式
        delete hid_controller_;
        hid_controller_ = nullptr;
        printf("[AIM] HID init failed - running in visual-only mode\n");
    } else {
        printf("[AIM] HID controller initialized\n");
    }
    
    initialized_ = true;
    enabled_ = true;   // 默认开启吸附
    printf("[AIM] AimSystem initialized (enabled by default)\n");
    return true;
}

void AimSystem::shutdown() {
    if (hid_controller_) {
        hid_controller_->shutdown();
        delete hid_controller_;
        hid_controller_ = nullptr;
    }
    
    if (aim_engine_) {
        delete aim_engine_;
        aim_engine_ = nullptr;
    }
    
    initialized_ = false;
    enabled_ = false;
}

bool AimSystem::isReady() const {
    return initialized_ && aim_engine_ && hid_controller_ && hid_controller_->isReady();
}

void AimSystem::update(const std::vector<PoseDetection>& detections, 
                       float screen_width, float screen_height) {
    if (!initialized_ || !aim_engine_) return;
    
    aim_engine_->updateTarget(detections, screen_width, screen_height);
}

cv::Point2f AimSystem::execute(float dt) {
    if (!enabled_ || !initialized_ || !aim_engine_ || !hid_controller_) {
        return {0, 0};
    }
    
    // 获取吸附位移
    cv::Point2f delta = aim_engine_->getAimDelta(dt);
    
    // 如果位移太小，不移动
    if (std::abs(delta.x) < 0.5f && std::abs(delta.y) < 0.5f) {
        return {0, 0};
    }
    
    // 获取强度
    float strength = 1.0f;
    if (aim_engine_->hasTarget()) {
        auto target = aim_engine_->getCurrentTarget();
        // 根据距离调整强度
        if (target.distance_estimate > 30.0f) {
            strength = 0.6f;
        } else if (target.distance_estimate > 10.0f) {
            strength = 0.8f;
        }
    }
    
    // 执行 HID 移动
    if (hid_controller_->aim(delta, strength)) {
        return delta;
    }
    
    return {0, 0};
}

bool AimSystem::moveToTarget(float screen_width, float screen_height) {
    if (!enabled_ || !initialized_ || !aim_engine_ || !hid_controller_) {
        return false;
    }
    
    if (!aim_engine_->hasTarget()) {
        return false;
    }
    
    // 获取从屏幕中心到目标的偏移
    cv::Point2f offset = aim_engine_->getTargetOffsetFromCenter(screen_width, screen_height);
    
    // 如果偏移太小，不移动
    if (std::abs(offset.x) < 1.0f && std::abs(offset.y) < 1.0f) {
        return true;  // 已经在目标上
    }
    
    // 应用强度曲线（远距离弱吸附）
    float strength = 1.0f;
    auto target = aim_engine_->getCurrentTarget();
    if (target.distance_estimate > 30.0f) {
        strength = 0.6f;
    } else if (target.distance_estimate > 10.0f) {
        strength = 0.8f;
    }
    
    // 应用灵敏度和强度
    offset = offset * hid_config_.sensitivity * strength;
    
    // 人类化（平滑 + 微抖动）
    offset = hid_controller_->humanizeDelta(offset);
    
    // 限制最大移动距离
    offset = hid_controller_->clampDelta(offset);
    
    // 发送鼠标移动命令
    return hid_controller_->move(static_cast<int>(offset.x), static_cast<int>(offset.y));
}

bool AimSystem::shouldFire() const {
    if (!enabled_ || !aim_engine_) return false;
    return aim_engine_->shouldFire();
}

bool AimSystem::hasTarget() const {
    if (!aim_engine_) return false;
    return aim_engine_->hasTarget();
}

aim::AimTarget AimSystem::getTarget() const {
    if (!aim_engine_) return {{0, 0}, 0, 0, aim::HEAD, 0};
    return aim_engine_->getCurrentTarget();
}

void AimSystem::setPreset(aim::AimPreset preset) {
    aim_config_ = aim::getPreset(preset);
    if (aim_engine_) {
        aim_engine_->setConfig(aim_config_);
    }
}

} // namespace hid
