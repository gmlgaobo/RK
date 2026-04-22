#include "aim_engine.h"
#include <cmath>
#include <algorithm>

namespace aim {

// ========== TargetSelector ==========

TargetSelector::TargetSelector(const TargetSelectorConfig& cfg) : cfg_(cfg) {}

AimTarget TargetSelector::select(const PoseDetection& det, float screen_height) {
    float bbox_h = det.h;
    float distance = estimateDistance(bbox_h, screen_height);
    
    AimTarget target;
    target.distance_estimate = distance;
    target.confidence = det.score;
    
    if (cfg_.adaptive_body_part) {
        if (distance < cfg_.close_distance) {
            target = getHeadTarget(det);
            target.hit_probability = 0.85f;
        } else if (distance < cfg_.mid_distance) {
            target = getNeckTarget(det, det.y - det.h * 0.35f);
            target.hit_probability = 0.75f;
        } else {
            target = getChestTarget(det);
            target.hit_probability = 0.65f;
        }
    } else {
        // 固定瞄头
        target = getHeadTarget(det);
    }
    
    target.distance_estimate = distance;
    
    // 综合得分
    float score = target.hit_probability * target.confidence;
    score *= (1.0f - distance * cfg_.distance_penalty * 0.02f);
    target.confidence = score;
    
    return target;
}

float TargetSelector::estimateDistance(float bbox_h, float screen_height) {
    // 简化的距离估计：bbox 越高越近
    float ratio = bbox_h / screen_height;
    return std::clamp(200.0f / (ratio * 100.0f + 1.0f), 1.0f, 100.0f);
}

AimTarget TargetSelector::getHeadTarget(const PoseDetection& det) {
    cv::Point2f head;
    // 使用鼻子关键点（索引 0）
    if (det.kp_score[0] > 0.4f) {
        head.x = det.kp[0][0];
        head.y = det.kp[0][1];
    } else {
        // 估算：bbox 顶部中心
        head.x = det.x;
        head.y = det.y - det.h * 0.35f;
    }
    return {head, 1.0f, 0.85f, HEAD, 0};
}

AimTarget TargetSelector::getNeckTarget(const PoseDetection& det, float head_y) {
    float neck_offset = det.h * 0.05f;
    return {
        {det.x, head_y + neck_offset},
        0.95f, 0.75f, NECK, 0
    };
}

AimTarget TargetSelector::getChestTarget(const PoseDetection& det) {
    // 使用肩膀关键点（索引 5, 6）
    float cx, cy;
    if (det.kp_score[5] > 0.3f && det.kp_score[6] > 0.3f) {
        cx = (det.kp[5][0] + det.kp[6][0]) * 0.5f;
        cy = (det.kp[5][1] + det.kp[6][1]) * 0.5f;
    } else {
        cx = det.x;
        cy = det.y - det.h * 0.1f;
    }
    float offset = det.h * 0.12f;
    return {{cx, cy + offset}, 0.9f, 0.65f, CHEST, 0};
}

AimTarget TargetSelector::getBellyTarget(const PoseDetection& det) {
    return {{det.x, det.y + det.h * 0.15f}, 0.8f, 0.5f, BELLY, 0};
}

// ========== SpringAim ==========

SpringAim::SpringAim(const SpringConfig& cfg) 
    : cfg_(cfg), 
      rng_(std::random_device{}()),
      jitter_dist_(-cfg.micro_jitter * 0.5f, cfg.micro_jitter * 0.5f) {
    reset({0, 0});
}

cv::Point2f SpringAim::update(cv::Point2f target, float dt) {
    // 反应延迟
    target_history_.push_back(target);
    if (target_history_.size() > reactionBufferSize()) {
        target_history_.pop_front();
    }
    cv::Point2f delayed_target = target_history_.front();
    
    // 弹簧力
    cv::Point2f displacement = delayed_target - current_pos_;
    cv::Point2f spring_force = displacement * cfg_.stiffness;
    
    // 阻尼力
    cv::Point2f damping_force = velocity_ * (-cfg_.damping);
    
    // 合力
    cv::Point2f total_force = spring_force + damping_force;
    
    // 限制加速度
    float force_mag = std::sqrt(total_force.x * total_force.x + total_force.y * total_force.y);
    if (force_mag > cfg_.max_accel * cfg_.mass) {
        total_force = total_force * (cfg_.max_accel * cfg_.mass / force_mag);
    }
    
    // 积分
    cv::Point2f accel = total_force / cfg_.mass;
    velocity_ = velocity_ + accel * dt;
    
    // 限制速度
    float speed = std::sqrt(velocity_.x * velocity_.x + velocity_.y * velocity_.y);
    if (speed > cfg_.max_speed) {
        velocity_ = velocity_ * (cfg_.max_speed / speed);
    }
    
    // 过冲模拟
    if (speed > 10.0f && approachingTarget(displacement, velocity_)) {
        velocity_ = velocity_ * (1.0f + cfg_.overshoot_ratio);
    }
    
    // 微抖动
    float jitter_x = jitter_dist_(rng_);
    float jitter_y = jitter_dist_(rng_);
    
    current_pos_ = current_pos_ + velocity_ * dt;
    current_pos_.x += jitter_x;
    current_pos_.y += jitter_y;
    
    return velocity_ * dt;
}

void SpringAim::reset(cv::Point2f start) {
    current_pos_ = start;
    velocity_ = {0, 0};
    target_history_.clear();
}

size_t SpringAim::reactionBufferSize() const {
    return static_cast<size_t>(cfg_.reaction_delay_ms / 16.67f) + 1;
}

bool SpringAim::approachingTarget(cv::Point2f displacement, cv::Point2f velocity) {
    float dot = displacement.x * velocity.x + displacement.y * velocity.y;
    return dot > 0;
}

// ========== StrengthCurve ==========

StrengthCurve::StrengthCurve(const StrengthCurveConfig& cfg) : cfg_(cfg) {}

float StrengthCurve::getStrength(float distance) const {
    if (distance < cfg_.close_distance) {
        return cfg_.close_strength;
    } else if (distance < cfg_.mid_distance) {
        float t = (distance - cfg_.close_distance) / (cfg_.mid_distance - cfg_.close_distance);
        return cfg_.close_strength + (cfg_.mid_strength - cfg_.close_strength) * t;
    } else if (distance < cfg_.far_distance) {
        float t = (distance - cfg_.mid_distance) / (cfg_.far_distance - cfg_.mid_distance);
        return cfg_.mid_strength + (cfg_.far_strength - cfg_.mid_strength) * t;
    } else {
        return cfg_.far_strength;
    }
}

// ========== Presets ==========

AimConfig getPreset(AimPreset preset) {
    AimConfig cfg;
    
    switch (preset) {
        case AimPreset::LEGIT:
            // 最像人类的配置
            cfg.selector.head_weight = 2.0f;
            cfg.selector.neck_weight = 2.5f;
            cfg.selector.adaptive_body_part = true;
            
            cfg.spring.stiffness = 5.0f;
            cfg.spring.damping = 3.0f;
            cfg.spring.max_speed = 80.0f;
            cfg.spring.micro_jitter = 1.2f;
            cfg.spring.reaction_delay_ms = 120.0f;
            cfg.spring.overshoot_ratio = 0.08f;
            
            cfg.strength.close_strength = 0.7f;
            cfg.strength.mid_strength = 0.4f;
            cfg.strength.far_strength = 0.15f;
            
            cfg.fire_delay_frames = 5;
            break;
            
        case AimPreset::SEMI_RAGE:
            // 平衡配置
            cfg.selector.head_weight = 3.0f;
            cfg.selector.neck_weight = 2.2f;
            cfg.selector.adaptive_body_part = true;
            
            cfg.spring.stiffness = 8.0f;
            cfg.spring.damping = 2.5f;
            cfg.spring.max_speed = 120.0f;
            cfg.spring.micro_jitter = 0.8f;
            cfg.spring.reaction_delay_ms = 80.0f;
            cfg.spring.overshoot_ratio = 0.06f;
            
            cfg.strength.close_strength = 1.0f;
            cfg.strength.mid_strength = 0.6f;
            cfg.strength.far_strength = 0.25f;
            
            cfg.fire_delay_frames = 3;
            break;
            
        case AimPreset::RAGE:
            // 激进配置
            cfg.selector.head_weight = 5.0f;
            cfg.selector.neck_weight = 3.0f;
            cfg.selector.adaptive_body_part = false;
            
            cfg.spring.stiffness = 15.0f;
            cfg.spring.damping = 1.5f;
            cfg.spring.max_speed = 200.0f;
            cfg.spring.micro_jitter = 0.3f;
            cfg.spring.reaction_delay_ms = 40.0f;
            cfg.spring.overshoot_ratio = 0.02f;
            
            cfg.strength.close_strength = 1.0f;
            cfg.strength.mid_strength = 0.9f;
            cfg.strength.far_strength = 0.6f;
            
            cfg.fire_delay_frames = 1;
            break;
    }
    
    return cfg;
}

// ========== AimEngine ==========

AimEngine::AimEngine(const AimConfig& cfg)
    : cfg_(cfg),
      selector_(cfg.selector),
      spring_(cfg.spring),
      strength_curve_(cfg.strength),
      has_target_(false),
      fire_delay_counter_(0),
      frames_locked_(0),
      rng_(std::random_device{}()),
      reaction_delay_dist_(80, 200) {
}

void AimEngine::updateTarget(const std::vector<PoseDetection>& detections,
                             float screen_width, float screen_height) {
    screen_width_ = screen_width;
    screen_height_ = screen_height;
    
    if (detections.empty()) {
        has_target_ = false;
        fire_delay_counter_ = 0;
        frames_locked_ = 0;
        return;
    }
    
    // 选择最佳目标（置信度最高）
    const PoseDetection* best_det = nullptr;
    float best_score = -1.0f;
    
    for (const auto& det : detections) {
        if (det.score < cfg_.trigger_threshold) continue;
        
        float score = det.score;
        if (score > best_score) {
            best_score = score;
            best_det = &det;
        }
    }
    
    if (!best_det) {
        has_target_ = false;
        return;
    }
    
    // 选择目标点
    current_target_ = selector_.select(*best_det, screen_height);
    
    if (!has_target_) {
        // 新目标，重置弹簧系统
        spring_.reset(current_target_.pos);
        fire_delay_counter_ = cfg_.fire_delay_frames + (rng_() % 3);
        frames_locked_ = 0;
        printf("[AIM] New target acquired: pos=(%.0f, %.0f) part=%s score=%.2f\n",
               current_target_.pos.x, current_target_.pos.y,
               current_target_.body_part == HEAD ? "HEAD" :
               current_target_.body_part == NECK ? "NECK" :
               current_target_.body_part == CHEST ? "CHEST" : "BELLY",
               current_target_.confidence);
    }
    
    has_target_ = true;
    frames_locked_++;
}

cv::Point2f AimEngine::getAimDelta(float dt) {
    if (!has_target_) {
        return {0, 0};
    }
    
    // 获取弹簧位移
    cv::Point2f delta = spring_.update(current_target_.pos, dt);
    
    // 应用强度曲线
    float strength = strength_curve_.getStrength(current_target_.distance_estimate);
    delta = delta * strength;
    
    return delta;
}

cv::Point2f AimEngine::getTargetOffsetFromCenter(float screen_width, float screen_height) const {
    if (!has_target_) {
        return {0, 0};
    }
    
    // 计算屏幕中心
    float center_x = screen_width / 2.0f;
    float center_y = screen_height / 2.0f;
    
    // 计算从中心到目标的偏移
    float offset_x = current_target_.pos.x - center_x;
    float offset_y = current_target_.pos.y - center_y;
    
    return {offset_x, offset_y};
}

bool AimEngine::shouldFire() const {
    if (!has_target_) return false;
    
    // 延迟开火
    if (fire_delay_counter_ > 0) {
        return false;
    }
    
    // 需要锁定一定帧数
    if (frames_locked_ < 3) {
        return false;
    }
    
    return true;
}

void AimEngine::reset() {
    has_target_ = false;
    fire_delay_counter_ = 0;
    frames_locked_ = 0;
    spring_.reset({0, 0});
}

} // namespace aim
