#include "hid_relay.h"
#include "usb_hid.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sched.h>
#include <algorithm>

namespace hid {

// ========== HIDRelay ==========

HIDRelay::HIDRelay(const RelayConfig& cfg)
    : cfg_(cfg), aim_system_(nullptr), running_(false), keyboard_fd_(-1), mouse_fd_(-1), keyboard_grabbed_(false), mouse_grabbed_(false) {
    // 初始化键盘报告
    memset(&current_keyboard_, 0, sizeof(current_keyboard_));
}

HIDRelay::~HIDRelay() {
    shutdown();
}

bool HIDRelay::init(AimSystem* aim_system) {
    aim_system_ = aim_system;
    
    if (cfg_.mouse_device) {
        if (usb_hid_mouse_init() == 0) {
            printf("[RELAY] USB HID 鼠标初始化成功\n");
        } else {
            printf("[RELAY] 警告: USB HID 鼠标初始化失败\n");
        }
    }
    
    if (cfg_.keyboard_device) {
        if (usb_hid_keyboard_init() == 0) {
            printf("[RELAY] USB HID 键盘初始化成功\n");
        } else {
            printf("[RELAY] 警告: USB HID 键盘初始化失败\n");
        }
    }
    
    return true; // 即使 HID 初始化失败，程序仍可运行（用于调试）
}

void HIDRelay::shutdown() {
    stop();
    
    // 清理 USB HID
    if (keyboard_fd_ >= 0) {
        grabDevice(keyboard_fd_, false);
        close(keyboard_fd_);
        keyboard_fd_ = -1;
    }
    
    if (mouse_fd_ >= 0) {
        grabDevice(mouse_fd_, false);
        close(mouse_fd_);
        mouse_fd_ = -1;
    }
    
    usb_hid_mouse_exit();
    usb_hid_keyboard_exit();
}

bool HIDRelay::start() {
    if (running_) {
        return true;
    }
    
    running_ = true;
    
    // 打开键盘设备
    if (cfg_.keyboard_device) {
        keyboard_fd_ = openInputDevice(cfg_.keyboard_device);
        if (keyboard_fd_ >= 0) {
            printf("[RELAY] 键盘设备已打开: %s\n", cfg_.keyboard_device);
            keyboard_thread_ = std::thread(&HIDRelay::keyboardThreadFunc, this);
        } else {
            fprintf(stderr, "[RELAY] 错误: 无法打开键盘设备 %s\n", cfg_.keyboard_device);
        }
    }
    
    // 打开鼠标设备
    if (cfg_.mouse_device) {
        mouse_fd_ = openInputDevice(cfg_.mouse_device);
        if (mouse_fd_ >= 0) {
            printf("[RELAY] 鼠标设备已打开: %s\n", cfg_.mouse_device);
            mouse_thread_ = std::thread(&HIDRelay::mouseThreadFunc, this);
        } else {
            fprintf(stderr, "[RELAY] 错误: 无法打开鼠标设备 %s\n", cfg_.mouse_device);
        }
    }
    
    // 启动组合报告线程
    combo_thread_ = std::thread(&HIDRelay::comboThreadFunc, this);
    
    printf("[RELAY] 已启动（游戏模式: %s）\n", game_mode_ ? "开启" : "关闭");
    return true;
}

void HIDRelay::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
    }
    
    if (mouse_thread_.joinable()) {
        mouse_thread_.join();
    }
    
    if (combo_thread_.joinable()) {
        combo_thread_.join();
    }
}

void HIDRelay::toggle_game_mode() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    game_mode_ = !game_mode_;
    printf("[RELAY] 游戏模式: %s\n", game_mode_ ? "开启" : "关闭");
    
    // 更新设备拦截状态
    updateGrabState();
}

bool HIDRelay::setRealtimePriority() {
    if (!cfg_.use_realtime_scheduling) {
        return true;
    }
    
    struct sched_param param;
    param.sched_priority = cfg_.thread_priority;
    
    pthread_t this_thread = pthread_self();
    
    int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &param);
    if (ret != 0) {
        fprintf(stderr, "[RELAY] 警告: pthread_setschedparam 失败 (错误: %d)\n", ret);
        return false;
    }
    
    printf("[RELAY] 线程优先级已设置为 SCHED_FIFO (%d)\n", cfg_.thread_priority);
    return true;
}

int HIDRelay::openInputDevice(const char* path) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror("openInputDevice");
        return -1;
    }
    return fd;
}

bool HIDRelay::readInputEvent(int fd, input_event& ev) {
    ssize_t n = read(fd, &ev, sizeof(ev));
    return (n == sizeof(ev));
}

bool HIDRelay::grabDevice(int fd, bool grab) {
    if (fd < 0) return false;
    
    if (grab) {
        if (ioctl(fd, EVIOCGRAB, 1) < 0) {
            perror("grabDevice (EVIOCGRAB)");
            return false;
        }
        printf("[RELAY] 设备已拦截 (EVIOCGRAB)\n");
    } else {
        ioctl(fd, EVIOCGRAB, 0);
        printf("[RELAY] 设备已释放\n");
    }
    return true;
}

void HIDRelay::updateGrabState() {
    if (game_mode_) {
        // 游戏模式开启: 拦截设备
        if (cfg_.keyboard_device && keyboard_fd_ >= 0 && !keyboard_grabbed_) {
            if (grabDevice(keyboard_fd_, true)) {
                keyboard_grabbed_ = true;
            }
        }
        if (cfg_.mouse_device && mouse_fd_ >= 0 && !mouse_grabbed_) {
            if (grabDevice(mouse_fd_, true)) {
                mouse_grabbed_ = true;
            }
        }
    } else {
        // 游戏模式关闭: 释放设备
        if (cfg_.keyboard_device && keyboard_fd_ >= 0 && keyboard_grabbed_) {
            grabDevice(keyboard_fd_, false);
            keyboard_grabbed_ = false;
        }
        if (cfg_.mouse_device && mouse_fd_ >= 0 && mouse_grabbed_) {
            grabDevice(mouse_fd_, false);
            mouse_grabbed_ = false;
        }
    }
}

bool HIDRelay::isFunctionKey(int keycode) const {
    return (keycode == cfg_.toggle_aim_key ||
            keycode == cfg_.preset_legit_key ||
            keycode == cfg_.preset_semirage_key ||
            keycode == cfg_.preset_rage_key ||
            keycode == cfg_.toggle_mode_key ||
            keycode == cfg_.sens_up_key ||
            keycode == cfg_.sens_down_key ||
            keycode == cfg_.toggle_game_mode_key);
}

void HIDRelay::handleKeyboardEvent(const input_event& ev) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 首先处理功能键（即使不在游戏模式也响应）
    if (ev.type == EV_KEY) {
        if (isFunctionKey(ev.code)) {
            if (ev.value == 1) { // 按键按下时触发
                if (ev.code == cfg_.toggle_aim_key) {
                    auto_fire_ = !auto_fire_;
                    printf("[RELAY] 自动开枪: %s\n", auto_fire_ ? "开启" : "关闭");
                } else if (ev.code == cfg_.preset_legit_key) {
                    if (aim_system_) aim_system_->setPreset(aim::AimPreset::LEGIT);
                    printf("[RELAY] 预设: Legit\n");
                } else if (ev.code == cfg_.preset_semirage_key) {
                    if (aim_system_) aim_system_->setPreset(aim::AimPreset::SEMI_RAGE);
                    printf("[RELAY] 预设: Semi-rage\n");
                } else if (ev.code == cfg_.preset_rage_key) {
                    if (aim_system_) aim_system_->setPreset(aim::AimPreset::RAGE);
                    printf("[RELAY] 预设: Rage\n");
                } else if (ev.code == cfg_.toggle_mode_key) {
                    if (aim_system_) {
                        bool absolute = !aim_system_->isAbsoluteMode();
                        aim_system_->setAbsoluteMode(absolute);
                        printf("[RELAY] 模式: %s\n", absolute ? "绝对" : "相对");
                    }
                } else if (ev.code == cfg_.sens_up_key) {
                    if (aim_system_) {
                        float new_sens = aim_system_->getSensitivity() + 0.1f;
                        aim_system_->setSensitivity(new_sens);
                        printf("[RELAY] 灵敏度: %.1f\n", new_sens);
                    }
                } else if (ev.code == cfg_.sens_down_key) {
                    if (aim_system_) {
                        float new_sens = std::max(0.1f, aim_system_->getSensitivity() - 0.1f);
                        aim_system_->setSensitivity(new_sens);
                        printf("[RELAY] 灵敏度: %.1f\n", new_sens);
                    }
                } else if (ev.code == cfg_.toggle_game_mode_key) {
                    toggle_game_mode();
                }
            }
            // 功能键不转发到 HID
            return;
        }
    }
    
    // 如果游戏模式关闭，不处理键盘事件（让系统处理）
    if (!game_mode_) {
        return;
    }
    
    // 处理普通键盘事件并转发到 HID
    if (ev.type == EV_KEY) {
        // 处理修饰键
        uint8_t modifier_bit = 0;
        if (ev.code == KEY_LEFTCTRL) modifier_bit = 1 << 0;
        else if (ev.code == KEY_LEFTSHIFT) modifier_bit = 1 << 1;
        else if (ev.code == KEY_LEFTALT) modifier_bit = 1 << 2;
        else if (ev.code == KEY_LEFTMETA) modifier_bit = 1 << 3;
        else if (ev.code == KEY_RIGHTCTRL) modifier_bit = 1 << 4;
        else if (ev.code == KEY_RIGHTSHIFT) modifier_bit = 1 << 5;
        else if (ev.code == KEY_RIGHTALT) modifier_bit = 1 << 6;
        else if (ev.code == KEY_RIGHTMETA) modifier_bit = 1 << 7;
        
        if (modifier_bit != 0) {
            if (ev.value == 1) {
                current_keyboard_.modifiers |= modifier_bit;
            } else if (ev.value == 0) {
                current_keyboard_.modifiers &= ~modifier_bit;
            }
        } else {
            // 处理普通按键
            uint8_t hid_code = ev.code; // 这里需要转换，暂时直接用
            
            if (ev.value == 1) {
                // 按键按下
                for (int i = 0; i < 6; i++) {
                    if (current_keyboard_.keycodes[i] == 0) {
                        current_keyboard_.keycodes[i] = hid_code;
                        break;
                    }
                }
            } else if (ev.value == 0) {
                // 按键释放
                for (int i = 0; i < 6; i++) {
                    if (current_keyboard_.keycodes[i] == hid_code) {
                        current_keyboard_.keycodes[i] = 0;
                        break;
                    }
                }
            }
        }
    }
}

void HIDRelay::handleMouseEvent(const input_event& ev) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 如果游戏模式关闭，不处理鼠标事件（让系统处理）
    if (!game_mode_) {
        return;
    }
    
    if (ev.type == EV_KEY) {
        // 鼠标按键
        uint8_t button_bit = 0;
        if (ev.code == BTN_LEFT) button_bit = 1 << 0;
        else if (ev.code == BTN_RIGHT) button_bit = 1 << 1;
        else if (ev.code == BTN_MIDDLE) button_bit = 1 << 2;
        
        if (button_bit != 0) {
            if (ev.value == 1) {
                current_mouse_buttons_ |= button_bit;
            } else if (ev.value == 0) {
                current_mouse_buttons_ &= ~button_bit;
            }
        }
    } else if (ev.type == EV_REL) {
        if (ev.code == REL_X) {
            accumulated_mouse_dx_ += static_cast<int8_t>(std::clamp<int>(ev.value, -127, 127));
        } else if (ev.code == REL_Y) {
            accumulated_mouse_dy_ += static_cast<int8_t>(std::clamp<int>(ev.value, -127, 127));
        }
    }
}

void HIDRelay::keyboardThreadFunc() {
    printf("[RELAY] 键盘事件线程已启动\n");
    setRealtimePriority();
    
    while (running_) {
        input_event ev;
        if (readInputEvent(keyboard_fd_, ev)) {
            handleKeyboardEvent(ev);
        }
    }
}

void HIDRelay::mouseThreadFunc() {
    printf("[RELAY] 鼠标事件线程已启动\n");
    setRealtimePriority();
    
    while (running_) {
        input_event ev;
        if (readInputEvent(mouse_fd_, ev)) {
            handleMouseEvent(ev);
        }
    }
}

bool HIDRelay::sendCombinedReport() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!game_mode_) {
        return true;
    }
    
    // 准备鼠标报告
    int8_t dx = accumulated_mouse_dx_;
    int8_t dy = accumulated_mouse_dy_;
    uint8_t buttons = current_mouse_buttons_;
    
    // 重置累计
    accumulated_mouse_dx_ = 0;
    accumulated_mouse_dy_ = 0;
    
    // 吸附引擎覆盖鼠标移动
    if (aim_system_ && aim_system_->isEnabled() && aim_system_->hasTarget()) {
        float dt = 1.0f / 1000.0f; // 1000Hz
        cv::Point2f aim_delta = aim_system_->execute(dt);
        dx = static_cast<int8_t>(std::clamp(aim_delta.x, -127.0f, 127.0f));
        dy = static_cast<int8_t>(std::clamp(aim_delta.y, -127.0f, 127.0f));
    }
    
    // 发送键盘报告
    if (usb_hid_keyboard_ready()) {
        hid_keyboard_report_t report;
        report.modifiers = current_keyboard_.modifiers;
        report.reserved = 0;
        memcpy(report.keycodes, current_keyboard_.keycodes, 6);
        usb_hid_keyboard_send(&report);
    }
    
    // 发送鼠标报告
    if (usb_hid_mouse_ready()) {
        hid_mouse_report_t report;
        report.buttons = buttons;
        report.dx = dx;
        report.dy = dy;
        report.scroll = 0;
        usb_hid_mouse_send(&report);
    }
    
    return true;
}

void HIDRelay::comboThreadFunc() {
    printf("[RELAY] 组合报告线程已启动 (1000Hz)\n");
    setRealtimePriority();
    
    // 初始化 OTG 状态（仅用于显示）
    last_otg_state_ = usb_hid_otg_connected();
    otg_connected_ = last_otg_state_;
    
    if (otg_connected_) {
        printf("[RELAY] USB OTG 已连接到 PC\n");
    } else {
        printf("[RELAY] USB OTG 未连接\n");
    }
    
    while (running_) {
        // 检测 OTG 状态变化（仅用于显示）
        if (usb_hid_otg_state_changed(&last_otg_state_)) {
            otg_connected_ = last_otg_state_;
            if (otg_connected_) {
                printf("\n[RELAY] ========== USB OTG 已连接 ==========\n");
            } else {
                printf("\n[RELAY] ========== USB OTG 已断开 ==========\n");
            }
        }
        
        // 发送报告
        sendCombinedReport();
        
        usleep(1000); // 1000Hz (1ms 间隔)
    }
}

} // namespace hid
