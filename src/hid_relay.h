#ifndef HID_RELAY_H
#define HID_RELAY_H

#include <thread>
#include <atomic>
#include <mutex>
#include "hid_controller.h"

// 防止和 OpenCV 的 KEY_* 宏冲突
#ifdef KEY_UP
#undef KEY_UP
#endif
#ifdef KEY_DOWN
#undef KEY_DOWN
#endif

#include <linux/input.h>
#include <linux/input-event-codes.h>

// 确保 F1-F12 按键码被定义（某些系统可能缺少）
#ifndef KEY_F1
#define KEY_F1          59
#define KEY_F2          60
#define KEY_F3          61
#define KEY_F4          62
#define KEY_F5          63
#define KEY_F6          64
#define KEY_F7          65
#define KEY_F8          66
#define KEY_F9          67
#define KEY_F10         68
#define KEY_F11         87
#define KEY_F12         88
#endif

namespace hid {

// 键盘报告结构
struct KeyboardReport {
    uint8_t modifiers;          // 修饰键
    uint8_t reserved;           // 保留
    uint8_t keycodes[6];        // 按键码
};

// 鼠标报告结构
struct MouseReport {
    uint8_t buttons;            // Buttons bitmask: bit0=left, bit1=right, bit2=middle
    int8_t dx;                  // X movement relative (-127 to +127)
    int8_t dy;                  // Y movement relative (-127 to +127)
    int8_t scroll;              // Scroll wheel
};

// HID 转发器配置
struct RelayConfig {
    // 设备路径
    const char* keyboard_device = nullptr;   // 键盘输入设备 (如 /dev/input/event0)
    const char* mouse_device = nullptr;      // 鼠标输入设备 (如 /dev/input/event1)
    
    // 按键映射（功能键用于控制吸附）
    int preset_legit_key = KEY_F1;        // Legit 预设
    int preset_semirage_key = KEY_F2;      // Semi-rage 预设
    int preset_rage_key = KEY_F3;          // Rage 预设
    int toggle_aim_key = KEY_F4;           // 切换自动开枪
    int toggle_mode_key = KEY_F5;          // 切换绝对/相对模式
    int sens_up_key = KEY_F6;              // 增加灵敏度
    int sens_down_key = KEY_F7;            // 降低灵敏度
    int toggle_game_mode_key = KEY_F9;     // 切换游戏模式
    
    // 性能
    int thread_priority = 99;             // 线程优先级 (0-99)
    bool use_realtime_scheduling = true;  // 实时调度
};

// HID 转发器
class HIDRelay {
public:
    explicit HIDRelay(const RelayConfig& cfg = RelayConfig());
    ~HIDRelay();
    
    // 初始化
    bool init(AimSystem* aim_system);
    void shutdown();
    bool isRunning() const { return running_; }
    
    // 开始/停止转发
    bool start();
    void stop();
    
    // 切换游戏模式
    void toggle_game_mode();
    
    // 获取游戏模式状态
    bool is_game_mode() const { return game_mode_; }
    
private:
    RelayConfig cfg_;
    AimSystem* aim_system_;
    
    std::atomic<bool> running_;
    std::thread keyboard_thread_;
    std::thread mouse_thread_;
    std::thread combo_thread_;
    
    // 设备文件描述符
    int keyboard_fd_ = -1;
    int mouse_fd_ = -1;
    
    // 当前状态
    std::mutex state_mutex_;
    KeyboardReport current_keyboard_;
    uint8_t current_mouse_buttons_ = 0;
    int8_t accumulated_mouse_dx_ = 0;
    int8_t accumulated_mouse_dy_ = 0;
    
    // OTG 连接状态
    bool otg_connected_ = false;
    bool last_otg_state_ = false;

    // 游戏模式
    bool game_mode_ = false;  // F9 控制

    // 自动开枪模式
    bool auto_fire_ = false;  // F1 控制
    
    // 设备是否已拦截
    bool keyboard_grabbed_ = false;
    bool mouse_grabbed_ = false;
    
    // 键盘事件处理线程
    void keyboardThreadFunc();
    
    // 鼠标事件处理线程
    void mouseThreadFunc();
    
    // 组合报告发送线程
    void comboThreadFunc();
    
    // 读取输入设备
    int openInputDevice(const char* path);
    bool readInputEvent(int fd, input_event& ev);
    
    // 拦截/释放设备
    bool grabDevice(int fd, bool grab);
    
    // 更新设备拦截状态（根据游戏模式）
    void updateGrabState();
    
    // 处理键盘事件
    void handleKeyboardEvent(const input_event& ev);
    
    // 处理鼠标事件
    void handleMouseEvent(const input_event& ev);
    
    // 发送组合报告
    bool sendCombinedReport();
    
    // 检查是否是功能键（不转发，仅用于内部控制）
    bool isFunctionKey(int keycode) const;
    
    // 设置实时线程优先级
    bool setRealtimePriority();
};

} // namespace hid

#endif // HID_RELAY_H
