#include <rclcpp/rclcpp.hpp>
#include <hello_moveit/msg/keyboard_state.hpp>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <thread>
#include <chrono>
#include <set>
#include <map>

class KeyboardPublisher : public rclcpp::Node
{
public:
    KeyboardPublisher() : Node("keyboard_publisher")
    {
        // 创建发布者
        key_publisher_ = this->create_publisher<hello_moveit::msg::KeyboardState>("keyboard_input", 10);
        
        // 设置终端为非缓冲模式
        setNonCanonicalMode();
        
        // 初始化按键状态
        initializeKeyStates();
        
        RCLCPP_INFO(this->get_logger(), "键盘输入节点已启动 - 切换控制模式");
        RCLCPP_INFO(this->get_logger(), "==== 键盘控制说明 ====");
        RCLCPP_INFO(this->get_logger(), "位置控制 (切换模式):");
        RCLCPP_INFO(this->get_logger(), "  w/s: X轴 前进/后退");
        RCLCPP_INFO(this->get_logger(), "  a/d: Y轴 左移/右移");
        RCLCPP_INFO(this->get_logger(), "  q/e: Z轴 上升/下降");
        RCLCPP_INFO(this->get_logger(), "姿态控制 (切换模式):");
        RCLCPP_INFO(this->get_logger(), "  i/k: Roll 旋转 +/-");
        RCLCPP_INFO(this->get_logger(), "  u/o: Pitch 旋转 +/-");
        RCLCPP_INFO(this->get_logger(), "  j/l: Yaw 旋转 +/-");
        RCLCPP_INFO(this->get_logger(), "特殊功能:");
        RCLCPP_INFO(this->get_logger(), "  r: 复位姿态");
        RCLCPP_INFO(this->get_logger(), "  p: 暂停所有运动");
        RCLCPP_INFO(this->get_logger(), "  x: 退出程序");
        RCLCPP_INFO(this->get_logger(), "===========================");
        RCLCPP_INFO(this->get_logger(), "使用方法: 按一次键开始运动，再按一次停止");
        RCLCPP_INFO(this->get_logger(), "支持多方向同时运动!");
        
        // 启动键盘检测线程
        keyboard_thread_ = std::thread(&KeyboardPublisher::keyboardLoop, this);
        
        // 启动固定频率发布定时器 (50Hz)
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&KeyboardPublisher::publishKeyboardState, this)
        );
    }
    
    ~KeyboardPublisher()
    {
        // 恢复终端模式
        restoreTerminalMode();
        
        // 等待线程结束
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    rclcpp::Publisher<hello_moveit::msg::KeyboardState>::SharedPtr key_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::thread keyboard_thread_;
    
    // 按键状态 - 每个按键的开/关状态
    std::set<char> active_keys_;
    std::mutex keys_mutex_;
    
    // 支持的按键列表
    const std::set<char> supported_keys_ = {'w', 's', 'a', 'd', 'q', 'e', 
                                           'i', 'k', 'u', 'o', 'j', 'l', 
                                           'r', 'p', 'x'};
    
    void initializeKeyStates()
    {
        // 初始化时所有按键都未按下
        active_keys_.clear();
    }
    
    // 检查是否有键盘输入可用
    bool kbhit() {
        struct timeval tv = {0L, 0L};
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(0, &fds);
        return select(1, &fds, NULL, NULL, &tv) > 0;
    }
    
    // 非阻塞获取键盘输入
    char getCharNonBlocking() {
        if (kbhit()) {
            return getchar();
        }
        return 0;  // 没有输入时返回0
    }
    
    // 设置终端为非缓冲模式
    void setNonCanonicalMode() {
        struct termios new_termios;
        tcgetattr(STDIN_FILENO, &new_termios);
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    }
    
    // 恢复终端正常模式
    void restoreTerminalMode() {
        struct termios new_termios;
        tcgetattr(STDIN_FILENO, &new_termios);
        new_termios.c_lflag |= (ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    }
    
    // 键盘检测循环
    void keyboardLoop() {
        while (rclcpp::ok()) {
            char key = getCharNonBlocking();
            
            // 检测到按键输入
            if (key != 0 && supported_keys_.count(key)) {
                {
                    std::lock_guard<std::mutex> lock(keys_mutex_);
                    
                    // 处理特殊功能按键
                    if (key == 'x') {
                        RCLCPP_INFO(this->get_logger(), "收到退出命令，关闭键盘节点");
                        rclcpp::shutdown();
                        break;
                    } else if (key == 'p') {
                        // 暂停所有运动
                        active_keys_.clear();
                        RCLCPP_INFO(this->get_logger(), "暂停所有运动");
                    } else if (key == 'r') {
                        // 复位姿态（不切换状态，直接触发）
                        RCLCPP_INFO(this->get_logger(), "复位姿态");
                    } else {
                        // 运动按键 - 切换状态
                        if (active_keys_.count(key)) {
                            // 按键已激活，关闭它
                            active_keys_.erase(key);
                            RCLCPP_INFO(this->get_logger(), "停止运动: %c", key);
                        } else {
                            // 按键未激活，开启它
                            active_keys_.insert(key);
                            RCLCPP_INFO(this->get_logger(), "开始运动: %c", key);
                        }
                    }
                    
                    // 显示当前活跃按键
                    if (!active_keys_.empty()) {
                        std::string active_str = "当前活跃: ";
                        for (char k : active_keys_) {
                            active_str += k;
                            active_str += " ";
                        }
                        RCLCPP_INFO(this->get_logger(), "%s", active_str.c_str());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "所有运动已停止");
                    }
                }
            }
            
            // 检测频率 - 10ms
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    // 固定频率发布按键状态
    void publishKeyboardState() {
        auto msg = hello_moveit::msg::KeyboardState();
        
        // 设置时间戳
        msg.stamp = this->get_clock()->now();
        
        // 获取当前活跃按键状态
        std::set<char> current_active_keys;
        {
            std::lock_guard<std::mutex> lock(keys_mutex_);
            current_active_keys = active_keys_;
        }
        
        // 设置各个按键状态
        msg.w_pressed = current_active_keys.count('w') ? 1 : 0;
        msg.s_pressed = current_active_keys.count('s') ? 1 : 0;
        msg.a_pressed = current_active_keys.count('a') ? 1 : 0;
        msg.d_pressed = current_active_keys.count('d') ? 1 : 0;
        msg.q_pressed = current_active_keys.count('q') ? 1 : 0;
        msg.e_pressed = current_active_keys.count('e') ? 1 : 0;
        
        msg.i_pressed = current_active_keys.count('i') ? 1 : 0;
        msg.k_pressed = current_active_keys.count('k') ? 1 : 0;
        msg.u_pressed = current_active_keys.count('u') ? 1 : 0;
        msg.o_pressed = current_active_keys.count('o') ? 1 : 0;
        msg.j_pressed = current_active_keys.count('j') ? 1 : 0;
        msg.l_pressed = current_active_keys.count('l') ? 1 : 0;
        
        msg.r_pressed = current_active_keys.count('r') ? 1 : 0;
        msg.x_pressed = current_active_keys.count('x') ? 1 : 0;
        
        // 发布消息
        key_publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardPublisher>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "键盘节点异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
} 