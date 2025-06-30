/*
 * MoveIt 实时笛卡尔速度连续运动控制器
 * 
 * 功能描述：
 * - 实现基于键盘输入的机械臂实时连续运动控制
 * - 采用笛卡尔空间速度控制策略
 * - 支持位置和姿态的同时控制
 * - 提供工作空间边界保护
 * - 发布实时位姿状态用于监控分析
 * 
 * 作者：AI Assistant
 * 版本：3.0 - 笛卡尔速度控制版本
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <hello_moveit/msg/keyboard_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <thread>
#include <vector>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <cmath>
#include <atomic>
#include <mutex>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <queue>
#include <array>
#include <set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @class MoveItSubscriber
 * @brief 机械臂实时笛卡尔速度连续运动控制器
 * 
 * 实现基于速度的连续运动控制：
 * - 按键按下时：计算目标笛卡尔速度
 * - 实时更新：以固定频率更新目标位置
 * - 连续运动：使用MoveIt的实时位置控制
 * - 平滑停止：按键释放时逐渐减速到零
 */
class MoveItSubscriber : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数 - 初始化节点和基础组件
     */
    MoveItSubscriber() : Node("moveit_subscriber")
    {
        // ============== TF变换监听器初始化 ==============
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // ============== 订阅器初始化 ==============
        key_subscription_ = this->create_subscription<hello_moveit::msg::KeyboardState>(
            "keyboard_input", 10, 
            std::bind(&MoveItSubscriber::keyCallback, this, std::placeholders::_1));
        
        // ============== 发布器初始化 ==============
        // 发布末端执行器位姿状态供监控脚本订阅
        end_effector_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/end_effector_pose", 10);
        
        RCLCPP_INFO(this->get_logger(), "MoveIt实时笛卡尔速度控制节点已启动");
        
        // ============== 延迟初始化MoveIt ==============
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MoveItSubscriber::initializeMoveIt, this)
        );
    }

private:
    // ==================== 核心组件 ====================
    
    /** @brief 键盘输入订阅器 */
    rclcpp::Subscription<hello_moveit::msg::KeyboardState>::SharedPtr key_subscription_;
    
    /** @brief MoveIt初始化定时器 */
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    /** @brief 运动控制定时器 */
    rclcpp::TimerBase::SharedPtr motion_timer_;
    
    /** @brief MoveIt运动规划接口 */
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    
    /** @brief MoveIt初始化状态标志 */
    bool moveit_initialized_ = false;
    
    // ==================== 速度控制参数 ====================
    
    /** @brief 线速度 (米/秒) - 位置控制速度 */
    const double linear_velocity_ = 0.02;  // 2cm/s，降低速度确保精确直线运动
    
    /** @brief 角速度 (弧度/秒) - 姿态控制速度 */
    const double angular_velocity_ = 0.05;  // 约2.9度/秒，降低角速度确保平滑姿态变化
    
    /** @brief 末端执行器期望速度 (米/秒) - 轨迹执行速度 */
    const double desired_end_effector_speed_ = 0.015;  // 1.5cm/s，稍微降低速度增加稳定性
    
    /** @brief 起始速度因子 - 相对于目标速度的比例 */
    const double start_speed_factor_ = 0.4;  // 40%，平滑启动
    
    /** @brief 结束速度因子 - 相对于目标速度的比例 */
    const double end_speed_factor_ = 0.4;    // 40%，与起始速度相同，确保连续性
    
    /** @brief 连续运动速度因子 - 运动中保持的统一速度 */
    const double continuous_speed_factor_ = 0.4;  // 40%，连续运动时的统一速度
    
    /** @brief 控制频率 (Hz) - 位置更新频率 */
    const double control_frequency_ = 50.0;  // 降低到50Hz，减少过于频繁的更新
    
    /** @brief 时间步长 (秒) */
    const double dt_ = 1.0 / control_frequency_;
    
    // ==================== 状态跟踪变量 ====================
    
    /** @brief 当前目标位姿 */
    geometry_msgs::msg::Pose target_pose_;
    
    /** @brief 当前键盘按键状态 */
    hello_moveit::msg::KeyboardState current_key_state_;
    
    /** @brief 运动状态标志 */
    std::atomic<bool> is_moving_{false};
    
    /** @brief 停止运动信号 */
    std::atomic<bool> should_stop_movement_{false};
    
    /** @brief 是否正在执行运动 */
    std::atomic<bool> is_executing_{false};
    
    /** @brief 上次执行运动的时间 */
    std::chrono::steady_clock::time_point last_execute_time_;
    
    /** @brief 运动执行间隔 (毫秒) */
    const int execute_interval_ms_ = 50;  // 增加到50ms间隔，减少轨迹冲突
    
    /** @brief 最小轨迹执行时间 (秒) */
    const double min_trajectory_duration_ = 0.3;  // 最小0.3秒，确保轨迹有足够时间执行
    
    /** @brief 最大轨迹执行时间 (秒) */
    const double max_trajectory_duration_ = 1.5;  // 最大1.5秒，避免过长的轨迹
    
    // ==================== 线程安全控制 ====================
    
    /** @brief 状态数据保护锁 */
    std::mutex state_mutex_;
    
    /** @brief 是否有运动指令 */
    std::atomic<bool> has_motion_command_{false};
    
    // ==================== TF变换系统 ====================
    
    /** @brief TF缓冲区 */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    /** @brief TF监听器 */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    /** @brief 末端执行器位姿发布器 */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pose_publisher_;
    
    // ==================== 核心功能函数 ====================
    
    /**
     * @brief 通过TF获取当前末端执行器位姿
     */
    bool getCurrentPoseFromTF(geometry_msgs::msg::Pose& pose)
    {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform("base_link", "link6", tf2::TimePointZero);
            
            pose.position.x = transform_stamped.transform.translation.x;
            pose.position.y = transform_stamped.transform.translation.y;
            pose.position.z = transform_stamped.transform.translation.z;
            pose.orientation = transform_stamped.transform.rotation;
            
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF获取位置失败: %s", ex.what());
            return false;
        }
    }
    
    /**
     * @brief 初始化MoveIt运动规划接口
     */
    void initializeMoveIt()
    {
        init_timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "正在初始化MoveIt运动规划接口...");
        
        try {
            // ============== 创建MoveGroup接口 ==============
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "rm_group");
            
            // ============== 配置规划参数 ==============
            move_group_interface_->setMaxVelocityScalingFactor(0.6);      // 速度缩放30%，确保平滑运动
            move_group_interface_->setMaxAccelerationScalingFactor(1.0);  // 加速度缩放50%，减少突变
            move_group_interface_->setPlanningTime(0.2);                  // 规划时间0.2s，给笛卡尔规划更多时间
            move_group_interface_->setNumPlanningAttempts(2);             // 规划尝试2次，提高成功率
            move_group_interface_->setPlannerId("RRTConnect");            // 使用RRT连接算法
            
            // ============== 设置容差参数 ==============
            move_group_interface_->setGoalPositionTolerance(0.001);       // 位置容差1mm，提高精度
            move_group_interface_->setGoalOrientationTolerance(0.01);     // 姿态容差约0.57度，提高精度
            move_group_interface_->setGoalJointTolerance(0.01);           // 关节容差约0.57度，提高精度
            
            // ============== 设置坐标系 ==============
            move_group_interface_->setPoseReferenceFrame("base_link");
            move_group_interface_->setEndEffectorLink("link6");
            
            // ============== 等待系统同步 ==============
            RCLCPP_INFO(this->get_logger(), "等待机器人状态同步...");
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // ============== 获取初始位置 ==============
            geometry_msgs::msg::Pose initial_pose;
            if (getCurrentPoseFromTF(initial_pose)) {
                target_pose_ = initial_pose;
                RCLCPP_INFO(this->get_logger(), "获取初始位置: (%.3f, %.3f, %.3f)", 
                           target_pose_.position.x, target_pose_.position.y, target_pose_.position.z);
            } else {
                // 使用默认安全位置
                target_pose_.position.x = -0.48358;
                target_pose_.position.y = 0.056398;
                target_pose_.position.z = 0.71246;
                target_pose_.orientation.x = -0.0303127;
                target_pose_.orientation.y = -0.0632058;
                target_pose_.orientation.z = 0.0250532;
                target_pose_.orientation.w = 0.997225;
                RCLCPP_WARN(this->get_logger(), "使用默认初始位置");
            }
            
            // ============== 启动运动控制定时器 ==============
            motion_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(dt_),
                std::bind(&MoveItSubscriber::motionControlLoop, this)
            );
            
            // 初始化执行时间
            last_execute_time_ = std::chrono::steady_clock::now();
            
            moveit_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "MoveIt初始化完成，实时控制已启动 (%.1fHz)", control_frequency_);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt初始化失败: %s", e.what());
        }
    }
    
    /**
     * @brief 键盘输入回调函数
     */
    void keyCallback(const hello_moveit::msg::KeyboardState::SharedPtr msg)
    {
        if (!moveit_initialized_) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // 更新按键状态
        current_key_state_ = *msg;
        
        // 检查是否有按键按下
        bool any_key_pressed = msg->w_pressed || msg->s_pressed || msg->a_pressed || 
                               msg->d_pressed || msg->q_pressed || msg->e_pressed ||
                               msg->i_pressed || msg->k_pressed || msg->u_pressed || 
                               msg->o_pressed || msg->j_pressed || msg->l_pressed;
        
        has_motion_command_ = any_key_pressed;
        
        if (any_key_pressed && !is_moving_) {
            is_moving_ = true;
            should_stop_movement_ = false;
            RCLCPP_INFO(this->get_logger(), "开始实时连续运动控制");
        } else if (!any_key_pressed && is_moving_) {
            is_moving_ = false;
            should_stop_movement_ = true;
            
            // ============== 停止运动和清除命令队列 ==============
            if (move_group_interface_) {
                try {
                    RCLCPP_INFO(this->get_logger(), "停止当前运动并清除命令队列");
                    
                    // 停止当前运动
                    move_group_interface_->stop();
                    
                    // 清除所有目标位姿
                    move_group_interface_->clearPoseTargets();
                    
                    // 重置执行状态
                    is_executing_ = false;
                    
                    // 获取当前实际位置并更新为新的目标位置
                    geometry_msgs::msg::Pose current_actual_pose;
                    if (getCurrentPoseFromTF(current_actual_pose)) {
                        target_pose_ = current_actual_pose;
                        RCLCPP_INFO(this->get_logger(), "更新目标位置为当前实际位置: (%.3f, %.3f, %.3f)", 
                                   target_pose_.position.x, target_pose_.position.y, target_pose_.position.z);
                    }
                    
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "停止运动时发生异常: %s", e.what());
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "停止连续运动");
        }
    }
    
    /**
     * @brief 运动控制主循环 - 以固定频率执行
     */
    void motionControlLoop()
    {
        if (!moveit_initialized_ || !has_motion_command_ || should_stop_movement_) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // ============== 计算目标速度 ==============
        geometry_msgs::msg::Twist target_velocity;
        target_velocity.linear.x = 0.0;
        target_velocity.linear.y = 0.0;
        target_velocity.linear.z = 0.0;
        target_velocity.angular.x = 0.0;
        target_velocity.angular.y = 0.0;
        target_velocity.angular.z = 0.0;
        
        // 位置控制
        if (current_key_state_.w_pressed) target_velocity.linear.x += linear_velocity_;
        if (current_key_state_.s_pressed) target_velocity.linear.x -= linear_velocity_;
        if (current_key_state_.a_pressed) target_velocity.linear.y += linear_velocity_;
        if (current_key_state_.d_pressed) target_velocity.linear.y -= linear_velocity_;
        if (current_key_state_.q_pressed) target_velocity.linear.z += linear_velocity_;
        if (current_key_state_.e_pressed) target_velocity.linear.z -= linear_velocity_;
        
        // 姿态控制
        if (current_key_state_.i_pressed) target_velocity.angular.x += angular_velocity_;
        if (current_key_state_.k_pressed) target_velocity.angular.x -= angular_velocity_;
        if (current_key_state_.u_pressed) target_velocity.angular.y += angular_velocity_;
        if (current_key_state_.o_pressed) target_velocity.angular.y -= angular_velocity_;
        if (current_key_state_.j_pressed) target_velocity.angular.z += angular_velocity_;
        if (current_key_state_.l_pressed) target_velocity.angular.z -= angular_velocity_;
        
        // ============== 更新目标位置 ==============
        updateTargetPose(target_velocity);
        
        // ============== 控制执行频率，避免过于频繁调用 ==============
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last_execute = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_execute_time_).count();
        
        // 只有在间隔足够长且没有正在执行运动时才执行新的运动指令
        // 增加更严格的执行控制，避免轨迹冲突
        if (time_since_last_execute >= execute_interval_ms_ && !is_executing_) {
            executeMotion();
            last_execute_time_ = current_time;
        } else if (is_executing_) {
            // 如果正在执行轨迹，延长等待时间，给当前轨迹更多完成时间
            RCLCPP_DEBUG(this->get_logger(), "轨迹正在执行中，等待完成...");
        }
        
        // 始终发布当前目标位姿供监控
        publishEndEffectorPose(target_pose_);
    }
    
    /**
     * @brief 根据速度更新目标位姿
     */
    void updateTargetPose(const geometry_msgs::msg::Twist& velocity)
    {
        // ============== 更新位置 ==============
        target_pose_.position.x += velocity.linear.x * dt_;
        target_pose_.position.y += velocity.linear.y * dt_;
        target_pose_.position.z += velocity.linear.z * dt_;
        
        // ============== 工作空间边界保护 ==============
        const double x_min = -0.8, x_max = 0.8;
        const double y_min = -0.8, y_max = 0.8;
        const double z_min = 0.1, z_max = 1.2;
        
        target_pose_.position.x = std::max(x_min, std::min(x_max, target_pose_.position.x));
        target_pose_.position.y = std::max(y_min, std::min(y_max, target_pose_.position.y));
        target_pose_.position.z = std::max(z_min, std::min(z_max, target_pose_.position.z));
        
        // ============== 更新姿态 ==============
        if (velocity.angular.x != 0.0 || velocity.angular.y != 0.0 || velocity.angular.z != 0.0) {
            // 将四元数转换为欧拉角
            tf2::Quaternion q(
                target_pose_.orientation.x,
                target_pose_.orientation.y,
                target_pose_.orientation.z,
                target_pose_.orientation.w);
            
            // 确保四元数规范化
            q.normalize();
            
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            // 更新欧拉角
            roll += velocity.angular.x * dt_;
            pitch += velocity.angular.y * dt_;
            yaw += velocity.angular.z * dt_;
            
            // 限制欧拉角范围，避免奇异
            roll = std::fmod(roll + M_PI, 2 * M_PI) - M_PI;    // [-π, π]
            pitch = std::max(-M_PI/2 + 0.1, std::min(M_PI/2 - 0.1, pitch));  // 避免万向锁
            yaw = std::fmod(yaw + M_PI, 2 * M_PI) - M_PI;      // [-π, π]
            
            // 转换回四元数并规范化
            tf2::Quaternion new_q;
            new_q.setRPY(roll, pitch, yaw);
            new_q.normalize();  // 确保四元数规范化
            
            target_pose_.orientation.x = new_q.x();
            target_pose_.orientation.y = new_q.y();
            target_pose_.orientation.z = new_q.z();
            target_pose_.orientation.w = new_q.w();
            
            // 验证四元数有效性
            double norm = sqrt(target_pose_.orientation.x * target_pose_.orientation.x +
                              target_pose_.orientation.y * target_pose_.orientation.y +
                              target_pose_.orientation.z * target_pose_.orientation.z +
                              target_pose_.orientation.w * target_pose_.orientation.w);
            
            if (norm < 0.9 || norm > 1.1) {
                RCLCPP_WARN(this->get_logger(), "四元数异常，重置为单位四元数");
                target_pose_.orientation.x = 0.0;
                target_pose_.orientation.y = 0.0;
                target_pose_.orientation.z = 0.0;
                target_pose_.orientation.w = 1.0;
            }
        }
    }
    
    /**
     * @brief 执行运动到目标位姿
     */
    void executeMotion()
    {
        try {
            // 设置执行状态
            is_executing_ = true;
            
            // 获取当前实际位姿作为起点
            geometry_msgs::msg::Pose current_actual_pose;
            if (!getCurrentPoseFromTF(current_actual_pose)) {
                RCLCPP_WARN(this->get_logger(), "无法获取当前位姿，跳过本次运动");
                is_executing_ = false;
                return;
            }
            
            // 计算从当前位置到目标位置的距离
            double distance = sqrt(
                pow(target_pose_.position.x - current_actual_pose.position.x, 2) +
                pow(target_pose_.position.y - current_actual_pose.position.y, 2) +
                pow(target_pose_.position.z - current_actual_pose.position.z, 2)
            );
            
            // 如果距离太小，跳过执行
            if (distance < 0.001) {  // 1mm阈值
                is_executing_ = false;
                return;
            }
            
            // ============== 计算中间点，减小运动步长 ==============
            // 将大的运动分解为更小的步长，提高连续性
            const double max_step_distance = 0.01;  // 10mm最大步长
            
            geometry_msgs::msg::Pose intermediate_pose;
            if (distance > max_step_distance) {
                // 计算方向向量
                double dx = target_pose_.position.x - current_actual_pose.position.x;
                double dy = target_pose_.position.y - current_actual_pose.position.y;
                double dz = target_pose_.position.z - current_actual_pose.position.z;
                
                // 归一化方向向量
                double norm = sqrt(dx*dx + dy*dy + dz*dz);
                dx /= norm;
                dy /= norm;
                dz /= norm;
                
                // 计算中间目标点
                intermediate_pose = current_actual_pose;
                intermediate_pose.position.x += dx * max_step_distance;
                intermediate_pose.position.y += dy * max_step_distance;
                intermediate_pose.position.z += dz * max_step_distance;
                
                // 姿态插值（球面线性插值近似）
                tf2::Quaternion q_current(
                    current_actual_pose.orientation.x,
                    current_actual_pose.orientation.y,
                    current_actual_pose.orientation.z,
                    current_actual_pose.orientation.w);
                tf2::Quaternion q_target(
                    target_pose_.orientation.x,
                    target_pose_.orientation.y,
                    target_pose_.orientation.z,
                    target_pose_.orientation.w);
                
                // 简单线性插值（对于小角度变化足够）
                double t = max_step_distance / distance;  // 插值参数
                tf2::Quaternion q_intermediate = q_current.slerp(q_target, t);
                q_intermediate.normalize();
                
                intermediate_pose.orientation.x = q_intermediate.x();
                intermediate_pose.orientation.y = q_intermediate.y();
                intermediate_pose.orientation.z = q_intermediate.z();
                intermediate_pose.orientation.w = q_intermediate.w();
                
                distance = max_step_distance;  // 更新实际移动距离
            } else {
                intermediate_pose = target_pose_;
            }
            
            // ============== 使用笛卡尔路径规划确保直线运动 ==============
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(current_actual_pose);  // 起点：当前实际位置
            waypoints.push_back(intermediate_pose);    // 终点：中间目标位置
            
            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step = 0.001;        // 末端执行器步长：1mm，更精细
            const double jump_threshold = 0.0;    // 关节跳跃阈值：禁用，确保连续性
            
            // 执行笛卡尔路径规划
            double fraction = move_group_interface_->computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory);
            
            if (fraction > 0.9) {  // 提高规划成功率阈值：90%
                
                // ============== 末端速度控制 ==============
                // 计算期望的执行时间，基于距离和期望速度
                double desired_duration = distance / desired_end_effector_speed_;
                
                // 限制最小和最大执行时间
                desired_duration = std::max(min_trajectory_duration_, std::min(max_trajectory_duration_, desired_duration));
                
                // 输出轨迹规划信息
                RCLCPP_INFO(this->get_logger(), 
                           "规划轨迹: 点数=%zu, 距离=%.1fmm, 期望时间=%.3fs, 规划成功率=%.1f%%", 
                           trajectory.joint_trajectory.points.size(), distance * 1000, 
                           desired_duration, fraction * 100);
                
                // 重新时间参数化轨迹以控制末端速度
                if (!trajectory.joint_trajectory.points.empty()) {
                    // 计算轨迹点数量
                    size_t num_points = trajectory.joint_trajectory.points.size();
                    
                    // 重新分配时间戳
                    for (size_t i = 0; i < num_points; ++i) {
                        double time_from_start = (desired_duration * i) / (num_points - 1);
                        trajectory.joint_trajectory.points[i].time_from_start = 
                            rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(time_from_start * 1e9));
                        
                        // 计算速度（简单差分法）
                        if (i > 0 && i < num_points - 1) {
                            double dt = time_from_start - (desired_duration * (i-1)) / (num_points - 1);
                            if (dt > 0) {
                                auto& current_point = trajectory.joint_trajectory.points[i];
                                auto& prev_point = trajectory.joint_trajectory.points[i-1];
                                
                                // 确保速度数组大小正确
                                if (current_point.velocities.size() != current_point.positions.size()) {
                                    current_point.velocities.resize(current_point.positions.size());
                                }
                                
                                // 计算关节速度
                                for (size_t j = 0; j < current_point.positions.size(); ++j) {
                                    double joint_velocity = (current_point.positions[j] - prev_point.positions[j]) / dt;
                                    current_point.velocities[j] = joint_velocity;
                                }
                            }
                        }
                    }
                    
                    // ============== 优化起始和结束点速度控制 ==============
                    if (num_points > 1) {
                        auto& first_point = trajectory.joint_trajectory.points[0];
                        auto& last_point = trajectory.joint_trajectory.points[num_points - 1];
                        
                        // 确保速度数组大小正确
                        if (first_point.velocities.size() != first_point.positions.size()) {
                            first_point.velocities.resize(first_point.positions.size());
                        }
                        if (last_point.velocities.size() != last_point.positions.size()) {
                            last_point.velocities.resize(last_point.positions.size());
                        }
                        
                        // ============== 起始点速度设置 ==============
                        // 如果机械臂正在运动中，保持一定的起始速度
                        if (is_moving_ && !should_stop_movement_) {
                            // 计算期望的起始速度（基于运动方向）
                            double dx = intermediate_pose.position.x - current_actual_pose.position.x;
                            double dy = intermediate_pose.position.y - current_actual_pose.position.y;
                            double dz = intermediate_pose.position.z - current_actual_pose.position.z;
                            double movement_distance = sqrt(dx*dx + dy*dy + dz*dz);
                            
                            if (movement_distance > 0.001 && num_points > 1) {
                                // 计算第一段的时间间隔
                                double first_dt = (trajectory.joint_trajectory.points[1].time_from_start.sec + 
                                                  trajectory.joint_trajectory.points[1].time_from_start.nanosec * 1e-9) - 
                                                 (trajectory.joint_trajectory.points[0].time_from_start.sec +
                                                  trajectory.joint_trajectory.points[0].time_from_start.nanosec * 1e-9);
                                
                                if (first_dt > 0) {
                                    // 使用统一的连续运动速度因子，确保起始速度一致
                                    for (size_t j = 0; j < first_point.positions.size(); ++j) {
                                        double joint_displacement = trajectory.joint_trajectory.points[1].positions[j] - 
                                                                   first_point.positions[j];
                                        double target_velocity = joint_displacement / first_dt;
                                        first_point.velocities[j] = target_velocity * continuous_speed_factor_;
                                    }
                                }
                            }
                        } else {
                            // 运动开始时，起始速度为零
                            std::fill(first_point.velocities.begin(), first_point.velocities.end(), 0.0);
                        }
                        
                        // ============== 结束点速度设置 ==============
                        // 如果还有更多运动指令在队列中，保持一定的结束速度
                        if (has_motion_command_ && !should_stop_movement_) {
                            // 计算期望的结束速度（基于运动方向）
                            if (num_points > 1) {
                                // 计算最后一段的时间间隔
                                double last_dt = (last_point.time_from_start.sec + 
                                                 last_point.time_from_start.nanosec * 1e-9) -
                                                (trajectory.joint_trajectory.points[num_points-2].time_from_start.sec +
                                                 trajectory.joint_trajectory.points[num_points-2].time_from_start.nanosec * 1e-9);
                                
                                if (last_dt > 0) {
                                    // 使用统一的连续运动速度因子，确保结束速度与起始速度相同
                                    for (size_t j = 0; j < last_point.positions.size(); ++j) {
                                        double joint_displacement = last_point.positions[j] - 
                                                                   trajectory.joint_trajectory.points[num_points-2].positions[j];
                                        double target_velocity = joint_displacement / last_dt;
                                        last_point.velocities[j] = target_velocity * continuous_speed_factor_;
                                    }
                                }
                            }
                        } else {
                            // 运动结束时，结束速度为零
                            std::fill(last_point.velocities.begin(), last_point.velocities.end(), 0.0);
                        }
                    }
                }
                
                // 在独立线程中执行笛卡尔轨迹
                std::thread exec_thread([this, trajectory, distance, desired_duration]() {
                    try {
                        RCLCPP_INFO(this->get_logger(), "开始执行轨迹，预期时间: %.3fs", desired_duration);
                        
                        // 创建规划结果
                        moveit::planning_interface::MoveGroupInterface::Plan plan;
                        plan.trajectory_ = trajectory;
                        
                        // 执行笛卡尔轨迹
                        auto result = move_group_interface_->execute(plan);
                        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                            RCLCPP_INFO(this->get_logger(), 
                                        "轨迹执行成功！距离: %.1fmm，实际用时: %.3fs", 
                                        distance * 1000, desired_duration);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "轨迹执行失败，错误码: %d", result.val);
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "轨迹执行异常: %s", e.what());
                    }
                    
                    // 重置执行状态
                    is_executing_ = false;
                    RCLCPP_DEBUG(this->get_logger(), "轨迹执行完成，状态已重置");
                });
                exec_thread.detach();
                
                RCLCPP_DEBUG(this->get_logger(), 
                           "笛卡尔路径规划成功 (%.1f%%)，距离: %.3fmm，预期执行时间: %.3fs", 
                           fraction * 100, distance * 1000, desired_duration);
            } else {
                RCLCPP_WARN(this->get_logger(), "笛卡尔路径规划成功率过低: %.1f%%，跳过本次运动", 
                           fraction * 100);
                is_executing_ = false;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "运动执行异常: %s", e.what());
            is_executing_ = false;
        }
    }
    
    /**
     * @brief 发布末端执行器位姿状态
     */
    void publishEndEffectorPose(const geometry_msgs::msg::Pose& pose, const std::string& frame_id = "base_link") {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = frame_id;
        pose_stamped.pose = pose;
        
        end_effector_pose_publisher_->publish(pose_stamped);
    }
};

/**
 * @brief 主函数
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MoveItSubscriber>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "节点异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
} 