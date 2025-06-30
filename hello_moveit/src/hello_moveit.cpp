#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <thread>
#include <vector>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <fcntl.h>
#include <sys/select.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 函数声明：检查是否有键盘输入可用
bool kbhit() {
  struct timeval tv = {0L, 0L};
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  return select(1, &fds, NULL, NULL, &tv) > 0;
}

// 函数声明：非阻塞获取键盘输入
char getCharNonBlocking() {
  if (kbhit()) {
    return getchar();
  }
  return 0;  // 没有输入时返回0
}

// 函数声明：设置终端为非缓冲模式
void setNonCanonicalMode() {
  struct termios new_termios;
  tcgetattr(STDIN_FILENO, &new_termios);
  new_termios.c_lflag &= ~(ICANON | ECHO);
  new_termios.c_cc[VMIN] = 0;
  new_termios.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

// 函数声明：恢复终端正常模式
void restoreTerminalMode() {
  struct termios new_termios;
  tcgetattr(STDIN_FILENO, &new_termios);
  new_termios.c_lflag |= (ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "rm_group");

  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  
  // 设置规划参数以最小化关节角转动
  move_group_interface.setPlanningTime(10.0);  // 增加规划时间以获得更好的解
  move_group_interface.setNumPlanningAttempts(20);  // 增加规划尝试次数
  
  // 设置规划器参数 - 使用RRTConnect算法，它通常能找到较好的路径
  move_group_interface.setPlannerId("RRTConnect");
  
  // 设置容差参数，适当放宽容差有助于找到更优路径
  move_group_interface.setGoalPositionTolerance(0.01);  // 位置容差 1cm
  move_group_interface.setGoalOrientationTolerance(0.01);  // 姿态容差 0.1弧度
  move_group_interface.setGoalJointTolerance(0.01);  // 关节角度容差
  
  // 获取并设置当前状态为起始状态，确保从当前位置开始规划
  RCLCPP_INFO(logger, "正在初始化机器人状态...");
  
  // 等待足够长的时间让状态监控器同步
  RCLCPP_INFO(logger, "等待机器人状态同步...");
  std::this_thread::sleep_for(std::chrono::seconds(3));
  rclcpp::spin_some(node);
  
  // 检查move_group是否可用
  bool move_group_available = false;
  int retry_count = 0;
  const int max_retries = 10;
  
  while(!move_group_available && retry_count < max_retries && rclcpp::ok()) {
    try {
      move_group_interface.setStartStateToCurrentState();
      move_group_available = true;
      RCLCPP_INFO(logger, "机器人状态获取成功！");
    } catch (const std::exception& e) {
      retry_count++;
      RCLCPP_WARN(logger, "尝试 %d/%d: 获取机器人状态失败: %s", retry_count, max_retries, e.what());
      std::this_thread::sleep_for(std::chrono::seconds(2));
      rclcpp::spin_some(node);
    }
  }
  
  if (!move_group_available) {
    RCLCPP_ERROR(logger, "无法获取机器人状态，请检查以下项目：");
    RCLCPP_ERROR(logger, "1. move_group 节点是否运行");
    RCLCPP_ERROR(logger, "2. joint_state_publisher 是否发布关节状态");
    RCLCPP_ERROR(logger, "3. robot_state_publisher 是否发布TF变换");
    RCLCPP_ERROR(logger, "4. 机器人URDF是否正确加载");
    return -1;
  }
  
  // 获取规划坐标系信息
  std::string planning_frame = move_group_interface.getPlanningFrame();
  std::string eef_link = "link6";
  
  RCLCPP_INFO(logger, "规划坐标系: %s", planning_frame.c_str());
  RCLCPP_INFO(logger, "末端执行器: %s", eef_link.c_str());
  
  // 设置参考坐标系为base_link
  move_group_interface.setPoseReferenceFrame("base_link");
  
  // 设置末端执行器链接
  move_group_interface.setEndEffectorLink(eef_link);
  
  RCLCPP_INFO(logger, "已设置参考坐标系: base_link");
  RCLCPP_INFO(logger, "已设置末端执行器: %s", eef_link.c_str());
  
  // 测试获取当前位置
  try {
    geometry_msgs::msg::PoseStamped test_pose = move_group_interface.getCurrentPose(eef_link);
    RCLCPP_INFO(logger, "测试位置获取成功: (%.3f, %.3f, %.3f)", 
                test_pose.pose.position.x, test_pose.pose.position.y, test_pose.pose.position.z);
    
    // 检查位置是否为零点，如果是则设置一个合理的起始位置
    if (std::abs(test_pose.pose.position.x) < 0.001 && 
        std::abs(test_pose.pose.position.y) < 0.001 && 
        std::abs(test_pose.pose.position.z) < 0.001) {
      
      RCLCPP_WARN(logger, "当前位置为零点，设置合理的起始位置...");
      
      geometry_msgs::msg::Pose start_pose;
      start_pose.position.x = 0.2;
      start_pose.position.y = 0.01;
      start_pose.position.z = 0.92;
      start_pose.orientation.w = 1.0;
      start_pose.orientation.x = 0.0;
      start_pose.orientation.y = 0.0;
      start_pose.orientation.z = 0.0;
      
      move_group_interface.setPoseTarget(start_pose);
      
      // 尝试规划到起始位置
      moveit::planning_interface::MoveGroupInterface::Plan start_plan;
      bool success = (move_group_interface.plan(start_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (success) {
        move_group_interface.execute(start_plan);
        RCLCPP_INFO(logger, "已移动到起始位置: (%.3f, %.3f, %.3f)", 
                    start_pose.position.x, start_pose.position.y, start_pose.position.z);
        
        // 等待移动完成
        std::this_thread::sleep_for(std::chrono::seconds(2));
        move_group_interface.setStartStateToCurrentState();
      } else {
        RCLCPP_WARN(logger, "无法移动到起始位置，继续使用当前位置");
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "测试位置获取失败: %s", e.what());
    return -1;
  }

  // 键盘控制参数
  const double step_size = 0.005;  // 减少到0.5mm - 更小步长实现更快停止
  const double angle_step = 0.02;   // 减少到0.01弧度（约0.5度）- 更小角度步长
  
  // 手动跟踪当前位置和姿态（避免getCurrentPose问题）
  geometry_msgs::msg::Pose current_tracked_pose;
  current_tracked_pose.position.x = 0.200;
  current_tracked_pose.position.y = 0.01;
  current_tracked_pose.position.z = 0.92;
  current_tracked_pose.orientation.w = 1.0;
  current_tracked_pose.orientation.x = 0.0;
  current_tracked_pose.orientation.y = 0.0;
  current_tracked_pose.orientation.z = 0.0;
  
  // 跟踪欧拉角（roll, pitch, yaw）
  double current_roll = 0.0;
  double current_pitch = 0.0;
  double current_yaw = 0.0;
  
  // 显示控制说明
  RCLCPP_INFO(logger, "==== 键盘控制机械臂笛卡尔运动 ====");
  RCLCPP_INFO(logger, "位置控制:");
  RCLCPP_INFO(logger, "  w/s: X轴 前进/后退");
  RCLCPP_INFO(logger, "  a/d: Y轴 左移/右移");
  RCLCPP_INFO(logger, "  q/e: Z轴 上升/下降");
  RCLCPP_INFO(logger, "姿态控制:");
  RCLCPP_INFO(logger, "  i/k: Roll 旋转 +/-");
  RCLCPP_INFO(logger, "  u/o: Pitch 旋转 +/-");
  RCLCPP_INFO(logger, "  j/l: Yaw 旋转 +/-");
  RCLCPP_INFO(logger, "其他:");
  RCLCPP_INFO(logger, "  r: 复位姿态");
  RCLCPP_INFO(logger, "  x: 退出程序");
  RCLCPP_INFO(logger, "  位置步长: %.4f米, 角度步长: %.1f度", step_size, angle_step * 180.0 / M_PI);
  RCLCPP_INFO(logger, "*** 按住按键持续移动，松开立即停止 ***");
  RCLCPP_INFO(logger, "================================");

  // 设置终端为非缓冲模式
  setNonCanonicalMode();

  // 添加强制清理函数
  auto clearMoveQueue = [&]() {
    // 只调用一次stop，避免过多的停止事件
    move_group_interface.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    rclcpp::spin_some(node);
  };

  // 键盘控制循环
  char last_key = 0;
  char current_active_key = 0;  // 当前活跃的按键
  bool is_moving = false;
  bool just_stopped = false;  // 添加标志避免重复停止
  auto last_move_time = std::chrono::steady_clock::now();
  const auto move_interval = std::chrono::milliseconds(20);  // 调整回20ms，更稳定
  
  while(rclcpp::ok()) {
    
    // 获取键盘输入
    char key = getCharNonBlocking();
    
    // 更新当前活跃按键状态
    if (key != 0) {
      current_active_key = key;  // 有按键输入时更新活跃按键
      just_stopped = false;      // 重置停止标志
    } else if (current_active_key != 0) {
      // 没有按键输入且之前有活跃按键，说明按键被松开
      current_active_key = 0;
    }
    
    // 处理按键状态变化
    if (current_active_key != last_key) {
      if (current_active_key != 0) {
        // 有新按键按下
        if (last_key == 0 && just_stopped) {
          clearMoveQueue();
          just_stopped = false;
        }
        
        // 显示当前状态（只在按键改变时显示）
        RCLCPP_INFO(logger, "按键: %c - 当前位置: (%.3f, %.3f, %.3f)", 
                    current_active_key,
                    current_tracked_pose.position.x, 
                    current_tracked_pose.position.y, 
                    current_tracked_pose.position.z);
        RCLCPP_INFO(logger, "当前姿态: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°", 
                    current_roll * 180.0 / M_PI,
                    current_pitch * 180.0 / M_PI, 
                    current_yaw * 180.0 / M_PI);
        
        if (current_active_key == 'x') {
          RCLCPP_INFO(logger, "退出程序");
          clearMoveQueue();
          break;
        } else if (current_active_key == 'r') {
          clearMoveQueue();
          current_roll = 0.0;
          current_pitch = 0.0;
          current_yaw = 0.0;
          RCLCPP_INFO(logger, "姿态复位到 (0°, 0°, 0°)");
        }
        
        is_moving = true;
      } else {
        // 按键松开
        if (last_key != 0 && !just_stopped) {
          RCLCPP_INFO(logger, "检测到按键松开 - 停止移动");
          clearMoveQueue();
          just_stopped = true;
        }
        is_moving = false;
      }
      
      last_key = current_active_key;
    }
    
    // 如果正在移动且到了移动时间间隔
    if (is_moving && current_active_key != 0 && current_active_key != 'x' && current_active_key != 'r') {
      auto current_time = std::chrono::steady_clock::now();
      if (current_time - last_move_time >= move_interval) {
        
        // 直接使用current_active_key，不进行额外的按键检测
        // 这样避免了在移动执行过程中的误判
        
        // 创建目标位置
        geometry_msgs::msg::Pose target_pose = current_tracked_pose;
        bool move_requested = true;
        
        // 根据按键设置目标位置和姿态
        switch(current_active_key) {
          case 'w': // X轴正方向
            target_pose.position.x += step_size;
            break;
          case 's': // X轴负方向
            target_pose.position.x -= step_size;
            break;
          case 'a': // Y轴正方向
            target_pose.position.y += step_size;
            break;
          case 'd': // Y轴负方向
            target_pose.position.y -= step_size;
            break;
          case 'q': // Z轴正方向
            target_pose.position.z += step_size;
            break;
          case 'e': // Z轴负方向
            target_pose.position.z -= step_size;
            break;
          case 'i': // Roll 正方向
            current_roll += angle_step;
            break;
          case 'k': // Roll 负方向
            current_roll -= angle_step;
            break;
          case 'u': // Pitch 正方向
            current_pitch += angle_step;
            break;
          case 'o': // Pitch 负方向
            current_pitch -= angle_step;
            break;
          case 'j': // Yaw 正方向
            current_yaw += angle_step;
            break;
          case 'l': // Yaw 负方向
            current_yaw -= angle_step;
            break;
          default:
            move_requested = false;
            break;
        }
        
        // 执行移动
        if(move_requested) {
          // 将欧拉角转换为四元数
          double cy = cos(current_yaw * 0.5);
          double sy = sin(current_yaw * 0.5);
          double cp = cos(current_pitch * 0.5);
          double sp = sin(current_pitch * 0.5);
          double cr = cos(current_roll * 0.5);
          double sr = sin(current_roll * 0.5);

          target_pose.orientation.w = cr * cp * cy + sr * sp * sy;
          target_pose.orientation.x = sr * cp * cy - cr * sp * sy;
          target_pose.orientation.y = cr * sp * cy + sr * cp * sy;
          target_pose.orientation.z = cr * cp * sy - sr * sp * cy;
          
          // 创建笛卡尔路径
          std::vector<geometry_msgs::msg::Pose> waypoints;
          waypoints.push_back(current_tracked_pose);
          waypoints.push_back(target_pose);
          
          // 计算笛卡尔路径
          moveit_msgs::msg::RobotTrajectory trajectory;
          const double eef_step = 0.0002;  // 调整为0.2mm，平衡速度和精度
          const double jump_threshold = 0.0;  // 跳跃阈值
          
          double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
          
          // 检查路径规划是否成功
          if(fraction > 0.30) {  // 调整回30%，更稳定
            // 执行轨迹
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            
            auto execution_result = move_group_interface.execute(plan);
            if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
              // 更新跟踪的位置
              current_tracked_pose = target_pose;
            } else {
              // 执行失败时才输出调试信息
              RCLCPP_DEBUG(logger, "轨迹执行失败");
            }
  } else {
            // 规划失败时才输出调试信息
            RCLCPP_DEBUG(logger, "路径规划成功率过低: %.1f%%", fraction * 100.0);
          }
        }
        
        last_move_time = current_time;
      }
    }
    
    // 处理ROS消息
    rclcpp::spin_some(node);
    
    // 适中的循环频率 - 2ms
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  
  // 恢复终端正常模式
  restoreTerminalMode();
  
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}