# MoveIt 分离节点架构 - 键盘控制

## 架构概述

这个新架构将键盘检测和运动规划分离成两个独立的ROS2节点，通过话题通信实现解耦：

```
┌─────────────────────┐    keyboard_input    ┌─────────────────────┐
│                     │    (std_msgs/Char)   │                     │
│  keyboard_publisher │ ──────────────────► │  moveit_subscriber  │
│                     │                      │                     │
│  - 键盘检测         │                      │  - 运动规划         │
│  - 状态发布         │                      │  - 轨迹执行         │
│  - 终端控制         │                      │  - MoveIt接口       │
└─────────────────────┘                      └─────────────────────┘
```

## 节点功能

### 1. keyboard_publisher (键盘发布节点)
- **功能**: 专门负责键盘输入检测
- **输出**: 发布按键状态到 `/keyboard_input` 话题
- **特点**: 
  - 高频率检测(1ms)
  - 状态变化时才发布
  - 独立的终端控制
  - 无MoveIt依赖，启动快速

### 2. moveit_subscriber (运动规划订阅节点)  
- **功能**: 专门负责机械臂运动控制
- **输入**: 订阅 `/keyboard_input` 话题
- **特点**:
  - 异步运动执行
  - 独立的MoveIt初始化
  - 线程化连续运动
  - 即时停止响应

## 使用方法

### 方法1: 使用启动文件 (推荐)
```bash
# 编译
cd ~/ros2_ws
colcon build --packages-select hello_moveit
source install/setup.bash

# 启动两个节点
ros2 launch hello_moveit keyboard_control_simple.launch.py
```

### 方法2: 手动启动节点
```bash
# 终端1: 启动键盘节点
cd ~/ros2_ws
source install/setup.bash
ros2 run hello_moveit keyboard_publisher

# 终端2: 启动运动规划节点  
cd ~/ros2_ws
source install/setup.bash
ros2 run hello_moveit moveit_subscriber
```

### 方法3: 分离终端启动
```bash
# 启动运动规划节点(后台)
ros2 run hello_moveit moveit_subscriber &

# 启动键盘节点(前台，用于输入)
ros2 run hello_moveit keyboard_publisher
```

## 控制说明

### 位置控制
- **W/S**: X轴 前进/后退 (5mm步长)
- **A/D**: Y轴 左移/右移 (5mm步长)  
- **Q/E**: Z轴 上升/下降 (5mm步长)

### 姿态控制
- **I/K**: Roll轴 旋转 +/- (1°步长)
- **U/O**: Pitch轴 旋转 +/- (1°步长)
- **J/L**: Yaw轴 旋转 +/- (1°步长)

### 特殊功能
- **R**: 复位姿态到 (0°, 0°, 0°)
- **X**: 退出程序

## 架构优势

### 1. 模块化设计
- **解耦**: 键盘检测与运动规划完全分离
- **独立性**: 每个节点可以独立开发和测试
- **可扩展**: 易于添加新的输入设备或控制算法

### 2. 实时性提升
- **专用线程**: 键盘检测有专用线程，不受MoveIt影响
- **异步执行**: 运动规划异步执行，不阻塞按键检测
- **即时响应**: 按键状态变化立即通过话题传递

### 3. 稳定性增强
- **故障隔离**: 一个节点崩溃不影响另一个
- **资源分离**: 避免资源竞争和冲突
- **调试友好**: 可以独立调试每个模块

### 4. 灵活部署
- **分布式**: 可以在不同机器上运行
- **多输入**: 支持多个键盘节点同时连接
- **热插拔**: 可以动态启停节点

## 话题通信

### `/keyboard_input` (std_msgs/Char)
```cpp
// 消息格式
std_msgs::msg::Char
{
  uint8 data  // 当前按键字符，0表示无按键
}
```

### 通信特点
- **状态驱动**: 只在按键状态变化时发布
- **可靠传输**: 使用ROS2的可靠QoS
- **低延迟**: 队列深度为10，保证实时性

## 监控和调试

### 查看话题
```bash
# 监控按键话题
ros2 topic echo /keyboard_input

# 查看话题信息
ros2 topic info /keyboard_input

# 检查节点状态
ros2 node list
ros2 node info /keyboard_publisher
ros2 node info /moveit_subscriber
```

### 性能监控
```bash
# 话题频率
ros2 topic hz /keyboard_input

# 节点资源使用
top -p $(pgrep keyboard_publisher)
top -p $(pgrep moveit_subscriber)
```

## 故障排除

### 1. 键盘节点无响应
```bash
# 检查节点是否运行
ros2 node list | grep keyboard_publisher

# 检查话题发布
ros2 topic echo /keyboard_input
```

### 2. 运动节点无响应
```bash
# 检查订阅状态
ros2 topic info /keyboard_input

# 检查MoveIt状态
ros2 topic list | grep move_group
```

### 3. 通信问题
```bash
# 检查话题连接
ros2 topic info /keyboard_input

# 重启节点
killall keyboard_publisher moveit_subscriber
ros2 launch hello_moveit keyboard_control_simple.launch.py
```

## 扩展开发

### 添加新输入设备
1. 创建新的发布节点
2. 发布到相同话题 `/keyboard_input`
3. 运动节点自动响应

### 添加新控制算法
1. 创建新的订阅节点
2. 订阅 `/keyboard_input` 话题
3. 实现自定义运动逻辑

### 多机器人控制
1. 为每个机器人创建独立话题
2. 使用命名空间区分: `/robot1/keyboard_input`
3. 配置对应的订阅节点

## 版本对比

| 特性 | 单节点版本 | 分离节点版本 |
|------|------------|--------------|
| 架构复杂度 | 简单 | 中等 |
| 实时性 | 一般 | 优秀 |
| 稳定性 | 一般 | 优秀 |
| 可扩展性 | 有限 | 优秀 |
| 调试难度 | 中等 | 简单 |
| 部署灵活性 | 有限 | 优秀 |

## 总结

分离节点架构通过ROS2的发布-订阅机制实现了键盘检测和运动规划的完全解耦，提供了更好的实时性、稳定性和可扩展性。这种设计模式特别适合复杂的机器人控制系统。 