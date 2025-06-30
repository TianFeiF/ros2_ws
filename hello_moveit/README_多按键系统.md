# MoveIt 多按键键盘控制系统

## 系统架构

本系统采用分离式节点架构，支持多按键同时按下进行复合运动：

```
keyboard_publisher → /keyboard_input (KeyboardState) → moveit_subscriber
```

### 节点说明

1. **keyboard_publisher**: 键盘检测节点
   - 高频检测键盘输入 (5ms轮询)
   - 固定频率发布按键状态 (50Hz)
   - 支持多按键同时检测

2. **moveit_subscriber**: 运动规划节点
   - 订阅键盘状态消息
   - 实时计算复合运动
   - 执行MoveIt路径规划

## 自定义消息类型

```
hello_moveit/msg/KeyboardState.msg
```

包含所有按键的0/1状态：
- 位置控制: w/s (X轴), a/d (Y轴), q/e (Z轴)
- 姿态控制: i/k (Roll), u/o (Pitch), j/l (Yaw)
- 特殊功能: r (复位), x (退出)

## 多按键功能

### 支持的复合运动
- **对角线移动**: 同时按下 `w+a` 实现X+Y方向移动
- **螺旋运动**: 同时按下 `q+j` 实现上升+旋转
- **复杂6DOF**: 可同时控制位置和姿态的任意组合

### 按键映射
```
位置控制 (可同时按下):
  w/s: X轴 前进/后退 (±10mm)
  a/d: Y轴 左移/右移 (±10mm)  
  q/e: Z轴 上升/下降 (±10mm)

姿态控制 (可同时按下):
  i/k: Roll 旋转 ±1°
  u/o: Pitch 旋转 ±1°
  j/l: Yaw 旋转 ±1°

特殊功能:
  r: 复位姿态到 (0°, 0°, 0°)
  x: 退出程序
```

## 技术特性

### 消息发布
- **发布频率**: 50Hz固定频率
- **检测频率**: 200Hz键盘轮询
- **消息格式**: 结构化按键状态 (0/1)
- **时间戳**: 每条消息包含准确时间戳

### 运动控制
- **步长**: 位置10mm, 角度1°
- **路径规划**: Cartesian路径，1mm精度
- **成功率阈值**: 30%
- **执行频率**: 20Hz运动更新

### 实时性能
- **响应延迟**: <50ms按键到运动
- **停止响应**: 立即停止 (<20ms)
- **多线程**: 异步执行避免阻塞
- **状态同步**: 线程安全的状态管理

## 启动方法

```bash
# 编译
colcon build --packages-select hello_moveit

# 加载环境
source install/setup.bash

# 启动系统
ros2 launch hello_moveit keyboard_control_simple.launch.py
```

## 系统优势

1. **真正的多按键支持**: 不再是单按键切换，而是多按键状态并发
2. **固定频率发布**: 稳定的50Hz消息流，避免事件驱动的不稳定性
3. **结构化消息**: 清晰的按键状态表示，便于扩展和调试
4. **模块化架构**: 键盘检测与运动规划完全解耦
5. **ROS2标准**: 使用标准的发布-订阅模式
6. **实时性能**: 高频检测+低延迟响应

## 调试工具

```bash
# 查看按键状态
ros2 topic echo /keyboard_input

# 查看话题频率
ros2 topic hz /keyboard_input

# 查看消息结构
ros2 interface show hello_moveit/msg/KeyboardState
```

这个系统实现了真正的多按键同时控制，支持复杂的6DOF复合运动，是专业级的机器人键盘控制解决方案。 