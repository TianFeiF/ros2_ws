# 机器人状态监控工具

这个工具包含了三个主要功能：实时监控、数据记录和数据分析。

## 安装依赖

```bash
pip3 install matplotlib pandas numpy
```

## 使用方法

### 方法1：使用启动脚本（推荐）

```bash
cd /path/to/ros2_ws
source install/setup.bash
./src/hello_moveit/scripts/start_monitor.sh
```

### 方法2：直接运行单个脚本

#### 1. 实时监控和绘图

显示6个实时更新的图表窗口：
- 关节位置、速度、加速度
- 末端位置、姿态
- 3D轨迹

```bash
python3 src/hello_moveit/scripts/robot_state_monitor.py
```

#### 2. 数据记录

将机器人状态数据记录到CSV文件：

```bash
python3 src/hello_moveit/scripts/data_logger.py
```

数据文件将保存在 `robot_data/` 目录下：
- `joint_data_YYYYMMDD_HHMMSS.csv` - 关节数据
- `end_effector_data_YYYYMMDD_HHMMSS.csv` - 末端位置数据

#### 3. 数据分析

分析已记录的数据并生成图表：

```bash
# 分析最新数据
python3 src/hello_moveit/scripts/analyze_data.py

# 分析指定数据文件
python3 src/hello_moveit/scripts/analyze_data.py --joint-file joint_data_xxx.csv --end-effector-file end_effector_data_xxx.csv

# 只生成报告，不显示图表
python3 src/hello_moveit/scripts/analyze_data.py --no-plot
```

## 功能特性

### 实时监控 (robot_state_monitor.py)
- **6个关节状态**：位置、速度、加速度实时曲线
- **末端位置**：X、Y、Z坐标实时显示
- **末端姿态**：Roll、Pitch、Yaw角度实时显示
- **3D轨迹**：末端执行器的3D运动轨迹
- **自动缩放**：图表坐标轴自动调整
- **滑动窗口**：保持最近1000个数据点（约100秒）

### 数据记录 (data_logger.py)
- **关节数据**：位置、速度、力矩，频率跟随 `/joint_states` 话题
- **末端数据**：位置、四元数、欧拉角，10Hz频率
- **CSV格式**：便于后续分析和处理
- **时间戳**：精确的时间记录
- **自动命名**：基于时间戳的文件命名

### 数据分析 (analyze_data.py)
- **关节分析**：
  - 位置、速度、力矩时间序列
  - 速度分布直方图
  - 位置变化范围统计
  - 速度统计（最大值、RMS值）
- **末端分析**：
  - 位置和姿态时间序列
  - 3D轨迹可视化
  - 末端速度分析
  - 运动距离统计
- **数据报告**：详细的数值分析报告

## 数据格式

### 关节数据 (joint_data_*.csv)
```
timestamp,time_sec,joint1_pos,joint2_pos,...,joint1_vel,joint2_vel,...,joint1_eff,joint2_eff,...
```

### 末端数据 (end_effector_data_*.csv)
```
timestamp,time_sec,pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w,roll,pitch,yaw
```

## 使用场景

1. **实时监控**：运行机械臂时实时观察状态
2. **性能分析**：记录运动数据进行离线分析
3. **轨迹验证**：验证规划的轨迹是否正确执行
4. **调试工具**：识别运动中的异常或问题
5. **数据收集**：为机器学习或优化算法收集训练数据

## 注意事项

- 确保ROS2环境已正确设置
- 确保机器人驱动和MoveIt正在运行
- 实时监控需要图形界面支持
- 数据文件可能会占用较多磁盘空间
- 建议定期清理旧的数据文件

## 故障排除

### 常见问题

1. **"未找到数据文件"**
   - 确保先运行数据记录功能
   - 检查 `robot_data/` 目录是否存在

2. **TF获取失败**
   - 确保机器人驱动正在运行
   - 检查TF树是否正确发布

3. **图表不显示**
   - 确保系统支持图形界面
   - 检查matplotlib是否正确安装

4. **权限错误**
   - 运行 `chmod +x src/hello_moveit/scripts/*`

### 依赖检查

```bash
# 检查Python包
python3 -c "import matplotlib, pandas, numpy, rclpy; print('所有依赖已安装')"

# 检查ROS话题
ros2 topic list | grep -E "(joint_states|tf)"
``` 