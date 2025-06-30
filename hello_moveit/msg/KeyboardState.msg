# 键盘状态消息
# 每个字段用0/1表示按键是否按下

# 位置控制按键
uint8 w_pressed    # X轴正方向
uint8 s_pressed    # X轴负方向  
uint8 a_pressed    # Y轴正方向
uint8 d_pressed    # Y轴负方向
uint8 q_pressed    # Z轴正方向
uint8 e_pressed    # Z轴负方向

# 姿态控制按键
uint8 i_pressed    # Roll正方向
uint8 k_pressed    # Roll负方向
uint8 u_pressed    # Pitch正方向
uint8 o_pressed    # Pitch负方向
uint8 j_pressed    # Yaw正方向
uint8 l_pressed    # Yaw负方向

# 特殊功能按键
uint8 r_pressed    # 复位姿态
uint8 x_pressed    # 退出程序

# 时间戳
builtin_interfaces/Time stamp 