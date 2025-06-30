#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rm_ros_interfaces/msg/jointpos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ServoToRealRobot : public rclcpp::Node
{
public:
    ServoToRealRobot() : Node("servo_to_real_robot")
    {
        // 订阅MoveIt Servo输出的轨迹命令
        trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/rm_group_controller/joint_trajectory",
            10,
            std::bind(&ServoToRealRobot::trajectory_callback, this, std::placeholders::_1));
        
        // 发布给真实机械臂的命令
        joint_cmd_pub_ = this->create_publisher<rm_ros_interfaces::msg::Jointpos>(
            "/rm_driver/movej_canfd_cmd", 10);
        
        // 订阅当前关节状态
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&ServoToRealRobot::joint_state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Servo to real robot bridge started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /rm_group_controller/joint_trajectory");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /rm_driver/movej_canfd_cmd");
    }

private:
    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
            return;
        }
        
        // 获取最后一个轨迹点的位置
        const auto& last_point = msg->points.back();
        
        if (last_point.positions.size() != 6)
        {
            RCLCPP_WARN(this->get_logger(), "Expected 6 joint positions, got %zu", 
                       last_point.positions.size());
            return;
        }
        
        // 创建Jointpos消息
        auto joint_cmd = rm_ros_interfaces::msg::Jointpos();
        joint_cmd.joint.resize(6);
        
        // 复制关节位置
        for (size_t i = 0; i < 6; ++i)
        {
            if(i == 4 || i == 0 || i == 3 || i == 5)
            {
                joint_cmd.joint[i] = -static_cast<float>(last_point.positions[i]);
            }
            else
            {
                joint_cmd.joint[i] = static_cast<float>(last_point.positions[i]);
            }
        }
        
        // 设置其他参数
        joint_cmd.follow = false;  // 不跟随模式
        joint_cmd.expand = 0.0;    // 扩展参数
        joint_cmd.dof = 6;         // 6自由度
        
        // 发布命令
        joint_cmd_pub_->publish(joint_cmd);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Published joint command: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    joint_cmd.joint[0], joint_cmd.joint[1], joint_cmd.joint[2],
                    joint_cmd.joint[3], joint_cmd.joint[4], joint_cmd.joint[5]);
    }
    
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 存储当前关节状态，用于调试或安全检查
        current_joint_states_ = msg;
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr joint_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState::SharedPtr current_joint_states_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoToRealRobot>());
    rclcpp::shutdown();
    return 0;
} 