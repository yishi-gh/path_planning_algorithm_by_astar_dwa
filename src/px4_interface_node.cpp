#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

class PX4InterfaceNode : public rclcpp::Node {
public:
    PX4InterfaceNode() : Node("px4_interface_node") {
        // 订阅无人机状态
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&PX4InterfaceNode::stateCallback, this, std::placeholders::_1));

        // 订阅来自 DWA 节点的控制指令
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel", 10,
            std::bind(&PX4InterfaceNode::cmdVelCallback, this, std::placeholders::_1));

        // 创建服务客户端，用于设置模式和解锁
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        // 定时器，用于周期性检查状态并发送指令
        auto timer_callback = [this]() -> void {
            // 确保无人机已连接且处于 Offboard 模式
            if (current_state_.mode != "OFFBOARD") {
                setOffboardMode();
            }
            if (current_state_.armed && current_state_.mode == "OFFBOARD") {
                // 如果已解锁且处于 Offboard 模式，则发送缓存的指令
                velocity_pub_->publish(current_cmd_vel_);
            }
        };

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50 Hz
            timer_callback);

        // 发布速度指令，需要在循环中持续发布以保持 Offboard 模式
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel", 10);
    }

private:
    // 成员变量
    mavros_msgs::msg::State current_state_; // 当前无人机状态
    geometry_msgs::msg::Twist current_cmd_vel_; // 当前缓存的速度指令
    
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief 状态回调函数，更新无人机状态
     */
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
    }

    /**
     * @brief 指令回调函数，缓存来自 DWA 的速度指令
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_cmd_vel_ = *msg;
    }

    /**
     * @brief 切换到 OFFBOARD 模式
     */
    void setOffboardMode() {
        RCLCPP_INFO(this->get_logger(), "正在请求 OFFBOARD 模式...");
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";
        
        // 异步调用服务
        set_mode_client_->async_send_request(request, [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            if (future.get()->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "成功切换到 OFFBOARD 模式。");
            } else {
                RCLCPP_WARN(this->get_logger(), "切换 OFFBOARD 模式失败。");
            }
        });
    }

    /**
     * @brief 解锁无人机
     */
    void arm() {
        RCLCPP_INFO(this->get_logger(), "正在请求解锁无人机...");
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        
        arming_client_->async_send_request(request, [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            if (future.get()->success) {
                RCLCPP_INFO(this->get_logger(), "成功解锁无人机。");
            } else {
                RCLCPP_WARN(this->get_logger(), "解锁无人机失败。");
            }
        });
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4InterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}