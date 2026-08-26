#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <algorithm>

class RemoteOverrideNode : public rclcpp::Node {
public:
    RemoteOverrideNode() : Node("remote_override") {
        vx_max_ = this->declare_parameter("vx_max", 1.5);
        vyaw_max_ = this->declare_parameter("vyaw_max", 1.0);
        threshold_ = this->declare_parameter("activation_threshold", 0.05);

        remote_sub_ = this->create_subscription<mavros_msgs::msg::ManualControl>(
            "/usv_1/control/body_cmd", 1,
            [this](const mavros_msgs::msg::ManualControl::SharedPtr msg) {
                onRemoteCmd(msg);
            });

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_remote", 1);

        RCLCPP_INFO(this->get_logger(), "Remote override node started");
    }

private:
    void onRemoteCmd(const mavros_msgs::msg::ManualControl::SharedPtr msg) {
        // 死区处理
        float y = std::abs(msg->y) < threshold_ ? 0.0f : msg->y;
        float z = std::abs(msg->z) < threshold_ ? 0.0f : msg->z;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(z * vx_max_, -vx_max_, vx_max_);
        cmd.linear.y = 0.0;
        cmd.angular.z = std::clamp(y * vyaw_max_, -vyaw_max_, vyaw_max_);

        cmd_vel_pub_->publish(cmd);
    }

    double vx_max_, vyaw_max_, threshold_;
    rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr remote_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RemoteOverrideNode>());
    rclcpp::shutdown();
    return 0;
}
