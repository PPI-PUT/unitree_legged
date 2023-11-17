#include "unitree_a1_joystick/unitree_a1_joystick.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>

double calcMappingdouble(const double input, const double sensitivity)
{
    const double exponent = 1.0 / (std::max(0.001, std::min(1.0, sensitivity)));
    return std::pow(input, exponent);
}

namespace unitree_a1_legged
{
    class UnitreeJoystickNode : public rclcpp::Node
    {
    public:
        explicit UnitreeJoystickNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("unitree_a1_joystick", options)
        {
            // Parameters
            update_rate_ = this->declare_parameter<double>("update_rate", 50.0);
            linear_ratio_ = this->declare_parameter<double>("linear_ratio", 0.5);
            angular_ratio_ = this->declare_parameter<double>("angular_ratio", 0.5);
            linear_x_sensitivity_ = this->declare_parameter<double>("linear_x_sensitivity", 0.5);
            linear_y_sensitivity_ = this->declare_parameter<double>("linear_y_sensitivity", 0.5);
            linear_velocity_limit_ = this->declare_parameter<double>("linear_velocity_limit", 0.5);
            angular_z_sensitivity_ = this->declare_parameter<double>("angular_z_sensitivity", 0.5);
            angular_velocity_limit_ = this->declare_parameter<double>("angular_velocity_limit", 0.5);
            // Create timer
            const auto period_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / update_rate_));
            timer_ = rclcpp::create_timer(
                this, get_clock(), period_ns, std::bind(&UnitreeJoystickNode::timerCallback, this));

            // Create publishers
            twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/cmd_vel", 1);
            // Create subscribers
            joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/unitree_lowlevel/joy", 1, std::bind(&UnitreeJoystickNode::receiveJoystickCallback, this, std::placeholders::_1));
        }

    private:
        double update_rate_;
        double linear_ratio_;
        double angular_ratio_;
        double linear_x_sensitivity_;
        double linear_y_sensitivity_;
        double linear_velocity_limit_;
        double angular_z_sensitivity_;
        double angular_velocity_limit_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
        rclcpp::Time last_joy_received_time_;
        std::shared_ptr<UnitreeJoystick> joy_;
        void receiveJoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            last_joy_received_time_ = msg->header.stamp;
            joy_ = std::make_shared<UnitreeJoystick>(*msg);
        }
        void publishTwist()
        {
            auto twist_msg = geometry_msgs::msg::TwistStamped();
            twist_msg.header.stamp = this->now();
            if (joy_->linear_x())
            {
                twist_msg.twist.linear.x =
                    linear_ratio_ * calcMappingdouble(static_cast<double>(joy_->linear_x()), linear_x_sensitivity_);
            }
            if (joy_->linear_y())
            {
                twist_msg.twist.linear.y =
                    linear_ratio_ * calcMappingdouble(static_cast<double>(joy_->linear_y()), linear_y_sensitivity_);
            }
            if (joy_->angular_z())
            {
                twist_msg.twist.angular.z =
                    angular_ratio_ * calcMappingdouble(static_cast<double>(joy_->angular_z()), angular_z_sensitivity_);
            }
            twist_publisher_->publish(twist_msg);
        }
        void timerCallback()
        {

            if (!isDataReady())
            {
                return;
            }
            publishTwist();
        }
        bool isDataReady()
        {
            if (!joy_)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
                    "waiting for joy msg...");
                return false;
            }
            constexpr auto timeout = 2.0;
            const auto time_diff = this->now() - last_joy_received_time_;
            if (time_diff.seconds() > timeout)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(), "joy msg is timeout");
                return false;
            }
            return true;
        }
    };
} // namespace unitree_a1_legged